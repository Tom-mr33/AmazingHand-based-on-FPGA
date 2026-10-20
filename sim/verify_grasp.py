"""MuJoCo 数字孪生 —— 抓取参数表批量验证。

流程（对应 grasp_table 每条参数）::

    angles_deg(8 关节, 相对中位) → position 执行器 ctrl(rad)
      → 前向仿真至稳定 → 指尖间距矩阵 + 可达性 + 自碰检查 → 判定报告

用途：
1. grasp_table.json 参数从 ``source=default`` 升级为 ``source=mujoco``；
2. 仿真/实物同角度对比的基准数据（render_compare.py 引用同一 PoseResult）。

引用模型：``AmazingHand/Demo/AHSimulation/AH_Right/mjcf/scene.xml``
（见 sim/ah_model/README.md，引入不改）。

Caveat：上游执行器参数（kp=10/forcerange=±10）为理想值，**不是** SCS0009
真实力矩特性——本工具当前验证"几何可达性与包络间隙"，接触力数值仅定性参考；
真实力控阈值以实机标定（待定项 2）为准。

运行::

    python sim/verify_grasp.py                 # 验证 grasp_table.json 全表
    python sim/verify_grasp.py --table <path>  # 指定参数表
    python sim/verify_grasp.py --pose 60,0,60,0,60,0,60,0   # 单姿态快查
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path

import mujoco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TABLE = REPO_ROOT / "host/app/strategy/grasp_table.json"
DEFAULT_MODEL = REPO_ROOT / "AmazingHand/Demo/AHSimulation/AHSimulation/AH_Right/mjcf/scene.xml"

N_JOINTS = 8
#: 每条参数的稳定化仿真时长（秒）
SETTLE_S = 1.5
#: 稳定判定：末段 qpos 最大变化（rad）
SETTLE_TOL_RAD = 1e-3
#: 可达判定：执行器目标与实际关节角残差（rad），超过视为被限位/自碰卡住
REACH_TOL_RAD = 0.02
#: 关节限位（模型定义 ±90°）
JOINT_LIMIT_RAD = math.pi / 2


@dataclass
class PoseResult:
    """单姿态仿真结论（render_compare.py 复用）。"""

    label: str
    converged: bool = False          # 仿真数值稳定
    all_reachable: bool = False      # 8 关节均到达目标（残差 < tol）
    max_residual_rad: float = 0.0
    tip_gap_mm: dict = field(default_factory=dict)  # "tipA-tipB" → 距离
    min_gap_mm: float = float("inf")
    self_contacts: int = 0           # 手部 geom 自碰数（不含地板）
    limit_hits: int = 0              # 撞限位的关节数
    floor_contacts: int = 0
    tip_xyz_mm: dict = field(default_factory=dict)


class HandSim:
    """AmazingHand 右手仿真封装（前向位置控制，无 IK 依赖）。"""

    def __init__(self, model_path: Path | str = DEFAULT_MODEL):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        # 执行器顺序 = finger1_motor1..2, finger2..., 即舵机 ID 1-8
        self.actuator_names = [self.model.actuator(i).name for i in range(self.model.nu)]
        if self.model.nu != N_JOINTS:
            raise RuntimeError(f"expect {N_JOINTS} actuators, got {self.model.nu}")
        self.tip_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"tip{i}")
            for i in range(1, 5)
        ]
        self.floor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self._t = 0.0

    # ------------------------------------------------------------------

    def apply_pose(self, angles_deg: tuple[float, ...], middle_deg: tuple[float, ...] = (0.0,) * N_JOINTS) -> None:
        """8 关节角（度，相对中位）写入 position 执行器目标。"""
        if len(angles_deg) != N_JOINTS:
            raise ValueError("need exactly 8 joint angles")
        for i, ang in enumerate(angles_deg):
            target_rad = math.radians(middle_deg[i] + ang)
            # 模型执行器 inheritrange ±90°，超限目标会被截断——记录为不可达
            self.data.ctrl[i] = max(-JOINT_LIMIT_RAD, min(JOINT_LIMIT_RAD, target_rad))

    def settle(self, duration_s: float = SETTLE_S) -> bool:
        """步进至稳定；返回是否数值收敛（末段 qpos 变化 < tol）。"""
        n_steps = int(duration_s / self.model.opt.timestep)
        q_tail: np.ndarray | None = None
        tail_steps = max(10, n_steps // 10)
        for k in range(n_steps):
            mujoco.mj_step(self.model, self.data)
            self._t += self.model.opt.timestep
            if k >= n_steps - tail_steps:
                if q_tail is None:
                    q_tail = np.zeros((tail_steps, self.model.nq))
                q_tail[k - (n_steps - tail_steps)] = self.data.qpos.copy()
        assert q_tail is not None
        return bool(float(np.abs(q_tail - q_tail[-1]).max()) < SETTLE_TOL_RAD)

    def metrics(self, label: str, middle_deg: tuple[float, ...] = (0.0,) * N_JOINTS) -> PoseResult:
        """当前状态的抓取指标。"""
        res = PoseResult(label=label)
        mujoco.mj_forward(self.model, self.data)

        # 可达性：|ctrl - qpos|（仅位置关节；模型 qpos 含四元数自由量，按 actuator 对应 joint 取）
        residuals = []
        for i in range(N_JOINTS):
            jid = self.model.actuator_trnid[i, 0]  # slider/hinge joint id
            qadr = self.model.jnt_qposadr[jid]
            residuals.append(abs(self.data.ctrl[i] - self.data.qpos[qadr]))
            lo, hi = self.model.jnt_range[jid]
            if self.data.qpos[qadr] <= lo + 1e-3 or self.data.qpos[qadr] >= hi - 1e-3:
                res.limit_hits += 1
        res.max_residual_rad = float(max(residuals))
        res.all_reachable = bool(res.max_residual_rad < REACH_TOL_RAD)

        # 指尖位置与两两间距
        tips = {f"tip{i + 1}": self.data.site_xpos[tid].copy() for i, tid in enumerate(self.tip_ids)}
        res.tip_xyz_mm = {k: [round(float(v) * 1000, 2) for v in xyz] for k, xyz in tips.items()}
        names = list(tips)
        for a in range(len(names)):
            for b in range(a + 1, len(names)):
                d_mm = float(np.linalg.norm(tips[names[a]] - tips[names[b]]) * 1000)
                res.tip_gap_mm[f"{names[a]}-{names[b]}"] = round(d_mm, 2)
                res.min_gap_mm = min(res.min_gap_mm, d_mm)

        # 接触：区分手-自碰与手-地板
        for c in range(self.data.ncon):
            g1 = self.model.geom(self.data.contact[c].geom1).id
            g2 = self.model.geom(self.data.contact[c].geom2).id
            if self.floor_id in (g1, g2):
                res.floor_contacts += 1
            else:
                res.self_contacts += 1
        return res


def verify_entry(entry: dict, sim: HandSim, label: str,
                 middle_deg: tuple[float, ...]) -> PoseResult:
    """验证 grasp_table 单条参数。"""
    sim.data.ctrl[:] = 0
    mujoco.mj_resetData(sim.model, sim.data)
    sim.apply_pose(tuple(entry["angles_deg"]), middle_deg)
    converged = sim.settle()
    res = sim.metrics(label, middle_deg)
    res.converged = converged
    return res


def verify_table(table_path: Path, out_path: Path | None = None,
                 middle_deg: tuple[float, ...] = (0.0,) * N_JOINTS) -> dict:
    """批量验证参数表：default + objects 全条目，输出 JSON 报告。"""
    raw = json.loads(table_path.read_text(encoding="utf-8"))
    sim = HandSim()
    report: dict = {"middle_pos_deg": list(middle_deg), "entries": {}}
    entries: list[tuple[str, dict]] = [("default", raw["default"])]
    entries += list(raw.get("objects", {}).items())

    for label, entry in entries:
        res = verify_entry(entry, sim, label, middle_deg)
        verdict = {
            "pass": res.converged and res.all_reachable and res.self_contacts == 0,
            **asdict(res),
        }
        report["entries"][label] = verdict
        flag = "PASS" if verdict["pass"] else "REVIEW"
        print(
            f"  {flag}  {label:<10} converged={res.converged} reachable={res.all_reachable} "
            f"max_resid={res.max_residual_rad:.4f}rad min_gap={res.min_gap_mm:.1f}mm "
            f"self_con={res.self_contacts} limit_hits={res.limit_hits}"
        )

    if out_path is not None:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"report -> {out_path}")
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="grasp_table MuJoCo 批量验证")
    parser.add_argument("--table", default=str(DEFAULT_TABLE))
    parser.add_argument("--out", default=str(REPO_ROOT / "data/metrics/grasp_table_validation.json"))
    parser.add_argument("--pose", default=None, help="单姿态快查：8 个逗号分隔角度")
    parser.add_argument("--middle", default="0,0,0,0,0,0,0,0", help="8 关节中位角（标定后传入）")
    args = parser.parse_args(argv)

    middle = tuple(float(x) for x in args.middle.split(","))

    if args.pose:
        angles = tuple(float(x) for x in args.pose.split(","))
        sim = HandSim()
        sim.apply_pose(angles, middle)
        sim.settle()
        res = sim.metrics("adhoc", middle)
        print(json.dumps(asdict(res), ensure_ascii=False, indent=2))
        return 0

    report = verify_table(Path(args.table), Path(args.out), middle)
    bad = [k for k, v in report["entries"].items() if not v["pass"]]
    if bad:
        print(f"\nREVIEW needed: {', '.join(bad)}")
        return 1
    print("\nall entries PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
