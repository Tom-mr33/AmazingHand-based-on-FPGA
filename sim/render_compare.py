"""仿真渲染 —— 仿真/实物同角度并排对比的素材生成。

输出（data/render/）：给定姿态的正视/侧视/俯视 PNG，可与实物同角度照片
并排拼图（演示视频素材）。

注意：离屏渲染需要 OpenGL 上下文；Windows 桌面环境通常直接可用。
若在无显示环境（CI/SSH）运行失败，属环境限制，不阻塞验证链路
（verify_grasp.py 不依赖渲染）。

运行::

    python sim/render_compare.py --pose 90,0,90,0,90,0,90,0
    python sim/render_compare.py --entry banana
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco
from PIL import Image

from verify_grasp import DEFAULT_MODEL, DEFAULT_TABLE, HandSim  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = REPO_ROOT / "data/render"

#: 相机位姿：正视（掌面）/ 侧视 / 俯视——与实物拍摄机位约定一致（lookat 对准手掌中心）
VIEWS = {
    "front": dict(azimuth=90, elevation=0, distance=0.55),
    "side": dict(azimuth=0, elevation=0, distance=0.55),
    "top": dict(azimuth=90, elevation=85, distance=0.55),
}
WIDTH, HEIGHT = 960, 720
LOOKAT = (0.03, 0.0, 0.13)  # 米，手掌中心附近


def render_pose(sim: HandSim, out_dir: Path, tag: str) -> list[Path]:
    """渲染当前姿态的三个机位，返回文件路径列表。"""
    out_dir.mkdir(parents=True, exist_ok=True)
    # 上游 XML offscreen 帧缓冲为默认 640×480（引入不改文件）——运行时在
    # mjModel 内存中放大，仅影响本次渲染
    sim.model.vis.global_.offwidth = WIDTH
    sim.model.vis.global_.offheight = HEIGHT
    renderer = mujoco.Renderer(sim.model, height=HEIGHT, width=WIDTH)
    written: list[Path] = []
    try:
        mujoco.mj_forward(sim.model, sim.data)
        for name, cam_cfg in VIEWS.items():
            cam = mujoco.MjvCamera()
            cam.azimuth = cam_cfg["azimuth"]
            cam.elevation = cam_cfg["elevation"]
            cam.distance = cam_cfg["distance"]
            cam.lookat[:] = LOOKAT
            renderer.update_scene(sim.data, camera=cam)
            png = renderer.render()
            out = out_dir / f"{tag}_{name}.png"
            Image.fromarray(png).save(out)
            written.append(out)
            print(f"  saved {out.relative_to(REPO_ROOT)}")
    finally:
        renderer.close()
    return written


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="姿态渲染（仿真/实物对比素材）")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--pose", help="8 个逗号分隔角度（相对中位，度）")
    group.add_argument("--entry", help="grasp_table 条目名（如 banana）")
    parser.add_argument("--table", default=str(DEFAULT_TABLE))
    parser.add_argument("--out-dir", default=str(OUT_DIR))
    args = parser.parse_args(argv)

    if args.entry:
        raw = json.loads(Path(args.table).read_text(encoding="utf-8"))
        entry = raw.get("objects", {}).get(args.entry) or raw["default"]
        angles = tuple(entry["angles_deg"])
        tag = args.entry
    else:
        angles = tuple(float(x) for x in args.pose.split(","))
        tag = "adhoc"

    sim = HandSim()
    sim.apply_pose(angles)
    sim.settle(1.0)
    try:
        written = render_pose(sim, Path(args.out_dir), tag)
        print(f"{len(written)} renders -> {args.out_dir}")
        return 0
    except Exception as exc:  # noqa: BLE001 —— 无 GL 环境降级说明
        print(f"render failed（无显示环境时属预期）: {type(exc).__name__}: {exc}")
        print("提示：验证链路不依赖渲染；本工具仅生成演示素材。")
        return 2


if __name__ == "__main__":
    sys.exit(main())
