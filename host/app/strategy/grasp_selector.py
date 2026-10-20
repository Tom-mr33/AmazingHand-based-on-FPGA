"""抓取策略 —— (类别, 尺寸, 朝向) → GraspPlan 查表选择器。

参数来源三层：
1. ``grasp_table.json`` 预标定参数（当前为占位，source 字段标注溯源）；
2. ``sim/verify_grasp.py`` MuJoCo 离线验证后回填（source=mujoco）；
3. 实机标定终值（source=calib）。

尺寸/朝向微调：骨架版仅按尺寸做力矩等级粗调（大物体降级防过载），
几何微调待数字孪生联调后扩展。
"""

from __future__ import annotations

import json
from pathlib import Path

from app.control.servo_base import GraspPlan

# 表文件位置：随包分发，亦可被 settings.yaml 覆盖
DEFAULT_TABLE = Path(__file__).with_name("grasp_table.json")


class GraspSelector:
    def __init__(self, table_path: Path | str = DEFAULT_TABLE):
        self._raw = json.loads(Path(table_path).read_text(encoding="utf-8"))
        self._default = self._raw["default"]
        self._objects: dict[str, dict] = self._raw.get("objects", {})

    @property
    def known_objects(self) -> list[str]:
        return sorted(self._objects)

    def select(self, category: str, size_mm: tuple[float, float, float] | None = None,
               yaw_deg: float | None = None) -> GraspPlan:
        """查表生成抓取计划；未收录类别回退 default（调用方需提示）。"""
        entry = self._objects.get(category, self._default)
        torque = list(entry["torque_levels"])
        if size_mm is not None:
            longest = max(size_mm)
            if longest > 120:  # 大物体：整体降 1 级防过载（骨架版粗调）
                torque = [max(1, t - 1) for t in torque]
        return GraspPlan(
            angles_deg=tuple(entry["angles_deg"]),
            pregrasp_deg=tuple(entry["pregrasp_deg"]),
            speed_raw=int(entry["speed_raw"]),
            torque_levels=tuple(torque),
            mode=entry["mode"],
            label=category if category in self._objects else f"{category}(fallback)",
        )
