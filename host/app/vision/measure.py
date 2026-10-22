"""俯视单目几何测尺寸 —— bbox（像素）→ 物理尺寸（mm）。

原理：俯视相机安装高度已知 → 地面 mm/pixel 恒定 →
物体顶面宽/长 = bbox_w/bbox_h × scale。

标定：``tools/calibrate_camera.py``（棋盘格已知间距 → 焦距 f（像素）），
``mm_per_px = H / f``（针孔模型，H=安装高度 mm）。标定产物存
``data/calibration/camera.yaml``；未标定时用占位系数（报告标注）。

高度方向：俯视无法测高，取类别先验（15 类物品典型高度表），
报告/策略侧标注置信度。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

#: 类别先验高度（mm）——占位值，待实物量取修订（不参与面积判定，仅展示）
DEFAULT_HEIGHT_MM: dict[str, float] = {
    "banana": 35.0, "apple": 70.0, "orange": 70.0, "bottle": 180.0, "cup": 100.0,
    "wine_glass": 150.0, "bowl": 60.0, "mouse": 40.0, "remote": 25.0,
    "keyboard": 20.0, "cell_phone": 10.0, "book": 30.0, "scissors": 15.0,
    "teddy_bear": 150.0, "toothbrush": 15.0,
}
FALLBACK_HEIGHT_MM = 50.0


@dataclass(frozen=True)
class CameraCalib:
    mm_per_px: float  # 地面标定系数（H/f）
    image_size: tuple[int, int] = (1280, 720)
    source: str = "placeholder"  # placeholder | calibrated

    @classmethod
    def load(cls, path: Path | str | None) -> "CameraCalib":
        """读 data/calibration/camera.yaml；缺失回退占位（H=600mm, f=800px → 0.75mm/px）。"""
        if path is not None and Path(path).exists():
            raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
            return cls(mm_per_px=float(raw["mm_per_px"]),
                       image_size=tuple(raw.get("image_size", (1280, 720))),
                       source="calibrated")
        return cls(mm_per_px=0.75)


def bbox_to_size_mm(
    bbox_xyxy: tuple[float, float, float, float],
    calib: CameraCalib,
    category: str | None = None,
) -> tuple[float, float, float]:
    """bbox → (长 mm, 宽 mm, 高 mm)。长= bbox 较长边；高=类别先验。"""
    x1, y1, x2, y2 = bbox_xyxy
    w_px, h_px = abs(x2 - x1), abs(y2 - y1)
    dims_px = sorted((w_px, h_px), reverse=True)
    length_mm = dims_px[0] * calib.mm_per_px
    width_mm = dims_px[1] * calib.mm_per_px
    height_mm = DEFAULT_HEIGHT_MM.get(category or "", FALLBACK_HEIGHT_MM)
    return round(length_mm, 1), round(width_mm, 1), round(height_mm, 1)
