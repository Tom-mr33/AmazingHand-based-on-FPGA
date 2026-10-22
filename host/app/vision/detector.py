"""检测调度器 —— 按 settings.yaml 选择 NCNN 后端。

类别体系：内部物品名（banana/apple/...，与 grasp_table.json 键一致）
↔ COCO 80 类 id（YOLO 预训练）。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

#: 内部物品名 → COCO 类 id（YOLOv8 COCO 预训练，零训练路线）
ITEM_TO_COCO: dict[str, int] = {
    "banana": 46,
    "apple": 47,
    "orange": 49,
    "bottle": 39,
    "cup": 41,
    "wine_glass": 40,
    "bowl": 45,
    "mouse": 64,
    "remote": 65,
    "keyboard": 66,
    "cell_phone": 67,
    "book": 73,
    "scissors": 76,
    "teddy_bear": 77,
    "toothbrush": 79,
}
#: 反查表（后端输出 COCO id → 内部名）
COCO_TO_ITEM: dict[int, str] = {v: k for k, v in ITEM_TO_COCO.items()}

#: 默认置信度阈值（现场光照波动时由 app 层自适应）
DEFAULT_CONF = 0.35
DEFAULT_IOU = 0.45


@dataclass(frozen=True)
class Detection:
    """单目标检测结果（图像坐标系，像素）。"""

    category: str
    conf: float
    bbox_xyxy: tuple[float, float, float, float]

    @property
    def bbox_xywh(self) -> tuple[float, float, float, float]:
        x1, y1, x2, y2 = self.bbox_xyxy
        return x1, y1, x2 - x1, y2 - y1


class BaseDetector(ABC):
    """检测后端抽象：输入 BGR 帧，输出已按 15 类物品过滤的检测。"""

    name: str = "base"

    def __init__(self, conf: float = DEFAULT_CONF, iou: float = DEFAULT_IOU):
        self.conf = conf
        self.iou = iou

    @abstractmethod
    def detect_raw(self, frame_bgr: np.ndarray) -> list[tuple[int, float, tuple[float, float, float, float]]]:
        """原始输出：[(coco_id, conf, xyxy)]，由基类完成类别过滤。"""

    def detect(self, frame_bgr: np.ndarray) -> list[Detection]:
        out: list[Detection] = []
        for coco_id, conf, xyxy in self.detect_raw(frame_bgr):
            item = COCO_TO_ITEM.get(coco_id)
            if item is not None:  # 非 15 类物品（如 person/bus）直接丢弃
                out.append(Detection(category=item, conf=conf, bbox_xyxy=xyxy))
        return out

    def close(self) -> None:  # noqa: B027 —— 可选资源释放
        pass


def build_detector(settings: dict) -> BaseDetector:
    """后端工厂：settings['vision']['backend'] = ncnn。"""
    vs = settings.get("vision", {})
    backend = vs.get("backend", "ncnn")
    conf = float(vs.get("conf_threshold", DEFAULT_CONF))
    iou = float(vs.get("iou_threshold", DEFAULT_IOU))
    if backend == "ncnn":
        from app.vision.backends.ncnn_det import NcnnDetector

        return NcnnDetector(model_dir=vs["ncnn"]["model_dir"],
                            input_size=int(vs["ncnn"].get("input_size", 416)),
                            conf=conf, iou=iou)
    raise ValueError(f"unknown vision backend: {backend}")
