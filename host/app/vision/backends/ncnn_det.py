"""NCNN 检测后端 —— 树莓派/PC CPU 推理，底层 ncnn API + YOLOv8 头后处理。

模型：``tools/export_yolo_ncnn.py`` 导出（yolov8n @416，COCO 预训练），
IO 张量名 in0/out0，输出 [1, 84, N]（4 box(xywh) + 80 cls，未 NMS）。

预处理（与 ultralytics 对齐，已用 bus.jpg 比对验证）：
letterbox→416 → BGR→RGB → /255 → float32 CHW。

PC 上同一份代码可跑（ncnn pip 跨平台），冒烟测试
``host/tests/smoke_test_vision.py`` 直接覆盖本模块。
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from app.vision.detector import BaseDetector

INPUT_NAME = "in0"
OUTPUT_NAME = "out0"
N_CLASSES = 80


def letterbox(
    frame_bgr: np.ndarray, size: int
) -> tuple[np.ndarray, float, tuple[float, float]]:
    """ultralytics 风格等比缩放填充。返回 (padded RGB, scale, (pad_x, pad_y))。"""
    h, w = frame_bgr.shape[:2]
    scale = min(size / h, size / w)
    nh, nw = round(h * scale), round(w * scale)
    resized = cv2.resize(frame_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
    pad_x, pad_y = (size - nw) / 2, (size - nh) / 2
    top, bottom = int(round(pad_y - 0.1)), int(round(pad_y + 0.1))
    left, right = int(round(pad_x - 0.1)), int(round(pad_x + 0.1))
    padded = cv2.copyMakeBorder(resized, top, bottom, left, right,
                                cv2.BORDER_CONSTANT, value=(114, 114, 114))
    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
    return rgb, scale, (left, top)


class NcnnDetector(BaseDetector):
    name = "ncnn"

    def __init__(self, model_dir: str | Path, input_size: int = 416,
                 conf: float = 0.35, iou: float = 0.45,
                 num_threads: int = 4):
        super().__init__(conf, iou)
        import ncnn  # 板端与 PC 同装 ncnn wheel

        model_dir = Path(model_dir)
        self._net = ncnn.Net()
        self._net.opt.num_threads = num_threads
        self._net.opt.use_vulkan_compute = False  # 树莓派/PC 无 GPU，CPU 多线程
        param = model_dir / "model.ncnn.param"
        binw = model_dir / "model.ncnn.bin"
        if not param.exists() or not binw.exists():
            raise FileNotFoundError(f"ncnn model not found in {model_dir}（先跑 tools/export_yolo_ncnn.py）")
        self._net.load_param(str(param))
        self._net.load_model(str(binw))
        self.input_size = input_size

    def detect_raw(self, frame_bgr: np.ndarray) -> list[tuple[int, float, tuple[float, float, float, float]]]:
        rgb, scale, (pad_x, pad_y) = letterbox(frame_bgr, self.input_size)
        # /255 + CHW + **ascontiguousarray（关键）**：transpose 产生非连续内存，
        # ncnn.Mat 按连续布局读取会拿到错位数据（首跑踩坑，见 2026-10-06 log）
        blob = np.ascontiguousarray((rgb.astype(np.float32) / 255.0).transpose(2, 0, 1))

        import ncnn

        with self._net.create_extractor() as ex:
            ex.input(INPUT_NAME, ncnn.Mat(blob))
            ret, out = ex.extract(OUTPUT_NAME)
        if ret != 0:
            raise RuntimeError(f"ncnn extract failed: {ret}")
        preds = np.array(out)  # (84, N)
        preds = preds.reshape(-1, preds.shape[-1])  # (84, N)

        boxes, scores, class_ids = [], [], []
        for i in range(preds.shape[1]):
            cls_scores = preds[4:, i]
            cid = int(cls_scores.argmax())
            score = float(cls_scores[cid])
            if score < self.conf:
                continue
            cx, cy, bw, bh = preds[:4, i]
            # letterbox 坐标 → 原图坐标
            x1 = (cx - bw / 2 - pad_x) / scale
            y1 = (cy - bh / 2 - pad_y) / scale
            x2 = (cx + bw / 2 - pad_x) / scale
            y2 = (cy + bh / 2 - pad_y) / scale
            boxes.append([x1, y1, x2 - x1, y2 - y1])
            scores.append(score)
            class_ids.append(cid)

        if not boxes:
            return []
        keep = cv2.dnn.NMSBoxes(boxes, scores, self.conf, self.iou)
        keep = np.array(keep).reshape(-1) if len(keep) else []
        return [(class_ids[i], scores[i],
                 (boxes[i][0], boxes[i][1],
                  boxes[i][0] + boxes[i][2], boxes[i][1] + boxes[i][3]))
                for i in keep]

    def close(self) -> None:
        self._net.clear()


def _probe_input_shape(param_path: Path) -> tuple[int, int]:  # pragma: no cover
    """调试辅助：从 param 文本解析输入尺寸（默认 416）。"""
    for line in Path(param_path).read_text(encoding="utf-8").splitlines():
        if line.startswith("InterResize") or "InterResize" in line:
            for tok in line.split():
                if tok.startswith("11="):
                    return int(tok[3:]), int(tok[3:])
    return 416, 416
