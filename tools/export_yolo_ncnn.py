"""YOLOv8n COCO → NCNN 导出（视觉链模型产物生成，PC 一次性执行）。

产物：``yolov8n_ncnn_model/``（model.ncnn.param/bin，IO=in0/out0，
416 输入 → 3549 anchors）。权重与导出产物不进 Git（.gitignore），
本脚本可从零重建（复现性）。

INT8 量化（树莓派 CPU 提速）：``--int8`` 需校准数据（COCO val 子集），
首次跑通 FP32 后再评估（其他推理后端另议）。

运行::

    python tools/export_yolo_ncnn.py            # 下载权重（若缺）+ 导出
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
WEIGHTS = REPO_ROOT / "yolov8n.pt"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--imgsz", type=int, default=416)
    parser.add_argument("--int8", action="store_true", help="INT8 量化（需校准数据）")
    args = parser.parse_args()

    if not WEIGHTS.exists():
        print(f"weights not found, downloading {WEIGHTS.name} ...")
        from urllib.request import urlretrieve

        urlretrieve(
            "https://github.com/ultralytics/assets/releases/download/v8.4.0/yolov8n.pt",
            WEIGHTS,
        )

    from ultralytics import YOLO

    out = YOLO(str(WEIGHTS)).export(
        format="ncnn", imgsz=args.imgsz, half=False, int8=args.int8, device="cpu"
    )
    print(f"exported -> {out}")
    print("下一步：python tools/test_detect_pc.py --image <jpg> 验证")
    return 0


if __name__ == "__main__":
    sys.exit(main())
