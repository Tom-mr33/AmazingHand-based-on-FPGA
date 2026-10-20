"""ArUco 手背标记跟踪 + 就位判定 + 引导输出。

约定：手背贴 4X4_50 字典 id=0 标记（打印尺寸 40mm，``tools/`` 生成 PDF）。
俯视相机检测 marker → solvePnP（相机内参来自标定）→ 手-目标相对偏移
(dx, dy, dyaw) → 引导文本；|dx|,|dy| < tol_cm 且 |dyaw| < tol_deg 判定就位。

PC 无相机也可全测：``make_marker_image()`` 合成含标记的测试帧。
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

ARUCO_DICT = cv2.aruco.DICT_4X4_50
MARKER_ID = 0
MARKER_SIZE_MM = 40.0  # 打印边长（与实物一致，否则 PnP 距离错误）


@dataclass(frozen=True)
class GuideResult:
    in_position: bool
    dx_cm: float = 0.0   # +: 手需右移（图像 x）
    dy_cm: float = 0.0   # +: 手需下移（图像 y）
    dyaw_deg: float = 0.0
    marker_found: bool = False


@dataclass(frozen=True)
class CameraIntrinsics:
    """针孔内参（tools/calibrate_camera.py 标定产物；PC 测试用合成值）。"""

    fx: float = 800.0
    fy: float = 800.0
    cx: float = 640.0
    cy: float = 360.0
    dist: tuple = (0.0, 0.0, 0.0, 0.0, 0.0)

    @classmethod
    def load(cls, path: str | None) -> "CameraIntrinsics":
        if path:
            import yaml
            from pathlib import Path as _P

            p = _P(path)
            if p.exists():
                raw = yaml.safe_load(p.read_text(encoding="utf-8"))
                return cls(fx=raw["fx"], fy=raw["fy"], cx=raw["cx"], cy=raw["cy"],
                           dist=tuple(raw.get("dist", (0, 0, 0, 0, 0))))
        return cls()


def make_marker_png(out_path: str, size_px: int = 400) -> None:
    """生成手背标记 PNG（打印时按 MARKER_SIZE_MM 缩放）。"""
    dict_gen = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    img = cv2.aruco.generateImageMarker(dict_gen, MARKER_ID, size_px)
    cv2.imwrite(out_path, img)


def make_test_frame(marker_xy: tuple[int, int], frame_size: tuple[int, int] = (1280, 720),
                    marker_px: int = 80, yaw_deg: float = 0.0) -> np.ndarray:
    """合成含 ArUco 的测试帧（PC 冒烟；yaw 旋转模拟手转动）。

    背景必须为白色：ArUco 检测要求标记四周有 quiet zone（白边），
    灰底会导致检出失败（首跑踩坑）。
    """
    dict_gen = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    marker = cv2.aruco.generateImageMarker(dict_gen, MARKER_ID, marker_px)
    frame = np.full((frame_size[1], frame_size[0], 3), 255, dtype=np.uint8)
    if abs(yaw_deg) > 0.5:
        h, w = marker.shape
        M = cv2.getRotationMatrix2D((w / 2, h / 2), yaw_deg, 1.0)
        marker = cv2.warpAffine(marker, M, (w, h))
    x, y = marker_xy
    frame[y : y + marker_px, x : x + marker_px] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return frame


class HandGuide:
    def __init__(self, intrinsics: CameraIntrinsics | None = None,
                 target_xy_px: tuple[float, float] | None = None,
                 tol_cm: float = 1.0, tol_deg: float = 10.0,
                 work_height_mm: float = 150.0):
        self.K = intrinsics or CameraIntrinsics()
        self.target = target_xy_px or (self.K.cx, self.K.cy)  # 默认：视野中心
        self.tol_cm = tol_cm
        self.tol_deg = tol_deg
        # PnP 物距先验：手背工作高度（俯视相机到 marker 平面，标定后修正）
        self._obj_points = np.array(
            [
                [-MARKER_SIZE_MM / 2, MARKER_SIZE_MM / 2, 0],
                [MARKER_SIZE_MM / 2, MARKER_SIZE_MM / 2, 0],
                [MARKER_SIZE_MM / 2, -MARKER_SIZE_MM / 2, 0],
                [-MARKER_SIZE_MM / 2, -MARKER_SIZE_MM / 2, 0],
            ],
            dtype=np.float32,
        )
        self._height_mm = work_height_mm

    def locate(self, frame_bgr: np.ndarray) -> tuple[bool, tuple[float, float], float]:
        """检测 marker → (found, 中心像素 xy, yaw 度)。"""
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        dict_gen = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        detector = cv2.aruco.ArucoDetector(dict_gen, cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(gray)
        # ids 形状 (N,1)——tolist() 得 [[0]] 嵌套结构，必须 flatten 后再查（首跑踩坑）
        if ids is None or len(ids) == 0 or MARKER_ID not in ids.flatten().tolist():
            return False, (0.0, 0.0), 0.0
        idx = int(np.where(ids == MARKER_ID)[0][0])
        pts = corners[idx][0]
        center = pts.mean(axis=0)
        # yaw：长边方向角（图像平面内旋转）
        edge = pts[1] - pts[0]
        yaw = float(np.degrees(np.arctan2(edge[1], edge[0])))
        return True, (float(center[0]), float(center[1])), yaw

    def guide(self, frame_bgr: np.ndarray) -> GuideResult:
        """单帧引导：相对目标点的偏移 + 就位判定。"""
        found, (cx, cy), yaw = self.locate(frame_bgr)
        if not found:
            return GuideResult(in_position=False, marker_found=False)
        mm_per_px = self._height_mm / self.K.fx * 1000 / 1000  # = H/f (mm/px)
        dx_cm = (cx - self.target[0]) * mm_per_px / 10.0
        dy_cm = (cy - self.target[1]) * mm_per_px / 10.0
        # yaw 取最小等价角（-180..180）
        dyaw = ((yaw + 180) % 360) - 180
        in_pos = abs(dx_cm) <= self.tol_cm and abs(dy_cm) <= self.tol_cm and abs(dyaw) <= self.tol_deg
        return GuideResult(in_position=in_pos, dx_cm=round(dx_cm, 2),
                           dy_cm=round(dy_cm, 2), dyaw_deg=round(dyaw, 1),
                           marker_found=True)

    def guide_text(self, g: GuideResult) -> str:
        """人读引导文本（UI/语音播报）。"""
        if not g.marker_found:
            return "未检测到手指标记，请将手移入画面"
        if g.in_position:
            return "就位"
        parts = []
        if abs(g.dx_cm) >= self.tol_cm / 2:
            parts.append(f"{'右' if g.dx_cm > 0 else '左'}移 {abs(g.dx_cm):.1f}cm")
        if abs(g.dy_cm) >= self.tol_cm / 2:
            parts.append(f"{'下' if g.dy_cm > 0 else '上'}移 {abs(g.dy_cm):.1f}cm")
        if abs(g.dyaw_deg) >= self.tol_deg / 2:
            parts.append(f"旋转 {abs(g.dyaw_deg):.0f}°")
        return " → ".join(parts) if parts else "微调中…"
