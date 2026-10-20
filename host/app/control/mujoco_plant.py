"""Real MuJoCo plant used by the local Host backend.

Raw conversion follows the repository's SIM profile, not calibrated SCS0009.
"""
from __future__ import annotations

import math
from pathlib import Path
import sys

import mujoco
import numpy as np

from app.control import scs_protocol as sp


class MujocoPlant:
    def __init__(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "sim"))
        from collision_model import build_collision_model

        self.model = build_collision_model()
        for geom in range(self.model.ngeom):
            body = self.model.geom_bodyid[geom]
            if self.model.body_mocapid[body] >= 0:
                self.model.geom_rgba[geom, 3] = 0
        self.data = mujoco.MjData(self.model)
        names = [self.model.actuator(i).name for i in range(self.model.nu)]
        expected = [f"finger{i}_motor{j}" for i in range(1, 5) for j in (1, 2)]
        if names != expected:
            raise ValueError(f"unexpected actuator mapping: {names}")
        self.qadr = self.model.jnt_qposadr[self.model.actuator_trnid[:, 0]]
        self.dadr = self.model.jnt_dofadr[self.model.actuator_trnid[:, 0]]
        self._gain = self.model.actuator_gainprm.copy()
        self._bias = self.model.actuator_biasprm.copy()
        self._targets = np.zeros(8)
        self._enabled = np.zeros(8, dtype=bool)
        self._speed = np.full(8, math.radians(60.0))
        self._remainder = 0.0
        # Settle the parallel linkage at model zero, without changing upstream XML.
        for _ in range(800):
            mujoco.mj_step(self.model, self.data)
        self.disable()

    @property
    def positions_rad(self):
        return self.data.qpos[self.qadr].copy()

    def apply_raw_targets(self, target_raw, torque_enabled, speed_raw=None):
        if len(target_raw) != 8 or any(type(x) is not int or not 0 <= x <= 4095 for x in target_raw):
            raise ValueError("target_raw must contain eight integers in 0..4095")
        enabled = np.asarray(torque_enabled, dtype=bool)
        if enabled.ndim == 0:
            enabled = np.full(8, bool(enabled))
        if enabled.shape != (8,):
            raise ValueError("torque mask must contain eight channels")
        targets = np.array([sp.raw_to_rad(x) for x in target_raw])
        self._targets[:] = np.clip(targets, *self.model.actuator_ctrlrange.T)
        self._enabled[:] = enabled
        self.model.actuator_gainprm[:] = self._gain
        self.model.actuator_biasprm[:] = self._bias
        self.model.actuator_gainprm[~enabled] = 0
        self.model.actuator_biasprm[~enabled] = 0
        if speed_raw is not None:
            # Deliberate simulation rate: raw 3 -> 60 deg/s; not a servo unit claim.
            self._speed[:] = np.radians([min(180.0, x * 20.0) if x else 180.0
                                         for x in speed_raw])
        if not enabled.any():
            self.disable()

    def disable(self):
        """Simulator safe stop: zero actuation and pause physics at measured pose."""
        self._enabled[:] = False
        self._targets[:] = self.positions_rad
        self.data.ctrl[:] = self.positions_rad
        self.data.qvel[:] = 0
        self.model.actuator_gainprm[:] = 0
        self.model.actuator_biasprm[:] = 0
        mujoco.mj_forward(self.model, self.data)

    def step(self, dt_s):
        if not math.isfinite(dt_s) or not 0 <= dt_s <= 0.1:
            self.disable()
            raise ValueError("plant dt must be finite and in 0..0.1 s")
        if not self._enabled.any():
            return
        self._remainder += dt_s
        step = float(self.model.opt.timestep)
        while self._remainder + 1e-12 >= step:
            delta = np.clip(self._targets - self.data.ctrl, -self._speed * step, self._speed * step)
            self.data.ctrl[self._enabled] += delta[self._enabled]
            mujoco.mj_step(self.model, self.data)
            self._remainder -= step
        if not np.all(np.isfinite(self.data.qpos)):
            self.disable()
            raise RuntimeError("non-finite MuJoCo state")

    def feedback_raw(self):
        return {
            "position_raw": [sp.rad_to_raw(float(x)) for x in self.positions_rad],
            "load_raw": [min(1023, int(abs(x) * 100)) for x in self.data.actuator_force],
            "online_mask": 255,
        }
