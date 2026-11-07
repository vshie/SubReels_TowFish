"""Frames, quaternions and the physical config loader.

Body frame is FRD (x forward, y starboard, z down) with its origin at the CG,
matching ArduPilot. The STEP model frame (towfish_geometry_report.md) is
-Y forward, +X up, Z spanwise (+Z starboard), in millimetres; `step_to_body`
maps one to the other. World frame is NED in metres about the SITL home.
"""
from __future__ import annotations

import hashlib
import math
from pathlib import Path

import numpy as np
import yaml

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "towfish.yaml"
G = 9.80665


class Config:
    """Read-only view of towfish.yaml: `cfg["cable.length"]` -> value."""

    def __init__(self, path: Path | str = CONFIG_PATH, overrides: dict | None = None):
        self.path = Path(path)
        raw = self.path.read_bytes()
        self.sha = hashlib.sha256(raw).hexdigest()[:12]
        self.tree = yaml.safe_load(raw)
        self.overrides = dict(overrides or {})

    def entry(self, key: str) -> dict:
        node = self.tree
        for part in key.split("."):
            node = node[part]
        return node

    def __getitem__(self, key: str):
        if key in self.overrides:
            return self.overrides[key]
        return self.entry(key)["value"]

    def status_table(self) -> list[tuple[str, object, str, str]]:
        """(key, value, status, source) for every leaf, for run records."""
        rows = []

        def walk(node, prefix):
            if isinstance(node, dict) and "value" in node:
                key = prefix
                value = self.overrides.get(key, node["value"])
                status = "OVERRIDE" if key in self.overrides else node.get("status", "")
                rows.append((key, value, status, node.get("source", "")))
                return
            if isinstance(node, dict):
                for k, v in node.items():
                    walk(v, f"{prefix}.{k}" if prefix else k)

        walk(self.tree, "")
        return rows


def step_to_body(p_mm, cg_mm) -> np.ndarray:
    """STEP model point(s) (mm, [x_up, y_aft, z_stbd]) -> body FRD metres about the CG."""
    p = np.asarray(p_mm, dtype=float)
    cg = np.asarray(cg_mm, dtype=float)
    d = (p - cg) / 1000.0
    return np.stack([-d[..., 1], d[..., 2], -d[..., 0]], axis=-1)


# --- quaternions: (w, x, y, z), rotating body vectors into NED --------------

def quat_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def quat_to_dcm(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def euler_from_quat(q: np.ndarray) -> tuple[float, float, float]:
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def quat_integrate(q: np.ndarray, omega_body: np.ndarray, dt: float) -> np.ndarray:
    """Advance q by a body rate over dt (exact for constant rate)."""
    rate = float(np.linalg.norm(omega_body))
    if rate < 1e-12:
        return q
    axis = omega_body / rate
    half = 0.5 * rate * dt
    dq = np.array([math.cos(half), *(axis * math.sin(half))])
    w1, x1, y1, z1 = q
    w2, x2, y2, z2 = dq
    out = np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])
    return out / np.linalg.norm(out)


# --- local tangent plane ----------------------------------------------------

EARTH_R = 6378137.0


def ll_to_ne(lat, lon, lat0, lon0):
    """Small-area equirectangular lat/lon (deg) -> north/east metres."""
    n = np.radians(np.asarray(lat) - lat0) * EARTH_R
    e = np.radians(np.asarray(lon) - lon0) * EARTH_R * math.cos(math.radians(lat0))
    return n, e
