# SPDX-License-Identifier: MulanPSL-2.0
"""Deployment configuration for the camera primitive (see ``config.spec``)."""

from __future__ import annotations

from dataclasses import dataclass, fields
import math

MAX_RATE_HZ = 5.0


def number(value, label):
    """A finite int or float, else ValueError."""
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    return float(value)


def size(value, label):
    if type(value) is not int or value <= 0:
        raise ValueError(f"{label} must be a positive integer")
    return value


def reject_unknown(cfg, cls, prefix=""):
    unknown = set(cfg) - {f.name for f in fields(cls)}
    if unknown:
        raise ValueError(f"unknown {prefix}config keys: {sorted(unknown)}")


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole + plumb_bob calibration at the size it was calibrated for."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    d: tuple

    @classmethod
    def parse(cls, cfg):
        if not isinstance(cfg, dict):
            raise ValueError("intrinsics must be a mapping")
        reject_unknown(cfg, cls, "intrinsics ")
        missing = {f.name for f in fields(cls)} - set(cfg)
        if missing:
            raise ValueError(f"intrinsics needs {sorted(missing)}")
        d = cfg["d"]
        if not isinstance(d, (list, tuple)) or len(d) != 5:
            raise ValueError("intrinsics.d must list the five plumb_bob coefficients")
        intr = cls(size(cfg["width"], "intrinsics.width"),
                   size(cfg["height"], "intrinsics.height"),
                   *(number(cfg[k], f"intrinsics.{k}") for k in ("fx", "fy", "cx", "cy")),
                   tuple(number(v, "intrinsics.d") for v in d))
        if intr.fx <= 0 or intr.fy <= 0:
            raise ValueError("intrinsics.fx and fy must be positive")
        if not (0 < intr.cx < intr.width and 0 < intr.cy < intr.height):
            raise ValueError("the principal point must lie inside the calibrated image")
        return intr

    def scaled(self, width, height):
        """The same camera seen through a resize to ``width`` x ``height``.

        Focal lengths scale with the image. The principal point scales about
        pixel centres (OpenCV's convention, pixel i covers [i - 0.5, i + 0.5]),
        which differs from a plain multiply by half a pixel per halving.
        Distortion acts on normalized coordinates, so it does not change.
        """
        sx, sy = width / self.width, height / self.height
        return Intrinsics(width, height, self.fx * sx, self.fy * sy,
                          (self.cx + 0.5) * sx - 0.5, (self.cy + 0.5) * sy - 0.5, self.d)


@dataclass(frozen=True)
class Settings:
    """Validated primitive configuration. Unknown keys are errors, not ignored."""

    transport: str
    host: str = ""
    port: int = 30210
    serial: str = ""
    timeout_s: float = 10.0
    width: int = 1224
    height: int = 1024
    rate_hz: float = 1.0
    frame_id: str = "camera_optical_frame"
    topic_prefix: str = "/mv_camera"
    intrinsics: Intrinsics | None = None

    @classmethod
    def parse(cls, cfg):
        cfg = dict(cfg or {})
        reject_unknown(cfg, cls)
        if cfg.get("intrinsics") is not None:
            cfg["intrinsics"] = Intrinsics.parse(cfg["intrinsics"])
        if "transport" not in cfg:
            raise ValueError("transport is required (mvs or aubo_vision)")
        s = cls(**cfg)
        if s.transport not in {"mvs", "aubo_vision"}:
            raise ValueError("transport must be mvs or aubo_vision")
        if s.transport == "aubo_vision" and not (isinstance(s.host, str) and s.host):
            raise ValueError("host is required for the aubo_vision transport")
        if type(s.port) is not int or not 0 < s.port < 65536:
            raise ValueError("port must be a TCP port number")
        if not isinstance(s.serial, str):
            raise ValueError("serial must be a string")
        size(s.width, "width")
        size(s.height, "height")
        if number(s.timeout_s, "timeout_s") <= 0:
            raise ValueError("timeout_s must be positive")
        if not 0 < number(s.rate_hz, "rate_hz") <= MAX_RATE_HZ:
            raise ValueError(f"rate_hz must be in (0, {MAX_RATE_HZ}]")
        for label in ("frame_id", "topic_prefix"):
            if not isinstance(getattr(s, label), str) or not getattr(s, label):
                raise ValueError(f"{label} must be a non-empty string")
        if not s.topic_prefix.startswith("/") or s.topic_prefix.endswith("/"):
            raise ValueError("topic_prefix must be an absolute ROS name without a trailing /")
        return s

    def topic(self, leaf):
        return f"{self.topic_prefix}/{leaf}"
