# SPDX-License-Identifier: MulanPSL-2.0
"""Robonix entry point: the standard camera contracts over ROS 2 and MCP.

INIT validates configuration only. ACTIVATE opens the frame source, fetches one
frame at the configured size, latches the intrinsics (when
configured), declares the image stream and starts it. DEACTIVATE stops the
stream and closes the source. The snapshot tool answers only while active.
"""

from __future__ import annotations

import logging
import threading
import time

from robonix_api import Err, Ok, Primitive

from .settings import Settings
from .source import jpeg, open_source

RGB = "robonix/primitive/camera/rgb"
SNAPSHOT = "robonix/primitive/camera/snapshot"
INTRINSICS = "robonix/primitive/camera/intrinsics"

log = logging.getLogger("mv_camera")
provider = Primitive(id="mv_camera", namespace="robonix/primitive/camera")
settings = None
source = None
publishers = {}
worker = None
stop_event = threading.Event()


def stamp_of(seconds):
    from builtin_interfaces.msg import Time
    return Time(sec=int(seconds), nanosec=int(seconds % 1 * 1e9))


def image_msg(frame, taken):
    from sensor_msgs.msg import Image

    msg = Image()
    msg.header.stamp = stamp_of(taken)
    msg.header.frame_id = settings.frame_id
    msg.height, msg.width = frame.height, frame.width
    msg.encoding = "mono8" if frame.channels == 1 else "rgb8"
    msg.step = frame.width * frame.channels
    msg.data = frame.data
    return msg


def camera_info_msg():
    """CameraInfo for the published size; monocular, so R is identity and P is K."""
    from sensor_msgs.msg import CameraInfo

    k = settings.intrinsics.scaled(settings.width, settings.height)
    msg = CameraInfo()
    msg.header.stamp = stamp_of(time.time())
    msg.header.frame_id = settings.frame_id
    msg.width, msg.height = k.width, k.height
    msg.distortion_model = "plumb_bob"
    msg.d = list(k.d)
    msg.k = [k.fx, 0.0, k.cx, 0.0, k.fy, k.cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [k.fx, 0.0, k.cx, 0.0, 0.0, k.fy, k.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


def stream():
    """Publish frames at up to ``rate_hz``; log each new failure once and keep trying."""
    previous_error = ""
    while not stop_event.is_set():
        began = time.monotonic()
        # Both sources expose at request time; the rest of the call is transfer.
        taken = time.time()
        try:
            frame = source.frame(settings.width, settings.height)
            publishers[RGB].publish(image_msg(frame, taken))
            if previous_error:
                log.info("frames flowing again")
            previous_error = ""
        except Exception as exc:
            if str(exc) != previous_error:
                log.error("frame failed: %s", exc)
                previous_error = str(exc)
        stop_event.wait(max(0.0, 1 / settings.rate_hz - (time.monotonic() - began)))


# MCP types come from `rbnx codegen --mcp`, run by scripts/build.sh.
import builtin_interfaces_mcp  # noqa: E402
import std_msgs_mcp  # noqa: E402
from sensor_msgs_mcp import Image as McpImage  # noqa: E402
from std_msgs_mcp import Empty  # noqa: E402


@provider.mcp(SNAPSHOT)
def snapshot(msg: Empty) -> McpImage:
    """Take one photo with the camera. Returns a sensor_msgs/Image whose
    `data` is a JPEG (`encoding` "jpeg"); a mono camera gives a grayscale
    JPEG with no colour information."""
    _ = msg
    if worker is None:
        raise RuntimeError("camera is not active")
    taken = time.time()
    frame = source.frame(settings.width, settings.height)
    data = jpeg(frame)
    return McpImage(
        header=std_msgs_mcp.Header(
            stamp=builtin_interfaces_mcp.Time(sec=int(taken), nanosec=int(taken % 1 * 1e9)),
            frame_id=settings.frame_id),
        height=frame.height, width=frame.width, encoding="jpeg",
        is_bigendian=0, step=len(data), data=data)


@provider.on_init
def init(cfg):
    """Validate the deployment config; do not touch the camera."""
    global settings
    try:
        parsed = Settings.parse(cfg)
    except Exception as exc:
        return Err(f"invalid mv_camera config: {exc}")
    settings = parsed
    log.info("configured %s transport, %dx%d at %.1f Hz, intrinsics %s", settings.transport,
             settings.width, settings.height, settings.rate_hz,
             "configured" if settings.intrinsics else "not configured")
    return Ok()


def check_calibrated_size(candidate):
    """Intrinsics only scale correctly from the full sensor frame they were calibrated on."""
    sensor = candidate.sensor_size()
    calibrated = (settings.intrinsics.width, settings.intrinsics.height)
    if sensor != calibrated:
        raise ValueError(f"intrinsics were calibrated at {calibrated[0]}x{calibrated[1]}"
                         f" but the sensor gives {sensor[0]}x{sensor[1]}")


@provider.on_activate
def activate():
    """Open the source, check it, publish intrinsics, then start the stream. Idempotent."""
    global source, worker
    from sensor_msgs.msg import CameraInfo, Image

    if worker is not None:
        return Ok()
    candidate = None
    try:
        candidate = open_source(settings)
        if settings.intrinsics is not None:
            check_calibrated_size(candidate)
        candidate.frame(settings.width, settings.height)
        if settings.intrinsics is not None and INTRINSICS not in publishers:
            publishers[INTRINSICS] = provider.create_publisher(
                INTRINSICS, topic=settings.topic("camera_info"), msg_type=CameraInfo,
                qos="latched")
            publishers[INTRINSICS].publish(camera_info_msg())
        if RGB not in publishers:
            publishers[RGB] = provider.create_publisher(
                RGB, topic=settings.topic("image"), msg_type=Image, qos="best_effort")
    except Exception as exc:
        log.error("activation failed: %s", exc)
        if candidate is not None:
            candidate.close()
        return Err(f"mv_camera activation failed: {exc}")
    source = candidate
    stop_event.clear()
    worker = threading.Thread(target=stream, name="mv-camera-stream", daemon=True)
    worker.start()
    return Ok()


@provider.on_deactivate
def deactivate():
    """Stop the stream and release the camera; the snapshot tool refuses until reactivated."""
    global source, worker
    stop_event.set()
    if worker is not None:
        worker.join(timeout=settings.timeout_s + 1)
        if worker.is_alive():
            return Err("stream worker did not stop")
        worker = None
    if source is not None:
        source.close()
        source = None
    return Ok()


@provider.on_shutdown
def shutdown():
    return deactivate()


if __name__ == "__main__":
    provider.run()
