# primitive-hikrobot-mv-camera-rbnx

Robonix primitive for Hikrobot MV-series industrial cameras (GigE and USB3).
It exposes the standard camera contracts, so skills and Pilot never see the
camera SDK. Frames come from one of two transports:

- `mvs`: the camera directly, through the Hikrobot MVS SDK. This is the
  intended path.
- `aubo_vision`: an AUBO controller's `visual_position` service (JSON-RPC,
  port 30210), which already owns the camera on AUBO cells. It is an
  unpublished alpha service, so treat it as a stopgap until the camera can be
  wired to the Robonix host.

Checked:

| Transport | Camera | Checked | Not checked |
|---|---|---|---|
| `aubo_vision` | MV-CS050-10GM (Mono8, 2448×2048) behind an AUBO i10H controller, `visual_position_standalone` 2.0.0-alpha.1 | `rbnx boot` in the robonix-aubo image: ACTIVE, snapshot over MCP (1224×1024 JPEG, 1.6 s), `mono8` stream at 0.5 Hz, latched intrinsics scaled to 1224×1024, clean shutdown | Pilot or Scene consuming it |
| `mvs` | none yet | unit tests against a fake SDK | everything on a real camera |

| Contract | Endpoint | |
|---|---|---|
| `robonix/primitive/camera/rgb` | `<prefix>/image` (ROS 2, best effort) | frames at up to `rate_hz`; `encoding` is `mono8` or `rgb8` by the camera's channels |
| `robonix/primitive/camera/snapshot` | MCP tool `snapshot` | one fresh frame as a JPEG `sensor_msgs/Image` (`encoding` `jpeg`) |
| `robonix/primitive/camera/intrinsics` | `<prefix>/camera_info` (ROS 2, latched) | only when `intrinsics` is configured |

A mono camera publishes grayscale on `camera/rgb`. Consumers that need
colour cannot get it from this camera; consumers that decode by `encoding`
(Scene does) handle `mono8`.

## Behaviour

- INIT validates config only. ACTIVATE opens the source, checks that the
  sensor size matches the calibration (when `intrinsics` is set), fetches one
  frame at `width` × `height`, latches the intrinsics, declares the stream and
  starts it. DEACTIVATE stops the stream and releases the camera.
- Frames are resized to `width` × `height`: by the controller for
  `aubo_vision` (a smaller frame also transfers faster: 2.6 s full size, 0.84 s
  at 1224×1024, 0.53 s at 612×512), by a box filter here for `mvs`.
- `intrinsics` holds the calibration of this camera and lens at full sensor
  size. It belongs in the deployment, not the package. The primitive scales K
  to the published size (focal lengths by the ratio, the principal point about
  pixel centres) and passes the plumb_bob distortion through unchanged.
- `mvs` opens the camera exclusively in software-trigger mode, so every frame
  is taken when requested. It changes only volatile trigger settings and the
  GigE packet size; exposure, gain and pixel format stay as the camera has
  them (set `PixelFormat` to Mono8 or RGB8 with the MVS viewer). If another
  client holds the camera, activation fails.
- A failed frame is logged once and retried at `rate_hz`; the snapshot tool
  returns the error to the caller.

## Transports

- `aubo_vision` needs network access to the controller; nothing to install.
- `mvs` needs the MVS runtime for Linux (`/opt/MVS`) in the runtime image,
  `LD_LIBRARY_PATH` with `/opt/MVS/lib/64`, and its Python binding
  (`MvCameraControl_class`, in `/opt/MVS/Samples/64/Python/MvImport`) on
  `PYTHONPATH`. A GigE camera must be on the host's wired subnet.

See `config.spec` for every field.

## Tests

```bash
PYTHONPATH=. python3 -m unittest discover -s tests -v
```

The tests use a fake service and a fake MVS module; no ROS, SDK or camera is
needed. The resize and JPEG tests need Pillow and are skipped without it.
