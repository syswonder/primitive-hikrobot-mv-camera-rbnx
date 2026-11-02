# Instance config for the mv_camera primitive (documentation, not a schema).
# Unknown keys are rejected at INIT.
config:
  transport: aubo_vision         # aubo_vision (AUBO visual_position service) | mvs (Hikrobot MVS SDK)

  # aubo_vision: the AUBO controller that runs visual_position.
  host: 192.168.1.10             # required for aubo_vision
  port: 30210                    # visual_position JSON-RPC port

  # mvs: the camera's serial number; may be empty when exactly one camera is visible.
  serial: ""                     # e.g. the serial printed on the camera label

  timeout_s: 10                  # per frame; a full 2448x2048 frame over aubo_vision takes about 2.6 s
  width: 1224                    # published image size; the sensor frame is resized to it
  height: 1024
  rate_hz: 1.0                   # stream rate cap (0 < rate <= 5)
  frame_id: camera_optical_frame # optical frame stamped on images and CameraInfo
  topic_prefix: /mv_camera       # topics: <prefix>/image, <prefix>/camera_info

  # Optional. The calibration of THIS camera and lens at full sensor size, as
  # produced by an OpenCV calibration (plumb_bob). The primitive checks the
  # sensor size at activation and scales K to width x height before
  # publishing. Omit it and intrinsics is not declared. The values below are
  # placeholders: fill in your own calibration.
  intrinsics:
    width: 2448                  # sensor size the calibration was made at
    height: 2048
    fx: 3500.0
    fy: 3500.0
    cx: 1224.0
    cy: 1024.0
    d: [0.0, 0.0, 0.0, 0.0, 0.0]  # k1 k2 p1 p2 k3

# Environment (mvs): the MVS runtime and its Python binding must be importable,
# e.g. LD_LIBRARY_PATH=/opt/MVS/lib/64 and PYTHONPATH=/opt/MVS/Samples/64/Python/MvImport.
