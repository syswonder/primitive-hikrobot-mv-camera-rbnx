---
description: Industrial camera (often mono, no colour); take a photo on demand, stream frames, intrinsics.
---

# Camera (mv_camera)

- `snapshot`: take one photo now. Returns a `sensor_msgs/Image` whose `data`
  is a JPEG. A mono camera gives a grayscale image: judge shape, position and
  brightness, never colour.
- `rgb`: the continuous frame stream for perception services; `encoding` is
  `mono8` or `rgb8`.
- `intrinsics`: the calibration for the streamed image size, when the
  deployment provides one.

A frame takes up to a few seconds. If `snapshot` fails, the camera or its
controller is unreachable; retrying will not help until that is fixed.
