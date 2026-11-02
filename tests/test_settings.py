# SPDX-License-Identifier: MulanPSL-2.0
import unittest

from mv_camera.settings import Intrinsics, Settings

CALIBRATION = {"width": 2448, "height": 2048, "fx": 3522.89, "fy": 3520.75,
               "cx": 1202.78, "cy": 1026.06,
               "d": [-0.12363, 0.36877, -0.00448, -0.00078, 0.0]}


class SettingsTest(unittest.TestCase):
    def test_defaults(self):
        s = Settings.parse({"transport": "aubo_vision", "host": "192.168.1.10"})
        self.assertEqual((s.port, s.width, s.height), (30210, 1224, 1024))
        self.assertIsNone(s.intrinsics)
        self.assertEqual(s.topic("image"), "/mv_camera/image")

    def test_rejects(self):
        self.assertEqual(Settings.parse({"transport": "mvs"}).serial, "")
        aubo = {"transport": "aubo_vision", "host": "h"}
        for bad in ({}, {"host": "h"}, {"transport": "usb"}, {"transport": "aubo_vision"},
                    {"transport": "mvs", "serial": 5}, {**aubo, "colour": True},
                    {**aubo, "rate_hz": 0}, {**aubo, "rate_hz": 6},
                    {**aubo, "width": 0}, {**aubo, "width": 12.5},
                    {**aubo, "topic_prefix": "aubo"},
                    {**aubo, "intrinsics": {**CALIBRATION, "d": [0, 0, 0, 0]}},
                    {**aubo, "intrinsics": {**CALIBRATION, "cx": 3000}},
                    {**aubo, "intrinsics": {**CALIBRATION, "k3": 0}},
                    {**aubo, "intrinsics": {k: v for k, v in CALIBRATION.items() if k != "fy"}}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                Settings.parse(bad)


class ScaleTest(unittest.TestCase):
    def test_identity(self):
        k = Intrinsics.parse(CALIBRATION)
        self.assertEqual(k.scaled(2448, 2048), k)

    def test_half_follows_pixel_centres(self):
        k = Intrinsics.parse(CALIBRATION).scaled(1224, 1024)
        self.assertAlmostEqual(k.fx, 3522.89 / 2)
        self.assertAlmostEqual(k.fy, 3520.75 / 2)
        # Full-size pixel centres 0 and 1 average to half-size pixel 0.
        self.assertAlmostEqual(k.cx, (1202.78 - 0.5) / 2)
        self.assertAlmostEqual(k.cy, (1026.06 - 0.5) / 2)
        self.assertEqual(k.d, tuple(CALIBRATION["d"]))

    def test_projection_survives_resize(self):
        """A 3D point lands at the same place in both images, up to the resize."""
        full = Intrinsics.parse(CALIBRATION)
        small = full.scaled(612, 512)
        x, y = 0.07, -0.03  # normalized coordinates
        u_full, v_full = full.fx * x + full.cx, full.fy * y + full.cy
        u_small, v_small = small.fx * x + small.cx, small.fy * y + small.cy
        self.assertAlmostEqual((u_full + 0.5) / 4 - 0.5, u_small)
        self.assertAlmostEqual((v_full + 0.5) / 4 - 0.5, v_small)


if __name__ == "__main__":
    unittest.main()
