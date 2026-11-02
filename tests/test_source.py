# SPDX-License-Identifier: MulanPSL-2.0
"""Frame sources against a fake visual_position service and a fake MVS SDK."""

import ctypes
import io
import json
import sys
import types
import unittest
from unittest import mock

from mv_camera import source
from mv_camera.settings import Settings

try:
    import PIL  # noqa: F401
except ImportError:
    PIL = None


def settings(**overrides):
    return Settings.parse({"transport": "aubo_vision", "host": "h", "timeout_s": 2, **overrides})


class Response(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass


def fake_service(width, height, channels=1, error=None):
    """urlopen stand-in for getCameraCurFrame; records each request's params."""
    calls = []

    def urlopen(request, timeout):
        params = json.loads(request.data)["params"]
        calls.append((params, timeout))
        if error:
            return Response(json.dumps({"error": error}).encode())
        w, h = (width, height) if params[0] else params[1:]
        result = {"width": w, "height": h, "channels": channels,
                  "data": [7] * (w * h * channels)}
        return Response(json.dumps({"result": result}).encode())
    return urlopen, calls


class AuboVisionTest(unittest.TestCase):
    def test_frame_at_requested_size(self):
        urlopen, calls = fake_service(2448, 2048)
        with mock.patch("urllib.request.urlopen", urlopen):
            cam = source.open_source(settings())
            f = cam.frame(612, 512)
            self.assertEqual((f.width, f.height, f.channels, len(f.data)), (612, 512, 1, 612 * 512))
            self.assertEqual(cam.sensor_size(), (2448, 2048))
        self.assertEqual(calls[0], ([False, 612, 512], 2))
        self.assertTrue(calls[1][0][0])

    def test_rejects_wrong_size_and_errors(self):
        urlopen, _ = fake_service(2448, 2048)

        def shrunk(request, timeout):
            reply = json.loads(urlopen(request, timeout).read())
            reply["result"]["width"] = 10
            return Response(json.dumps(reply).encode())
        with mock.patch("urllib.request.urlopen", shrunk):
            with self.assertRaises(source.FrameError):
                source.open_source(settings()).frame(612, 512)
        urlopen, _ = fake_service(0, 0, error={"code": -1, "message": "camera offline"})
        with mock.patch("urllib.request.urlopen", urlopen):
            with self.assertRaisesRegex(source.FrameError, "camera offline"):
                source.open_source(settings()).frame(612, 512)

    def test_checked(self):
        with self.assertRaises(source.FrameError):
            source.checked(2, 2, 1, b"\0" * 3)
        with self.assertRaises(source.FrameError):
            source.checked(2, 2, 4, b"\0" * 16)
        self.assertEqual(source.checked(2, 2, 3, b"\0" * 12).channels, 3)


class GigEInfo(ctypes.Structure):
    _fields_ = [("chSerialNumber", ctypes.c_ubyte * 16)]


class Special(ctypes.Union):
    _fields_ = [("stGigEInfo", GigEInfo), ("stUsb3VInfo", GigEInfo)]


class DeviceInfo(ctypes.Structure):
    _fields_ = [("nTLayerType", ctypes.c_uint), ("SpecialInfo", Special)]


class DeviceList(ctypes.Structure):
    _fields_ = [("nDeviceNum", ctypes.c_uint),
                ("pDeviceInfo", ctypes.POINTER(DeviceInfo) * 256)]


class FrameInfo(ctypes.Structure):
    _fields_ = [("nWidth", ctypes.c_ushort), ("nHeight", ctypes.c_ushort),
                ("enPixelType", ctypes.c_uint), ("nFrameLen", ctypes.c_uint)]


class FrameOut(ctypes.Structure):
    _fields_ = [("pBufAddr", ctypes.POINTER(ctypes.c_ubyte)), ("stFrameInfo", FrameInfo)]


def fake_mvs(serials, pixel_type=0x01080001, width=8, height=4, open_ret=0):
    """A MvCameraControl_class stand-in with the same structures and names."""
    log = []
    infos = []
    for serial in serials:
        info = DeviceInfo(nTLayerType=1)
        info.SpecialInfo.stGigEInfo.chSerialNumber[:len(serial)] = list(serial.encode())
        infos.append(info)
    buffer = (ctypes.c_ubyte * (width * height * 3))(*range(width * height * 3))

    class MvCamera:
        @staticmethod
        def MV_CC_EnumDevices(layers, found):
            found.nDeviceNum = len(infos)
            for i, info in enumerate(infos):
                found.pDeviceInfo[i] = ctypes.pointer(info)
            return 0

        def __getattr__(self, name):
            def call(*args):
                log.append((name,) + args)
                if name == "MV_CC_CreateHandle":
                    log.append(("serial", bytes(args[0].SpecialInfo.stGigEInfo.chSerialNumber)
                                .split(b"\0")[0].decode()))
                if name == "MV_CC_OpenDevice":
                    return open_ret
                if name == "MV_CC_GetOptimalPacketSize":
                    return 1500
                if name == "MV_CC_GetImageBuffer":
                    out = args[0]
                    out.pBufAddr = ctypes.cast(buffer, ctypes.POINTER(ctypes.c_ubyte))
                    channels = 3 if pixel_type == 0x02180014 else 1
                    out.stFrameInfo = FrameInfo(width, height, pixel_type, width * height * channels)
                return 0
            return call

    module = types.SimpleNamespace(
        MvCamera=MvCamera, MV_CC_DEVICE_INFO_LIST=DeviceList, MV_CC_DEVICE_INFO=DeviceInfo,
        MV_FRAME_OUT=FrameOut, MV_GIGE_DEVICE=1, MV_USB_DEVICE=4, MV_ACCESS_Exclusive=1,
        MV_TRIGGER_MODE_ON=1, MV_TRIGGER_SOURCE_SOFTWARE=7,
        PixelType_Gvsp_Mono8=0x01080001, PixelType_Gvsp_RGB8_Packed=0x02180014)
    return module, log


class MvsTest(unittest.TestCase):
    def open(self, module, **overrides):
        with mock.patch.dict(sys.modules, {"MvCameraControl_class": module}):
            return source.open_source(settings(transport="mvs", **overrides))

    def names(self, log):
        return [entry[0] for entry in log]

    def test_opens_by_serial_with_software_trigger(self):
        module, log = fake_mvs(["SN0001", "OTHER"])
        cam = self.open(module, serial="SN0001")
        self.assertIn(("serial", "SN0001"), log)
        self.assertIn(("MV_CC_SetEnumValue", "TriggerSource", 7), log)
        f = cam.grab()
        self.assertEqual((f.width, f.height, f.channels), (8, 4, 1))
        self.assertEqual(f.data, bytes(range(32)))
        self.assertEqual(self.names(log)[-3:], ["MV_CC_SetCommandValue", "MV_CC_GetImageBuffer",
                                                "MV_CC_FreeImageBuffer"])
        cam.close()
        self.assertEqual(self.names(log)[-3:], ["MV_CC_StopGrabbing", "MV_CC_CloseDevice",
                                                "MV_CC_DestroyHandle"])

    def test_device_selection(self):
        module, _ = fake_mvs(["A", "B"])
        with self.assertRaisesRegex(source.FrameError, "set serial"):
            self.open(module)
        with self.assertRaisesRegex(source.FrameError, "not found"):
            self.open(module, serial="C")
        module, log = fake_mvs(["ONLY"])
        self.open(module)
        self.assertIn(("serial", "ONLY"), log)

    def test_held_camera_releases_handle(self):
        module, log = fake_mvs(["A"], open_ret=0x80000203)
        with self.assertRaisesRegex(source.FrameError, "another client"):
            self.open(module)
        self.assertEqual(self.names(log)[-1], "MV_CC_DestroyHandle")
        self.assertNotIn("MV_CC_CloseDevice", self.names(log))

    def test_rgb_and_unsupported_formats(self):
        module, _ = fake_mvs(["A"], pixel_type=0x02180014)
        self.assertEqual(self.open(module).grab().channels, 3)
        module, log = fake_mvs(["A"], pixel_type=0x010C0004)  # Mono12
        with self.assertRaisesRegex(source.FrameError, "Mono8 or RGB8"):
            self.open(module).grab()
        self.assertEqual(self.names(log)[-1], "MV_CC_FreeImageBuffer")


@unittest.skipIf(PIL is None, "needs Pillow")
class ImageTest(unittest.TestCase):
    def test_box_resize_averages(self):
        f = source.Frame(4, 2, 1, bytes([0, 2, 10, 20, 4, 6, 30, 40]))
        small = source.resized(f, 2, 1)
        self.assertEqual(list(small.data), [3, 25])
        self.assertIs(source.resized(f, 4, 2), f)

    def test_jpeg(self):
        from PIL import Image
        for channels, mode in ((1, "L"), (3, "RGB")):
            f = source.Frame(16, 8, channels, bytes(16 * 8 * channels))
            image = Image.open(io.BytesIO(source.jpeg(f)))
            self.assertEqual((image.format, image.mode, image.size), ("JPEG", mode, (16, 8)))


if __name__ == "__main__":
    unittest.main()
