# SPDX-License-Identifier: MulanPSL-2.0
"""Frame sources. Each returns Mono8 or RGB8 frames at a requested size.

``aubo_vision`` asks an AUBO controller's visual_position service (JSON-RPC,
default port 30210) for a frame; it resizes on the controller, so a small
frame also transfers faster. ``mvs`` opens the camera directly with the
Hikrobot MVS SDK and resizes here.
"""

from __future__ import annotations

from dataclasses import dataclass
import ctypes
import json
import threading
import urllib.request


@dataclass(frozen=True)
class Frame:
    width: int
    height: int
    channels: int  # 1 = mono8, 3 = rgb8
    data: bytes    # row-major, no padding


class FrameError(RuntimeError):
    pass


def checked(width, height, channels, data):
    if channels not in (1, 3):
        raise FrameError(f"unsupported frame with {channels} channels")
    if len(data) != width * height * channels:
        raise FrameError(f"{width}x{height}x{channels} frame carries {len(data)} bytes")
    return Frame(width, height, channels, data)


def pil_image(frame):
    from PIL import Image
    return Image.frombytes("L" if frame.channels == 1 else "RGB",
                           (frame.width, frame.height), frame.data)


def resized(frame, width, height):
    """Box-filter resize; PIL samples at pixel centres, matching Intrinsics.scaled."""
    if (frame.width, frame.height) == (width, height):
        return frame
    from PIL import Image
    image = pil_image(frame).resize((width, height), Image.BOX)
    return Frame(width, height, frame.channels, image.tobytes())


def jpeg(frame, quality=85):
    from io import BytesIO
    buf = BytesIO()
    pil_image(frame).save(buf, format="JPEG", quality=quality)
    return buf.getvalue()


class AuboVision:
    """``getCameraCurFrame(original, width, height)``: with ``original`` false
    the service only resizes the sensor frame (checked against a box-filtered
    full frame: no undistortion or crop); with it true it ignores the size."""

    def __init__(self, s):
        self.url = f"http://{s.host}:{s.port}/jsonrpc"
        self.timeout_s = s.timeout_s
        self.lock = threading.Lock()

    def call(self, method, params):
        body = json.dumps({"jsonrpc": "2.0", "method": method, "params": params, "id": 1})
        request = urllib.request.Request(self.url, body.encode(),
                                         {"Content-Type": "application/json"})
        with self.lock, urllib.request.urlopen(request, timeout=self.timeout_s) as response:
            reply = json.loads(response.read())
        if "error" in reply:
            raise FrameError(f"{method}: {reply['error']}")
        return reply["result"]

    def grab(self, width, height, original=False):
        r = self.call("getCameraCurFrame", [original, width, height])
        return checked(r["width"], r["height"], r["channels"], bytes(r["data"]))

    def frame(self, width, height):
        f = self.grab(width, height)
        if (f.width, f.height) != (width, height):
            raise FrameError(f"asked for {width}x{height}, got {f.width}x{f.height}")
        return f

    def sensor_size(self):
        f = self.grab(1, 1, original=True)
        return f.width, f.height

    def close(self):
        pass


class Mvs:
    """Hikrobot MVS SDK, software-triggered so each frame is taken on request.

    Opens the camera exclusively: another client holding it (an AUBO
    visual_position service, the MVS viewer) makes activation fail. Changes
    only the volatile trigger settings; nothing is saved to the camera.
    """

    def __init__(self, s):
        import MvCameraControl_class as sdk  # from /opt/MVS/Samples/64/Python/MvImport
        self.sdk = sdk
        self.timeout_ms = int(s.timeout_s * 1000)
        self.lock = threading.Lock()
        if hasattr(sdk.MvCamera, "MV_CC_Initialize"):  # MVS 4.x
            sdk.MvCamera.MV_CC_Initialize()
        self.cam = sdk.MvCamera()
        self.handle = self.opened = self.grabbing = False
        try:
            self.check(self.cam.MV_CC_CreateHandle(self.find(s.serial)), "CreateHandle")
            self.handle = True
            self.check(self.cam.MV_CC_OpenDevice(sdk.MV_ACCESS_Exclusive, 0),
                       "OpenDevice (is another client holding the camera?)")
            self.opened = True
            packet = self.cam.MV_CC_GetOptimalPacketSize()  # GigE only; USB returns <= 0
            if packet > 0:
                self.check(self.cam.MV_CC_SetIntValue("GevSCPSPacketSize", packet), "packet size")
            self.check(self.cam.MV_CC_SetEnumValue("TriggerMode", sdk.MV_TRIGGER_MODE_ON), "TriggerMode")
            self.check(self.cam.MV_CC_SetEnumValue("TriggerSource", sdk.MV_TRIGGER_SOURCE_SOFTWARE),
                       "TriggerSource")
            self.check(self.cam.MV_CC_StartGrabbing(), "StartGrabbing")
            self.grabbing = True
        except Exception:
            self.close()
            raise

    @staticmethod
    def check(ret, what):
        if ret != 0:
            raise FrameError(f"MVS {what} failed: 0x{ret & 0xFFFFFFFF:08x}")

    def find(self, serial):
        """The device with ``serial``, or the only one when ``serial`` is empty."""
        sdk = self.sdk
        found = sdk.MV_CC_DEVICE_INFO_LIST()
        self.check(sdk.MvCamera.MV_CC_EnumDevices(sdk.MV_GIGE_DEVICE | sdk.MV_USB_DEVICE, found),
                   "EnumDevices")
        devices = {}
        for i in range(found.nDeviceNum):
            info = ctypes.cast(found.pDeviceInfo[i], ctypes.POINTER(sdk.MV_CC_DEVICE_INFO)).contents
            special = (info.SpecialInfo.stGigEInfo if info.nTLayerType == sdk.MV_GIGE_DEVICE
                       else info.SpecialInfo.stUsb3VInfo)
            devices[bytes(special.chSerialNumber).split(b"\0")[0].decode()] = info
        if serial:
            if serial not in devices:
                raise FrameError(f"camera {serial} not found; visible: {sorted(devices)}")
            return devices[serial]
        if len(devices) != 1:
            raise FrameError(f"{len(devices)} cameras visible {sorted(devices)}; set serial")
        return next(iter(devices.values()))

    def grab(self):
        sdk = self.sdk
        out = sdk.MV_FRAME_OUT()
        with self.lock:
            self.check(self.cam.MV_CC_SetCommandValue("TriggerSoftware"), "TriggerSoftware")
            self.check(self.cam.MV_CC_GetImageBuffer(out, self.timeout_ms), "GetImageBuffer")
            try:
                info = out.stFrameInfo
                channels = {sdk.PixelType_Gvsp_Mono8: 1,
                            sdk.PixelType_Gvsp_RGB8_Packed: 3}.get(info.enPixelType)
                if channels is None:
                    raise FrameError(f"pixel format 0x{info.enPixelType:08x} unsupported; "
                                     "set PixelFormat to Mono8 or RGB8")
                data = ctypes.string_at(out.pBufAddr, info.nFrameLen)
            finally:
                self.cam.MV_CC_FreeImageBuffer(out)
        return checked(info.nWidth, info.nHeight, channels, data)

    def frame(self, width, height):
        return resized(self.grab(), width, height)

    def sensor_size(self):
        f = self.grab()
        return f.width, f.height

    def close(self):
        if self.grabbing:
            self.cam.MV_CC_StopGrabbing()
            self.grabbing = False
        if self.opened:
            self.cam.MV_CC_CloseDevice()
            self.opened = False
        if self.handle:
            self.cam.MV_CC_DestroyHandle()
            self.handle = False


def open_source(s):
    return AuboVision(s) if s.transport == "aubo_vision" else Mvs(s)
