"""Exercise the OpenNI depth binding without native libraries or hardware."""

import ctypes
import pathlib
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist.sensors import depth  # noqa: E402


class OpenNITests(unittest.TestCase):
    def setUp(self):
        self.library = Mock()
        for name in (
            "oniDeviceOpen", "oniDeviceCreateStream", "oniStreamSetProperty",
            "oniStreamGetProperty", "oniStreamStart", "oniWaitForAnyStream",
        ):
            getattr(self.library, name).return_value = 0
        self.modes = (depth._Mode * 2)(
            depth._Mode(depth._FORMAT_DEPTH_1MM, 640, 480, 30),
            depth._Mode(depth._FORMAT_DEPTH_1MM, 320, 240, 30),
        )
        self.info = depth._SensorInfo(depth._SENSOR_DEPTH, 2, self.modes)
        self.library.oniStreamGetSensorInfo.return_value = ctypes.pointer(self.info)
        with patch.object(depth._OpenNI, "_lib", self.library):
            self.camera = depth._OpenNI("unused")

    def test_selects_requested_depth_mode(self):
        selected = []

        def set_property(stream, prop, pointer, size):
            if prop == depth._PROP_VIDEO_MODE:
                mode = ctypes.cast(pointer, ctypes.POINTER(depth._Mode)).contents
                selected.append((mode.format, mode.width, mode.height, mode.fps))
            return 0

        self.library.oniStreamSetProperty.side_effect = set_property
        self.camera.open_device(320, 240)
        self.assertEqual(selected, [(depth._FORMAT_DEPTH_1MM, 320, 240, 30)])
        self.library.oniStreamStart.assert_called_once()

    def test_reads_padded_frame_and_copies_before_release(self):
        pixels = np.array([[600, 700, 999], [800, 900, 999]], dtype=np.uint16)
        frame = depth._Frame(
            pixels.nbytes, pixels.ctypes.data, depth._SENSOR_DEPTH, 123, 1,
            2, 2, depth._Mode(depth._FORMAT_DEPTH_1MM, 2, 2, 30),
            0, 0, 0, pixels.strides[0],
        )

        def read_frame(stream, pointer):
            ctypes.cast(pointer, ctypes.POINTER(ctypes.POINTER(depth._Frame)))[0] = ctypes.pointer(frame)
            return 0

        self.library.oniStreamReadFrame.side_effect = read_frame
        self.library.oniFrameRelease.side_effect = lambda pointer: pixels.fill(0)
        image = self.camera.read()
        np.testing.assert_array_equal(image, [[600, 700], [800, 900]])
        self.assertEqual(image.dtype, np.uint16)
        self.assertEqual((self.camera.width, self.camera.height), (2, 2))
        self.library.oniFrameRelease.assert_called_once()

    def test_timeout_does_not_read_or_release_frame(self):
        self.library.oniWaitForAnyStream.return_value = 102
        self.assertIsNone(self.camera.read())
        self.library.oniStreamReadFrame.assert_not_called()
        self.library.oniFrameRelease.assert_not_called()

    def test_failed_open_does_not_close_invalid_handle(self):
        def fail_open(uri, pointer):
            ctypes.cast(pointer, ctypes.POINTER(ctypes.c_void_p))[0] = 123
            return 1

        self.library.oniDeviceOpen.side_effect = fail_open
        self.library.oniGetExtendedError.return_value = b"device unavailable"
        with self.assertRaisesRegex(OSError, "device unavailable"):
            self.camera.open_device(320, 240)
        self.camera.close()
        self.library.oniDeviceClose.assert_not_called()
        self.library.oniStreamDestroy.assert_not_called()


if __name__ == "__main__":
    unittest.main()