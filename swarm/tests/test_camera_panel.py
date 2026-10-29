"""The camera panel (F3) and the `sim camera` commands it sends."""

import contextlib
import io
import os
import unittest
from types import SimpleNamespace

from camera_properties import CameraProperties

try:
    import tkinter as tk
    tk.Tk().destroy()
    HAVE_DISPLAY = True
except Exception:
    HAVE_DISPLAY = False


class FakeDrone:
    def __init__(self):
        self.calls = []

    def set_aperture(self, camera, f):
        self.calls.append(("aperture", camera, f))
        return True


def sim_session():
    camera = CameraProperties()
    payloads = [SimpleNamespace(drone=FakeDrone(), camera=camera) for _ in range(2)]
    return SimpleNamespace(camera=camera, station=SimpleNamespace(cameras=payloads))


def run(session, line):
    from px4_gimbal_cesium import camera_command
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        camera_command(session, line.split())
    return out.getvalue()


class CameraCommandTests(unittest.TestCase):
    def setUp(self):
        try:
            import px4_gimbal_cesium  # noqa: F401  (needs the projectairsim client)
        except ImportError as err:
            self.skipTest(f"px4_gimbal_cesium won't import: {err}")
        self.session = sim_session()
        self.effects = self.session.camera.effects

    def test_distortion(self):
        self.assertIn("k1=-0.2 k2=0.03", run(self.session, "distortion -0.2 0.03"))
        self.assertIsNotNone(self.effects.distortion)
        self.assertIn("off", run(self.session, "distortion off"))
        self.assertIsNone(self.effects.distortion)
        run(self.session, "distortion on")
        self.assertEqual(self.effects.distortion_coeffs[:2], (-0.2, 0.03))

    def test_vibration(self):
        self.assertIn("0.05 deg at 12 Hz", run(self.session, "vibration 0.05 12"))
        self.assertIn("off", run(self.session, "vibration off"))
        self.assertEqual(self.effects.vibration_deg, 0.0)

    def test_noise_with_temperature(self):
        run(self.session, "noise 12 33")
        self.assertEqual((self.effects.gain_db, self.effects.temperature_c), (12.0, 33.0))
        self.assertGreater(self.effects.sigma, 0)

    def test_aperture_goes_to_every_drone_and_is_remembered(self):
        run(self.session, "aperture 5.6")
        for payload in self.session.station.cameras:
            self.assertEqual(payload.drone.calls, [("aperture", "GimbalCam", 5.6)])
        self.assertEqual(self.session.camera.aperture_f, 5.6)
        self.assertIn("Usage", run(self.session, "aperture 0"))

    def test_bad_input_shows_the_usage(self):
        for line in ("", "distortion", "vibration fast", "zoom 3"):
            self.assertIn("Usage", run(self.session, line), line)


@unittest.skipUnless(HAVE_DISPLAY, "needs a display for Tk")
class CameraPanelTests(unittest.TestCase):
    def setUp(self):
        from gcs.camera_panel import CameraPanel
        from gcs.dashboard import CameraWindows
        self.sent = []
        self.windows = CameraWindows(on_gimbal=lambda *a: None, on_zoom=lambda *a: None)
        self.windows.add_controls(["Drone1"], self.sent.append)
        self.camera = CameraProperties(os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), "camera_properties.json"))
        self.panel = CameraPanel(self.windows, self.camera)
        self.windows.add_camera_drawer(self.panel.load)

    def tearDown(self):
        self.windows.root.destroy()

    def test_f3_and_f2_share_the_dock(self):
        w = self.windows
        w.toggle_camera_drawer()
        self.assertTrue(w.camera_drawer_open)
        w.toggle_drawer(True)  # the safety drawer replaces it
        self.assertFalse(w.camera_drawer_open)
        w.toggle_camera_drawer(True)
        self.assertFalse(w.drawer_open)

    def test_controls_send_sim_camera_commands(self):
        self.windows.toggle_camera_drawer(True)
        p = self.panel
        self.assertEqual(p._res.get(), "1080p")  # from camera_properties.json
        p._res.set("720p")
        p._apply_resolution()
        p._aperture.set("4")
        p._apply_aperture()
        for var, value in zip(p._coeffs, ("-0.1", "0.02", "0", "0", "0.001")):
            var.set(value)
        p._apply_distortion()
        p._amplitude.set("0.03")
        p._apply_vibration()
        p._vibration_on.set(False)
        self.assertEqual(self.sent, ["sim camera res 720p", "sim camera aperture 4",
                                     "sim camera distortion -0.1 0.02 0 0 0.001",
                                     "sim camera vibration 0.03 30"])
        p._gain.set("loud")
        p._apply_noise()
        self.assertEqual(len(self.sent), 4)  # not a number: nothing sent

    def test_tick_boxes_follow_the_camera(self):
        self.windows.toggle_camera_drawer(True)
        self.assertFalse(self.panel._noise_on.get())
        self.camera.effects.set_noise(True)  # e.g. typed in the console
        self.panel.refresh()
        self.assertTrue(self.panel._noise_on.get())
        self.assertIn("noise      2.0 levels", self.panel._now.cget("text"))


if __name__ == "__main__":
    unittest.main()
