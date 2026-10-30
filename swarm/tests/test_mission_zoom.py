"""Mission `zoom` steps: parsed, placed at the right item, applied by the
ground station as PX4 reaches them, and turned into the camera's percentage."""

import unittest
from types import SimpleNamespace

from gcs.mission import DroneMission, MissionControl, build_items, zoom_spec
from gcs.station import DroneCamera
from gcs.mavlink_link import M


class ZoomSpecTests(unittest.TestCase):
    def test_factors_and_percentages(self):
        self.assertEqual(zoom_spec("2x"), {"factor": 2.0})
        self.assertEqual(zoom_spec("1.5X"), {"factor": 1.5})
        self.assertEqual(zoom_spec(25), {"percent": 25.0})

    def test_bad_zooms(self):
        for bad in ("0.5x", "twox", 150, -1, True, "2"):
            with self.assertRaises(ValueError, msg=bad):
                zoom_spec(bad)


class ZoomItemTests(unittest.TestCase):
    def test_zoom_applies_on_arrival_with_the_camera_steps(self):
        dm = DroneMission("Drone1", [
            {"takeoff": 40},
            {"goto": {"north": 100, "alt": 150}, "heading": 123},
            {"gimbal": {"pitch": -90}, "zoom": "2x", "hold": 300},
            {"rtl": True, "zoom": "1x"}])
        items = build_items(dm, (1, 103), (1, 103), 0, False, 1)
        (at, spec), (back_at, back) = dm.zoom_steps
        self.assertEqual(spec, {"factor": 2.0})
        # When PX4 moves on from the waypoint to the step's gimbal items
        self.assertEqual(items[at - 1][1], M.MAV_CMD_NAV_WAYPOINT)
        self.assertIn(items[at][1], (M.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                                     M.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW))
        # Back to 1x as the return home starts
        self.assertEqual(back, {"factor": 1.0})
        self.assertEqual(items[back_at][1], M.MAV_CMD_NAV_RETURN_TO_LAUNCH)


class FakeCamera(DroneCamera):
    def __init__(self, max_zoom):
        self.link = SimpleNamespace(name="Drone1")
        self.max_zoom = max_zoom
        self.set = []

    def set_zoom(self, percent):
        self.set.append(percent)


class ZoomToTests(unittest.TestCase):
    def test_factor_to_percent_on_a_10x_lens(self):
        camera = FakeCamera(10.0)
        self.assertAlmostEqual(camera.zoom_to({"factor": 2.0}), 100 / 9)  # 2x of 10x: 11.1%
        self.assertEqual(camera.zoom_to({"factor": 1.0}), 0.0)
        self.assertEqual(camera.zoom_to({"percent": 40.0}), 40.0)
        self.assertEqual(camera.zoom_to({"factor": 20.0}), 100.0)  # past the lens: its longest

    def test_a_real_camera_without_a_known_range_needs_a_percentage(self):
        with self.assertRaises(ValueError):
            FakeCamera(None).zoom_to({"factor": 2.0})


class MissionZoomHookTests(unittest.TestCase):
    def test_the_hook_gets_the_drone_and_spec_and_failures_only_warn(self):
        calls = []
        mc = MissionControl([SimpleNamespace(name="Drone1")], None, lambda: None, "/tmp",
                            on_zoom=lambda i, spec: calls.append((i, spec)))
        mc._zoom(0, {"factor": 2.0})
        self.assertEqual(calls, [(0, {"factor": 2.0})])

        def broken(i, spec):
            raise ValueError("no range")
        mc.on_zoom = broken
        with self.assertLogs(level="WARNING"):
            mc._zoom(0, {"factor": 2.0})


if __name__ == "__main__":
    unittest.main()
