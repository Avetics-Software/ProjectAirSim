"""Every mission file that ships loads, and the starter files build for their aircraft:
python -m unittest discover -s tests -v."""

import contextlib
import io
import unittest
from pathlib import Path

from gcs.airframe import airframe_of
from gcs.mission import build_items, load_mission, main

MISSIONS = Path(__file__).resolve().parent.parent / "missions"
NAMES = [f"Drone{i}" for i in range(1, 5)]


class ShippedMissionTests(unittest.TestCase):
    def test_every_mission_file_loads(self):
        files = sorted(MISSIONS.glob("*.yml"))
        self.assertGreaterEqual(len(files), 6)
        for path in files:
            with self.subTest(mission=path.name):
                self.assertTrue(load_mission(str(path), NAMES).drones)

    def test_starters_build_for_their_aircraft(self):
        for name, model in (("template.yml", "quad_px4"),
                            ("template_fixedwing.yml", "x8_arduplane"),
                            ("x8_circuit.yml", "x8_arduplane"), ("x8_track.yml", "x8_arduplane")):
            with self.subTest(mission=name):
                mission = load_mission(str(MISSIONS / name), NAMES)
                for i, dm in mission.drones.items():
                    items = build_items(dm, (1.4, 103.8), (1.4, 103.8), 0.0, False, i + 1,
                                        airframe=airframe_of(model))
                    self.assertGreater(len(items), 3)

    def test_the_fixed_wing_starter_also_flies_on_a_quad(self):
        # `hold` and `land: {...}` are valid for both aircraft
        mission = load_mission(str(MISSIONS / "template_fixedwing.yml"), NAMES)
        for model in ("quad_px4", "x8_arduplane"):
            build_items(mission.drones[0], (1.4, 103.8), (1.4, 103.8), 0.0, False, 1,
                        airframe=airframe_of(model))


class CheckerTests(unittest.TestCase):
    def run_main(self, *args):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = main([*args])
        return code, out.getvalue(), err.getvalue()

    def test_prints_what_the_aircraft_would_upload(self):
        code, out, _ = self.run_main(str(MISSIONS / "x8_circuit.yml"), "--model", "x8_arduplane",
                                     "--drones", "1")
        self.assertEqual(code, 0)
        self.assertIn("x8_arduplane, fixed-wing", out)
        for command in ("NAV_TAKEOFF", "DO_LAND_START", "NAV_LAND"):
            self.assertIn(command, out)

    def test_a_mistake_is_reported_and_exits_1(self):
        bad = Path(__file__).parent / "_bad_mission.yml"
        bad.write_text("drones:\n  Drone1:\n    steps:\n      - goto: {north: 10}\n"
                       "        speeed: 5\n")
        try:
            code, _, err = self.run_main(str(bad), "--drones", "1")
        finally:
            bad.unlink()
        self.assertEqual(code, 1)
        self.assertIn("speeed", err)

    def test_a_fixed_wing_mission_is_refused_a_heading(self):
        bad = Path(__file__).parent / "_bad_mission2.yml"
        bad.write_text("drones:\n  Drone1:\n    steps:\n      - goto: {north: 10}\n"
                       "        heading: 90\n")
        try:
            code, _, err = self.run_main(str(bad), "--model", "x8_arduplane", "--drones", "1")
        finally:
            bad.unlink()
        self.assertEqual(code, 1)
        self.assertIn("heading", err)


if __name__ == "__main__":
    unittest.main()
