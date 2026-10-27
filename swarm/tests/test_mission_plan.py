"""The map planner's data side (gcs/mission_plan.py) and the editor's mission handling
(actor_editor.py): python -m unittest discover -s tests -v."""

import os
import tempfile
import unittest
from pathlib import Path

import yaml

import actor_editor
from gcs import mission_plan
from gcs.mission import load_mission

ORIGIN = (1.413425, 103.865)
MISSIONS = Path(__file__).resolve().parent.parent / "missions"


def quad(steps, **extra):
    return {"mission": "t", "drones": {"Drone1": {"steps": steps}}, **extra}


class NormalizeTests(unittest.TestCase):
    def test_north_east_become_lat_lon(self):
        data = mission_plan.normalize(quad([{"takeoff": 30}, {"goto": {"north": 111, "east": 0, "alt": 40}},
                                            {"look_at": {"north": 0, "east": 100}}]), ORIGIN)
        goto = data["drones"]["Drone1"]["steps"][1]["goto"]
        self.assertAlmostEqual(goto["lat"], ORIGIN[0] + 0.000998, places=4)
        self.assertEqual(goto["alt"], 40)
        self.assertNotIn("north", goto)
        self.assertIn("lon", data["drones"]["Drone1"]["steps"][2]["look_at"])

    def test_drones_by_number_and_input_untouched(self):
        raw = {"drones": {1: {"steps": [{"goto": {"north": 5, "east": 5}}]}}}
        data = mission_plan.normalize(raw, ORIGIN, ["Alpha"])
        self.assertEqual(list(data["drones"]), ["Alpha"])
        self.assertIn("north", raw["drones"][1]["steps"][0]["goto"])


class PreviewTests(unittest.TestCase):
    def test_route_and_length(self):
        data = mission_plan.normalize(quad([{"takeoff": 30}, {"goto": {"north": 100, "east": 0}},
                                            {"goto": {"north": 100, "east": 100}}, {"rtl": True}],
                                           speed=10), ORIGIN)
        r = mission_plan.preview(data, ORIGIN)["Drone1"]
        self.assertNotIn("error", r)
        self.assertEqual(len(r["route"]), 2)
        self.assertAlmostEqual(r["length_m"], 200, delta=3)
        self.assertEqual(r["time_s"], 20)

    def test_error_names_the_step_and_spares_other_drones(self):
        data = {"drones": {"A": {"steps": [{"goto": {"lat": 1.4, "lon": 103.8}, "heading": 90}]},
                           "B": {"steps": [{"goto": {"lat": 1.4, "lon": 103.8}}]}}}
        out = mission_plan.preview(data, ORIGIN, {"A": "x8_arduplane", "B": "x8_arduplane"})
        self.assertEqual(out["A"]["error_step"], 1)
        self.assertIn("heading", out["A"]["error"])
        self.assertNotIn("error", out["B"])
        # a quad can turn on the spot
        self.assertNotIn("error", mission_plan.preview(data, ORIGIN, {"A": "quad_px4"})["A"])

    def test_empty_drone(self):
        self.assertEqual(mission_plan.preview({"drones": {"A": {"steps": []}}}, ORIGIN)["A"]["steps"], 0)


class DumpTests(unittest.TestCase):
    def test_round_trip_loads(self):
        data = mission_plan.normalize(quad([{"takeoff": 30}, {"goto": {"lat": 1.414, "lon": 103.866}},
                                            {"gimbal": {"pitch": -45}}, {"land": True}], speed=6),
                                      ORIGIN)
        text = mission_plan.dump(data)
        self.assertTrue(text.startswith("#"))
        self.assertLess(text.index("mission:"), text.index("speed:"))
        self.assertLess(text.index("speed:"), text.index("drones:"))
        with tempfile.NamedTemporaryFile("w", suffix=".yml", delete=False) as f:
            f.write(text)
        try:
            self.assertEqual(len(load_mission(f.name, ["Drone1"]).drones[0].steps), 4)
        finally:
            os.unlink(f.name)

    def test_nothing_to_fly(self):
        with self.assertRaises(ValueError):
            mission_plan.dump({"drones": {"A": {"steps": []}}})

    def test_header_kept(self):
        with tempfile.NamedTemporaryFile("w", suffix=".yml", delete=False) as f:
            f.write("# my notes\n# more\nmission: m\n")
        try:
            self.assertEqual(mission_plan.header_of(f.name), "# my notes\n# more\n")
        finally:
            os.unlink(f.name)

    def test_shipped_missions_survive_the_planner(self):
        """Open, convert to lat/lon and dump each shipped mission: it still loads."""
        names = [f"Drone{i}" for i in range(1, 5)]
        for path in sorted(MISSIONS.glob("*.yml")):
            with self.subTest(mission=path.name):
                raw = yaml.safe_load(path.read_text())
                text = mission_plan.dump(mission_plan.normalize(raw, ORIGIN, names))
                # beside the original, so its relative `actors:` file is found
                with tempfile.NamedTemporaryFile("w", suffix=".yml", dir=MISSIONS, delete=False) as f:
                    f.write(text)
                try:
                    before = load_mission(str(path), names)
                    after = load_mission(f.name, names)
                finally:
                    os.unlink(f.name)
                self.assertEqual({d.name: len(d.steps) for d in before.drones.values()},
                                 {d.name: len(d.steps) for d in after.drones.values()})


class EditorTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def editor(self, name="m.yml", text=None, **kw):
        path = os.path.join(self.tmp.name, name)
        if text is not None:
            Path(path).write_text(text)
        return actor_editor.Editor(os.path.join(self.tmp.name, "traffic.yml"), *ORIGIN, path, None, **kw)

    def test_new_mission_state(self):
        state = self.editor().mission_state()
        self.assertFalse(state["exists"])
        self.assertEqual(state["data"]["mission"], "m")
        self.assertEqual(state["drones"], ["Drone1"])
        self.assertTrue(state["known_models"]["quad_px4"]["hovers"])
        self.assertFalse(state["known_models"]["x8_arduplane"]["hovers"])

    def test_default_model_applies(self):
        state = self.editor(model="x8_arduplane").mission_state()
        self.assertEqual(state["models"]["Drone1"], "x8_arduplane")

    def test_save_keeps_backup_and_header_and_reports_problems(self):
        ed = self.editor(text="# keep me\nmission: m\ndrones:\n  Drone1:\n    steps: [{takeoff: 20}]\n")
        data = ed.mission_state()["data"]
        data["drones"]["Drone1"]["steps"] = [{"goto": {"lat": 1.4, "lon": 103.8}, "heading": 90}]
        problems = ed.mission_save(data, {"Drone1": "x8_arduplane"})
        self.assertEqual(len(problems), 1)
        self.assertIn("Drone1", problems[0])
        text = Path(ed.mission_path).read_text()
        self.assertTrue(text.startswith("# keep me"))
        self.assertIn("takeoff: 20", Path(ed.mission_path + ".bak").read_text())

    def test_resolve_mission(self):
        self.assertTrue(actor_editor.resolve_mission("x8_circuit").endswith("missions/x8_circuit.yml"))
        self.assertTrue(actor_editor.resolve_mission("brand_new").endswith("missions/brand_new.yml"))

    def test_actors_file_from_the_mission(self):
        path = os.path.join(self.tmp.name, "m.yml")
        Path(path).write_text("actors: traffic_a.yml\ndrones: {}\n")
        self.assertEqual(actor_editor.mission_actors_path(path), os.path.join(self.tmp.name, "traffic_a.yml"))
        self.assertIsNone(actor_editor.mission_actors_path(os.path.join(self.tmp.name, "none.yml")))


if __name__ == "__main__":
    unittest.main()
