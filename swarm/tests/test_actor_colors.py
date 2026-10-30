"""Moving actors' `color:` and the ReID scenario that uses it."""

import os
import unittest

import yaml

try:
    from moving_actors import COLORS, MovingActorManager, expand_actors, parse_color
except ImportError as err:  # needs the projectairsim client
    raise unittest.SkipTest(f"moving_actors won't import: {err}")

SWARM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class FakeWorld:
    def __init__(self, slots=("SKM_SportsCar/Body",)):
        self.calls, self.slots = [], list(slots)

    def set_object_color(self, name, rgb, parameter, slot_filter):
        self.calls.append((name, rgb, parameter, slot_filter))
        return self.slots


class ParseColorTests(unittest.TestCase):
    def test_names_and_lists(self):
        self.assertEqual(parse_color("Red"), COLORS["red"])
        self.assertEqual(parse_color([0, 0.5, 1]), [0.0, 0.5, 1.0])

    def test_bad_colors(self):
        for bad in ("mauve", [1, 0], [0, 0, 2], [-0.1, 0, 0]):
            with self.assertRaises(ValueError, msg=bad):
                parse_color(bad)


class PaintTests(unittest.TestCase):
    def manager(self, world):
        m = object.__new__(MovingActorManager)
        m.world = world
        return m

    def test_vehicles_tint_the_body_by_default(self):
        (kind, opts), = expand_actors({"actors": [{"name": "Car", "kind": "vehicle",
                                                   "color": "blue"}]})
        world = FakeWorld()
        self.manager(world)._paint("Car", opts)
        self.assertEqual(world.calls, [("Car", COLORS["blue"], "MetalPartsTint", "Body")])

    def test_parameter_and_slots_can_be_changed(self):
        world = FakeWorld()
        self.manager(world)._paint("Box", {"color": [1, 1, 0], "color_parameter": "Tint",
                                           "color_slots": ""})
        self.assertEqual(world.calls, [("Box", [1.0, 1.0, 0.0], "Tint", "")])

    def test_nothing_painted_is_a_warning_not_an_error(self):
        with self.assertLogs(level="WARNING") as logs:
            self.manager(FakeWorld(slots=[]))._paint("Car", {"color": "red"})
        self.assertIn("no material took color red", "\n".join(logs.output))


class ReidScenarioTests(unittest.TestCase):
    def test_four_differently_coloured_cars_on_closed_runway_loops(self):
        with open(os.path.join(SWARM, "missions", "scenarios", "reid_vehicles.yml")) as f:
            actors = expand_actors(yaml.safe_load(f))
        self.assertEqual(len(actors), 4)
        colors = [opts["color"] for _, opts in actors]
        self.assertEqual(len(set(colors)), 4)
        for kind, opts in actors:
            parse_color(opts["color"])
            self.assertEqual(kind, "vehicle")
            self.assertFalse(opts["follow_roads"])
            self.assertEqual(opts["mode"], "loop")
            self.assertGreaterEqual(len(opts["waypoints"]), 3)


if __name__ == "__main__":
    unittest.main()
