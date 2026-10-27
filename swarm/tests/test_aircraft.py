"""Aircraft registry, autopilot dialects and fixed-wing missions, offline:
python -m unittest discover -s tests -v."""

import math
import unittest
from types import SimpleNamespace

from aircraft import Aircraft, list_models
from gcs.airframe import Airframe, airframe_of
from gcs.autopilot import dialect
from gcs.mavlink_link import M, VehicleLink
from gcs.mission import DroneMission, build_items

FIXEDWING = airframe_of("x8_arduplane")


def by_id(items, key):
    return {item[key]: item for item in items}


class RegistryTests(unittest.TestCase):
    def test_every_model_passes_its_audit(self):
        for name in list_models():
            errors = [m for level, m in Aircraft(name).audit() if level == "ERROR"]
            self.assertEqual(errors, [], name)

    def test_every_model_matches_the_client_schema(self):
        from projectairsim.utils import load_text_resource, validate_json
        schema = load_text_resource("schema/robot_config_schema.jsonc")
        for name in list_models():
            for index in (0, 1):
                validate_json(Aircraft(name).robot(index, home=(1.4, 103.8)), schema)

    def test_ports_follow_the_instance(self):
        quad = Aircraft("quad_px4").robot(2)["controller"]["px4-settings"]
        self.assertEqual((quad["tcp-port"], quad["control-port"], quad["gimbal-port"]),
                         (4562, 14542, 13282))
        plane = Aircraft("x8_arduplane")
        self.assertEqual(plane.instance(1), 6)
        self.assertEqual(plane.robot(1)["controller"]["arduplane-settings"]["local-host-udp-port"],
                         9062)

    def test_px4_estimator_origin_is_the_scene_home(self):
        params = Aircraft("quad_px4").robot(0, home=(1.3, 103.7))["controller"]["px4-settings"][
            "parameters"]
        self.assertEqual((params["LPE_LAT"], params["LPE_LON"]), (1.3, 103.7))

    def test_every_airframe_carries_the_same_gimbal_camera(self):
        for name in list_models():
            robot = Aircraft(name).robot(0)
            sensors, actuators = by_id(robot["sensors"], "id"), by_id(robot["actuators"], "name")
            self.assertEqual(sensors["GimbalCam"]["gimbal"]["gimbal-id"], "Gimbal_Payload_Actuator")
            self.assertIn("gimbal-settings", actuators["Gimbal_Payload_Actuator"])
        gimbal = Aircraft("x8_arduplane").robot(0)["controller"]["arduplane-settings"]["gimbal"]
        self.assertEqual((gimbal["pitch-min-deg"], gimbal["pitch-max-deg"]), (-90, 25))

    def test_swarm_needs_nothing_from_the_example_scripts(self):
        import pathlib
        swarm = pathlib.Path(__file__).resolve().parent.parent
        users = [str(p.relative_to(swarm)) for p in swarm.rglob("*")
                 if p.is_file() and p.suffix in (".py", ".yml", ".jsonc", ".parm", ".sh")
                 and "tests" not in p.parts and "runs" not in p.parts
                 and "example_user_scripts" in p.read_text(errors="ignore")]
        self.assertEqual(users, [])

    def test_later_drones_save_gpu(self):
        sensors = by_id(Aircraft("x8_arduplane").robot(1)["sensors"], "id")
        self.assertEqual(sensors["Chase"]["capture-settings"][0]["width"], 640)

    def test_sitl_spec(self):
        sitl = Aircraft("x8_arduplane").sitl(2)
        self.assertEqual((sitl.family, sitl.instance, sitl.sysid), ("ardupilot", 7, 3))
        self.assertTrue(sitl.params[-1].endswith("arduplane_swarm.parm"))
        self.assertEqual(Aircraft("quad_px4").sitl(0).frame, "none_iris")


class CopiedGcsTests(unittest.TestCase):
    """gcs/ alone on a field laptop, without aircraft/models.yml."""

    def test_default_drones_need_no_registry(self):
        from unittest import mock
        import gcs.airframe
        with mock.patch.object(gcs.airframe, "MODELS_FILE", "/nonexistent/models.yml"):
            self.assertEqual(gcs.airframe.airframe_of(None), Airframe())
            self.assertEqual(gcs.airframe.airframe_of("quad_px4"), Airframe())
            with self.assertRaisesRegex(ValueError, "copy the swarm's aircraft/"):
                gcs.airframe.airframe_of("x8_arduplane")


class DialectTests(unittest.TestCase):
    def test_modes_share_the_ground_stations_names(self):
        px4, ap = dialect("px4"), dialect("ardupilot")
        self.assertEqual(px4.mode((4 << 16) | (4 << 24)), ("MISSION", "MISSION"))
        self.assertEqual(ap.mode(10), ("MISSION", "AUTO"))
        self.assertEqual(ap.mode(12), ("HOLD", "LOITER"))
        self.assertEqual(ap.mode(5), ("STABILIZED", "FBWA"))
        for mode in ("HOLD", "MISSION"):
            main, sub = ap.set_mode_params(mode)
            self.assertEqual(ap.mode(main)[0], mode)

    def test_integer_parameters(self):
        px4, ap = dialect("px4"), dialect("ardupilot")
        ptype, raw = px4.encode_param(2)
        self.assertEqual(px4.decode_param(raw, ptype), 2)
        self.assertNotEqual(raw, 2.0)  # bytewise
        ptype, raw = ap.encode_param(7)
        self.assertEqual(raw, 7.0)
        self.assertEqual(ap.decode_param(7.0, M.MAV_PARAM_TYPE_INT8), 7)


def fake_link(airframe=None):
    """A VehicleLink whose uploads are answered at once; .sent has the items."""
    sent = []
    link = None

    def deliver(msg):
        msg._header.srcSystem = 1
        msg._header.srcComponent = 1
        link._on_message(msg)

    def count(sysid, compid, total, mission_type):
        for seq in range(total):
            deliver(M.MAVLink_mission_request_int_message(245, 190, seq, mission_type))
        deliver(M.MAVLink_mission_ack_message(245, 190, M.MAV_MISSION_ACCEPTED, mission_type))

    mav = SimpleNamespace(mission_count_send=count,
                          mission_item_int_send=lambda *args: sent.append(args))
    conn = SimpleNamespace(source_component=190, source_system=245,
                           subscribe=lambda callback: None,
                           send_as=lambda compid, send, bypass_outage=False: send(mav))
    link = VehicleLink(conn, 1, "Drone1", airframe)
    link.position = (1.0, 103.0, 40.0, 30.0)
    link.sent = sent
    link.deliver = deliver
    return link


class ArduPilotMissionTests(unittest.TestCase):
    def test_home_slot_keeps_item_numbers(self):
        link = fake_link(FIXEDWING)
        first, total = link.upload_mission([(1.001, 103.0), (1.002, 103.0)], 50)
        self.assertEqual((first, total), (0, 2))
        self.assertEqual(len(link.sent), 3)  # home placeholder + 2 waypoints
        self.assertEqual([a[2] for a in link.sent], [0, 1, 2])
        link.deliver(M.MAVLink_mission_current_message(1))
        self.assertEqual(link.mission_seq, 0)  # our first waypoint
        link.deliver(M.MAVLink_mission_item_reached_message(2))
        self.assertEqual(link.mission_reached, 1)

    def test_px4_has_no_home_slot(self):
        link = fake_link()
        link.upload_mission([(1.001, 103.0)], 50)
        self.assertEqual(len(link.sent), 1)


class FixedWingMissionTests(unittest.TestCase):
    def build(self, steps, airframe=FIXEDWING, airborne=True):
        return build_items(DroneMission("Drone1", steps), (1, 103), (1, 103), 30, airborne, 1,
                           airframe=airframe)

    def test_hold_circles(self):
        items = self.build([{"goto": {"north": 100}, "hold": 20}, {"wait": 5}])
        commands = [i[1] for i in items]
        self.assertNotIn(M.MAV_CMD_NAV_DELAY, commands)
        loiters = [i for i in items if i[1] == M.MAV_CMD_NAV_LOITER_TIME]
        self.assertEqual([i[2][0] for i in loiters], [20, 5])

    def test_no_gimbal_claim_items_for_ardupilot(self):
        items = self.build([{"gimbal": {"pitch": -30}}, {"track": {"target": "Vehicle1"}}])
        self.assertNotIn(M.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE, [i[1] for i in items])
        self.assertIn(M.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW, [i[1] for i in items])
        quad = self.build([{"gimbal": {"pitch": -30}}], airframe=Airframe())
        self.assertIn(M.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE, [i[1] for i in quad])

    def test_track_steps_start_after_their_waypoint(self):
        dm = DroneMission("Drone1", [{"goto": {"north": 100}, "track": {"target": "Car"}},
                                     {"goto": {"north": 200}}, {"track": "off"},
                                     {"goto": {"north": 300}}])
        items = build_items(dm, (1, 103), (1, 103), 30, True, 1, airframe=FIXEDWING)
        (start, spec), (end, none) = dm.track_steps
        self.assertEqual(spec["target"], "Car")
        self.assertIsNone(none)
        waypoints = [n for n, i in enumerate(items) if i[1] == M.MAV_CMD_NAV_WAYPOINT]
        # Tracking starts once the first waypoint is done, ends after the second
        self.assertEqual((start, end), (waypoints[0] + 1, waypoints[1] + 1))

    def test_no_item_carries_a_nan_ardupilot_refuses(self):
        # ArduPilot rejects a NaN in parameters 1-4 except p4 of waypoint, land, takeoff
        dm = DroneMission("Drone1", [
            {"takeoff": 40}, {"goto": {"north": 100}, "gimbal": {"pitch": -60, "yaw": 0}},
            {"look_at": {"north": 200, "east": 0}}, {"track": {"target": "Car"}},
            {"goto": {"north": 300}, "hold": 5}, {"track": "off"}, {"look_at": "none"},
            {"gimbal": {"pitch": -30, "heading": 90}}, {"photo": True}, {"wait": 3},
            {"land": {"north": 0, "east": 0, "heading": 33}}])
        items = build_items(dm, (1, 103), (1, 103), 0, False, 1, airframe=FIXEDWING)
        p4_ok = (M.MAV_CMD_NAV_WAYPOINT, M.MAV_CMD_NAV_LAND, M.MAV_CMD_NAV_TAKEOFF)
        for frame, command, params, *_ in items:
            for n, value in enumerate(params, start=1):
                if math.isnan(value):
                    self.assertTrue(n == 4 and command in p4_ok, (command, n))

    def test_gimbal_step_needs_a_pitch_on_ardupilot(self):
        with self.assertRaisesRegex(ValueError, "pitch"):
            self.build([{"gimbal": {"heading": 90}}])
        self.build([{"gimbal": {"heading": 90}}], airframe=Airframe())  # PX4 keeps NaN

    def test_takeoff_climbs_out_and_waypoints_use_its_own_radius(self):
        items = self.build([{"goto": {"north": 100}}], airborne=False)
        takeoff = next(i for i in items if i[1] == M.MAV_CMD_NAV_TAKEOFF)
        self.assertGreater(takeoff[2][0], 0)  # minimum pitch
        waypoint = next(i for i in items if i[1] == M.MAV_CMD_NAV_WAYPOINT)
        self.assertEqual(waypoint[2][1], 0)
        self.assertFalse(math.isnan(waypoint[2][3]))

    def test_what_a_fixed_wing_cant_do(self):
        with self.assertRaisesRegex(ValueError, "heading"):
            self.build([{"heading": 90}])
        with self.assertRaisesRegex(ValueError, "runway"):
            self.build([{"goto": {"north": 10}, "land": True}])
        self.build([{"heading": "travel", "goto": {"north": 10}}])  # fine

    def test_lands_on_a_runway_from_an_approach(self):
        items = self.build([{"goto": {"north": 500}},
                            {"land": {"north": 0, "east": 0, "heading": 90, "approach_m": 300}}])
        start, approach, land = items[-3:]
        self.assertEqual([start[1], approach[1], land[1]],
                         [M.MAV_CMD_DO_LAND_START, M.MAV_CMD_NAV_WAYPOINT, M.MAV_CMD_NAV_LAND])
        # Landing eastwards: the approach is 300 m west of the touchdown, at 30 m
        self.assertAlmostEqual(land[3], 1.0, places=6)
        self.assertAlmostEqual((approach[4] - land[4]) * 111_320 * math.cos(math.radians(1)), -300, delta=1)
        self.assertEqual((approach[5], land[5]), (30, 0))

    def test_land_step_is_validated(self):
        from gcs.mission import _validate_step
        _validate_step({"land": {"north": 0, "east": 0, "heading": 33}}, "step 1")
        _validate_step({"land": True}, "step 1")
        with self.assertRaisesRegex(ValueError, "heading"):
            _validate_step({"land": {"north": 0, "east": 0}}, "step 1")
        with self.assertRaisesRegex(ValueError, "approach_m"):
            _validate_step({"land": {"north": 0, "east": 0, "heading": 3, "approach_m": 0}}, "step 1")


class FixedWingLinkTests(unittest.TestCase):
    def test_no_heading_and_no_turning(self):
        link = fake_link(FIXEDWING)
        self.assertIsNone(link.turn_to(90))
        sent = []
        link.conn.send_as = lambda compid, send, bypass_outage=False: send(
            SimpleNamespace(command_int_send=lambda *a: sent.append(a)))
        link.fly_to(1.01, 103.0, 50, heading_deg=90)
        self.assertTrue(math.isnan(sent[0][9]))  # param4: ArduPlane's loiter direction


if __name__ == "__main__":
    unittest.main()


class FixedWingSearchTests(unittest.TestCase):
    def test_consecutive_passes_leave_room_to_turn(self):
        from gcs.search_planner import pass_order
        for count, spacing, radius in ((10, 40, 35), (12, 25, 35), (30, 15, 50)):
            order = pass_order(count, spacing, radius)
            self.assertEqual(sorted(order), list(range(count)))
            steps = [abs(b - a) * spacing for a, b in zip(order, order[1:])]
            tight = [s for s in steps if s < 2 * radius]
            self.assertLessEqual(len(tight), 2, (count, spacing, radius, order))

    def test_multirotors_still_fly_in_order(self):
        from gcs.search_planner import pass_order
        self.assertEqual(pass_order(5, 40, 0), [0, 1, 2, 3, 4])
        self.assertEqual(pass_order(5, 100, 35), [0, 1, 2, 3, 4])  # passes already 2R apart

    def test_mixed_fleet_covers_the_same_passes(self):
        from gcs.search_planner import plan_area_search
        box = [(1.40, 103.86), (1.40, 103.87), (1.41, 103.87), (1.41, 103.86)]
        quads, _ = plan_area_search(box, 2, 40)
        mixed, _ = plan_area_search(box, 2, 40, [0.0, 35.0])
        self.assertEqual(quads[0], mixed[0])
        self.assertEqual(sorted(quads[1]), sorted(mixed[1]))
        self.assertNotEqual(quads[1], mixed[1])
