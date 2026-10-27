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

    def test_later_drones_save_gpu(self):
        sensors = by_id(Aircraft("x8_arduplane").robot(1)["sensors"], "id")
        self.assertEqual(sensors["Chase"]["capture-settings"][0]["width"], 640)

    def test_sitl_spec(self):
        sitl = Aircraft("x8_arduplane").sitl(2)
        self.assertEqual((sitl.family, sitl.instance, sitl.sysid), ("ardupilot", 7, 3))
        self.assertTrue(sitl.params[-1].endswith("arduplane_swarm.parm"))
        self.assertEqual(Aircraft("quad_px4").sitl(0).frame, "none_iris")


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
        with self.assertRaisesRegex(ValueError, "land"):
            self.build([{"goto": {"north": 10}, "land": True}])
        self.build([{"heading": "travel", "goto": {"north": 10}}])  # fine


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
