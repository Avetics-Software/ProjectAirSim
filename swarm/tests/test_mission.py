"""Offline mission checks: python -m unittest discover -s tests -v."""

import asyncio
import csv
import math
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import yaml

from gcs.airframe import Airframe
from gcs.autopilot import dialect
from gcs.companion import CompanionLink
from gcs.mavlink_link import M, VehicleLink
from gcs.mission import (DroneMission, MissionControl, build_items, compare_runs,
                         fence_corners, load_mission)
from gcs.safety import SafetyMonitor
from gcs.swarm import SwarmControl


class MissionFileTests(unittest.TestCase):
    def load(self, data):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test.yml"
            path.write_text(yaml.safe_dump(data))
            return load_mission(path, ["Drone1", "Drone2"])

    def test_bad_inputs_are_rejected_before_upload(self):
        steps = [
            {"goto": {"lat": 91, "lon": 103}},
            {"goto": {"north": math.nan}},
            {"goto": {"lat": 1, "lon": 103, "east": 10}},
            {"gimbal": {"yaw": 1, "heading": 2}},
            {"gimbal": "down"}, {"gimbal": {"pitch": -100}},
            {"speed": -1}, {"wait": math.inf}, {"takeoff": True},
            {"land": "false"}, {"land": True, "goto": {"north": 1}},
        ]
        for step in steps:
            with self.subTest(step=step), self.assertRaises(ValueError):
                self.load({"drones": {"Drone1": {"steps": [step]}}})
        for data in ([], {"drones": []}, {"drones": {"Drone1": {"steps": []}}},
                     {"mission": "../escape", "drones": {"Drone1": {"steps": [{"rtl": True}]}}}):
            with self.subTest(data=data), self.assertRaises(ValueError):
                self.load(data)

    def test_fence_corners_and_ceiling(self):
        steps = {"Drone1": {"steps": [{"rtl": True}]}}
        corners = [{"north": -10, "east": -10}, {"north": 10, "east": -10},
                   {"lat": 1.0001, "lon": 103.0001}]
        mission = self.load({"drones": steps, "fence": {"corners": corners, "ceiling": 150}})
        self.assertEqual(mission.fence["ceiling"], 150)
        lat, lon = fence_corners(mission.fence, (1, 103))[0]
        self.assertAlmostEqual((lat - 1) * 111_320, -10, places=3)
        self.assertLess(lon, 103)
        self.assertIsNone(self.load({"drones": steps, "fence": {"corners": corners}})
                          .fence["ceiling"])
        for fence in ([], {"corners": corners[:2]}, {"corners": corners, "ceiling": 0},
                      {"corners": corners, "floor": 5}, {"corners": corners + [{"alt": 3}]}):
            with self.subTest(fence=fence), self.assertRaises(ValueError):
                self.load({"drones": steps, "fence": fence})

    def test_terminal_actions_and_duplicate_drones(self):
        with self.assertRaisesRegex(ValueError, "after land"):
            self.load({"drones": {"Drone1": {"steps": [{"land": True}, {"wait": 1}]}}})
        with self.assertRaisesRegex(ValueError, "twice"):
            self.load({"drones": {"Drone1": {"steps": [{"wait": 1}]},
                                   1: {"steps": [{"wait": 2}]}}})

    def test_heading_camera_frames_and_hold_order(self):
        dm = DroneMission("Drone1", [
            {"takeoff": 30},
            {"goto": {"north": 100, "east": 20, "alt": 30}, "heading": 90,
             "gimbal": {"pitch": -45, "heading": 270}, "hold": 5},
            {"look_at": {"north": 200}}, {"look_at": "none"},
            {"photo": True}, {"rtl": True}], speed=6)
        items = build_items(dm, (1, 103), (1, 103), 0, False, 2)
        commands = [item[1] for item in items]
        wp = items[commands.index(M.MAV_CMD_NAV_WAYPOINT)]
        self.assertEqual(wp[2][3], 90)
        self.assertEqual(wp[2][0], 0)  # hold follows the camera change
        self.assertAlmostEqual(wp[3], 1 + 100 / 111320)
        claim = items[commands.index(M.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE)]
        self.assertEqual(claim[2][:2], (2, 1))
        camera_index = commands.index(M.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)
        camera = items[camera_index]
        self.assertEqual(camera[2][:2], (-45, 270))
        self.assertTrue(camera[3] & M.GIMBAL_MANAGER_FLAGS_YAW_LOCK)
        self.assertEqual(items[camera_index + 1][1], M.MAV_CMD_NAV_DELAY)
        self.assertEqual(items[camera_index + 1][2][0], 5)
        self.assertIn(M.MAV_CMD_DO_SET_ROI_LOCATION, commands)
        self.assertIn(M.MAV_CMD_IMAGE_START_CAPTURE, commands)
        self.assertEqual(commands[-1], M.MAV_CMD_NAV_RETURN_TO_LAUNCH)

    def test_heading_holds_until_changed_or_travel(self):
        dm = DroneMission("Drone1", [
            {"goto": {"north": 10}, "heading": 0}, {"goto": {"north": 20}},
            {"goto": {"north": 30}, "heading": "travel"}, {"goto": {"north": 40}}])
        items = build_items(dm, (1, 103), (1, 103), 20, True, 1)
        headings = [i[2][3] for i in items if i[1] == M.MAV_CMD_NAV_WAYPOINT]
        self.assertEqual(headings[:2], [0, 0])
        self.assertTrue(all(math.isnan(h) for h in headings[2:]))

    def test_look_at_is_reaimed_at_each_waypoint_before_its_hold(self):
        dm = DroneMission("Drone1", [
            {"look_at": {"north": 40}}, {"goto": {"east": 10}, "hold": 4},
            {"look_at": "none"}, {"goto": {"east": 20}}])
        items = build_items(dm, (1, 103), (1, 103), 20, True, 1)
        commands = [i[1] for i in items]
        roi = M.MAV_CMD_DO_SET_ROI_LOCATION
        self.assertEqual(commands.count(roi), 2)
        wp = commands.index(M.MAV_CMD_NAV_WAYPOINT)
        self.assertEqual(items[wp][2][0], 0)  # the hold waits for the re-aim
        self.assertEqual(commands[wp + 1:wp + 3], [roi, M.MAV_CMD_NAV_DELAY])
        self.assertEqual(commands[-1], M.MAV_CMD_NAV_WAYPOINT)  # none after look_at: none

    def test_track_hands_the_gimbal_over_and_takes_it_back(self):
        configure = M.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
        dm = DroneMission("Drone1", [
            {"takeoff": 30}, {"look_at": {"north": 40}},
            {"track": {"target": "Vehicle1"}}, {"goto": {"north": 50}},
            {"track": "off"}, {"track": "Vehicle1"}, {"gimbal": {"pitch": -30}}])
        items = build_items(dm, (1, 103), (1, 103), 0, False, 3)
        commands = [i[1] for i in items]
        # The mission claims the gimbal right after takeoff, so X stands down
        self.assertEqual(commands[:2], [M.MAV_CMD_NAV_TAKEOFF, configure])
        self.assertEqual(items[1][2][:2], (3, 1))
        (on1, spec), (off1, none1), (on2, _), (off2, none2) = dm.track_steps
        self.assertEqual(spec, {"target": "Vehicle1", "tracker": "auto"})
        self.assertIsNone(none1)
        self.assertIsNone(none2)
        self.assertEqual(items[on1][2][:2], (0, 0))  # to nobody: the tracker takes it
        # The look_at ends first, its pointing kept as a fixed compass heading
        self.assertEqual(commands[on1 - 3:on1],
                         [M.MAV_CMD_DO_SET_ROI_NONE, configure,
                          M.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW])
        pitch, heading = items[on1 - 1][2][:2]
        self.assertAlmostEqual(heading, 0.0, places=3)  # due north of the takeoff point
        self.assertAlmostEqual(pitch, -math.degrees(math.atan2(30, 40)), places=1)
        self.assertTrue(items[on1 - 1][3] & M.GIMBAL_MANAGER_FLAGS_YAW_LOCK)
        self.assertEqual(items[off1][2][:2], (3, 1))
        self.assertEqual(items[off2][2][:2], (3, 1))  # the gimbal step's claim ends it
        self.assertEqual(commands[off2 + 1], M.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)
        # No re-aim of the look_at while tracking
        self.assertEqual(commands.count(M.MAV_CMD_DO_SET_ROI_LOCATION), 1)

    def test_track_values(self):
        from gcs.mission import track_spec
        self.assertIsNone(track_spec("off"))
        self.assertEqual(track_spec(True), {"target": None, "tracker": "auto"})
        self.assertEqual(track_spec({"target": "Car", "tracker": "truth"})["tracker"], "truth")
        with self.assertRaises(ValueError):
            track_spec({"target": "Car", "tracker": "radar"})
        with self.assertRaises(ValueError):
            self.load({"drones": {"Drone1": {"steps": [{"track": "Car", "gimbal": {"pitch": 0}}]}}})

    def test_airborne_and_variable_altitudes(self):
        dm = DroneMission("Drone1", [{"takeoff": 30}, {"goto": {"north": 10, "alt": 40}},
                                     {"goto": {"north": 20, "alt": 50}}])
        items = build_items(dm, (1, 103), (1, 103), 20, True, 1)
        self.assertNotIn(M.MAV_CMD_NAV_TAKEOFF, [i[1] for i in items])
        self.assertIsNone(dm.route_alt)  # crossing check must not claim one safe height

    def test_wire_encoding_preserves_flags_without_geographic_scaling(self):
        mav = Mock()
        conn = SimpleNamespace(source_component=190, source_system=245,
                               subscribe=lambda callback: None,
                               send_as=lambda compid, send: send(mav))
        link = VehicleLink(conn, 1, "Drone1")
        dm = DroneMission("Drone1", [{"gimbal": {"pitch": -30, "heading": 90}}])
        link._mission_items = build_items(dm, (1, 103), (1, 103), 30, True, 1)
        link._send_mission_item([i[1] for i in link._mission_items].index(
            M.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW))
        args = mav.mission_item_int_send.call_args.args
        self.assertEqual(args[3], M.MAV_FRAME_MISSION)
        self.assertEqual(args[11], M.GIMBAL_MANAGER_FLAGS_ROLL_LOCK
                         | M.GIMBAL_MANAGER_FLAGS_PITCH_LOCK | M.GIMBAL_MANAGER_FLAGS_YAW_LOCK)

    def test_compare_includes_camera_angles_and_wraps_yaw(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = [Path(directory) / f"{i}.csv" for i in range(2)]
            for i, path in enumerate(paths):
                with path.open("w") as f:
                    writer = csv.writer(f)
                    writer.writerow(["drone", "t", "lat", "lon", "height", "heading",
                                     "gimbal_pitch", "gimbal_yaw"])
                    writer.writerow(["Drone1", 0, 1, 103, 30, 359 if i == 0 else 1,
                                     -30 + i * 3, 179 if i == 0 else -179])
            row = compare_runs(*paths)[0]
            self.assertEqual(row[2:5], (0, 0, 2))
            self.assertEqual(row[5:], (3, 2))

    def test_native_upload_request_ack_protocol(self):
        """Exercise VehicleLink's actual upload with MAVLink request/ack objects."""
        received = []
        link = None

        def deliver(msg):
            msg._header.srcSystem = 1
            msg._header.srcComponent = 1
            link._on_message(msg)

        def request(seq):
            deliver(M.MAVLink_mission_request_int_message(245, 190, seq, M.MAV_MISSION_TYPE_MISSION))

        def count(sysid, compid, total, mission_type):
            self.assertEqual((sysid, compid, mission_type), (1, 1, M.MAV_MISSION_TYPE_MISSION))
            request(0)

        def item(*args):
            received.append(args)
            if len(received) < len(items):
                request(len(received))
            else:
                deliver(M.MAVLink_mission_ack_message(245, 190, M.MAV_MISSION_ACCEPTED,
                                                      M.MAV_MISSION_TYPE_MISSION))

        mav = SimpleNamespace(mission_count_send=count, mission_item_int_send=item)
        conn = SimpleNamespace(source_component=190, source_system=245,
                               subscribe=lambda callback: None,
                               send_as=lambda compid, send: send(mav))
        link = VehicleLink(conn, 1, "Drone1")
        dm = DroneMission("Drone1", [{"takeoff": 30}, {"goto": {"north": 50},
                                                      "heading": 120,
                                                      "gimbal": {"pitch": -40, "yaw": 15}},
                                     {"rtl": True}])
        items = build_items(dm, (1, 103), (1, 103), 0, False, 1)
        link.upload_items(items, timeout=0.1)
        self.assertEqual(len(received), len(items))
        self.assertEqual([a[2] for a in received], list(range(len(items))))
        waypoint = next(a for a in received if a[4] == M.MAV_CMD_NAV_WAYPOINT)
        self.assertEqual(waypoint[10], 120)  # aircraft heading (param4)
        self.assertEqual(link.mission_seq, -1)


class MissionLifecycleTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.link = SimpleNamespace(name="Drone1", sysid=1, position=(1, 103, 50, 30),
                                    armed=True, mode="HOLD", mode_name="HOLD", mission_seq=-1,
                                    mission_reached=-1, mission_state=0, mission_time=0, heading=0,
                                    gimbal_attitude=None, airframe=Airframe(), ap=dialect("px4"))
        self.link.upload_items = Mock()
        self.link.hold = Mock(side_effect=self.hold)
        self.link.land = Mock(return_value=M.MAV_RESULT_ACCEPTED)
        self.link.return_home = Mock(return_value=M.MAV_RESULT_ACCEPTED)
        self.link.start_mission = Mock(side_effect=self.start)
        self.link.clear_mission = Mock()
        self.companion = Mock()
        self.swarm = SwarmControl([self.link])
        self.control = MissionControl([self.link], self.swarm, lambda: (1, 103),
                                      self.directory.name, companions={0: self.companion})
        path = Path(self.directory.name) / "mission.yml"
        path.write_text(yaml.safe_dump({"mission": "test", "drones": {"Drone1": {"steps": [
            {"goto": {"north": 10}, "gimbal": {"pitch": -30}}, {"wait": 2}]}}}))
        self.control.load(path)
        await self.control.upload([0])

    def hold(self):
        self.link.mode = "HOLD"
        return M.MAV_RESULT_ACCEPTED

    def start(self):
        self.link.mode = "MISSION"
        self.link.mission_state = M.MISSION_STATE_ACTIVE
        self.link.mission_time = time.monotonic()
        return M.MAV_RESULT_ACCEPTED

    async def asyncTearDown(self):
        self.control.close()
        self.swarm.stop()
        await asyncio.sleep(0)
        self.directory.cleanup()

    async def test_final_actions_finish_before_camera_is_released(self):
        await self.control.start([0])
        await asyncio.sleep(0.05)
        self.link.mission_seq = len(self.control.mission.drones[0].items) - 1
        self.link.mission_reached = 0  # waypoint reached, final delay still running
        await asyncio.sleep(0.55)
        self.assertIn(0, self.control.flying)
        self.link.hold.assert_not_called()
        self.link.mission_state = M.MISSION_STATE_COMPLETE
        self.link.mission_time = time.monotonic()
        await asyncio.wait_for(self.swarm.wait_idle(), 2)
        self.link.hold.assert_called_once()
        self.companion.set_mission_active.assert_any_call(True)
        self.companion.set_mission_active.assert_called_with(False)
        self.assertFalse(self.control.flying)
        self.assertNotIn(0, self.swarm.plans)

    async def test_cannot_replace_or_clear_a_running_mission(self):
        await self.control.start([0])
        for action in (self.control.upload, self.control.start, self.control.clear):
            with self.assertRaisesRegex(ValueError, "running"):
                await action([0])
        with self.assertRaisesRegex(ValueError, "running"):
            self.control.load("anything.yml")
        self.control.stop()
        await self.swarm.wait_idle()
        await asyncio.sleep(0)
        self.assertFalse(self.control.flying)

    async def test_failed_reupload_cannot_start_old_items(self):
        self.link.upload_items.side_effect = RuntimeError("rejected")
        await self.control.upload([0])
        self.assertNotIn(0, self.control.uploaded)
        with self.assertRaisesRegex(ValueError, "upload it first"):
            await self.control.start([0])

    async def test_variable_height_route_checks_highest_altitude_against_fence(self):
        self.control.mission.drones[0].steps.append({"goto": {"north": 20, "alt": 130}})
        await self.control.upload([0])
        self.assertIsNone(self.swarm.plan_alts[0])
        monitor = SafetyMonitor([self.link], self.swarm, {})
        monitor.fence = [(0.99, 102.99), (0.99, 103.01), (1.01, 103.01), (1.01, 102.99)]
        found = {}
        monitor._check_routes(found)
        self.assertIn("above the fence ceiling", found["route:0"][2])

    async def test_a_mission_fence_goes_up_before_the_mission(self):
        order = []
        self.link.upload_items.side_effect = lambda items: order.append("mission")

        async def on_fence(corners, ceiling):
            order.append(("fence", corners, ceiling))
        self.control.on_fence = on_fence
        self.control.mission.fence = {"corners": [{"north": 0, "east": 0}, {"north": 50},
                                                  {"east": 50}], "ceiling": 120}
        await self.control.upload([0])
        self.assertEqual(order[1], "mission")
        self.assertEqual(order[0][0], "fence")
        self.assertEqual(order[0][1][0], (1, 103))
        self.assertEqual(order[0][2], 120)

    async def test_a_fence_that_fails_stops_the_mission_upload(self):
        async def on_fence(corners, ceiling):
            raise RuntimeError("the mission's geofence didn't reach Drone1")
        self.control.on_fence = on_fence
        self.control.mission.fence = {"corners": [{"north": 0}, {"north": 50}, {"east": 50}],
                                      "ceiling": None}
        self.link.upload_items.reset_mock()
        self.control.uploaded.clear()
        with self.assertRaisesRegex(RuntimeError, "geofence"):
            await self.control.upload([0])
        self.link.upload_items.assert_not_called()
        self.assertNotIn(0, self.control.uploaded)

    async def test_repeated_runs_reset_progress_and_restore_plans(self):
        for _ in range(2):
            await self.control.start([0])
            self.assertEqual(self.link.mission_reached, -1)
            await asyncio.sleep(0.05)
            self.assertIn(0, self.swarm.plans)
            self.link.mission_reached = len(self.control.mission.drones[0].items) - 1
            await asyncio.wait_for(self.swarm.wait_idle(), 2)
            await asyncio.sleep(0)
        self.assertEqual(self.link.start_mission.call_count, 2)
        self.assertEqual(len(self.control.runs()), 2)
        self.control.invalidate()
        self.assertFalse(self.control.uploaded)


class CompanionMissionTests(unittest.TestCase):
    def test_manual_gimbal_input_is_blocked_but_heartbeat_continues(self):
        companion = CompanionLink.__new__(CompanionLink)
        companion.mission_active = False
        companion.tracking = True
        companion.track_point = (10, 20)
        companion.striking = False
        companion.sock = Mock()
        companion.addr = ("127.0.0.1", 5556)
        companion.name = "test"
        companion.set_mission_active(True)
        sent = [call.args[0] for call in companion.sock.sendto.call_args_list]
        self.assertEqual(sent, [b"CANCEL", b"PITCH:STOP", b"YAW:STOP"])
        companion.sock.reset_mock()
        companion.click(1, 2)
        companion.jog("YAW", "RIGHT")
        companion.center_gimbal()
        self.assertFalse(companion.tracking)
        companion.send("HEARTBEAT:123")
        companion.sock.sendto.assert_called_once_with(b"HEARTBEAT:123", companion.addr)
        companion.set_mission_active(False)
        companion.click(1, 2)
        self.assertTrue(companion.tracking)

    def test_mission_releases_strike_and_refuses_arming(self):
        companion = CompanionLink.__new__(CompanionLink)
        companion.mission_active = False
        companion.tracking = False
        companion.track_point = None
        companion.striking = False
        companion.strike_command = "STRIKE"
        companion.sock = Mock()
        companion.addr = ("127.0.0.1", 5556)
        companion.name = "test"
        self.assertTrue(companion.strike(True))
        companion.set_mission_active(True)
        sent = [call.args[0] for call in companion.sock.sendto.call_args_list]
        self.assertEqual(sent[:2], [b"STRIKE:START", b"STRIKE:STOP"])
        self.assertFalse(companion.striking)
        companion.sock.reset_mock()
        self.assertFalse(companion.strike(True))
        companion.sock.sendto.assert_not_called()
        self.assertTrue(companion.strike(False))  # releasing always goes out
        companion.sock.sendto.assert_called_once_with(b"STRIKE:STOP", companion.addr)


if __name__ == "__main__":
    unittest.main()
