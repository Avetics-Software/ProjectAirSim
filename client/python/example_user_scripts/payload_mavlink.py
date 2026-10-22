"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

MAVLink side of the simulated gimbal payload and swarm flight control, on PX4's
onboard payload link (PX4 instance N sends to UDP 14030+N; its system id is N+1):

- gimbal control through PX4's gimbal manager (GIMBAL_MANAGER_SET_ATTITUDE), the
  same way a companion-computer tracker would steer a real gimbal
- a MAVLink camera component (MAV_COMP_ID_CAMERA) so QGroundControl finds the
  RTSP video stream on its own and its zoom controls drive the camera's FOV
- flight commands (arm, takeoff, fly to a point, hold, land, return) and the
  drone's position and flight mode, for controlling a swarm from the console.
  They go to PX4 like a companion computer's would, so QGC stays in sync.

PX4 forwards messages between this link and the ground-station link, so QGC
talks to the camera component through PX4.
"""

import math
import os
import queue
import threading
import time

os.environ["MAVLINK20"] = "1"  # gimbal v2 and camera messages need MAVLink 2
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

from projectairsim.utils import projectairsim_log

PAYLOAD_PORT = 14030  # + PX4 instance
TRACKER_COMPID = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
CAMERA_COMPID = mavutil.mavlink.MAV_COMP_ID_CAMERA

# PX4's gimbal manager runs in the autopilot; it ignores setpoints sent to the
# broadcast component, so always address it explicitly
PX4_COMPID = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
GIMBAL_DEVICE_COMPID = mavutil.mavlink.MAV_COMP_ID_GIMBAL

GIMBAL_FLAGS = (
    mavutil.mavlink.GIMBAL_MANAGER_FLAGS_ROLL_LOCK
    | mavutil.mavlink.GIMBAL_MANAGER_FLAGS_PITCH_LOCK
)

M = mavutil.mavlink

# PX4 custom_mode: main mode in byte 2, sub mode (of AUTO) in byte 3
PX4_MAIN_MODES = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 5: "ACRO", 6: "OFFBOARD",
                  7: "STABILIZED", 8: "RATTITUDE"}
PX4_AUTO_MODES = {1: "READY", 2: "TAKEOFF", 3: "HOLD", 4: "MISSION", 5: "RTL", 6: "LAND",
                  8: "FOLLOW", 9: "PRECLAND"}


def wrap_180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def px4_mode_name(custom_mode):
    main = (custom_mode >> 16) & 0xFF
    sub = (custom_mode >> 24) & 0xFF
    if main == 4:
        return PX4_AUTO_MODES.get(sub, "AUTO")
    return PX4_MAIN_MODES.get(main, "?")


class PayloadLink:
    """One UDP link to PX4 shared by the gimbal controller and camera component.

    Zoom requests from QGC are queued in `zoom_requests` as ("range", percent)
    or ("continuous", -1/0/1) for the main thread to apply to the sim, since the
    sim client must not be used from this link's reader thread.
    """

    def __init__(self, video_uri, camera, instance=0):
        self.instance = instance
        self.sysid = instance + 1  # PX4's, and ours: we are part of that vehicle
        self.port = PAYLOAD_PORT + instance
        self.video_uri = video_uri
        self.camera = camera  # CameraProperties
        self.zoom_percent = 0.0
        self._published_zooms = []  # (time.monotonic(), percent) reported in the last second
        self.zoom_requests = queue.Queue()
        self.capture_requests = queue.Queue()
        self.image_count = 0
        # Telemetry, updated by the reader thread
        self.armed = False
        self.mode = "?"
        self.position = None  # (lat, lon, alt AMSL m, alt above home m)
        self.velocity = (0.0, 0.0, 0.0)  # north, east, down m/s
        self.position_time = 0.0  # time.monotonic() of the last position
        self.position_count = 0  # positions received, to measure the rate
        self.landed = True
        self.status_text = ""
        self.gimbal_attitude = None  # (pitch, yaw relative to the drone) in degrees
        self.gimbal_attitude_time = 0.0
        self.mission_seq = -1  # PX4's current mission item
        self.mission_reached = -1  # the last mission item PX4 reached

        self._mav = mavutil.mavlink_connection(
            f"udpin:0.0.0.0:{self.port}", source_system=self.sysid,
            source_component=TRACKER_COMPID,
        )
        self._send_lock = threading.Lock()
        self._acks = queue.Queue()
        self._mission_items = []  # being uploaded; PX4 requests them one by one
        self._mission_ack = queue.Queue()
        self._boot = time.monotonic()
        self._stop = threading.Event()
        self._px4_seen = threading.Event()
        self._threads = [
            threading.Thread(target=self._read_loop, daemon=True),
            threading.Thread(target=self._heartbeat_loop, daemon=True),
        ]
        for t in self._threads:
            t.start()

    def close(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)
        self._mav.close()

    def wait_for_px4(self, timeout=30):
        """Wait for the next PX4 heartbeat (e.g. from a freshly started PX4)."""
        self._px4_seen.clear()
        if not self._px4_seen.wait(timeout):
            raise RuntimeError(
                f"No heartbeat from PX4 {self.instance} on the payload link (UDP {self.port})"
            )
        projectairsim_log().info(f"Payload link {self.instance + 1} up (UDP {self.port})")

    # ------------------------------------------------------------------
    # Gimbal

    def take_gimbal_control(self):
        """Make this link the gimbal manager's primary controller.

        Whoever configures last is in control, so QGC can take it back at any
        time and this can take it again.
        """
        self._send_as(
            TRACKER_COMPID,
            lambda mav: mav.command_long_send(
                self.sysid,
                PX4_COMPID,
                M.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                0,
                self.sysid,  # primary control sysid
                TRACKER_COMPID,  # primary control compid
                -1,  # secondary control sysid: unchanged
                -1,  # secondary control compid: unchanged
                0,
                0,
                0,  # gimbal device id: all
            ),
        )

    def set_gimbal_attitude(self, pitch_deg, yaw_deg, pitch_rate_dps=math.nan,
                            yaw_rate_dps=math.nan):
        """Stream a gimbal setpoint. Angles are relative to the vehicle heading."""
        q = QuaternionBase([0.0, math.radians(pitch_deg), math.radians(yaw_deg)]).q
        self._send_as(
            TRACKER_COMPID,
            lambda mav: mav.gimbal_manager_set_attitude_send(
                self.sysid,
                PX4_COMPID,
                GIMBAL_FLAGS,
                0,  # gimbal device id: all
                [float(v) for v in q],
                math.nan,
                math.radians(pitch_rate_dps),
                math.radians(yaw_rate_dps),
            ),
        )

    # ------------------------------------------------------------------
    # Flight (swarm control)

    def command(self, command, *params, timeout=3.0):
        """Send a COMMAND_LONG to PX4 and return its MAV_RESULT (None if no ack)."""
        params = list(params) + [0.0] * (7 - len(params))
        while not self._acks.empty():
            self._acks.get_nowait()
        self._send_as(
            TRACKER_COMPID,
            lambda mav: mav.command_long_send(self.sysid, PX4_COMPID, command, 0, *params),
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                acked, result = self._acks.get(timeout=max(0.0, deadline - time.monotonic()))
            except queue.Empty:
                break
            if acked == command:
                return result
        return None

    def arm(self):
        return self.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1)

    def takeoff(self, alt_m):
        """Take off to alt_m above home. Arms first; PX4 refuses to arm until its
        estimator has settled, so the caller may need to retry."""
        if self.position is None:
            return None
        if not self.armed:
            result = self.arm()
            if result != M.MAV_RESULT_ACCEPTED:
                return result
        lat, lon, amsl, rel = self.position
        return self.command(M.MAV_CMD_NAV_TAKEOFF, -1, 0, 0, math.nan, math.nan, math.nan,
                            amsl - rel + alt_m)

    def fly_to(self, lat, lon, alt_m, speed_mps=-1.0, heading_deg=None):
        """Fly to lat/lon at alt_m above home and hover there (PX4 Hold), facing
        heading_deg (degrees from north) if given, else keeping its heading."""
        if self.position is None:
            return None
        _, _, amsl, rel = self.position
        if speed_mps > 0:
            self.command(M.MAV_CMD_DO_CHANGE_SPEED, 1, speed_mps, -1)
        with self._send_lock:
            self._mav.mav.srcComponent = TRACKER_COMPID
            self._mav.mav.command_int_send(
                self.sysid, PX4_COMPID, M.MAV_FRAME_GLOBAL, M.MAV_CMD_DO_REPOSITION, 0, 0,
                # PX4 reads the yaw (param 4) in radians
                -1, M.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 0,
                math.nan if heading_deg is None else math.radians(wrap_180(heading_deg)),
                int(lat * 1e7), int(lon * 1e7), amsl - rel + alt_m,
            )
        return M.MAV_RESULT_ACCEPTED

    def turn_to(self, heading_deg):
        """Turn in place to face heading_deg (degrees from north)."""
        if self.position is None:
            return None
        lat, lon, _, rel = self.position
        return self.fly_to(lat, lon, rel, heading_deg=heading_deg)

    def hold(self):
        # PX4 custom mode: AUTO (4) / LOITER (3)
        return self.command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 3)

    def land(self):
        return self.command(M.MAV_CMD_NAV_LAND, 0, 0, 0, math.nan, math.nan, math.nan, math.nan)

    def return_home(self):
        return self.command(M.MAV_CMD_NAV_RETURN_TO_LAUNCH)

    # ------------------------------------------------------------------
    # Missions

    def upload_mission(self, waypoints, alt_m, takeoff=False, speed_mps=None, timeout=15.0):
        """Upload a PX4 mission: an optional takeoff, an optional speed, then the
        [(lat, lon), ...] waypoints at alt_m above home. Returns the index of
        the first waypoint item; raises if PX4 rejects it."""
        F_REL, F_MISSION = M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, M.MAV_FRAME_MISSION
        items = []  # (frame, command, params 1-4, lat, lon, alt)
        if takeoff and self.position:
            lat, lon = self.position[:2]
            items.append((F_REL, M.MAV_CMD_NAV_TAKEOFF, (0, 0, 0, math.nan), lat, lon, alt_m))
        if speed_mps:
            items.append((F_MISSION, M.MAV_CMD_DO_CHANGE_SPEED, (1, speed_mps, -1, 0), 0, 0, 0))
        first_waypoint = len(items)
        for lat, lon in waypoints:
            # param2: acceptance radius (m)
            items.append((F_REL, M.MAV_CMD_NAV_WAYPOINT, (0, 3.0, 0, math.nan), lat, lon, alt_m))
        self._mission_items = items
        while not self._mission_ack.empty():
            self._mission_ack.get_nowait()
        self._send_as(
            TRACKER_COMPID,
            lambda mav: mav.mission_count_send(self.sysid, PX4_COMPID, len(items), 0),
        )
        try:
            result = self._mission_ack.get(timeout=timeout)
        except queue.Empty:
            raise RuntimeError("PX4 didn't confirm the mission upload")
        if result != M.MAV_MISSION_ACCEPTED:
            raise RuntimeError(f"PX4 rejected the mission (MAV_MISSION_RESULT {result})")
        self.mission_seq = self.mission_reached = -1
        return first_waypoint, len(items)

    def _send_mission_item(self, seq):
        frame, command, (p1, p2, p3, p4), lat, lon, alt = self._mission_items[seq]
        self._send_as(
            TRACKER_COMPID,
            lambda mav: mav.mission_item_int_send(
                self.sysid, PX4_COMPID, seq, frame, command, 0, 1, p1, p2, p3, p4,
                int(lat * 1e7), int(lon * 1e7), alt, 0,
            ),
        )

    def start_mission(self):
        """Fly the uploaded mission from its start (arm first if on the ground)."""
        return self.command(M.MAV_CMD_MISSION_START, 0, 0)

    def resume_mission(self):
        """Continue the mission from where it was interrupted (PX4 Mission mode)."""
        return self.command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 4)

    def distance_to(self, lat, lon):
        """Horizontal distance (m) from the drone to lat/lon, or None."""
        if self.position is None:
            return None
        dn = (lat - self.position[0]) * 111_320.0
        de = (lon - self.position[1]) * 111_320.0 * math.cos(math.radians(lat))
        return math.hypot(dn, de)

    # ------------------------------------------------------------------
    # Camera component

    def publish_zoom(self, zoom_percent):
        """Report the current zoom to QGC (0 = widest, 100 = most zoomed)."""
        self.zoom_percent = zoom_percent
        now = time.monotonic()
        self._published_zooms = [(t, z) for t, z in self._published_zooms if now - t < 1.0]
        self._published_zooms.append((now, zoom_percent))
        self._send_camera_settings()

    def _is_zoom_echo(self, percent):
        """QGC's zoom slider follows CAMERA_SETTINGS and sends the level back as
        a range zoom, which would stop a continuous zoom (from the companion or
        the keys) a few ms after it starts. A level we reported in the last
        second is that echo, not a request."""
        now = time.monotonic()
        return any(now - t < 1.0 and abs(z - percent) < 0.5 for t, z in self._published_zooms)

    def report_image_captured(self, path):
        self.image_count += 1
        self._send_as(
            CAMERA_COMPID,
            lambda mav: mav.camera_image_captured_send(
                self._ms(), int(time.time() * 1e6), 0, 0, 0, 0, 0,
                [1.0, 0.0, 0.0, 0.0], self.image_count - 1, 1,
                path.encode()[:205],
            ),
        )

    def _ms(self):
        return int((time.monotonic() - self._boot) * 1000)

    def _send_as(self, compid, send):
        with self._send_lock:
            self._mav.mav.srcComponent = compid
            send(self._mav.mav)

    def _heartbeat_loop(self):
        while not self._stop.wait(1.0):
            for compid, mav_type in (
                (TRACKER_COMPID, M.MAV_TYPE_ONBOARD_CONTROLLER),
                (CAMERA_COMPID, M.MAV_TYPE_CAMERA),
            ):
                self._send_as(
                    compid,
                    lambda mav, t=mav_type: mav.heartbeat_send(
                        t, M.MAV_AUTOPILOT_INVALID, 0, 0, M.MAV_STATE_ACTIVE
                    ),
                )

    def _read_loop(self):
        while not self._stop.is_set():
            msg = self._mav.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            msg_type = msg.get_type()
            from_px4 = msg.get_srcSystem() == self.sysid and msg.get_srcComponent() == PX4_COMPID
            if msg_type == "HEARTBEAT":
                if from_px4:
                    self._px4_seen.set()
                    self.armed = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.mode = px4_mode_name(msg.custom_mode)
            elif msg_type == "GLOBAL_POSITION_INT" and from_px4:
                self.position = (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0,
                                 msg.relative_alt / 1000.0)
                self.velocity = (msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0)
                self.position_time = time.monotonic()
                self.position_count += 1
            elif msg_type == "EXTENDED_SYS_STATE" and from_px4:
                self.landed = msg.landed_state in (M.MAV_LANDED_STATE_ON_GROUND,
                                                   M.MAV_LANDED_STATE_UNDEFINED)
            elif msg_type == "GIMBAL_DEVICE_ATTITUDE_STATUS" and msg.get_srcSystem() == self.sysid:
                _, pitch, yaw = QuaternionBase(list(msg.q)).euler
                self.gimbal_attitude = (math.degrees(pitch), wrap_180(math.degrees(yaw)))
                self.gimbal_attitude_time = time.monotonic()
            elif msg_type == "STATUSTEXT" and from_px4:
                self.status_text = msg.text
            elif msg_type == "COMMAND_ACK" and from_px4:
                self._acks.put((msg.command, msg.result))
            elif msg_type in ("MISSION_REQUEST_INT", "MISSION_REQUEST") and from_px4:
                if getattr(msg, "mission_type", 0) == 0 and msg.seq < len(self._mission_items):
                    self._send_mission_item(msg.seq)
            elif msg_type == "MISSION_ACK" and from_px4:
                if getattr(msg, "mission_type", 0) == 0:
                    self._mission_ack.put(msg.type)
            elif msg_type == "MISSION_CURRENT" and from_px4:
                self.mission_seq = msg.seq
            elif msg_type == "MISSION_ITEM_REACHED" and from_px4:
                self.mission_reached = msg.seq
            elif msg_type == "COMMAND_LONG" and msg.target_component == CAMERA_COMPID:
                self._handle_camera_command(msg)

    def _ack(self, msg, result):
        self._send_as(
            CAMERA_COMPID,
            lambda mav: mav.command_ack_send(
                msg.command, result, 0, 0, msg.get_srcSystem(), msg.get_srcComponent()
            ),
        )

    def _handle_camera_command(self, msg):
        cmd = msg.command
        projectairsim_log().debug(
            f"Camera {self.instance + 1}: command {cmd} ({msg.param1:g}, {msg.param2:g}) "
            f"from {msg.get_srcSystem()}/{msg.get_srcComponent()}"
        )
        accepted, unsupported = M.MAV_RESULT_ACCEPTED, M.MAV_RESULT_UNSUPPORTED

        if cmd == M.MAV_CMD_REQUEST_MESSAGE:
            requested = int(msg.param1)
            senders = {
                M.MAVLINK_MSG_ID_CAMERA_INFORMATION: self._send_camera_information,
                M.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION: self._send_video_stream_information,
                M.MAVLINK_MSG_ID_CAMERA_SETTINGS: self._send_camera_settings,
                M.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS: self._send_capture_status,
            }
            sender = senders.get(requested)
            self._ack(msg, accepted if sender else unsupported)
            if sender:
                sender()
        elif cmd == M.MAV_CMD_REQUEST_CAMERA_INFORMATION:
            self._ack(msg, accepted)
            self._send_camera_information()
        elif cmd == M.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
            self._ack(msg, accepted)
            self._send_video_stream_information()
        elif cmd == M.MAV_CMD_REQUEST_CAMERA_SETTINGS:
            self._ack(msg, accepted)
            self._send_camera_settings()
        elif cmd == M.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
            self._ack(msg, accepted)
            self._send_capture_status()
        elif cmd == M.MAV_CMD_SET_CAMERA_ZOOM:
            zoom_type, value = int(msg.param1), msg.param2
            if zoom_type == M.ZOOM_TYPE_RANGE and self._is_zoom_echo(value):
                pass  # QGC's slider sending back a level we just reported
            elif zoom_type == M.ZOOM_TYPE_RANGE:
                self.zoom_requests.put(("range", max(0.0, min(100.0, value))))
            elif zoom_type == M.ZOOM_TYPE_CONTINUOUS:
                self.zoom_requests.put(("continuous", int(math.copysign(1, value)) if value else 0))
            elif zoom_type == M.ZOOM_TYPE_STEP:
                self.zoom_requests.put(("range", max(0.0, min(100.0, self.zoom_percent + 10 * value))))
            else:
                self._ack(msg, unsupported)
                return
            self._ack(msg, accepted)
        elif cmd == M.MAV_CMD_IMAGE_START_CAPTURE:
            self.capture_requests.put(True)
            self._ack(msg, accepted)
        elif cmd == M.MAV_CMD_SET_CAMERA_MODE:
            self._ack(msg, accepted)
        else:
            self._ack(msg, unsupported)

    def _send_camera_information(self):
        flags = (
            M.CAMERA_CAP_FLAGS_CAPTURE_IMAGE
            | M.CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM
            | M.CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM
        )
        self._send_as(
            CAMERA_COMPID,
            lambda mav: mav.camera_information_send(
                self._ms(),
                list(b"ProjectAirSim".ljust(32, b"\0")),
                list(self.camera.name.encode()[:32].ljust(32, b"\0")),
                0,
                self.camera.focal_min_mm,
                self.camera.sensor_width_mm,
                self.camera.sensor_height_mm,
                self.camera.resolution[0],
                self.camera.resolution[1],
                0,
                flags,
                0,
                b"",
                GIMBAL_DEVICE_COMPID,
            ),
        )

    def _send_video_stream_information(self):
        self._send_as(
            CAMERA_COMPID,
            lambda mav: mav.video_stream_information_send(
                1,
                1,
                M.VIDEO_STREAM_TYPE_RTSP,
                M.VIDEO_STREAM_STATUS_FLAGS_RUNNING,
                30.0,
                self.camera.resolution[0],
                self.camera.resolution[1],
                4_000_000,
                0,
                int(self.camera.hfov_deg()),
                b"GimbalCam",
                self.video_uri.encode(),
                M.VIDEO_STREAM_ENCODING_H264,
            ),
        )

    def _send_camera_settings(self):
        self._send_as(
            CAMERA_COMPID,
            lambda mav: mav.camera_settings_send(
                self._ms(), M.CAMERA_MODE_VIDEO, self.zoom_percent, math.nan
            ),
        )

    def _send_capture_status(self):
        self._send_as(
            CAMERA_COMPID,
            lambda mav: mav.camera_capture_status_send(
                self._ms(), 0, 0, 0.0, 0, math.nan, self.image_count
            ),
        )
