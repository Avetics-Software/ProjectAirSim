"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

MAVLink to the drones' autopilots (PX4 or ArduPilot):

- MavConnection: one pymavlink connection and its reader thread. Every drone
  reachable through it shares it: a telemetry radio or mesh carries the whole
  fleet (each drone with its own system id), the sim gives each autopilot its
  own UDP port. Messages are passed to every subscriber; each picks out its own.
- VehicleLink: one drone on a connection. Telemetry (mode, arming, position,
  gimbal angle), flight commands (arm, takeoff, fly to a point, hold, land,
  return), missions, the gimbal (through the autopilot's gimbal manager) and
  the camera's zoom. Commands go to the autopilot like a companion computer's
  or ground station's would, so QGC stays in sync and can take over at any time.
  Where PX4 and ArduPilot differ (modes, parameters, mission numbering) it asks
  the drone's dialect (gcs/autopilot.py); where a multirotor and a fixed-wing
  differ, its airframe (gcs/airframe.py).
"""

import math
import os
import queue
import threading
import time

os.environ["MAVLINK20"] = "1"  # gimbal v2 and camera messages need MAVLink 2
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

from .airframe import Airframe
from .autopilot import dialect
from .log import log

M = mavutil.mavlink

# The gimbal manager runs in the autopilot; PX4's ignores setpoints sent to the
# broadcast component, so always address it explicitly
PX4_COMPID = M.MAV_COMP_ID_AUTOPILOT1  # PX4's and ArduPilot's
CAMERA_COMPID = M.MAV_COMP_ID_CAMERA
GIMBAL_FLAGS = M.GIMBAL_MANAGER_FLAGS_ROLL_LOCK | M.GIMBAL_MANAGER_FLAGS_PITCH_LOCK

# Telemetry the safety monitor needs, requested at 1 Hz in case the link's
# stream set lacks them: SYS_STATUS, GPS_RAW_INT, BATTERY_STATUS
HEALTH_MESSAGES = (1, 24, 147)
GIMBAL_ATTITUDE_MSG = 285  # GIMBAL_DEVICE_ATTITUDE_STATUS
GIMBAL_MANAGER_STATUS_MSG = 281  # who controls the gimbal
MISSION, FENCE = M.MAV_MISSION_TYPE_MISSION, M.MAV_MISSION_TYPE_FENCE
GPS_FIX_NAMES = {0: "none", 1: "none", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK", 6: "RTK"}
FIXEDWING_TAKEOFF_PITCH = 15.0  # deg, a fixed-wing mission takeoff's minimum climb pitch


def wrap_180(deg):
    return (deg + 180.0) % 360.0 - 180.0


class MavConnection:
    """A pymavlink connection with a reader thread and heartbeats.

    source_system/source_component are who we are on the link: the ground
    station's own ids on a real fleet link, or (in the sim) the drone's system
    id with the onboard-computer component, like a companion computer.
    """

    def __init__(self, url, source_system, source_component,
                 heartbeat_type=M.MAV_TYPE_GCS):
        self.url = url
        self.source_system = source_system
        self.source_component = source_component
        self._mav = mavutil.mavlink_connection(
            url, source_system=source_system, source_component=source_component,
        )
        self._send_lock = threading.Lock()
        self._subscribers = []
        self._heartbeats = [(source_component, heartbeat_type)]  # (compid, MAV_TYPE)
        # Simulated radio outage (the sim's fault controls): while set, nothing
        # is received and nothing sent, heartbeats included
        self.outage = False
        self._boot = time.monotonic()
        self._stop = threading.Event()
        self._threads = [
            threading.Thread(target=self._read_loop, name=f"mavlink {url}", daemon=True),
            threading.Thread(target=self._heartbeat_loop, daemon=True),
        ]
        for t in self._threads:
            t.start()

    def subscribe(self, callback):
        """Call callback(msg) for every message received (reader thread)."""
        self._subscribers.append(callback)

    def add_heartbeat(self, compid, mav_type):
        """Also send a heartbeat as this component (e.g. a MAVLink camera)."""
        self._heartbeats.append((compid, mav_type))

    def send_as(self, compid, send, bypass_outage=False):
        """send(mav) with the given source component, e.g.
        send_as(c, lambda mav: mav.command_long_send(...)). Dropped during a
        simulated outage unless bypass_outage (the sim's own fault injection)."""
        if self.outage and not bypass_outage:
            return
        with self._send_lock:
            self._mav.mav.srcComponent = compid
            send(self._mav.mav)

    def ms(self):
        """Milliseconds since this connection started, for time_boot_ms."""
        return int((time.monotonic() - self._boot) * 1000)

    def close(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)
        self._mav.close()

    def _heartbeat_loop(self):
        while not self._stop.wait(1.0):
            for compid, mav_type in self._heartbeats:
                self.send_as(
                    compid,
                    lambda mav, t=mav_type: mav.heartbeat_send(
                        t, M.MAV_AUTOPILOT_INVALID, 0, 0, M.MAV_STATE_ACTIVE
                    ),
                )

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                msg = self._mav.recv_match(blocking=True, timeout=0.5)
            except OSError as err:  # e.g. a serial radio unplugged
                log.warning(f"MAVLink {self.url}: {err}")
                self._stop.wait(1.0)
                continue
            if msg is None or msg.get_type() == "BAD_DATA" or self.outage:
                continue
            for callback in self._subscribers:
                try:
                    callback(msg)
                except Exception as err:
                    log.debug(f"MAVLink {self.url}: handling {msg.get_type()} failed: {err}")


class VehicleLink:
    """One drone (its autopilot's system id `sysid`) on a MavConnection.
    airframe: gcs.airframe.Airframe, what it is (default a PX4 multirotor)."""

    def __init__(self, conn, sysid, name, airframe=None):
        self.conn = conn
        self.sysid = sysid
        self.name = name
        self.airframe = airframe or Airframe()
        self.ap = dialect(self.airframe.autopilot)
        self.compid = conn.source_component  # what we send as
        # Telemetry, updated by the connection's reader thread
        self.armed = False
        self.mode = "?"  # the ground station's name for it (gcs/autopilot.py)
        self.mode_name = "?"  # the autopilot's own, for display
        self.position = None  # (lat, lon, alt AMSL m, alt above home m)
        self.velocity = (0.0, 0.0, 0.0)  # north, east, down m/s
        self.heading = None  # degrees from north
        self.position_time = 0.0  # time.monotonic() of the last position
        self.position_count = 0  # positions received, to measure the rate
        self._streams_requested = 0.0  # time.monotonic() of the last request for them
        self.landed = True
        self.status_text = ""
        self.status_severity = M.MAV_SEVERITY_INFO
        self.status_time = 0.0  # time.monotonic() of the last STATUSTEXT
        self.system_status = M.MAV_STATE_UNINIT  # the autopilot's HEARTBEAT state (CRITICAL, EMERGENCY...)
        self.last_message_time = 0.0  # time.monotonic() of any message from the autopilot
        # Health (None until reported)
        self.battery_pct = None
        self.battery_v = None
        self.battery_time_s = None  # flight time left, if PX4 estimates it
        self.gps_fix = None  # GPS_FIX_TYPE
        self.gps_sats = None
        self.gps_eph_m = None  # horizontal accuracy
        self.gimbal_attitude = None  # (pitch, yaw relative to the drone) in degrees
        self.gimbal_attitude_time = 0.0
        self.gimbal_owner = None  # (sysid, compid) in primary control of the gimbal
        # Mission items as uploaded (0 = the first), whatever the autopilot numbers them
        self.mission_seq = -1  # the autopilot's current mission item
        self.mission_reached = -1  # the last mission item it reached
        self.mission_state = 0  # MAVLink MISSION_STATE (0 on older firmware)
        self.mission_time = 0.0
        self.zoom_percent = 0.0  # the camera zoom last asked for (real cameras)

        self._acks = queue.Queue()
        self._mission_items = []  # being uploaded; PX4 requests them one by one
        self._mission_type = MISSION  # of the upload in progress
        self._mission_ack = queue.Queue()
        self._params = {}  # name -> value, from PARAM_VALUE
        self._status_id = 0  # STATUSTEXT chunk id being joined
        self._status_raw = ""  # its text so far, unstripped
        self._param_event = threading.Condition()
        self._px4_seen = threading.Event()
        conn.subscribe(self._on_message)

    def wait_for_autopilot(self, timeout=30):
        """Wait for the next autopilot heartbeat (e.g. from a freshly started one)."""
        self._px4_seen.clear()
        if not self._px4_seen.wait(timeout):
            raise RuntimeError(
                f"No heartbeat from {self.name}'s {self.ap.label} (system {self.sysid}) "
                f"on {self.conn.url}"
            )
        log.info(f"{self.name} MAVLink link up ({self.ap.label}, {self.conn.url})")

    wait_for_px4 = wait_for_autopilot

    @property
    def connected(self):
        return self.position_time > 0 and time.monotonic() - self.position_time < 3.0

    @property
    def link_age(self):
        """Seconds since anything arrived from the autopilot (None before the first)."""
        return time.monotonic() - self.last_message_time if self.last_message_time else None

    def _request_health_streams(self):
        self._streams_requested = time.monotonic()
        # Health at 1 Hz; where the gimbal points at 5 Hz (the dashboard and
        # mission run logs show it)
        rates = [(m, 1_000_000) for m in HEALTH_MESSAGES] + [
            (GIMBAL_ATTITUDE_MSG, 200_000), (GIMBAL_MANAGER_STATUS_MSG, 500_000),
            *self.ap.stream_messages]
        for msg_id, interval_us in rates:
            self._send(lambda mav, m=msg_id, us=interval_us: mav.command_long_send(
                self.sysid, PX4_COMPID, M.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                m, us, 0, 0, 0, 0, 0))

    # ------------------------------------------------------------------
    # Parameters

    def set_param(self, name, value, timeout=3.0, bypass_outage=False):
        """Set an autopilot parameter (an int value sets an integer parameter)
        and wait for it to echo it. Returns the value it reports, or None."""
        ptype, raw = self.ap.encode_param(value)
        with self._param_event:
            self._params.pop(name, None)
        self.conn.send_as(
            self.compid,
            lambda mav: mav.param_set_send(self.sysid, PX4_COMPID, name.encode(), raw, ptype),
            bypass_outage=bypass_outage,
        )
        return self._wait_param(name, timeout)

    def get_param(self, name, timeout=3.0):
        """Read an autopilot parameter, or None if it doesn't answer."""
        with self._param_event:
            self._params.pop(name, None)
        self._send(lambda mav: mav.param_request_read_send(self.sysid, PX4_COMPID,
                                                             name.encode(), -1))
        return self._wait_param(name, timeout)

    def _wait_param(self, name, timeout):
        with self._param_event:
            self._param_event.wait_for(lambda: name in self._params, timeout)
            return self._params.get(name)

    def _send(self, send):
        self.conn.send_as(self.compid, send)

    # ------------------------------------------------------------------
    # Gimbal and camera

    @property
    def autopilot_has_gimbal(self):
        """True if the autopilot itself (a mission) controls the gimbal; None if unknown."""
        if self.gimbal_owner is None:
            return None
        return self.gimbal_owner == (self.sysid, PX4_COMPID)

    def take_gimbal_control(self):
        """Make this link the gimbal manager's primary controller.

        Whoever configures last is in control, so QGC can take it back at any
        time and this can take it again.
        """
        self._send(
            lambda mav: mav.command_long_send(
                self.sysid,
                PX4_COMPID,
                M.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                0,
                self.conn.source_system,  # primary control sysid
                self.compid,  # primary control compid
                -1,  # secondary control sysid: unchanged
                -1,  # secondary control compid: unchanged
                0,
                0,
                0,  # gimbal device id: all
            ),
        )

    def give_gimbal(self, to_autopilot):
        """Hand the gimbal to the autopilot itself (as a PX4 mission's claim
        item does) or to nobody (a mission's `track` step), as a command: an
        ArduPilot mission can't carry DO_GIMBAL_MANAGER_CONFIGURE items. Who
        holds it is what a companion (X) stands down on."""
        sysid, compid = (self.sysid, PX4_COMPID) if to_autopilot else (0, 0)
        return self.command(M.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE, sysid, compid, -1, -1, 0, 0, 0)

    def set_gimbal_attitude(self, pitch_deg, yaw_deg, pitch_rate_dps=math.nan,
                            yaw_rate_dps=math.nan, yaw_lock=False):
        """Stream a gimbal setpoint. The yaw is from the drone's nose, or with
        yaw_lock a compass heading the gimbal holds while the drone turns."""
        q = QuaternionBase([0.0, math.radians(pitch_deg), math.radians(yaw_deg)]).q
        flags = GIMBAL_FLAGS | (M.GIMBAL_MANAGER_FLAGS_YAW_LOCK if yaw_lock else 0)
        self._send(
            lambda mav: mav.gimbal_manager_set_attitude_send(
                self.sysid,
                PX4_COMPID,
                flags,
                0,  # gimbal device id: all
                [float(v) for v in q],
                math.nan,
                math.radians(pitch_rate_dps),
                math.radians(yaw_rate_dps),
            ),
        )

    def set_zoom(self, percent):
        """Zoom the drone's MAVLink camera (0 = widest, 100 = longest)."""
        self.zoom_percent = max(0.0, min(100.0, percent))
        self._send(
            lambda mav: mav.command_long_send(
                self.sysid, CAMERA_COMPID, M.MAV_CMD_SET_CAMERA_ZOOM, 0,
                M.ZOOM_TYPE_RANGE, self.zoom_percent, 0, 0, 0, 0, 0,
            ),
        )

    # ------------------------------------------------------------------
    # Flight (swarm control)

    def command(self, command, *params, timeout=3.0):
        """Send a COMMAND_LONG to the autopilot and return its MAV_RESULT (None if no ack)."""
        params = list(params) + [0.0] * (7 - len(params))
        while not self._acks.empty():
            self._acks.get_nowait()
        self._send(lambda mav: mav.command_long_send(self.sysid, PX4_COMPID, command, 0, *params))
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

    def disarm(self, force=False):
        """Stop the motors. The autopilot refuses in flight unless forced (the
        magic number in param 2), which drops the aircraft."""
        return self.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196 if force else 0)

    def set_mode(self, mode):
        """Switch to a mode by the ground station's name (HOLD, MISSION, ...;
        the dialect's SET_MODES)."""
        return self.command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            *self.ap.set_mode_params(mode))

    def takeoff(self, alt_m):
        """Take off to alt_m above home. Arms first; the autopilot refuses to arm
        until its estimator has settled, so the caller may need to retry.

        A fixed-wing takes off in its TAKEOFF mode, along the way it faces
        (launched when armed), and circles once at alt_m."""
        if self.position is None:
            return None
        if not self.airframe.hovers:
            self.set_param("TKOFF_ALT", float(alt_m))
            result = self.set_mode("TAKEOFF")
            if result != M.MAV_RESULT_ACCEPTED or self.armed:
                return result
            return self.arm()
        if self.ap.takeoff_mode:
            # ArduCopter takes off only in GUIDED
            result = self.set_mode(self.ap.takeoff_mode)
            if result != M.MAV_RESULT_ACCEPTED:
                return result
        if not self.armed:
            result = self.arm()
            if result != M.MAV_RESULT_ACCEPTED:
                return result
        lat, lon, amsl, rel = self.position
        takeoff_alt = alt_m if self.ap.takeoff_alt_relative else amsl - rel + alt_m
        return self.command(M.MAV_CMD_NAV_TAKEOFF, -1, 0, 0, math.nan, math.nan, math.nan,
                            takeoff_alt)

    def fly_to(self, lat, lon, alt_m, speed_mps=-1.0, heading_deg=None):
        """Fly to lat/lon at alt_m above home and hover there (PX4 Hold), facing
        heading_deg (degrees from north). Without it PX4 turns the drone to face
        the way it flies. A fixed-wing (GUIDED) circles the point instead, and
        has no heading of its own."""
        if self.position is None:
            return None
        _, _, amsl, rel = self.position
        if speed_mps > 0:
            self.command(M.MAV_CMD_DO_CHANGE_SPEED, 1 if self.airframe.hovers else 0, speed_mps, -1)
        face = heading_deg is not None and self.airframe.hovers
        if not face or not self.ap.reposition_sets_heading:
            yaw = math.nan  # ArduPlane reads a non-zero param 4 as "circle anticlockwise"
        else:
            yaw = math.radians(wrap_180(heading_deg))  # PX4 reads it in radians
        self._send(
            lambda mav: mav.command_int_send(
                self.sysid, PX4_COMPID, M.MAV_FRAME_GLOBAL, M.MAV_CMD_DO_REPOSITION, 0, 0,
                -1, M.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 0, yaw,
                int(lat * 1e7), int(lon * 1e7), amsl - rel + alt_m,
            ),
        )
        if face and not self.ap.reposition_sets_heading:
            # ArduCopter ignores the repositioning's heading: turn it with its own command
            self.face(heading_deg)
        return M.MAV_RESULT_ACCEPTED

    def face(self, heading_deg):
        """Turn to face heading_deg (degrees from north), at the autopilot's own
        yaw rate: MAV_CMD_CONDITION_YAW, absolute."""
        return self.command(M.MAV_CMD_CONDITION_YAW, heading_deg % 360.0, 0, 0, 0)

    def turn_to(self, heading_deg):
        """Turn in place to face heading_deg (degrees from north). None for an
        aircraft that can't (a fixed-wing)."""
        if self.position is None or not self.airframe.hovers:
            return None
        lat, lon, _, rel = self.position
        return self.fly_to(lat, lon, rel, heading_deg=heading_deg)

    def hold(self):
        """Hold where it is: hover (PX4 Hold, or ArduCopter flying to where it
        is), or circle (ArduPlane LOITER)."""
        if self.ap.hold_by_repositioning and self.airframe.hovers and self.position:
            lat, lon, _, rel = self.position
            return self.fly_to(lat, lon, rel)
        return self.set_mode("HOLD")

    def land(self):
        """Land where it is; a fixed-wing flies its mission's landing sequence
        (DO_LAND_START), and is refused without one."""
        if not self.airframe.hovers:
            return self.command(M.MAV_CMD_DO_LAND_START)
        return self.command(M.MAV_CMD_NAV_LAND, 0, 0, 0, math.nan, math.nan, math.nan, math.nan)

    def return_home(self):
        return self.command(M.MAV_CMD_NAV_RETURN_TO_LAUNCH)

    # ------------------------------------------------------------------
    # Missions

    def takeoff_item(self, lat, lon, alt_m):
        """A mission's takeoff item: straight up, or (fixed-wing) climbing out
        along the way it faces at FIXEDWING_TAKEOFF_PITCH or more."""
        params = (0, 0, 0, math.nan) if self.airframe.hovers else (FIXEDWING_TAKEOFF_PITCH, 0, 0, 0)
        return (M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, M.MAV_CMD_NAV_TAKEOFF, params, lat, lon, alt_m)

    @property
    def acceptance_radius_m(self):
        """A waypoint's acceptance radius: 3 m, or (0) the fixed-wing's own (WP_RADIUS)."""
        return 3.0 if self.airframe.hovers else 0.0

    def upload_mission(self, waypoints, alt_m, takeoff=False, speed_mps=None, timeout=15.0):
        """Upload a mission: an optional takeoff, an optional speed, then the
        [(lat, lon), ...] waypoints at alt_m above home. Returns the index of
        the first waypoint item; raises if the autopilot rejects it."""
        F_REL, F_MISSION = M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, M.MAV_FRAME_MISSION
        items = []  # (frame, command, params 1-4, lat, lon, alt)
        if takeoff and self.position:
            items.append(self.takeoff_item(*self.position[:2], alt_m))
        if speed_mps:
            items.append((F_MISSION, M.MAV_CMD_DO_CHANGE_SPEED,
                          (1 if self.airframe.hovers else 0, speed_mps, -1, 0), 0, 0, 0))
        first_waypoint = len(items)
        for lat, lon in waypoints:
            # param2: acceptance radius (m)
            items.append((F_REL, M.MAV_CMD_NAV_WAYPOINT, (0, self.acceptance_radius_m, 0, math.nan),
                          lat, lon, alt_m))
        self._upload_items(items, MISSION, timeout, "mission")
        self.mission_seq = self.mission_reached = -1
        return first_waypoint, len(items)

    def upload_items(self, items, timeout=15.0):
        """Upload a mission of ready-made items (frame, command, (p1..p4),
        lat, lon, alt); raises if the autopilot rejects it."""
        self._upload_items(items, MISSION, timeout, "mission")
        self.mission_seq = self.mission_reached = -1
        self.mission_state = 0

    def clear_mission(self, timeout=15.0):
        """Remove the autopilot's stored mission (an empty upload)."""
        self._upload_items([], MISSION, timeout, "mission")
        self.mission_seq = self.mission_reached = -1

    def upload_fence(self, polygon, timeout=15.0):
        """Replace the autopilot's geofence with an inclusion polygon
        [(lat, lon), ...]; an empty polygon clears it. Raises if it's rejected."""
        n = len(polygon)
        items = [(M.MAV_FRAME_GLOBAL, M.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                  (n, 0, 0, 0), lat, lon, 0.0) for lat, lon in polygon]
        self._upload_items(items, FENCE, timeout, "geofence")

    def _upload_items(self, items, mission_type, timeout, what):
        """The MAVLink mission protocol: send the count, answer the autopilot's
        requests for each item, wait for its ack."""
        if mission_type == MISSION and items and self.ap.mission_offset:
            # ArduPilot's item 0 is home, which it sets itself: a placeholder
            lat, lon, amsl, _ = self.position or (0.0, 0.0, 0.0, 0.0)
            items = [(M.MAV_FRAME_GLOBAL, M.MAV_CMD_NAV_WAYPOINT, (0, 0, 0, 0), lat, lon, amsl)] + items
        self._mission_items, self._mission_type = items, mission_type
        while not self._mission_ack.empty():
            self._mission_ack.get_nowait()
        self._send(lambda mav: mav.mission_count_send(self.sysid, PX4_COMPID, len(items),
                                                      mission_type))
        try:
            result = self._mission_ack.get(timeout=timeout)
        except queue.Empty:
            raise RuntimeError(f"{self.ap.label} didn't confirm the {what} upload")
        if result != M.MAV_MISSION_ACCEPTED:
            raise RuntimeError(f"{self.ap.label} rejected the {what} (MAV_MISSION_RESULT {result})")

    def _send_mission_item(self, seq):
        frame, command, (p1, p2, p3, p4), lat, lon, alt = self._mission_items[seq]
        # x/y are degrees x 1e7 for a position; an item without one (frame
        # MISSION) carries params 5 and 6 there as they are
        scale = 1 if frame == M.MAV_FRAME_MISSION else 1e7
        self._send(
            lambda mav: mav.mission_item_int_send(
                self.sysid, PX4_COMPID, seq, frame, command, 0, 1, p1, p2, p3, p4,
                int(lat * scale), int(lon * scale), alt, self._mission_type,
            ),
        )

    def start_mission(self):
        """Fly the uploaded mission from its start (arm first if on the ground)."""
        return self.command(M.MAV_CMD_MISSION_START, 0, 0)

    def resume_mission(self):
        """Continue the mission from where it was interrupted (MISSION / AUTO mode)."""
        return self.set_mode("MISSION")

    def distance_to(self, lat, lon):
        """Horizontal distance (m) from the drone to lat/lon, or None."""
        if self.position is None:
            return None
        dn = (lat - self.position[0]) * 111_320.0
        de = (lon - self.position[1]) * 111_320.0 * math.cos(math.radians(lat))
        return math.hypot(dn, de)

    # ------------------------------------------------------------------
    # Receiving

    def _for_us(self, msg):
        """A reply to us, not to QGC or another station sharing the link."""
        target = getattr(msg, "target_system", 0)
        return target in (0, self.conn.source_system)

    def _on_message(self, msg):
        if msg.get_srcSystem() != self.sysid:
            return  # another drone on a shared link
        msg_type = msg.get_type()
        if msg_type == "GIMBAL_DEVICE_ATTITUDE_STATUS":
            # From PX4 or the gimbal device itself
            _, pitch, yaw = QuaternionBase(list(msg.q)).euler
            self.gimbal_attitude = (math.degrees(pitch), wrap_180(math.degrees(yaw)))
            self.gimbal_attitude_time = time.monotonic()
            return
        if msg_type == "GIMBAL_MANAGER_STATUS":
            self.gimbal_owner = (msg.primary_control_sysid, msg.primary_control_compid)
            return
        if msg.get_srcComponent() != PX4_COMPID:
            return
        now = time.monotonic()
        if msg_type == "HEARTBEAT":
            if not self.last_message_time or now - self.last_message_time > 5.0:
                # A new or restarted PX4, or a link back after an outage
                self._request_health_streams()
            elif (self.ap.stream_messages and now - self._streams_requested > 5.0
                  and now - self.position_time > 3.0):
                # An autopilot that streams only on request can drop the request
                # while it starts up: ask again until the position arrives
                self._request_health_streams()
            self._px4_seen.set()
            self.armed = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
            self.mode, self.mode_name = self.ap.mode(msg.custom_mode)
            self.system_status = msg.system_status
        self.last_message_time = now
        if msg_type == "GLOBAL_POSITION_INT":
            self.position = (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0,
                             msg.relative_alt / 1000.0)
            self.velocity = (msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0)
            if msg.hdg != 65535:  # UINT16_MAX: unknown
                self.heading = msg.hdg / 100.0
            self.position_time = time.monotonic()
            self.position_count += 1
        elif msg_type == "EXTENDED_SYS_STATE":
            self.landed = msg.landed_state in (M.MAV_LANDED_STATE_ON_GROUND,
                                               M.MAV_LANDED_STATE_UNDEFINED)
        elif msg_type == "STATUSTEXT":
            text = msg.text
            # Long texts arrive in chunks sharing an id, split anywhere (even
            # mid-word); join them as they are
            chunk_id, chunk = getattr(msg, "id", 0), getattr(msg, "chunk_seq", 0)
            if chunk_id and chunk > 0 and chunk_id == self._status_id:
                text = self._status_raw + text
            self._status_raw = text
            text = text.strip()
            self._status_id = chunk_id
            self.status_text, self.status_severity, self.status_time = text, msg.severity, now
        elif msg_type == "SYS_STATUS":
            if msg.battery_remaining >= 0:
                self.battery_pct = msg.battery_remaining
            if msg.voltage_battery != 65535:
                self.battery_v = msg.voltage_battery / 1000.0
        elif msg_type == "BATTERY_STATUS":
            if msg.battery_remaining >= 0:
                self.battery_pct = msg.battery_remaining
            time_left = getattr(msg, "time_remaining", 0)
            self.battery_time_s = time_left if time_left > 0 else None
        elif msg_type == "GPS_RAW_INT":
            self.gps_fix, self.gps_sats = msg.fix_type, msg.satellites_visible
            self.gps_eph_m = msg.eph / 100.0 if msg.eph != 65535 else None
        elif msg_type == "PARAM_VALUE":
            value = self.ap.decode_param(msg.param_value, msg.param_type)
            name = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
            with self._param_event:
                self._params[name.rstrip("\0")] = value
                self._param_event.notify_all()
        elif msg_type == "COMMAND_ACK" and self._for_us(msg):
            self._acks.put((msg.command, msg.result))
        elif msg_type in ("MISSION_REQUEST_INT", "MISSION_REQUEST") and self._for_us(msg):
            if (getattr(msg, "mission_type", 0) == self._mission_type
                    and msg.seq < len(self._mission_items)):
                self._send_mission_item(msg.seq)
        elif msg_type == "MISSION_ACK" and self._for_us(msg):
            if getattr(msg, "mission_type", 0) == self._mission_type:
                self._mission_ack.put(msg.type)
        elif msg_type == "MISSION_CURRENT":
            self.mission_seq = msg.seq - self.ap.mission_offset
            self.mission_state = getattr(msg, "mission_state", 0)
            self.mission_time = time.monotonic()
        elif msg_type == "MISSION_ITEM_REACHED":
            self.mission_reached = msg.seq - self.ap.mission_offset
