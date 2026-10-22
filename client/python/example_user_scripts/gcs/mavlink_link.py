"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

MAVLink to the drones' PX4s:

- MavConnection: one pymavlink connection and its reader thread. Every drone
  reachable through it shares it: a telemetry radio or mesh carries the whole
  fleet (each drone with its own system id), the sim gives each PX4 its own
  UDP port. Messages are passed to every subscriber; each picks out its own.
- VehicleLink: one drone on a connection. Telemetry (mode, arming, position,
  gimbal angle), flight commands (arm, takeoff, fly to a point, hold, land,
  return), PX4 missions, the gimbal (through PX4's gimbal manager) and the
  camera's zoom. Commands go to PX4 like a companion computer's or ground
  station's would, so QGC stays in sync and can take over at any time.
"""

import math
import os
import queue
import threading
import time

os.environ["MAVLINK20"] = "1"  # gimbal v2 and camera messages need MAVLink 2
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

from .log import log

M = mavutil.mavlink

# PX4's gimbal manager runs in the autopilot; it ignores setpoints sent to the
# broadcast component, so always address it explicitly
PX4_COMPID = M.MAV_COMP_ID_AUTOPILOT1
CAMERA_COMPID = M.MAV_COMP_ID_CAMERA
GIMBAL_FLAGS = M.GIMBAL_MANAGER_FLAGS_ROLL_LOCK | M.GIMBAL_MANAGER_FLAGS_PITCH_LOCK

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

    def send_as(self, compid, send):
        """send(mav) with the given source component, e.g.
        send_as(c, lambda mav: mav.command_long_send(...))."""
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
            if msg is None or msg.get_type() == "BAD_DATA":
                continue
            for callback in self._subscribers:
                try:
                    callback(msg)
                except Exception as err:
                    log.debug(f"MAVLink {self.url}: handling {msg.get_type()} failed: {err}")


class VehicleLink:
    """One drone (PX4 system id `sysid`) on a MavConnection."""

    def __init__(self, conn, sysid, name):
        self.conn = conn
        self.sysid = sysid
        self.name = name
        self.compid = conn.source_component  # what we send as
        # Telemetry, updated by the connection's reader thread
        self.armed = False
        self.mode = "?"
        self.position = None  # (lat, lon, alt AMSL m, alt above home m)
        self.velocity = (0.0, 0.0, 0.0)  # north, east, down m/s
        self.heading = None  # degrees from north
        self.position_time = 0.0  # time.monotonic() of the last position
        self.position_count = 0  # positions received, to measure the rate
        self.landed = True
        self.status_text = ""
        self.gimbal_attitude = None  # (pitch, yaw relative to the drone) in degrees
        self.gimbal_attitude_time = 0.0
        self.mission_seq = -1  # PX4's current mission item
        self.mission_reached = -1  # the last mission item PX4 reached
        self.zoom_percent = 0.0  # the camera zoom last asked for (real cameras)

        self._acks = queue.Queue()
        self._mission_items = []  # being uploaded; PX4 requests them one by one
        self._mission_ack = queue.Queue()
        self._px4_seen = threading.Event()
        conn.subscribe(self._on_message)

    def wait_for_px4(self, timeout=30):
        """Wait for the next PX4 heartbeat (e.g. from a freshly started PX4)."""
        self._px4_seen.clear()
        if not self._px4_seen.wait(timeout):
            raise RuntimeError(
                f"No heartbeat from {self.name}'s PX4 (system {self.sysid}) on {self.conn.url}"
            )
        log.info(f"{self.name} MAVLink link up ({self.conn.url})")

    @property
    def connected(self):
        return self.position_time > 0 and time.monotonic() - self.position_time < 3.0

    def _send(self, send):
        self.conn.send_as(self.compid, send)

    # ------------------------------------------------------------------
    # Gimbal and camera

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

    def set_gimbal_attitude(self, pitch_deg, yaw_deg, pitch_rate_dps=math.nan,
                            yaw_rate_dps=math.nan):
        """Stream a gimbal setpoint. Angles are relative to the vehicle heading."""
        q = QuaternionBase([0.0, math.radians(pitch_deg), math.radians(yaw_deg)]).q
        self._send(
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
        """Send a COMMAND_LONG to PX4 and return its MAV_RESULT (None if no ack)."""
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
        self._send(
            lambda mav: mav.command_int_send(
                self.sysid, PX4_COMPID, M.MAV_FRAME_GLOBAL, M.MAV_CMD_DO_REPOSITION, 0, 0,
                # PX4 reads the yaw (param 4) in radians
                -1, M.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 0,
                math.nan if heading_deg is None else math.radians(wrap_180(heading_deg)),
                int(lat * 1e7), int(lon * 1e7), amsl - rel + alt_m,
            ),
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
        self._send(lambda mav: mav.mission_count_send(self.sysid, PX4_COMPID, len(items), 0))
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
        self._send(
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
        if msg.get_srcComponent() != PX4_COMPID:
            return
        if msg_type == "HEARTBEAT":
            self._px4_seen.set()
            self.armed = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
            self.mode = px4_mode_name(msg.custom_mode)
        elif msg_type == "GLOBAL_POSITION_INT":
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
            self.status_text = msg.text
        elif msg_type == "COMMAND_ACK" and self._for_us(msg):
            self._acks.put((msg.command, msg.result))
        elif msg_type in ("MISSION_REQUEST_INT", "MISSION_REQUEST") and self._for_us(msg):
            if getattr(msg, "mission_type", 0) == 0 and msg.seq < len(self._mission_items):
                self._send_mission_item(msg.seq)
        elif msg_type == "MISSION_ACK" and self._for_us(msg):
            if getattr(msg, "mission_type", 0) == 0:
                self._mission_ack.put(msg.type)
        elif msg_type == "MISSION_CURRENT":
            self.mission_seq = msg.seq
        elif msg_type == "MISSION_ITEM_REACHED":
            self.mission_reached = msg.seq
