"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Where PX4 and ArduPilot differ on MAVLink, for VehicleLink. Everything else
(telemetry, the mission protocol, the gimbal manager, the camera) is common
MAVLink and stays in mavlink_link.py.

Modes: the ground station speaks one vocabulary, PX4's names, whichever
autopilot flies the drone, so the rest of it compares modes without knowing
which: MISSION, HOLD, RTL, LAND, TAKEOFF, GUIDED, and the manual ones
(MANUAL, STABILIZED, ALTCTL, POSCTL, ACRO). VehicleLink.mode is that name;
VehicleLink.mode_name is the autopilot's own, for display.
"""

import os
import struct

os.environ["MAVLINK20"] = "1"  # as mavlink_link.py: set before pymavlink loads
from pymavlink import mavutil  # noqa: E402

M = mavutil.mavlink

MANUAL_MODES = ("MANUAL", "STABILIZED", "ALTCTL", "POSCTL", "ACRO")

INT_PARAM_TYPES = (M.MAV_PARAM_TYPE_UINT8, M.MAV_PARAM_TYPE_INT8, M.MAV_PARAM_TYPE_UINT16,
                   M.MAV_PARAM_TYPE_INT16, M.MAV_PARAM_TYPE_UINT32, M.MAV_PARAM_TYPE_INT32)


class Px4:
    name = "px4"
    label = "PX4"

    # custom_mode: main mode in byte 2, sub mode (of AUTO) in byte 3
    MAIN_MODES = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 5: "ACRO", 6: "OFFBOARD",
                  7: "STABILIZED", 8: "RATTITUDE"}
    AUTO_MODES = {1: "READY", 2: "TAKEOFF", 3: "HOLD", 4: "MISSION", 5: "RTL", 6: "LAND",
                  8: "FOLLOW", 9: "PRECLAND"}
    # The modes the ground station sets: (main, sub)
    SET_MODES = {"HOLD": (4, 3), "MISSION": (4, 4)}

    # Mission items: PX4 numbers them from 0, as uploaded
    mission_offset = 0
    # A mission claims the gimbal (DO_GIMBAL_MANAGER_CONFIGURE items): PX4 obeys
    # its gimbal items only while the autopilot holds the gimbal
    mission_gimbal_claim = True

    # Telemetry the autopilot sends only when asked: (MAVLink message id, interval
    # in microseconds). PX4's and ArduPlane's default streams carry what the
    # ground station reads; ArduCopter streams nothing until asked
    stream_messages = ()

    # A multirotor's takeoff: the mode to be in first (None: any), and whether
    # MAV_CMD_NAV_TAKEOFF's altitude is above home (PX4: above sea level)
    takeoff_mode = None
    takeoff_alt_relative = False
    # DO_REPOSITION carries the heading to face (PX4), or the autopilot has to
    # be told with MAV_CMD_CONDITION_YAW
    reposition_sets_heading = True
    # A mission waypoint's param4 is the heading to face there (PX4), or the
    # mission needs a CONDITION_YAW item to turn the nose (ArduCopter)
    waypoint_sets_heading = True
    # The mode a multirotor is put in to be armed, unless it is in it already (None:
    # PX4 arms in any but a manual mode). ArduCopter stays in LAND or RTL after
    # landing, which it won't arm in
    arm_mode = None
    # A multirotor holds by flying to where it is, rather than by a hold mode
    # (ArduCopter's LOITER takes its climb rate from a throttle stick)
    hold_by_repositioning = False

    # Link-loss failsafe: the parameters, and what their values do
    link_loss_action_param = "NAV_DLL_ACT"
    link_loss_timeout_param = "COM_DL_LOSS_T"
    LINK_LOSS_ACTIONS = {0: "nothing", 1: "hold", 2: "return home", 3: "land",
                         5: "terminate", 6: "lockdown"}
    # Geofence: what PX4 does at it, and its ceiling above home
    fence_action_param = "GF_ACTION"
    fence_ceiling_param = "GF_MAX_VER_DIST"
    FENCE_ACTIONS = {"warn": 1, "hold": 2, "rtl": 3, "land": 5}
    # Parameters that turn an uploaded fence on (PX4: none, it uses any it has)
    FENCE_ON = {}
    FENCE_OFF = {}

    def mode(self, custom_mode):
        """(the ground station's name, the autopilot's own) for HEARTBEAT.custom_mode."""
        main = (custom_mode >> 16) & 0xFF
        sub = (custom_mode >> 24) & 0xFF
        name = self.AUTO_MODES.get(sub, "AUTO") if main == 4 else self.MAIN_MODES.get(main, "?")
        return name, name

    def set_mode_params(self, mode):
        """MAV_CMD_DO_SET_MODE params 2 and 3 for a mode of SET_MODES."""
        main, sub = self.SET_MODES[mode]
        return main, sub

    def encode_param(self, value):
        """(MAV_PARAM_TYPE, the float PARAM_SET carries) for a value; an int
        sets an integer parameter."""
        if isinstance(value, int):
            # PX4 encodes integer parameters bytewise in the float field
            return M.MAV_PARAM_TYPE_INT32, struct.unpack("<f", struct.pack("<i", value))[0]
        return M.MAV_PARAM_TYPE_REAL32, float(value)

    def decode_param(self, value, param_type):
        if param_type in INT_PARAM_TYPES:
            return struct.unpack("<i", struct.pack("<f", value))[0]
        return value


class ArduPilot(Px4):
    """ArduPlane (the modes are ArduPlane's; ArduCopter's are below)."""
    name = "ardupilot"
    label = "ArduPilot"

    # ArduPlane's custom_mode numbers, and the ground station's names for them
    MODES = {0: "MANUAL", 1: "CIRCLE", 2: "STABILIZE", 3: "TRAINING", 4: "ACRO", 5: "FBWA",
             6: "FBWB", 7: "CRUISE", 8: "AUTOTUNE", 10: "AUTO", 11: "RTL", 12: "LOITER",
             13: "TAKEOFF", 14: "AVOID_ADSB", 15: "GUIDED", 16: "INITIALISING", 17: "QSTABILIZE",
             18: "QHOVER", 19: "QLOITER", 20: "QLAND", 21: "QRTL", 22: "QAUTOTUNE", 23: "QACRO",
             24: "THERMAL", 25: "LOITER_ALT_QLAND", 26: "AUTOLAND"}
    COMMON_NAMES = {"MANUAL": "MANUAL", "STABILIZE": "STABILIZED", "TRAINING": "MANUAL",
                    "ACRO": "ACRO", "FBWA": "STABILIZED", "FBWB": "ALTCTL", "CRUISE": "POSCTL",
                    "AUTO": "MISSION", "RTL": "RTL", "LOITER": "HOLD", "CIRCLE": "HOLD",
                    "TAKEOFF": "TAKEOFF", "GUIDED": "GUIDED", "QLAND": "LAND", "QRTL": "RTL",
                    "AUTOLAND": "LAND", "QSTABILIZE": "STABILIZED", "QHOVER": "ALTCTL",
                    "QLOITER": "POSCTL", "QACRO": "ACRO"}
    SET_MODES = {"HOLD": 12, "MISSION": 10, "TAKEOFF": 13, "GUIDED": 15, "RTL": 11}

    # Mission item 0 is home, which ArduPilot keeps itself: an upload starts
    # with a placeholder for it, and item N here is ArduPilot's N + 1
    mission_offset = 1
    # ArduPilot obeys a mission's gimbal items whoever holds the gimbal, and
    # has no DO_GIMBAL_MANAGER_CONFIGURE mission item (it rejects the upload)
    mission_gimbal_claim = False

    link_loss_action_param = "FS_LONG_ACTN"
    link_loss_timeout_param = "FS_LONG_TIMEOUT"
    LINK_LOSS_ACTIONS = {0: "carry on", 1: "return home", 2: "glide", 3: "deploy its parachute",
                         4: "fly its mission", 5: "land (AUTOLAND)"}
    fence_action_param = "FENCE_ACTION"
    fence_ceiling_param = "FENCE_ALT_MAX"
    # A plane can't hold or land on the spot: "hold" is GUIDED (it flies back
    # inside and circles), "land" returns home
    FENCE_ACTIONS = {"warn": 0, "hold": 6, "rtl": 1, "land": 1}
    FENCE_ON = {"FENCE_TYPE": 5, "FENCE_ENABLE": 1}  # polygon (4) + ceiling (1)
    FENCE_OFF = {"FENCE_ENABLE": 0}

    def mode(self, custom_mode):
        own = self.MODES.get(custom_mode, "?")
        return self.COMMON_NAMES.get(own, own), own

    def set_mode_params(self, mode):
        return self.SET_MODES[mode], 0

    def encode_param(self, value):
        # ArduPilot carries every parameter as the float value, whatever its type
        return (M.MAV_PARAM_TYPE_INT32 if isinstance(value, int) else M.MAV_PARAM_TYPE_REAL32,
                float(value))

    def decode_param(self, value, param_type):
        return int(value) if param_type in INT_PARAM_TYPES else value


class ArduCopter(ArduPilot):
    """ArduCopter: the ArduPilot dialect with a multirotor's modes. Where it
    differs from ArduPlane (and PX4): it takes off in GUIDED, to an altitude
    above home; it faces a heading only when told (CONDITION_YAW); and a
    multirotor hovers, so HOLD is LOITER."""
    name = "ardupilot-copter"
    label = "ArduCopter"

    MODES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
             6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP",
             15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB",
             20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG",
             25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL"}
    COMMON_NAMES = {"STABILIZE": "STABILIZED", "ACRO": "ACRO", "ALT_HOLD": "ALTCTL",
                    "AUTO": "MISSION", "GUIDED": "GUIDED", "GUIDED_NOGPS": "GUIDED",
                    "LOITER": "HOLD", "BRAKE": "HOLD", "POSHOLD": "POSCTL", "RTL": "RTL",
                    "SMART_RTL": "RTL", "AUTO_RTL": "RTL", "LAND": "LAND"}
    # HOLD (LOITER) is here for a drone on the ground; see hold_by_repositioning
    SET_MODES = {"HOLD": 5, "MISSION": 3, "GUIDED": 4, "RTL": 6, "LAND": 9}

    # GLOBAL_POSITION_INT, EXTENDED_SYS_STATE (landed), MISSION_CURRENT
    stream_messages = ((33, 200_000), (245, 500_000), (42, 500_000))

    takeoff_mode = "GUIDED"
    takeoff_alt_relative = True
    reposition_sets_heading = False
    waypoint_sets_heading = False
    hold_by_repositioning = True
    arm_mode = "GUIDED"

    # FS_GCS_ENABLE: what the copter does when the ground station goes quiet
    link_loss_action_param = "FS_GCS_ENABLE"
    link_loss_timeout_param = "FS_GCS_TIMEOUT"
    LINK_LOSS_ACTIONS = {0: "carry on", 1: "return home", 2: "carry on in a mission, else return",
                         3: "return home (smart)", 4: "land (smart return first)", 5: "land",
                         6: "fly its landing sequence"}
    # FENCE_ACTION: report only, return or land, land, smart return, brake or land
    FENCE_ACTIONS = {"warn": 0, "hold": 4, "rtl": 1, "land": 2}


DIALECTS = {d.name: d() for d in (Px4, ArduPilot, ArduCopter)}


def dialect(name):
    try:
        return DIALECTS[name]
    except KeyError:
        raise ValueError(f"unknown autopilot {name!r} (known: {', '.join(DIALECTS)})") from None
