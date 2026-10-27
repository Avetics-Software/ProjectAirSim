"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

The swarm's aircraft registry (aircraft/models.yml): builds each drone's robot
config and says how to start its autopilot, from three layers that each hold
one kind of fact:

  airframe   aircraft/airframes/<name>.jsonc: the physical aircraft, its output
             order per autopilot family, where the gimbal camera mounts
  autopilot  models.yml autopilots: the link (controller type, ports, clock)
  model      models.yml models: an airframe + autopilot pair, its SITL
             parameter files and its capabilities (gcs/airframe.py)

Every drone gets the same gimbal camera (aircraft/payload_gimbal.jsonc) at its
airframe's mount. Drone `index` (0-based) talks to autopilot instance index +
the autopilot's instance-offset, whose ports are offset by instance x stride.

  python aircraft.py --list
  python aircraft.py --dump x8_arduplane [--index 1]   # the generated robot config
  python aircraft.py --audit                           # cross-check the layers
"""

import argparse
import copy
import json
import os
import sys
from dataclasses import dataclass, field

import commentjson
import yaml

from gcs.airframe import airframe_of

SWARM_DIR = os.path.dirname(os.path.abspath(__file__))
AIRCRAFT_DIR = os.path.join(SWARM_DIR, "aircraft")
MODELS_FILE = os.path.join(AIRCRAFT_DIR, "models.yml")
PAYLOAD_FILE = os.path.join(AIRCRAFT_DIR, "payload_gimbal.jsonc")
GIMBAL_CAMERA = "GimbalCam"
# Keys of an airframe file that describe it to aircraft.py, not to the sim
AIRFRAME_ONLY_KEYS = ("extends", "remove", "autopilot", "gimbal-mount")


def expand(path):
    """${REPO}, ${SWARM}, ${PX4_DIR}, ${ARDUPILOT_DIR} and ~ in a path."""
    values = {
        "REPO": os.path.dirname(SWARM_DIR),
        "SWARM": SWARM_DIR,
        "PX4_DIR": os.path.expanduser(os.environ.get("PX4_DIR", "~/Documents/PX4-Autopilot")),
        "ARDUPILOT_DIR": os.path.expanduser(os.environ.get("ARDUPILOT_DIR", "~/Documents/ardupilot")),
    }
    for name, value in values.items():
        path = path.replace("${" + name + "}", value)
    return os.path.expanduser(path)


def merge(base, over):
    """`over` deep-merged onto a copy of `base`: mappings merge, anything else
    (lists included) is replaced."""
    out = copy.deepcopy(base)
    for key, value in over.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def load_jsonc(path):
    with open(path) as f:
        return commentjson.load(f)


def outside_swarm(path):
    """True for a file the swarm reads from outside swarm/, other than the
    autopilots' own checkouts (their binaries and stock parameter files)."""
    path = os.path.realpath(path)
    allowed = [SWARM_DIR] + [os.path.realpath(expand("${" + d + "}")) for d in ("PX4_DIR", "ARDUPILOT_DIR")]
    return not any(path == a or path.startswith(a + os.sep) for a in allowed)


def load_airframe(name):
    """An airframe file, with the file it extends merged under it."""
    path = os.path.join(AIRCRAFT_DIR, "airframes", f"{name}.jsonc")
    airframe = load_jsonc(path)
    if "extends" in airframe:
        base = load_jsonc(expand(airframe["extends"]))
        for key in airframe.get("remove", []):
            base.pop(key, None)
        airframe = merge(base, airframe)
    return airframe


def load_models(path=MODELS_FILE):
    with open(path) as f:
        return yaml.safe_load(f)


def read_params(path):
    """{NAME: value} from a .parm file ("NAME VALUE", '#' comments) or a QGC
    .params export ("sysid compid NAME VALUE type")."""
    params = {}
    with open(path) as f:
        for line in f:
            parts = line.split("#")[0].replace(",", " ").split()
            if len(parts) >= 5 and parts[0].isdigit() and parts[1].isdigit():
                parts = parts[2:4]
            if len(parts) >= 2 and parts[0][:1].isalpha() and parts[0].upper() == parts[0]:
                try:
                    params[parts[0]] = float(parts[1])
                except ValueError:
                    pass
    return params


@dataclass
class Sitl:
    """How to start one drone's autopilot (px4_sitl.Px4Sitl or
    ardupilot_sitl.ArduPilotSitl take it)."""
    model: str
    family: str  # px4 | ardupilot
    index: int  # the drone, 0-based
    instance: int  # the autopilot's instance number
    params: list = field(default_factory=list)  # parameter files, applied in order
    frame: str = ""  # PX4: PX4_SIM_MODEL
    binary: str = ""  # ArduPilot: the SITL executable

    @property
    def sysid(self):
        return self.index + 1


class Aircraft:
    """One model of models.yml, resolved for any drone index."""

    def __init__(self, model, models=None):
        self.models = models or load_models()
        if model not in self.models["models"]:
            raise ValueError(f"unknown aircraft model {model!r} "
                             f"(known: {', '.join(self.models['models'])})")
        self.name = model
        self.entry = self.models["models"][model]
        self.autopilot = self.models["autopilots"][self.entry["autopilot"]]
        self.family = self.autopilot["family"]
        self.airframe = load_airframe(self.entry["airframe"])
        self.capabilities = airframe_of(model, self.models)
        # The X profile its companion computer runs in the sim
        self.companion_profile = self.entry.get("companion-profile", "")

    def instance(self, index):
        return index + self.autopilot.get("instance-offset", 0)

    def ports(self, index):
        stride = self.autopilot.get("port-stride", 1)
        return {name: base + stride * self.instance(index)
                for name, base in self.autopilot.get("ports", {}).items()}

    def robot(self, index=0, home=None):
        """The robot config (a dict) for drone `index`. home: (lat, lon) of
        the scene, which PX4's estimator needs."""
        own = self.airframe.get("autopilot", {}).get(self.family)
        if own is None:
            raise ValueError(f"airframe {self.entry['airframe']} has nothing for "
                             f"{self.family} autopilots (its \"autopilot\" block)")
        robot = {k: copy.deepcopy(v) for k, v in self.airframe.items()
                 if k not in AIRFRAME_ONLY_KEYS}

        settings = merge(self.autopilot.get("settings", {}), own.get("settings", {}))
        settings.update(self.ports(index))
        if home and self.family == "px4":
            settings.setdefault("parameters", {}).update({"LPE_LAT": home[0], "LPE_LON": home[1]})
        settings["actuator-order"] = copy.deepcopy(own["actuator-order"])
        robot["controller"] = merge(self.autopilot["controller"], own.get("controller", {}))
        robot["controller"][self.autopilot["settings-key"]] = settings

        mount = self.airframe.get("gimbal-mount")
        if mount:
            self._add_gimbal(robot, mount, settings)

        if index > 0:
            # Save GPU for the swarm: a smaller chase view, no down camera
            for sensor in robot.get("sensors", []):
                if sensor["id"] == "Chase":
                    for setting in sensor.get("capture-settings", []):
                        setting["width"], setting["height"] = 640, 360
                elif sensor["id"] == "DownCamera":
                    sensor["enabled"] = False
        return robot

    def _add_gimbal(self, robot, mount, settings):
        payload = load_jsonc(PAYLOAD_FILE)
        actuator = dict(payload["actuator"], **{k: copy.deepcopy(mount[k]) for k in
                                               ("parent-link", "child-link", "origin")})
        camera = dict(payload["camera"], **{"parent-link": mount["parent-link"],
                                            "origin": copy.deepcopy(mount["origin"])})
        robot.setdefault("actuators", []).append(actuator)
        sensors = robot.setdefault("sensors", [])
        sensors.insert(1 if sensors and sensors[0]["id"] == "Chase" else 0, camera)
        if self.family == "ardupilot":
            # ArduPilot aims it through mount servo outputs, decoded across
            # the gimbal's travel (which must match MNT1_*; see --audit)
            travel = actuator["gimbal-settings"]
            channels = self.autopilot["gimbal-channels"]
            roll = self.autopilot.get("gimbal-roll-deg", [0, 0])
            settings["gimbal"] = {
                "id": actuator["name"],
                "roll-channel": channels["roll"],
                "pitch-channel": channels["pitch"],
                "yaw-channel": channels["yaw"],
                "roll-min-deg": roll[0], "roll-max-deg": roll[1],
                "pitch-min-deg": travel["pitch-min-deg"], "pitch-max-deg": travel["pitch-max-deg"],
                "yaw-min-deg": travel["yaw-min-deg"], "yaw-max-deg": travel["yaw-max-deg"],
            }

    def sitl(self, index):
        sitl = self.entry.get("sitl", {})
        return Sitl(
            model=self.name, family=self.family, index=index, instance=self.instance(index),
            params=[expand(p) for p in sitl.get("params", [])],
            frame=sitl.get("frame", ""), binary=expand(sitl.get("binary", "")),
        )

    def audit(self):
        """Problems with this model: (level, message), level ERROR or NOTE."""
        problems = []
        sitl = self.sitl(0)
        for path in sitl.params:
            if not os.path.exists(path):
                problems.append(("ERROR", f"parameter file missing: {path}"))
        extends = load_jsonc(os.path.join(AIRCRAFT_DIR, "airframes",
                                          f"{self.entry['airframe']}.jsonc")).get("extends")
        for path in sitl.params + ([expand(extends)] if extends else []):
            if outside_swarm(path):
                problems.append(("ERROR", f"reads {path}, outside swarm/: keep what the "
                                          "swarm flies in swarm/aircraft/"))
        if sitl.binary and not os.path.exists(sitl.binary):
            problems.append(("NOTE", f"SITL binary not built yet: {sitl.binary}"))
        try:
            robot = self.robot(0)
        except Exception as err:
            return problems + [("ERROR", f"can't build the robot config: {err}")]
        settings = robot["controller"][self.autopilot["settings-key"]]
        names = {a["name"] for a in robot.get("actuators", [])}
        for item in settings["actuator-order"]:
            if item["id"] not in names:
                problems.append(("ERROR", f"actuator-order names {item['id']}, which the "
                                          "airframe doesn't have"))
        if self.family == "ardupilot":
            problems += self._audit_ardupilot(settings, sitl)
        return problems

    def _audit_ardupilot(self, settings, sitl):
        """The two places that describe the same aircraft agree: SERVOn_FUNCTION
        has an output for every actuator-order entry, and the mount's travel
        (MNT1_*) is the one the sim decodes its PWM across."""
        problems = []
        params = {}
        for path in sitl.params:
            if os.path.exists(path):
                params.update(read_params(path))
        for i, item in enumerate(settings["actuator-order"], start=1):
            if not params.get(f"SERVO{i}_FUNCTION"):
                problems.append(("ERROR", f"SERVO{i}_FUNCTION is 0 or unset, but output {i} "
                                          f"drives {item['id']}"))
        gimbal = settings.get("gimbal")
        if gimbal:
            if params.get("MNT1_TYPE") not in (7.0,):
                problems.append(("ERROR", f"MNT1_TYPE is {params.get('MNT1_TYPE')}; the sim's "
                                          "gimbal is self-stabilising (BrushlessPWM, 7)"))
            for axis, function in (("roll", 8), ("pitch", 7), ("yaw", 6)):
                channel = gimbal[f"{axis}-channel"]
                if params.get(f"SERVO{channel}_FUNCTION") != function:
                    problems.append(("ERROR", f"SERVO{channel}_FUNCTION should be {function} "
                                              f"(mount {axis})"))
                for end, limit in (("min", "MIN"), ("max", "MAX")):
                    if params.get(f"SERVO{channel}_{limit}") != {"min": 1000, "max": 2000}[end]:
                        problems.append(("ERROR", f"SERVO{channel}_{limit} should be "
                                                  f"{ {'min': 1000, 'max': 2000}[end]}"))
                    sim = gimbal[f"{axis}-{end}-deg"]
                    ap = params.get(f"MNT1_{axis.upper()}_{limit}")
                    if ap != sim:
                        problems.append(("ERROR", f"MNT1_{axis.upper()}_{limit} is {ap}, but the "
                                                  f"sim decodes the mount across {sim} deg"))
        elif params.get("MNT1_TYPE"):
            problems.append(("NOTE", "MNT1_TYPE is set but the airframe has no gimbal-mount"))
        return problems


def list_models(models=None):
    models = models or load_models()
    return {name: entry.get("label", "") for name, entry in models["models"].items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--list", action="store_true", help="the models")
    group.add_argument("--dump", metavar="MODEL", help="print a model's robot config")
    group.add_argument("--audit", action="store_true",
                       help="check every model; exits 1 on an error")
    parser.add_argument("--index", type=int, default=0, help="drone index for --dump (0-based)")
    args = parser.parse_args()

    if args.list:
        for name, label in list_models().items():
            print(f"{name:20} {label}")
    elif args.dump:
        aircraft = Aircraft(args.dump)
        print(json.dumps(aircraft.robot(args.index), indent=2))
        print(f"// SITL: {aircraft.sitl(args.index)}", file=sys.stderr)
    else:
        errors = 0
        for name in list_models():
            problems = Aircraft(name).audit()
            errors += sum(level == "ERROR" for level, _ in problems)
            print(f"{name}: {'ok' if not problems else ''}")
            for level, message in problems:
                print(f"  {level}: {message}")
        sys.exit(1 if errors else 0)


if __name__ == "__main__":
    main()
