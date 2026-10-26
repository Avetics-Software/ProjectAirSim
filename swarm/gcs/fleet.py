"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

The config file: the drones, how to reach each one, and the ground station's
settings (swarm, deconfliction, safety, controls). See fleet.example.yml for
every key; swarm/config.yml is the sim's.

Real drones read all of it. The sim reads how many drones there are, each
one's companion and the settings: it makes one PX4 per drone, named Drone1,
Drone2, ..., and works out their MAVLink ports, system ids and video streams
itself.
"""

from dataclasses import dataclass, field
from typing import Optional

import yaml

from .safety import SafetySettings
from .settings import ControlSettings, DeconflictionSettings, SwarmSettings

DEFAULT_GCS_SYSID = 245  # not 255, QGC's, so both can share a link
DEFAULT_GCS_COMPID = 190  # MAV_COMP_ID_MISSIONPLANNER


@dataclass
class DroneSpec:
    name: str
    sysid: int  # its PX4's MAVLink system id
    mavlink: Optional[str] = None  # its own MAVLink link, if not the fleet's shared one
    video: Optional[str] = None  # gimbal camera stream
    chase_video: Optional[str] = None  # a second camera, shown as the ChaseCam
    companion: Optional[str] = None  # IP of its companion computer (X pipeline)


@dataclass
class Fleet:
    drones: list
    mavlink: Optional[str] = None  # the link carrying every drone without its own
    gcs_sysid: int = DEFAULT_GCS_SYSID
    gcs_compid: int = DEFAULT_GCS_COMPID
    home: Optional[tuple] = None  # (lat, lon): map origin; else the first position fix
    # Settings sections (gcs/settings.py, gcs/safety.py); defaults if absent
    swarm: SwarmSettings = field(default_factory=SwarmSettings)
    deconfliction: DeconflictionSettings = field(default_factory=DeconflictionSettings)
    safety: SafetySettings = field(default_factory=SafetySettings)
    controls: ControlSettings = field(default_factory=ControlSettings)

    @property
    def companions(self):
        """{drone index: companion IP} for the drones that have one."""
        return {i: d.companion for i, d in enumerate(self.drones) if d.companion}

    def link_url(self, drone):
        return drone.mavlink or self.mavlink


SECTIONS = {"swarm": SwarmSettings, "deconfliction": DeconflictionSettings,
            "safety": SafetySettings, "controls": ControlSettings}
TOP_KEYS = {"drones", "mavlink", "gcs_sysid", "gcs_compid", "home", "layer_step", *SECTIONS}


def load_fleet(path):
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    unknown = set(data) - TOP_KEYS
    if unknown:
        raise ValueError(f"{path}: unknown key(s) {', '.join(sorted(unknown))}")
    entries = data.get("drones") or []
    if not entries:
        raise ValueError(f"{path}: no drones listed")
    drones = []
    for i, entry in enumerate(entries):
        entry = dict(entry or {})
        unknown = set(entry) - set(DroneSpec.__dataclass_fields__)
        if unknown:
            raise ValueError(f"{path}: drone {i + 1}: unknown key(s) {', '.join(sorted(unknown))}")
        entry.setdefault("name", f"Drone{i + 1}")
        entry.setdefault("sysid", i + 1)
        drones.append(DroneSpec(**entry))
    for key in ("name", "sysid"):
        values = [getattr(d, key) for d in drones]
        if len(set(values)) != len(values):
            raise ValueError(f"{path}: two drones have the same {key}")
    home = data.get("home")
    try:
        sections = {name: cls.from_dict(data.get(name), name) for name, cls in SECTIONS.items()}
    except ValueError as err:
        raise ValueError(f"{path}: {err}") from None
    if "layer_step" in data:  # the older top-level spelling
        sections["swarm"].set("layer_step_m", data["layer_step"])
    return Fleet(
        drones=drones,
        mavlink=data.get("mavlink"),
        gcs_sysid=int(data.get("gcs_sysid", DEFAULT_GCS_SYSID)),
        gcs_compid=int(data.get("gcs_compid", DEFAULT_GCS_COMPID)),
        home=tuple(home) if home else None,
        **sections,
    )


def sim_drones(fleet, drones=None, pi_ip=None):
    """For the sim: (how many drones, {index: companion IP}). The command line's
    --drones and --pi-ip win over the file: extra drones have no companion, and
    --pi-ip is Drone1's companion."""
    count = drones if drones else (len(fleet.drones) if fleet else 1)
    companions = {i: ip for i, ip in (fleet.companions if fleet else {}).items() if i < count}
    if pi_ip:
        companions[0] = pi_ip
    return count, companions


def check_for_real_drones(fleet, path):
    """Every drone needs a MAVLink link when flying real drones."""
    missing = [d.name for d in fleet.drones if not fleet.link_url(d)]
    if missing:
        raise ValueError(f"{path}: no MAVLink link for {', '.join(missing)}: set the fleet's "
                         "`mavlink:` or the drone's own")
