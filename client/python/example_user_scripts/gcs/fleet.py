"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

The fleet file: which drones there are and how to reach each one. See
fleet.example.yml for every key.

Real drones read all of it. The sim (run_cesium_sim.sh --fleet) reads only how
many drones there are and each one's companion: it makes one PX4 per drone,
named Drone1, Drone2, ..., and works out their MAVLink ports, system ids and
video streams itself.
"""

from dataclasses import dataclass
from typing import Optional

import yaml

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
    layer_step: float = 5.0  # swarm height layers (m)

    @property
    def companions(self):
        """{drone index: companion IP} for the drones that have one."""
        return {i: d.companion for i, d in enumerate(self.drones) if d.companion}

    def link_url(self, drone):
        return drone.mavlink or self.mavlink


def load_fleet(path):
    with open(path) as f:
        data = yaml.safe_load(f) or {}
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
    fleet = Fleet(
        drones=drones,
        mavlink=data.get("mavlink"),
        gcs_sysid=int(data.get("gcs_sysid", DEFAULT_GCS_SYSID)),
        gcs_compid=int(data.get("gcs_compid", DEFAULT_GCS_COMPID)),
        home=tuple(home) if home else None,
        layer_step=float(data.get("layer_step", 5.0)),
    )
    return fleet


def check_for_real_drones(fleet, path):
    """Every drone needs a MAVLink link when flying real drones."""
    missing = [d.name for d in fleet.drones if not fleet.link_url(d)]
    if missing:
        raise ValueError(f"{path}: no MAVLink link for {', '.join(missing)}: set the fleet's "
                         "`mavlink:` or the drone's own")
