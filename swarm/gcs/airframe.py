"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

What kind of aircraft a drone is, for the ground station: which autopilot
speaks for it, and what it can be asked to do. A drone's `model:` in the config
file names an entry of aircraft/models.yml (the sim builds the same aircraft
from it); this reads only that entry's autopilot family and capabilities, so
the ground station needs nothing from the sim.

The commands ask these capabilities rather than the kind of aircraft:

  hovers           stops on a point, climbs in place, turns on the spot. Without
                   it the aircraft circles a held point (loiter_radius_m), gives
                   way by changing height only, and has no heading of its own.
  loiter_radius_m  how far from a held point it circles (0: holds the point)
  cruise_mps       its normal airspeed; stall_mps the slowest it can fly
"""

import os
from dataclasses import dataclass, fields, replace

import yaml

MODELS_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           "aircraft", "models.yml")
DEFAULT_MODEL = "quad_px4"


@dataclass(frozen=True)
class Airframe:
    model: str = DEFAULT_MODEL
    autopilot: str = "px4"  # MAVLink dialect: px4 | ardupilot
    hovers: bool = True
    loiter_radius_m: float = 0.0
    cruise_mps: float = 0.0
    stall_mps: float = 0.0

    @property
    def label(self):
        return "multirotor" if self.hovers else "fixed-wing"


# The kinds a model's `capabilities:` can name, and their values. A model may
# also give a mapping: {kind: fixedwing, loiter_radius_m: 50, ...}.
KINDS = {
    "multirotor": {"hovers": True},
    # ArduPlane's stock SITL plane (WP_LOITER_RAD 80) at the X8's speeds
    "fixedwing": {"hovers": False, "loiter_radius_m": 80.0, "cruise_mps": 18.0,
                  "stall_mps": 13.0},
}


def load_models(path=MODELS_FILE):
    with open(path) as f:
        return yaml.safe_load(f)


def airframe_of(model, models=None):
    """The Airframe of a models.yml model name (None: the default model)."""
    model = model or DEFAULT_MODEL
    models = models or load_models()
    entry = models["models"].get(model)
    if entry is None:
        raise ValueError(f"unknown model {model!r} (known: {', '.join(models['models'])})")
    family = models["autopilots"][entry["autopilot"]]["family"]
    caps = entry.get("capabilities", "multirotor")
    if isinstance(caps, str):
        caps = {"kind": caps}
    caps = dict(caps)
    kind = caps.pop("kind", "multirotor")
    if kind not in KINDS:
        raise ValueError(f"model {model}: capabilities kind must be one of {', '.join(KINDS)}")
    unknown = set(caps) - {f.name for f in fields(Airframe)}
    if unknown:
        raise ValueError(f"model {model}: unknown capabilities {', '.join(sorted(unknown))}")
    return replace(Airframe(model=model, autopilot=family), **KINDS[kind], **caps)
