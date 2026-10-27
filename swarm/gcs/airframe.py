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
  turn_radius_m    the tightest turn it flies between search passes (0: turns
                   on the spot)
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
    turn_radius_m: float = 0.0
    cruise_mps: float = 0.0
    stall_mps: float = 0.0

    @property
    def label(self):
        return "multirotor" if self.hovers else "fixed-wing"


# The kinds a model's `capabilities:` can name, and their values. A model may
# also give a mapping: {kind: fixedwing, loiter_radius_m: 50, ...}.
KINDS = {
    "multirotor": {"hovers": True},
    # ArduPlane's stock SITL plane (WP_LOITER_RAD 80) at the X8's speeds; a
    # turn at cruise and 45 deg of bank: v^2 / (g tan 45) = 33 m. (Flown in the
    # sim it turns tighter, 12-15 m, as the stock plane banks to 65 deg: this
    # is the planning figure, not the limit)
    "fixedwing": {"hovers": False, "loiter_radius_m": 80.0, "turn_radius_m": 35.0,
                  "cruise_mps": 18.0, "stall_mps": 13.0},
}


def load_models(path=MODELS_FILE):
    with open(path) as f:
        return yaml.safe_load(f)


def airframe_of(model, models=None):
    """The Airframe of a models.yml model name (None: the default model).

    gcs/ copied alone to a field laptop has no aircraft/models.yml: every drone
    is then the default (a PX4 multirotor), and another model is an error that
    says what to copy."""
    model = model or DEFAULT_MODEL
    if models is None:
        if not os.path.exists(MODELS_FILE):
            if model == DEFAULT_MODEL:
                return Airframe()
            raise ValueError(f"model {model!r}: {MODELS_FILE} not found; copy the swarm's "
                             "aircraft/ folder next to gcs/ to fly other aircraft")
        models = load_models()
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
