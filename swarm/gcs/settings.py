"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

The ground station's settings, one dataclass per section of the config file
(swarm/config.yml for the sim, a fleet file for real drones):

  swarm:          height layers, default height, formation and search spacing
  deconfliction:  separation, look-ahead, giving way
  safety:         gcs.safety.SafetySettings (thresholds and responses)
  controls:       gimbal and zoom steps

Every value has a default here; a config file only needs what it changes.
"""

from dataclasses import dataclass, fields


class Settings:
    """from_dict() and set() for a settings dataclass."""

    CHOICES = {}  # key -> allowed values, for text settings

    @classmethod
    def from_dict(cls, data, section=None):
        names = {f.name for f in fields(cls)}
        unknown = set(data or {}) - names
        if unknown:
            where = f"{section}: " if section else ""
            raise ValueError(f"{where}unknown setting(s) {', '.join(sorted(unknown))} "
                             f"(known: {', '.join(sorted(names))})")
        settings = cls()
        for key, value in (data or {}).items():
            settings.set(key, value)
        return settings

    def set(self, key, value):
        """Set one setting from text or a number, keeping its type."""
        current = getattr(self, key)
        if isinstance(current, str):
            value = str(value).lower()
            allowed = self.CHOICES.get(key)
            if allowed and value not in allowed:
                raise ValueError(f"{key} must be one of {', '.join(allowed)}")
        else:
            value = type(current)(float(value))
        setattr(self, key, value)


@dataclass
class SwarmSettings(Settings):
    layer_step_m: float = 5.0  # drone N flies (N-1) x this above the height asked for; 0 = off
    default_alt_m: float = 30.0  # takeoff / search height when none is given (above home)
    formation_spacing_m: float = 15.0  # several drones sent to one point form a line this far apart
    search_spacing_m: float = 40.0  # pass spacing of `search` when none is given
    arrive_m: float = 4.0  # a goto has arrived within this distance (then turns to its heading)
    waypoint_timeout_s: float = 180.0  # give up on a goto after this (plus 2 s per 1 m to fly)


@dataclass
class DeconflictionSettings(Settings):
    separation_h_m: float = 6.0  # closer than this horizontally...
    separation_v_m: float = 3.0  # ...and this vertically is a conflict
    lookahead_s: float = 6.0  # predict conflicts this far ahead from the drones' velocities
    give_way_climb_m: float = 6.0  # the drone giving way stops and climbs this much
    clear_h_m: float = 10.0  # and resumes once this far from the other and not closing
    max_gps_margin_m: float = 10.0  # cap on the separation added for GPS inaccuracy


@dataclass
class ControlSettings(Settings):
    gimbal_step_deg: float = 3.0  # each W/A/S/D press, for drones without a companion
    zoom_step_percent: float = 5.0  # each wheel notch or +/-, without a companion
    companion_zoom_pulse_s: float = 0.4  # with a companion: each notch zooms this long
