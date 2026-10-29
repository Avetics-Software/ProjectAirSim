"""
Time of day for the sim: a clock time at the site (its time zone, with daylight
saving) turned into what Unreal's SunSky takes, and where the sun then is.

The SunSky positions the sun from a latitude, longitude, time zone, date and
"solar time" (the clock time in that zone). On a GIS scene the sim gives it the
home point, but a time zone of longitude / 15 h: UTC+6.93 at Seletar, where
clocks run at UTC+8. So a site time is converted to UTC and then into the
SunSky's own zone (`sunsky_time`). The SunSky has no daylight saving; the site's
zone takes care of it.

`sun_position` is NOAA's solar position approximation (about 0.1 deg here), for
reporting where the sun is, e.g. "sun 8 deg up".
"""

import datetime as dt
import math
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError

DEFAULT_TIMEZONE = "Asia/Singapore"
# Sun elevations that name the light, for the reply to `sim time`
TWILIGHT_DEG = (-6.0, 0.0, 10.0)  # civil dusk, horizon, low sun


def site_zone(name):
    try:
        return ZoneInfo(name)
    except (ZoneInfoNotFoundError, ValueError):
        raise ValueError(f"unknown time zone '{name}' (use an IANA name, e.g. Asia/Singapore)")


def parse_site_time(text, zone, date=None, now=None):
    """The site's time as an aware datetime. text: "HH:MM", "HH:MM:SS" or
    "now"; date: "YYYY-MM-DD" or None for today at the site."""
    now = now or dt.datetime.now(dt.timezone.utc)
    today = now.astimezone(zone).date()
    if text.lower() == "now":
        if date:
            raise ValueError("'now' takes no date")
        return now.astimezone(zone).replace(microsecond=0)
    try:
        day = dt.date.fromisoformat(date) if date else today
    except ValueError:
        raise ValueError(f"bad date '{date}' (YYYY-MM-DD)")
    for fmt in ("%H:%M", "%H:%M:%S"):
        try:
            clock = dt.datetime.strptime(text, fmt).time()
            break
        except ValueError:
            continue
    else:
        raise ValueError(f"bad time '{text}' (HH:MM, HH:MM:SS or now)")
    return dt.datetime.combine(day, clock, tzinfo=zone)


def sunsky_time(site_time, longitude):
    """The SunSky's date and clock time (naive) for an aware site time: the
    same instant in its time zone of longitude / 15 hours."""
    utc = site_time.astimezone(dt.timezone.utc).replace(tzinfo=None)
    return utc + dt.timedelta(hours=longitude / 15.0)


def zone_mismatch_h(site_time, longitude):
    """How far (hours) the site's UTC offset is from the sun's at this
    longitude. A few hours means the time zone is likely the wrong one for
    the place (e.g. after `goto` to another country)."""
    offset = site_time.utcoffset().total_seconds() / 3600.0
    return offset - longitude / 15.0


def sun_position(latitude, longitude, when):
    """(elevation, azimuth) of the sun in degrees, azimuth from north through
    east, at an aware datetime. NOAA's approximation, without refraction."""
    utc = when.astimezone(dt.timezone.utc)
    day_of_year = utc.timetuple().tm_yday
    hour = utc.hour + utc.minute / 60.0 + utc.second / 3600.0
    g = 2.0 * math.pi / 365.0 * (day_of_year - 1 + (hour - 12.0) / 24.0)
    eq_time = 229.18 * (0.000075 + 0.001868 * math.cos(g) - 0.032077 * math.sin(g)
                        - 0.014615 * math.cos(2 * g) - 0.040849 * math.sin(2 * g))
    decl = (0.006918 - 0.399912 * math.cos(g) + 0.070257 * math.sin(g)
            - 0.006758 * math.cos(2 * g) + 0.000907 * math.sin(2 * g)
            - 0.002697 * math.cos(3 * g) + 0.00148 * math.sin(3 * g))
    true_solar_min = hour * 60.0 + eq_time + 4.0 * longitude
    hour_angle = math.radians(true_solar_min / 4.0 - 180.0)
    lat = math.radians(latitude)
    cos_zenith = (math.sin(lat) * math.sin(decl)
                  + math.cos(lat) * math.cos(decl) * math.cos(hour_angle))
    zenith = math.acos(max(-1.0, min(1.0, cos_zenith)))
    azimuth = math.degrees(math.atan2(
        math.sin(hour_angle),
        math.cos(hour_angle) * math.sin(lat) - math.tan(decl) * math.cos(lat))) + 180.0
    return 90.0 - math.degrees(zenith), azimuth % 360.0


def describe_sun(elevation):
    civil, horizon, low = TWILIGHT_DEG
    if elevation < civil:
        return "night"
    if elevation < horizon:
        return "twilight"
    if elevation < low:
        return "low sun"
    return "day"
