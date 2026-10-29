"""Time of day: site time to the SunSky's, and where the sun is."""

import contextlib
import datetime as dt
import io
import unittest
from types import SimpleNamespace

from sun_time import (describe_sun, parse_site_time, site_zone, sun_position, sunsky_time,
                      zone_mismatch_h)

SGT = site_zone("Asia/Singapore")
SELETAR = (1.413425, 103.865)


def sgt(text):
    return dt.datetime.fromisoformat(text).replace(tzinfo=SGT)


class SunPositionTests(unittest.TestCase):
    def test_noon_is_nearly_overhead_in_singapore_at_the_equinox(self):
        # Solar noon there is about 13:12 SGT: clocks run an hour ahead of the sun
        elevation, _ = sun_position(*SELETAR, sgt("2026-03-20T13:12"))
        self.assertGreater(elevation, 87.5)
        self.assertLess(sun_position(*SELETAR, sgt("2026-03-20T12:12"))[0], elevation - 10)

    def test_sunrise_east_and_sunset_west(self):
        # Singapore: sunrise about 07:10, sunset about 19:15 SGT in March
        up, az_rise = sun_position(*SELETAR, sgt("2026-03-20T07:10"))
        down, az_set = sun_position(*SELETAR, sgt("2026-03-20T19:15"))
        self.assertLess(abs(up), 1.5)
        self.assertLess(abs(down), 1.5)
        self.assertAlmostEqual(az_rise, 90.0, delta=2.0)
        self.assertAlmostEqual(az_set, 270.0, delta=2.0)

    def test_midnight_is_dark_and_the_names(self):
        elevation, _ = sun_position(*SELETAR, sgt("2026-03-20T01:00"))
        self.assertLess(elevation, -60)
        self.assertEqual([describe_sun(e) for e in (-30, -3, 5, 40)],
                         ["night", "twilight", "low sun", "day"])

    def test_london_summer_with_daylight_saving(self):
        london = site_zone("Europe/London")
        # BST: solar noon about 13:02 local on the solstice, the sun 62 deg up
        elevation, azimuth = sun_position(51.5, -0.12, dt.datetime(2026, 6, 21, 13, 2,
                                                                   tzinfo=london))
        self.assertAlmostEqual(elevation, 62.0, delta=0.5)
        self.assertAlmostEqual(azimuth, 180.0, delta=2.0)


class SunSkyTimeTests(unittest.TestCase):
    def test_the_sunsky_zone_is_longitude_over_15(self):
        # 18:30 SGT = 10:30 UTC; the SunSky's zone at 103.865 E is UTC+6.924 h
        got = sunsky_time(sgt("2026-03-20T18:30"), SELETAR[1])
        want = dt.datetime(2026, 3, 20, 10, 30) + dt.timedelta(hours=103.865 / 15)
        self.assertEqual(got, want)
        self.assertIsNone(got.tzinfo)

    def test_the_date_rolls_over_with_the_zone(self):
        # 07:00 SGT on the 21st is 23:00 UTC on the 20th; at 30 W that is 21:00 on the 20th
        got = sunsky_time(sgt("2026-03-21T07:00"), -30.0)
        self.assertEqual(got, dt.datetime(2026, 3, 20, 21, 0))

    def test_daylight_saving_is_the_site_zones_business(self):
        ny = site_zone("America/New_York")
        winter = sunsky_time(dt.datetime(2026, 1, 15, 12, 0, tzinfo=ny), -75.0)
        summer = sunsky_time(dt.datetime(2026, 7, 15, 12, 0, tzinfo=ny), -75.0)
        self.assertEqual((winter.hour, summer.hour), (12, 11))  # EDT clocks run an hour ahead

    def test_a_zone_far_from_the_longitude_is_flagged(self):
        self.assertAlmostEqual(zone_mismatch_h(sgt("2026-03-20T12:00"), SELETAR[1]), 1.08,
                               delta=0.01)
        london = dt.datetime(2026, 3, 20, 12, tzinfo=site_zone("Europe/London"))
        self.assertLess(zone_mismatch_h(london, SELETAR[1]), -6)


class ParseTests(unittest.TestCase):
    NOW = dt.datetime(2026, 10, 28, 20, 0, 30, tzinfo=dt.timezone.utc)  # 04:00:30 SGT on the 29th

    def test_times_and_dates(self):
        self.assertEqual(parse_site_time("18:30", SGT, now=self.NOW), sgt("2026-10-29T18:30"))
        self.assertEqual(parse_site_time("06:45:10", SGT, "2026-12-01", now=self.NOW),
                         sgt("2026-12-01T06:45:10"))
        self.assertEqual(parse_site_time("now", SGT, now=self.NOW), sgt("2026-10-29T04:00:30"))

    def test_bad_input(self):
        for text, date in (("25:00", None), ("dusk", None), ("18:30", "29/10/2026"),
                           ("now", "2026-10-29")):
            with self.assertRaises(ValueError, msg=(text, date)):
                parse_site_time(text, SGT, date, now=self.NOW)
        with self.assertRaises(ValueError):
            site_zone("Mars/Olympus")


class SimTimeCommandTests(unittest.TestCase):
    def setUp(self):
        try:
            import px4_gimbal_cesium
        except ImportError as err:
            self.skipTest(f"px4_gimbal_cesium won't import: {err}")
        self.module = px4_gimbal_cesium
        self.sent = []
        world = SimpleNamespace(
            home_geo_point={"latitude": SELETAR[0], "longitude": SELETAR[1]},
            set_sun_position_from_date_time=lambda when: self.sent.append(when) or True)
        self.session = object.__new__(px4_gimbal_cesium.Session)
        self.session.world, self.session.zone, self.session.site_time = world, SGT, None

    def run_command(self, *args):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.module.time_command(self.session, list(args))
        return out.getvalue()

    def test_sets_the_sun_in_the_sunskys_zone_and_reports_it(self):
        reply = self.run_command("19:15", "2026-03-20")
        self.assertEqual(self.sent, [sunsky_time(sgt("2026-03-20T19:15"), SELETAR[1])])
        self.assertIn("2026-03-20 19:15 +08: sun", reply)
        self.assertIn("azimuth 270 deg", reply)
        self.assertNotIn("time zone", reply)  # Singapore's is right for Seletar
        self.assertIn("2026-03-20 19:15", self.run_command())

    def test_a_wrong_looking_zone_is_pointed_out(self):
        self.session.zone = site_zone("America/Los_Angeles")
        self.assertIn("is it this place's time zone?", self.run_command("12:00"))

    def test_no_sunsky(self):
        self.session.world.set_sun_position_from_date_time = lambda when: False
        self.assertIn("no SunSky", self.run_command("12:00"))
        self.assertIsNone(self.session.site_time)

    def test_usage(self):
        self.assertIn("map's own sun", self.run_command())
        self.assertIn("Usage", self.run_command("noon"))
        self.assertIn("Usage", self.run_command("12:00", "2026-01-01", "extra"))


if __name__ == "__main__":
    unittest.main()
