"""Camera defects on the sim's frames, offline: python -m unittest discover -s tests -v."""

import math
import unittest

import cv2
import numpy as np

from camera_effects import CameraEffects, noise_sigma_levels
from camera_properties import CameraProperties, parse_resolution

W, H, HFOV = 640, 360, 80.0


def dot_at(x, y, value=255):
    img = np.zeros((H, W, 3), np.uint8)
    cv2.circle(img, (x, y), 2, (value,) * 3, -1)
    return img


def brightest(img):
    gray = img.astype(np.float32).sum(axis=2)
    ys, xs = np.nonzero(gray > 0.5 * gray.max())
    return xs.mean(), ys.mean()


class DistortionTests(unittest.TestCase):
    def test_a_point_lands_where_the_lens_model_puts_it(self):
        lens = {"distortion_enabled": True, "distortion_k1": -0.25, "distortion_k2": 0.04,
                "distortion_p1": 0.002, "distortion_p2": -0.001}
        fx = (W / 2) / math.tan(math.radians(HFOV) / 2)
        k = np.array([[fx, 0, (W - 1) / 2], [0, fx, (H - 1) / 2], [0, 0, 1]])
        dist = np.array([lens["distortion_k1"], lens["distortion_k2"], lens["distortion_p1"],
                         lens["distortion_p2"], 0.0])
        for x, y in [(100, 60), (540, 300), (320, 180), (560, 40)]:
            want, _ = cv2.projectPoints(
                np.array([[(x - k[0, 2]) / fx, (y - k[1, 2]) / fx, 1.0]]),
                np.zeros(3), np.zeros(3), k, dist)
            out = CameraEffects({"lens": lens}).apply(dot_at(x, y), HFOV)
            got = brightest(out)
            self.assertAlmostEqual(got[0], want[0, 0, 0], delta=1.0, msg=(x, y))
            self.assertAlmostEqual(got[1], want[0, 0, 1], delta=1.0, msg=(x, y))

    def test_the_segmentation_follows_the_scene(self):
        lens = {"distortion_enabled": True, "distortion_k1": -0.3}
        scene, seg = dot_at(500, 80), dot_at(500, 80)
        effects = CameraEffects({"lens": lens})
        a = brightest(effects.apply(scene, HFOV))
        b = brightest(effects.apply_to_segmentation(seg, HFOV))
        self.assertAlmostEqual(a[0], b[0], delta=1.0)
        self.assertAlmostEqual(a[1], b[1], delta=1.0)

    def test_off_means_the_same_frame(self):
        img = dot_at(10, 10)
        effects = CameraEffects({})
        self.assertFalse(effects.active)
        self.assertIs(effects.apply(img, HFOV), img)
        self.assertIs(effects.apply_to_segmentation(img, HFOV), img)


class VibrationTests(unittest.TestCase):
    PROPS = {"platform": {"vibration_enabled": True, "vibration_amplitude_deg": 0.1,
                          "vibration_frequency_hz": 7.0}}

    def test_shift_is_in_pixels_for_the_field_of_view(self):
        effects = CameraEffects(self.PROPS)
        peak = max(abs(effects.vibration_px(t / 1000, HFOV, W)[0]) for t in range(1000))
        self.assertAlmostEqual(peak, 0.1 * W / HFOV, delta=0.01)

    def test_zooming_in_shakes_more(self):
        effects = CameraEffects(self.PROPS)
        wide = max(abs(effects.vibration_px(t / 1000, 80.0, W)[0]) for t in range(1000))
        tele = max(abs(effects.vibration_px(t / 1000, 8.0, W)[0]) for t in range(1000))
        self.assertAlmostEqual(tele / wide, 10.0, delta=0.1)

    def test_same_time_same_frame(self):
        img = dot_at(300, 100)
        a = CameraEffects(self.PROPS).apply(img, 10.0, 0.123)
        b = CameraEffects(self.PROPS).apply(img, 10.0, 0.123)
        self.assertTrue(np.array_equal(a, b))

    def test_it_moves_the_picture(self):
        img = dot_at(300, 100)
        effects = CameraEffects(self.PROPS)
        moved = [brightest(effects.apply(img, 10.0, t / 30)) for t in range(30)]
        xs = [m[0] for m in moved]
        self.assertGreater(max(xs) - min(xs), 3.0)

    def test_segmentation_moves_with_the_scene(self):
        effects = CameraEffects(self.PROPS)
        img = dot_at(300, 100)
        for t in (0.01, 0.07, 0.2):
            a = brightest(effects.apply(img, 10.0, t))
            b = brightest(effects.apply_to_segmentation(img, 10.0, t))
            self.assertAlmostEqual(a[0], b[0], delta=0.6)
            self.assertAlmostEqual(a[1], b[1], delta=0.6)


class NoiseTests(unittest.TestCase):
    def test_noise_level_follows_gain_and_temperature(self):
        self.assertAlmostEqual(noise_sigma_levels(0, 25), 1.0)
        self.assertAlmostEqual(noise_sigma_levels(20, 25), 10.0)
        self.assertAlmostEqual(noise_sigma_levels(0, 33), 2.0)

    def test_measured_noise_matches_the_model(self):
        props = {"sensor": {"noise_enabled": True, "noise_gain_db": 12.0,
                            "noise_temperature_c": 25.0}}
        effects = CameraEffects(props)
        flat = np.full((H, W, 3), 128, np.uint8)
        noise = effects.apply(flat, HFOV).astype(np.float32) - 128
        self.assertAlmostEqual(float(noise.std()), noise_sigma_levels(12, 25), delta=0.15)
        self.assertAlmostEqual(float(noise.mean()), 0.0, delta=0.1)

    def test_noise_can_be_turned_on_and_off_while_running(self):
        effects = CameraEffects({})
        flat = np.full((H, W, 3), 128, np.uint8)
        self.assertFalse(effects.active)
        effects.set_noise(True, gain_db=12.0)
        self.assertTrue(effects.active)
        self.assertAlmostEqual(float(effects.apply(flat, HFOV).astype(np.float32).std()),
                               noise_sigma_levels(12, 25), delta=0.15)
        effects.set_noise(True, gain_db=20.0)  # a new level, not the old pool's
        self.assertAlmostEqual(float(effects.apply(flat, HFOV).astype(np.float32).std()),
                               noise_sigma_levels(20, 25), delta=0.4)
        effects.set_noise(False)
        self.assertFalse(effects.active)
        self.assertIs(effects.apply(flat, HFOV), flat)
        effects.set_noise(True)  # back on at the gain it had
        self.assertAlmostEqual(effects.sigma, noise_sigma_levels(20, 25))

    def test_noise_changes_from_frame_to_frame(self):
        effects = CameraEffects({"sensor": {"noise_enabled": True}})
        flat = np.full((H, W, 3), 128, np.uint8)
        self.assertFalse(np.array_equal(effects.apply(flat, HFOV), effects.apply(flat, HFOV)))

    def test_noise_does_not_touch_the_segmentation(self):
        effects = CameraEffects({"sensor": {"noise_enabled": True}})
        seg = dot_at(5, 5)
        self.assertIs(effects.apply_to_segmentation(seg, HFOV), seg)

    def test_the_input_frame_is_left_alone(self):
        effects = CameraEffects({"sensor": {"noise_enabled": True, "noise_gain_db": 20.0}})
        flat = np.full((H, W, 3), 128, np.uint8)
        effects.apply(flat, HFOV)
        self.assertTrue((flat == 128).all())


class LiveSwitchingTests(unittest.TestCase):
    """The camera panel (F3) and `sim camera` turn effects on and off while
    frames are being made."""

    def test_distortion_on_off_and_new_coefficients(self):
        img = dot_at(560, 40)
        effects = CameraEffects({})
        effects.set_distortion(True, [-0.3])
        self.assertEqual(effects.distortion_coeffs, (-0.3, 0.0, 0.0, 0.0, 0.0))
        barrel = brightest(effects.apply(img, HFOV))
        self.assertGreater(math.dist(barrel, (560, 40)), 5)
        effects.set_distortion(True, [-0.1])  # a new map, not the cached one
        milder = brightest(effects.apply(img, HFOV))
        self.assertLess(math.dist(milder, (560, 40)), math.dist(barrel, (560, 40)))
        effects.set_distortion(False)
        self.assertIs(effects.apply(img, HFOV), img)
        self.assertEqual(effects.distortion_coeffs[0], -0.1)  # kept for turning it back on

    def test_vibration_keeps_its_settings_while_off(self):
        effects = CameraEffects(VibrationTests.PROPS)
        effects.set_vibration(False)
        self.assertEqual(effects.vibration_px(0.01, HFOV, W), (0.0, 0.0))
        effects.set_vibration(True)
        self.assertEqual(effects.vibration_deg, 0.1)
        effects.set_vibration(True, amplitude_deg=0.5, frequency_hz=3.0)
        self.assertEqual((effects.vibration_deg, effects.vibration_hz), (0.5, 3.0))

    def test_settings_report_everything(self):
        effects = CameraEffects({"sensor": {"noise_enabled": True, "noise_gain_db": 12.0}})
        s = effects.settings()
        self.assertTrue(s["noise"])
        self.assertFalse(s["distortion"] or s["vibration"])
        self.assertEqual(s["noise_gain_db"], 12.0)
        self.assertEqual(CameraProperties().settings()["resolution"], (1280, 720))


class PropertiesTests(unittest.TestCase):
    def test_resolution_names_and_sizes(self):
        self.assertEqual(parse_resolution("1080p"), (1920, 1080))
        self.assertEqual(parse_resolution("4K"), (3840, 2160))
        self.assertEqual(parse_resolution("1024x576"), (1024, 576))
        for bad in ("huge", "axb", ""):
            with self.assertRaises(ValueError):
                parse_resolution(bad)

    def test_the_effects_come_from_the_properties_file(self):
        import json
        import os
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "camera_properties.json")
            with open(path, "w") as f:
                json.dump({"platform": {"vibration_enabled": True},
                           "sensor": {"noise_enabled": True}}, f)
            camera = CameraProperties(path)
        self.assertTrue(camera.effects.active)
        self.assertIn("vibration", camera.describe())
        self.assertFalse(CameraProperties().effects.active)


if __name__ == "__main__":
    unittest.main()
