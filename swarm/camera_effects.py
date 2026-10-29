"""
Image defects of a real payload camera, added to the sim's clean frames:
lens distortion, platform vibration and sensor noise. They are set in
camera_properties.json (lens.distortion_*, platform.vibration_*,
sensor.noise_*) and applied wherever the frames leave the sim: the RTSP stream
and the dashboard, and the experiment and ground-truth recordings.

Distortion and vibration move pixels, so a segmentation image is moved the same
way (nearest neighbour) and the boxes taken from it stay on the objects.
"""

import math

import cv2
import numpy as np

# Segmentation background (ID 0) in BGR, for the corners a distortion leaves empty
SEG_BACKGROUND_BGR = (57, 181, 55)

GRID_STEP_PX = 16  # the distortion map is worked out on this grid, then interpolated
NOISE_PAD_PX = 64  # the noise pool is this much larger than a frame, to crop at random


def noise_sigma_levels(gain_db, temperature_c):
    """Noise (standard deviation, in 8-bit levels) of the sensor model: 1 level
    at 0 dB and 25 C, up with gain, and doubling every 8 C like dark current.
    An indicative model, not a calibrated one."""
    return 10.0 ** (gain_db / 20.0) * 2.0 ** ((temperature_c - 25.0) / 8.0)


class CameraEffects:
    """The enabled defects of one camera. `apply()` takes a frame and the
    camera's current horizontal field of view, which sets how many pixels a
    vibration of a given angle moves it, so zooming in shakes the picture more,
    as it does on the real camera."""

    def __init__(self, props, seed=0):
        """props: the camera_properties.json dict."""
        lens, platform, sensor = (props.get(k, {}) for k in ("lens", "platform", "sensor"))
        self.distortion = None
        if lens.get("distortion_enabled"):
            # OpenCV's model: a point of the ideal image lands at the distorted one
            self.distortion = np.array([float(lens.get(k, 0.0)) for k in
                                        ("distortion_k1", "distortion_k2", "distortion_p1",
                                         "distortion_p2", "distortion_k3")])
        self.vibration_deg = self.vibration_hz = 0.0
        if platform.get("vibration_enabled"):
            self.vibration_deg = float(platform.get("vibration_amplitude_deg", 0.01))
            self.vibration_hz = float(platform.get("vibration_frequency_hz", 30.0))
        self.gain_db = float(sensor.get("noise_gain_db", 6.0))
        self.temperature_c = float(sensor.get("noise_temperature_c", 25.0))
        self.sigma = 0.0
        if sensor.get("noise_enabled"):
            self.sigma = noise_sigma_levels(self.gain_db, self.temperature_c)
        rng = np.random.default_rng(seed)
        self._phase = rng.uniform(0.0, 2.0 * math.pi, 2)
        self._rng = rng
        self._maps = {}  # (width, height, hfov) -> (map_x, map_y) of the distortion
        self._pool = None  # noise, as int16 levels, larger than a frame

    @property
    def active(self):
        return self.distortion is not None or self.vibration_deg > 0.0 or self.sigma > 0.0

    def describe(self):
        parts = []
        if self.distortion is not None:
            parts.append("lens distortion k1=%g k2=%g p1=%g p2=%g k3=%g" % tuple(self.distortion))
        if self.vibration_deg > 0.0:
            parts.append(f"vibration {self.vibration_deg:g} deg at {self.vibration_hz:g} Hz")
        if self.sigma > 0.0:
            parts.append(f"sensor noise {self.sigma:.1f} levels")
        return ", ".join(parts) or "none"

    def set_noise(self, enabled, gain_db=None, temperature_c=None):
        """Turn the sensor noise on or off, optionally at another gain or
        temperature (kept for the next time it is turned on)."""
        if gain_db is not None:
            self.gain_db = float(gain_db)
        if temperature_c is not None:
            self.temperature_c = float(temperature_c)
        self._pool = None
        self.sigma = noise_sigma_levels(self.gain_db, self.temperature_c) if enabled else 0.0

    def vibration_px(self, t_sec, hfov_deg, width):
        """The (dx, dy) shift in pixels at sim time t_sec: the camera shaking
        by `vibration_deg`, at `vibration_hz` sideways and a little off that
        rate up and down."""
        if self.vibration_deg <= 0.0:
            return 0.0, 0.0
        px_per_deg = width / hfov_deg
        amplitude = self.vibration_deg * px_per_deg
        w = 2.0 * math.pi * self.vibration_hz
        return (amplitude * math.sin(w * t_sec + self._phase[0]),
                amplitude * math.sin(1.31 * w * t_sec + self._phase[1]))

    def _distortion_map(self, width, height, hfov_deg):
        """Where each pixel of the distorted image comes from in the ideal one.
        Worked out by undistorting a coarse grid, then interpolated up to a
        full-size map; the lens model only ever goes ideal -> distorted."""
        key = (width, height, round(hfov_deg, 1))
        cached = self._maps.get(key)
        if cached is not None:
            return cached
        fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        cx, cy = (width - 1) / 2.0, (height - 1) / 2.0
        k = np.array([[fx, 0, cx], [0, fx, cy], [0, 0, 1]])
        gw, gh = (width - 1) // GRID_STEP_PX + 2, (height - 1) // GRID_STEP_PX + 2
        us, vs = np.linspace(0, width - 1, gw), np.linspace(0, height - 1, gh)
        grid = np.stack(np.meshgrid(us, vs), -1).reshape(-1, 1, 2)
        ideal = cv2.undistortPoints(
            grid, k, self.distortion,
            criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, 1e-9),
        ).reshape(gh, gw, 2)
        coarse_x = (ideal[..., 0] * fx + cx).astype(np.float32)
        coarse_y = (ideal[..., 1] * fx + cy).astype(np.float32)
        # Interpolate with the grid's own coordinates, so its ends meet the image's
        gx = np.tile(np.linspace(0, gw - 1, width, dtype=np.float32), (height, 1))
        gy = np.tile(np.linspace(0, gh - 1, height, dtype=np.float32)[:, None], (1, width))
        maps = (cv2.remap(coarse_x, gx, gy, cv2.INTER_LINEAR),
                cv2.remap(coarse_y, gx, gy, cv2.INTER_LINEAR))
        if len(self._maps) > 32:  # zooming sweeps through many field-of-views
            self._maps.clear()
        self._maps[key] = maps
        return maps

    def _warp(self, image, hfov_deg, shift, interpolation, border):
        h, w = image.shape[:2]
        dx, dy = shift
        if self.distortion is not None:
            map_x, map_y = self._distortion_map(w, h, hfov_deg)
            if dx or dy:
                map_x, map_y = map_x - dx, map_y - dy
            return cv2.remap(image, map_x, map_y, interpolation,
                             borderMode=cv2.BORDER_CONSTANT, borderValue=border)
        if dx or dy:
            # Edge pixels are repeated rather than blacked out: the real scene
            # carries on past the edge of the frame
            return cv2.warpAffine(image, np.float32([[1, 0, -dx], [0, 1, -dy]]), (w, h),
                                  flags=interpolation | cv2.WARP_INVERSE_MAP,
                                  borderMode=cv2.BORDER_REPLICATE)
        return image

    def _noise(self, shape):
        h, w = shape[:2]
        pool = self._pool
        if pool is None or pool.shape[0] < h + NOISE_PAD_PX or pool.shape[1] < w + NOISE_PAD_PX:
            noise = self._rng.standard_normal((h + NOISE_PAD_PX, w + NOISE_PAD_PX, 3),
                                              dtype=np.float32)
            self._pool = pool = np.rint(noise * self.sigma).astype(np.int16)
        oy = int(self._rng.integers(0, pool.shape[0] - h + 1))
        ox = int(self._rng.integers(0, pool.shape[1] - w + 1))
        crop = pool[oy:oy + h, ox:ox + w]
        return cv2.flip(crop, int(self._rng.integers(-1, 2)))

    def apply(self, bgr, hfov_deg, t_sec=0.0):
        """The scene image with the defects on. A new array; `bgr` is not changed."""
        if not self.active:
            return bgr
        out = self._warp(bgr, hfov_deg, self.vibration_px(t_sec, hfov_deg, bgr.shape[1]),
                         cv2.INTER_LINEAR, (0, 0, 0))
        if self.sigma > 0.0:
            out = cv2.add(out, self._noise(out.shape), dtype=cv2.CV_8U)
        return out

    def apply_to_segmentation(self, seg_bgr, hfov_deg, t_sec=0.0):
        """A segmentation image moved as `apply()` moves the scene image (it has
        no noise), so what is labelled stays where it is seen."""
        if self.distortion is None and self.vibration_deg <= 0.0:
            return seg_bgr
        return self._warp(seg_bgr, hfov_deg,
                          self.vibration_px(t_sec, hfov_deg, seg_bgr.shape[1]),
                          cv2.INTER_NEAREST, SEG_BACKGROUND_BGR)
