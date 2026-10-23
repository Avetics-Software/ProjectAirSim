"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

The simulated gimbal payload's MAVLink camera, on PX4's onboard payload link
(PX4 instance N sends to UDP 14030+N; its system id is N+1).

A real drone's camera announces itself to QGroundControl; the sim's GimbalCam
can't, so this does it: a MAVLink camera component (MAV_COMP_ID_CAMERA) that
points QGC at the RTSP video and turns its zoom controls into zoom requests
for the sim. PX4 forwards messages between this link and QGC's.

The flight, gimbal and telemetry side of the same link is the ground
station's gcs.mavlink_link.VehicleLink, which flies real drones the same way.
"""

import math
import queue
import time

from projectairsim.utils import projectairsim_log

from gcs.mavlink_link import CAMERA_COMPID, M, MavConnection

PAYLOAD_PORT = 14030  # + PX4 instance
TRACKER_COMPID = M.MAV_COMP_ID_ONBOARD_COMPUTER
GIMBAL_DEVICE_COMPID = M.MAV_COMP_ID_GIMBAL


def payload_connection(instance):
    """PX4 instance N's payload link. We take PX4's system id there, as part of
    that vehicle, and the onboard-computer component, like a companion computer."""
    return MavConnection(
        f"udpin:0.0.0.0:{PAYLOAD_PORT + instance}", source_system=instance + 1,
        source_component=TRACKER_COMPID, heartbeat_type=M.MAV_TYPE_ONBOARD_CONTROLLER,
    )


class SimCamera:
    """The sim GimbalCam's MAVLink camera component, for drone `instance`.

    Zoom requests (from QGC, or a companion computer through PX4) are queued in
    `zoom_requests` as ("range", percent) or ("continuous", -1/0/1) for the main
    thread to apply to the sim, since the sim client must not be used from the
    connection's reader thread. Photo requests go to `capture_requests`.
    """

    def __init__(self, conn, instance, video_uri, camera):
        self.conn = conn
        self.instance = instance
        self.video_uri = video_uri
        self.camera = camera  # CameraProperties
        self.zoom_percent = 0.0
        self._published_zooms = []  # (time.monotonic(), percent) reported in the last second
        self.zoom_requests = queue.Queue()
        self.capture_requests = queue.Queue()
        self.image_count = 0
        conn.add_heartbeat(CAMERA_COMPID, M.MAV_TYPE_CAMERA)
        conn.subscribe(self._on_message)

    def publish_zoom(self, zoom_percent):
        """Report the current zoom to QGC (0 = widest, 100 = most zoomed)."""
        self.zoom_percent = zoom_percent
        now = time.monotonic()
        self._published_zooms = [(t, z) for t, z in self._published_zooms if now - t < 1.0]
        self._published_zooms.append((now, zoom_percent))
        self._send_camera_settings()

    def _is_zoom_echo(self, percent):
        """QGC's zoom slider follows CAMERA_SETTINGS and sends the level back as
        a range zoom, which would stop a continuous zoom (from the companion or
        the keys) a few ms after it starts. A level we reported in the last
        second is that echo, not a request."""
        now = time.monotonic()
        return any(now - t < 1.0 and abs(z - percent) < 0.5 for t, z in self._published_zooms)

    def report_image_captured(self, path):
        self.image_count += 1
        self._send(
            lambda mav: mav.camera_image_captured_send(
                self.conn.ms(), int(time.time() * 1e6), 0, 0, 0, 0, 0,
                [1.0, 0.0, 0.0, 0.0], self.image_count - 1, 1,
                path.encode()[:205],
            ),
        )

    def _send(self, send):
        self.conn.send_as(CAMERA_COMPID, send)

    def _on_message(self, msg):
        if msg.get_type() == "COMMAND_LONG" and msg.target_component == CAMERA_COMPID:
            self._handle_camera_command(msg)

    def _ack(self, msg, result):
        self._send(
            lambda mav: mav.command_ack_send(
                msg.command, result, 0, 0, msg.get_srcSystem(), msg.get_srcComponent()
            ),
        )

    def _handle_camera_command(self, msg):
        cmd = msg.command
        projectairsim_log().debug(
            f"Camera {self.instance + 1}: command {cmd} ({msg.param1:g}, {msg.param2:g}) "
            f"from {msg.get_srcSystem()}/{msg.get_srcComponent()}"
        )
        accepted, unsupported = M.MAV_RESULT_ACCEPTED, M.MAV_RESULT_UNSUPPORTED

        if cmd == M.MAV_CMD_REQUEST_MESSAGE:
            requested = int(msg.param1)
            senders = {
                M.MAVLINK_MSG_ID_CAMERA_INFORMATION: self._send_camera_information,
                M.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION: self._send_video_stream_information,
                M.MAVLINK_MSG_ID_CAMERA_SETTINGS: self._send_camera_settings,
                M.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS: self._send_capture_status,
            }
            sender = senders.get(requested)
            self._ack(msg, accepted if sender else unsupported)
            if sender:
                sender()
        elif cmd == M.MAV_CMD_REQUEST_CAMERA_INFORMATION:
            self._ack(msg, accepted)
            self._send_camera_information()
        elif cmd == M.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
            self._ack(msg, accepted)
            self._send_video_stream_information()
        elif cmd == M.MAV_CMD_REQUEST_CAMERA_SETTINGS:
            self._ack(msg, accepted)
            self._send_camera_settings()
        elif cmd == M.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
            self._ack(msg, accepted)
            self._send_capture_status()
        elif cmd == M.MAV_CMD_SET_CAMERA_ZOOM:
            zoom_type, value = int(msg.param1), msg.param2
            if zoom_type == M.ZOOM_TYPE_RANGE and self._is_zoom_echo(value):
                pass  # QGC's slider sending back a level we just reported
            elif zoom_type == M.ZOOM_TYPE_RANGE:
                self.zoom_requests.put(("range", max(0.0, min(100.0, value))))
            elif zoom_type == M.ZOOM_TYPE_CONTINUOUS:
                self.zoom_requests.put(("continuous", int(math.copysign(1, value)) if value else 0))
            elif zoom_type == M.ZOOM_TYPE_STEP:
                self.zoom_requests.put(("range", max(0.0, min(100.0, self.zoom_percent + 10 * value))))
            else:
                self._ack(msg, unsupported)
                return
            self._ack(msg, accepted)
        elif cmd == M.MAV_CMD_IMAGE_START_CAPTURE:
            self.capture_requests.put(True)
            self._ack(msg, accepted)
        elif cmd == M.MAV_CMD_SET_CAMERA_MODE:
            self._ack(msg, accepted)
        else:
            self._ack(msg, unsupported)

    def _send_camera_information(self):
        flags = (
            M.CAMERA_CAP_FLAGS_CAPTURE_IMAGE
            | M.CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM
            | M.CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM
        )
        self._send(
            lambda mav: mav.camera_information_send(
                self.conn.ms(),
                list(b"ProjectAirSim".ljust(32, b"\0")),
                list(self.camera.name.encode()[:32].ljust(32, b"\0")),
                0,
                self.camera.focal_min_mm,
                self.camera.sensor_width_mm,
                self.camera.sensor_height_mm,
                self.camera.resolution[0],
                self.camera.resolution[1],
                0,
                flags,
                0,
                b"",
                GIMBAL_DEVICE_COMPID,
            ),
        )

    def _send_video_stream_information(self):
        self._send(
            lambda mav: mav.video_stream_information_send(
                1,
                1,
                M.VIDEO_STREAM_TYPE_RTSP,
                M.VIDEO_STREAM_STATUS_FLAGS_RUNNING,
                30.0,
                self.camera.resolution[0],
                self.camera.resolution[1],
                4_000_000,
                0,
                int(self.camera.hfov_deg()),
                b"GimbalCam",
                self.video_uri.encode(),
                M.VIDEO_STREAM_ENCODING_H264,
            ),
        )

    def _send_camera_settings(self):
        self._send(
            lambda mav: mav.camera_settings_send(
                self.conn.ms(), M.CAMERA_MODE_VIDEO, self.zoom_percent, math.nan
            ),
        )

    def _send_capture_status(self):
        self._send(
            lambda mav: mav.camera_capture_status_send(
                self.conn.ms(), 0, 0, 0.0, 0, math.nan, self.image_count
            ),
        )
