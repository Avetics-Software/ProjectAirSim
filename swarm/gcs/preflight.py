"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Checks before flying real drones (run_real.sh runs this first):

  python -m gcs.preflight --config config.yml [--listen 6]

- the config is valid for real drones (a MAVLink link for every drone, unique
  system ids)
- the MAVLink link can be opened: a UDP port free to listen on, a serial
  radio present
- every drone's PX4 is heard on it (listens --listen seconds for heartbeats)
- each drone's video stream, and each companion (ping, and X's tracker video)

Exits 1 only when the ground station couldn't start (a bad config, a busy
port, a missing radio); everything else is a warning, so you can start before
every drone is powered on.
"""

import argparse
import os
import shutil
import socket
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

from .companion import TRACKER_PORT  # noqa: E402
from .fleet import check_for_real_drones, load_fleet  # noqa: E402

OK, WARN, FAIL = "ok  ", "WARN", "FAIL"


def check_link(url):
    """(status, text): can the ground station open this MAVLink link?"""
    kind, _, rest = url.partition(":")
    if kind == "udpin":
        host, _, port = rest.rpartition(":")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((host or "0.0.0.0", int(port)))
        except OSError as err:
            return FAIL, f"{url}: can't listen on UDP {port} ({err.strerror}); is QGC or " \
                         "another ground station using it?"
        finally:
            sock.close()
        return OK, f"{url}: UDP {port} free"
    if url.startswith("/dev/"):
        device = url.split(",")[0]
        if not os.path.exists(device):
            return FAIL, f"{url}: no such device {device}; is the radio plugged in?"
        return OK, f"{url}: {device} present"
    return OK, f"{url}: (not checked)"


def listen_for_drones(url, sysids, seconds):
    """System ids heard (PX4 heartbeats) on a link within `seconds`."""
    heard = set()
    try:
        mav = mavutil.mavlink_connection(url, source_system=254, source_component=190)
    except Exception:
        return heard
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline and not sysids <= heard:
            msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if msg and msg.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                heard.add(msg.get_srcSystem())
    finally:
        mav.close()
    return heard


def check_rtsp(url, timeout=6):
    """True if an RTSP stream answers with a video stream."""
    if not shutil.which("ffprobe"):
        return None
    try:
        out = subprocess.run(
            ["ffprobe", "-v", "error", "-rtsp_transport", "tcp", "-select_streams", "v:0",
             "-show_entries", "stream=width,height", "-of", "csv=p=0", url],
            capture_output=True, text=True, timeout=timeout,
        )
        return out.stdout.strip() or False
    except subprocess.TimeoutExpired:
        return False


def ping(host):
    try:
        return subprocess.run(["ping", "-c", "1", "-W", "1", host],
                              capture_output=True).returncode == 0
    except FileNotFoundError:
        return None


def main():
    parser = argparse.ArgumentParser(prog="python -m gcs.preflight", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", required=True)
    parser.add_argument("--listen", type=float, default=6.0,
                        help="seconds to listen for the drones' heartbeats (default 6)")
    args = parser.parse_args()

    results = []  # (status, text)
    try:
        fleet = load_fleet(args.config)
        check_for_real_drones(fleet, args.config)
    except (OSError, ValueError) as err:
        print(f"{FAIL}  config: {err}")
        return 1
    results.append((OK, f"config {args.config}: {len(fleet.drones)} drones"))

    links = {}
    for d in fleet.drones:
        links.setdefault(fleet.link_url(d), []).append(d)
    fatal = False
    for url, drones in links.items():
        status, text = check_link(url)
        results.append((status, text))
        if status == FAIL:
            fatal = True
            continue
        heard = listen_for_drones(url, {d.sysid for d in drones}, args.listen)
        for d in drones:
            results.append((OK, f"{d.name}: PX4 (system {d.sysid}) heard on {url}") if d.sysid in heard
                           else (WARN, f"{d.name}: no heartbeat from system {d.sysid} on {url} "
                                 f"in {args.listen:g} s (powered on? MAV_SYS_ID {d.sysid}?)"))

    def drone_checks(d):
        out = []
        if d.video:
            found = check_rtsp(d.video)
            out.append((OK, f"{d.name}: video {d.video} ({found})") if found
                       else (WARN, f"{d.name}: no video at {d.video}") if found is False
                       else (WARN, f"{d.name}: video not checked (no ffprobe)"))
        elif not d.companion:
            out.append((WARN, f"{d.name}: no video: and no companion, so no camera on the dashboard"))
        if d.companion:
            up = ping(d.companion)
            out.append((OK, f"{d.name}: companion {d.companion} answers ping") if up
                       else (WARN, f"{d.name}: companion {d.companion} doesn't answer ping"))
            tracker = f"rtsp://{d.companion}:{TRACKER_PORT}/tracker"
            found = check_rtsp(tracker)
            out.append((OK, f"{d.name}: X's tracker video up ({found})") if found
                       else (WARN, f"{d.name}: X's tracker video not up yet ({tracker}); "
                             "is X running?"))
        return out

    with ThreadPoolExecutor(max_workers=8) as pool:
        for out in pool.map(drone_checks, fleet.drones):
            results.extend(out)

    for status, text in results:
        print(f"{status}  {text}")
    warnings = sum(1 for s, _ in results if s == WARN)
    if fatal:
        print("The ground station can't start; fix the FAIL lines above.")
        return 1
    print(f"Preflight: {warnings} warning{'s' if warnings != 1 else ''}" if warnings
          else "Preflight: all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
