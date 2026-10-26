"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

The swarm ground station for real drones:

  python -m gcs --fleet fleet.yml

Same dashboard and commands as the sim (px4_gimbal_cesium.py), with the drones
reached over the MAVLink links, video streams and companion computers listed
in the fleet file (see gcs/fleet.example.yml). Ctrl+C to stop.
"""

import argparse
import asyncio
import logging
import os
import signal

from .fleet import check_for_real_drones, load_fleet
from .log import log
from .mavlink_link import M, MavConnection, VehicleLink
from .station import DroneCamera, GroundStation


def parse_args():
    parser = argparse.ArgumentParser(
        prog="python -m gcs", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--fleet", required=True, help="fleet file (see gcs/fleet.example.yml)")
    parser.add_argument("--log-file", default="gcs.log",
                        help="debug log, with every message (default gcs.log)")
    parser.add_argument("--verbose", action="store_true",
                        help="also print debug messages in the terminal")
    return parser.parse_args()


def setup_logging(log_file, verbose):
    log.setLevel(logging.DEBUG)
    terminal = logging.StreamHandler()
    terminal.setLevel(logging.DEBUG if verbose else logging.INFO)
    terminal.setFormatter(logging.Formatter("%(asctime)s %(message)s", datefmt="%H:%M:%S"))
    log.addHandler(terminal)
    if log_file:
        to_file = logging.FileHandler(log_file)
        to_file.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
        log.addHandler(to_file)


async def main():
    args = parse_args()
    setup_logging(args.log_file, args.verbose)
    fleet = load_fleet(args.fleet)
    check_for_real_drones(fleet, args.fleet)

    # One connection per distinct link; drones on a shared link share it
    connections = {}
    for drone in fleet.drones:
        url = fleet.link_url(drone)
        if url not in connections:
            connections[url] = MavConnection(url, fleet.gcs_sysid, fleet.gcs_compid, M.MAV_TYPE_GCS)
            log.info(f"MAVLink {url} as system {fleet.gcs_sysid}/{fleet.gcs_compid}")
    links = [VehicleLink(connections[fleet.link_url(d)], d.sysid, d.name) for d in fleet.drones]
    specs = {link.name: d for link, d in zip(links, fleet.drones)}

    station = GroundStation(
        links,
        lambda i, link, windows: DroneCamera(
            link, windows, specs[link.name].video, specs[link.name].chase_video
        ),
        companions=fleet.companions,
        home=fleet.home,
        config=fleet,
    )
    station.start()
    log.info(f"Ground station up for {', '.join(d.name for d in fleet.drones)}; "
             "type 'help' for commands. Ctrl+C to stop.")

    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGINT, stop.set)
    loop.add_signal_handler(signal.SIGTERM, stop.set)
    try:
        await stop.wait()
    finally:
        station.close()
        for conn in connections.values():
            conn.close()


if __name__ == "__main__":
    asyncio.run(main())
    os._exit(0)  # don't wait on the console thread blocked reading stdin
