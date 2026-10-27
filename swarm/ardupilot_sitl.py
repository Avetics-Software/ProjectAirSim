"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Starts and stops ArduPilot SITL (e.g. ArduPlane) for one simulator scene, as
px4_sitl.Px4Sitl does for PX4: the same interface, so a swarm can mix them.

ArduPilot talks to the sim over its JSON interface (the robot's arduplane-api
controller, UDP 9002+10*I for instance I) and starts afresh (-w) every time, so
parameters come only from the model's files (aircraft/models.yml), and no
mission or fence survives from an earlier session. Its MAVLink links are the
PX4 drones' equivalents, so the ground station, QGC and the sim's MAVLink
camera (payload_mavlink.py) find drone N (0-based) where they would find a PX4:

  SERIAL1  udpclient 127.0.0.1:14030+N   payload link: ground station + camera
  SERIAL2  udpclient 127.0.0.1:14550     QGC (it listens there)
  SERIAL5+ udpclient <companion>:<port>  the drone's companion computer, if any
  SERIAL0  TCP 5760+10*I                 MAVProxy or a GCS, if wanted

(SERIAL3 and 4 are SITL's simulated GPSs.)

Environment: ARDUPILOT_DIR (default ~/Documents/ardupilot, for the binary),
PI_PORTS and LOG_DIR as for PX4.
"""

import os
import subprocess
import time

from projectairsim.utils import projectairsim_log

from payload_mavlink import PAYLOAD_PORT

QGC_PORT = 14550
COMPANION_SERIALS = (5, 6)  # SERIALn for the companion computer's ports


class ArduPilotSitl:
    def __init__(self, spec, pi_ip=""):
        """spec: aircraft.Sitl (binary, parameter files, instance)."""
        self.spec = spec
        self.instance = spec.instance
        self.pi_ip = pi_ip
        self.pi_ports = [p for p in os.environ.get("PI_PORTS", "15540,15541").split(",") if p]
        self.log_dir = os.environ.get("LOG_DIR", "/tmp/projectairsim-run")
        self.workdir = os.path.join(self.log_dir, f"ardupilot_{self.instance}")
        self.log_path = os.path.join(self.log_dir, f"ardupilot_{self.instance}.log")
        self.pid_path = os.path.join(self.log_dir, f"ardupilot_{self.instance}.pid")
        self.home = None  # (lat, lon, alt MSL, heading): the scene's home
        self.proc = None

    def _instance_params(self):
        """The parameters that differ per drone, as a file after the model's."""
        lines = [
            # Its ground station is the payload link, which speaks as the drone's
            # own system id (payload_mavlink.payload_connection)
            f"MAV_GCS_SYSID {self.spec.sysid}",
        ]
        if self.pi_ip:
            lines += [f"SERIAL{n}_PROTOCOL 2" for n in COMPANION_SERIALS[:len(self.pi_ports)]]
        path = os.path.join(self.workdir, "instance.parm")
        with open(path, "w") as f:
            f.write("# Written by ardupilot_sitl.py for this drone; overwritten at each start\n")
            f.write("\n".join(lines) + "\n")
        return path

    def command(self):
        if self.home is None:
            raise RuntimeError("ArduPilot SITL needs the scene's home before it starts")
        lat, lon, alt, heading = self.home
        defaults = self.spec.params + [self._instance_params()]
        cmd = [
            self.spec.binary, "-w", "--model", "JSON", "--speedup", "1",
            "-I", str(self.instance), "--sysid", str(self.spec.sysid),
            "--home", f"{lat},{lon},{alt},{heading}",
            "--defaults", ",".join(defaults),
            # Without this, SITL waits for something to connect to SERIAL0
            "--serial0", "tcp:0",
            "--serial1", f"udpclient:127.0.0.1:{PAYLOAD_PORT + self.spec.index}",
            "--serial2", f"udpclient:127.0.0.1:{QGC_PORT}",
        ]
        if self.pi_ip:
            for n, port in zip(COMPANION_SERIALS, self.pi_ports):
                cmd += [f"--serial{n}", f"udpclient:{self.pi_ip}:{port}"]
        return cmd

    def start(self, timeout=30):
        """Start ArduPilot and wait until its JSON interface is up."""
        if not os.path.exists(self.spec.binary):
            raise RuntimeError(f"No ArduPilot SITL binary at {self.spec.binary}; build it "
                               "(./waf configure --board sitl && ./waf plane) or set ARDUPILOT_DIR")
        os.makedirs(self.workdir, exist_ok=True)
        log = open(self.log_path, "w")
        cmd = self.command()
        log.write(" ".join(cmd) + "\n")
        log.flush()
        # Its working directory takes the eeprom, logs and terrain cache
        self.proc = subprocess.Popen(cmd, cwd=self.workdir, stdout=log, stderr=subprocess.STDOUT)
        with open(self.pid_path, "w") as f:
            f.write(str(self.proc.pid))

        deadline = time.monotonic() + timeout
        while not self._log_has("JSON control interface set to"):
            if self.proc.poll() is not None:
                raise RuntimeError(f"ArduPilot exited during startup; see {self.log_path}")
            if time.monotonic() > deadline:
                raise RuntimeError(f"ArduPilot did not start within {timeout}s; see {self.log_path}")
            time.sleep(0.5)
        projectairsim_log().info(
            f"ArduPilot SITL {self.instance} started ({self.spec.model}, system id "
            f"{self.spec.sysid}); log: {self.log_path}"
        )
        if self.pi_ip:
            projectairsim_log().info(
                f"MAVLink to companion computer {self.pi_ip} on UDP {', '.join(self.pi_ports)}"
            )

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self.proc = None

    def restart_modules(self, modules=None, timeout=15):
        """PX4 can restart its simulated hardware; ArduPilot SITL has no
        equivalent short of a restart (goto). Returns what failed."""
        return ["ArduPilot SITL can't restart its simulated hardware; use goto to restart it"]

    def _log_has(self, text):
        try:
            with open(self.log_path) as f:
                return text in f.read()
        except FileNotFoundError:
            return False
