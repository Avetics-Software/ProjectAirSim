"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Starts and stops PX4 SITL for one simulator scene. PX4 SITL can only connect to
one scene, so moving to a new location means loading the new scene and then
starting a fresh PX4 for it.

Each drone of a swarm has its own PX4 instance (`px4 -i N`). PX4 offsets its
ports by N (simulator TCP 4560+N, QGC link 18570+N, payload link to 14030+N,
gimbal link to 13280+N) and uses MAVLink system id N+1. Instance 0 runs in
PX4's rootfs as before; instance N in build/px4_sitl_default/instance_N.

Settings come from the environment (set by run_cesium_sim.sh):
  PX4_DIR, PX4_MODEL       PX4-Autopilot checkout and SITL model [none_iris]
  PX4_PARAM_<NAME>=value   parameters PX4 applies at boot
  PI_PORTS                 companion-computer MAVLink ports [15540,15541]; each
                           drone's companion IP comes from the launcher (--fleet,
                           or PI_IP for the first drone)
  LOG_DIR                  where px4.log and px4.pid go (px4_N.* for instance N)
  PX4_KEEP_LOGS            PX4 flight logs (.ulg) to keep, counting the new one [5]
"""

import glob
import os
import subprocess
import threading
import time

from projectairsim.utils import projectairsim_log


class Px4Sitl:
    def __init__(self, instance=0, pi_ip=None):
        self.instance = instance
        self.px4_dir = os.path.expanduser(os.environ.get("PX4_DIR", "~/Documents/PX4-Autopilot"))
        self.model = os.environ.get("PX4_MODEL", "none_iris")
        # MAVLink links to this drone's companion computer, if it has one
        if pi_ip is None:
            pi_ip = os.environ.get("PI_IP", "") if instance == 0 else ""
        self.pi_ip = pi_ip
        self.pi_ports = [p for p in os.environ.get("PI_PORTS", "15540,15541").split(",") if p]
        self.log_dir = os.environ.get("LOG_DIR", "/tmp/projectairsim-run")
        build = os.path.join(self.px4_dir, "build", "px4_sitl_default")
        self.bin = os.path.join(build, "bin", "px4")
        self.etc = os.path.join(build, "etc")
        self.rootfs = os.path.join(build, "rootfs" if instance == 0 else f"instance_{instance}")
        self.keep_logs = int(os.environ.get("PX4_KEEP_LOGS", "5"))
        suffix = "" if instance == 0 else f"_{instance}"
        self.log_path = os.path.join(self.log_dir, f"px4{suffix}.log")
        self.pid_path = os.path.join(self.log_dir, f"px4{suffix}.pid")
        self.proc = None

    def start(self, timeout=30):
        """Start PX4 and wait until it is trying to reach the simulator."""
        self.prune_flight_logs(self.keep_logs - 1)  # PX4 starts a new one
        os.makedirs(self.log_dir, exist_ok=True)
        os.makedirs(self.rootfs, exist_ok=True)
        log = open(self.log_path, "w")
        env = dict(os.environ, PX4_SIM_MODEL=self.model)
        self.proc = subprocess.Popen(
            [self.bin, "-i", str(self.instance), "-d", self.etc, "-s", "etc/init.d-posix/rcS"],
            cwd=self.rootfs, env=env, stdout=log, stderr=subprocess.STDOUT,
        )
        with open(self.pid_path, "w") as f:
            f.write(str(self.proc.pid))

        deadline = time.monotonic() + timeout
        while not self._log_has("Waiting for simulator"):
            if self.proc.poll() is not None:
                raise RuntimeError(f"PX4 exited during startup; see {self.log_path}")
            if time.monotonic() > deadline:
                raise RuntimeError(f"PX4 did not start within {timeout}s; see {self.log_path}")
            time.sleep(0.5)
        projectairsim_log().info(
            f"PX4 SITL {self.instance} started ({self.model}); log: {self.log_path}"
        )

        if self.pi_ip:
            threading.Thread(target=self._add_companion_links, daemon=True).start()

    def prune_flight_logs(self, keep):
        """Delete all but the newest `keep` PX4 flight logs (rootfs/log/<date>/*.ulg)."""
        log_root = os.path.join(self.rootfs, "log")
        if not os.path.isdir(log_root):
            return
        logs = sorted(glob.glob(os.path.join(log_root, "*", "*.ulg")),
                      key=os.path.getmtime, reverse=True)
        old = logs[max(keep, 0):]
        freed = 0
        for path in old:
            freed += os.path.getsize(path)
            os.remove(path)
        for day in os.listdir(log_root):
            day_dir = os.path.join(log_root, day)
            if os.path.isdir(day_dir) and not os.listdir(day_dir):
                os.rmdir(day_dir)
        if old:
            projectairsim_log().info(
                f"Removed {len(old)} old PX4 flight logs ({freed / 1e6:.0f} MB); "
                f"keeping the newest {self.keep_logs}"
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

    def _log_has(self, text):
        try:
            with open(self.log_path) as f:
                return text in f.read()
        except FileNotFoundError:
            return False

    def _add_companion_links(self):
        # PX4 starts its own MAVLink links once the sim connects; add the
        # companion-computer links after the last of them (the gimbal link).
        # Local ports 14590+10N.., so several drones' PX4s don't collide.
        proc = self.proc
        while not self._log_has("mode: Gimbal"):
            if proc is not self.proc or proc.poll() is not None:
                return
            time.sleep(0.5)
        with open(self.log_path, "a") as log:
            for i, port in enumerate(self.pi_ports):
                subprocess.run(
                    [self.bin + "-mavlink", "--instance", str(self.instance), "start", "-x", "-u", str(14590 + 10 * self.instance + i),
                     "-r", "200000", "-m", "onboard", "-t", self.pi_ip, "-o", port, "-f"],
                    cwd=self.rootfs, stdout=log, stderr=subprocess.STDOUT,
                )
        projectairsim_log().info(
            f"MAVLink to companion computer {self.pi_ip} on UDP {', '.join(self.pi_ports)}"
        )
