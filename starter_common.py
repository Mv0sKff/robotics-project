#!/usr/bin/env python3
"""Gemeinsame Helfer fuer die Robotik-Starter."""

from __future__ import annotations

import os
import shlex
import signal
import subprocess
import time
import textwrap
from pathlib import Path
from typing import Callable

ROS_DISTRO = "jazzy"
PACKAGE_NAME = "robotik_projekt"
ROBOT_MODEL = "iiwa14"
NAMESPACE = "/lbr"
HARDWARE_CTRL = "joint_trajectory_controller"

WAIT_AFTER_MOCK = 5
WAIT_AFTER_MOVEIT = 8
WAIT_AFTER_HARDWARE = 8
HARDWARE_READY_TIMEOUT = 75
HARDWARE_STABLE_SECONDS = 2.0
MOVE_GROUP_READY_TIMEOUT = 45

WORKSPACE = Path(__file__).resolve().parent
LOG_DIR = WORKSPACE / "log" / "starter"

StatusFn = Callable[[str], None]


def ros_cmd(command: str) -> str:
    """Erzeugt einen Bash-Befehl mit ROS- und Workspace-Sourcing."""
    setup_ws = WORKSPACE / "install" / "setup.bash"
    source_ws = f"[ -f {shlex.quote(str(setup_ws))} ] && source {shlex.quote(str(setup_ws))}"
    return (
        f"set +u && "
        f"source {shlex.quote(f'/opt/ros/{ROS_DISTRO}/setup.bash')} && "
        f"{source_ws} && "
        f"set -u && exec {command}"
    )


class ProcessManager:
    """Verwaltet Hintergrundprozesse und Log-Dateien."""

    def __init__(self):
        self._procs: list[subprocess.Popen] = []
        self._logs: list = []

    def _open_log(self, name: str):
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        path = LOG_DIR / f"{time.strftime('%Y%m%d_%H%M%S')}_{name}.log"
        handle = open(path, "w", encoding="utf-8")
        self._logs.append(handle)
        return handle, path

    def run_blocking(self, command: str, log_name: str, status: StatusFn | None = None) -> int:
        log, path = self._open_log(log_name)
        if status:
            status(f"Starte: {command}\nLog: {path}")
        proc = subprocess.Popen(
            ["bash", "-lc", command],
            stdout=log,
            stderr=subprocess.STDOUT,
            cwd=WORKSPACE,
            preexec_fn=os.setsid,
            text=True,
        )
        self._procs.append(proc)
        rc = proc.wait()
        if proc in self._procs:
            self._procs.remove(proc)
        if status:
            status(f"Befehl beendet (Code {rc}): {command}")
        return rc

    def run_ros_capture(
        self,
        command: str,
        log_name: str,
        timeout: float,
        status: StatusFn | None = None,
    ) -> tuple[int, str]:
        log, path = self._open_log(log_name)
        display_command = command if len(command) <= 160 else f"{command[:157]}..."
        if status:
            status(f"Pruefe: {display_command}\nLog: {path}")
        proc = subprocess.Popen(
            ["bash", "-lc", ros_cmd(command)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            cwd=WORKSPACE,
            preexec_fn=os.setsid,
            text=True,
        )
        self._procs.append(proc)
        try:
            output, _ = proc.communicate(timeout=timeout)
            rc = proc.returncode
        except subprocess.TimeoutExpired:
            self.terminate(proc)
            output, _ = proc.communicate()
            output = (output or "") + f"\nTimeout nach {timeout:.0f} s\n"
            rc = 124
        finally:
            if proc in self._procs:
                self._procs.remove(proc)

        log.write(output or "")
        log.flush()
        if status and output:
            for line in output.strip().splitlines()[-8:]:
                status(line)
        if status:
            status(f"Pruefung beendet (Code {rc}): {display_command}")
        return rc, output or ""

    def start_background(
        self,
        command: str,
        log_name: str,
        status: StatusFn | None = None,
    ) -> subprocess.Popen:
        log, path = self._open_log(log_name)
        if status:
            status(f"Hintergrund: {command}\nLog: {path}")
        proc = subprocess.Popen(
            ["bash", "-lc", ros_cmd(command)],
            stdout=log,
            stderr=subprocess.STDOUT,
            cwd=WORKSPACE,
            preexec_fn=os.setsid,
            text=True,
        )
        self._procs.append(proc)
        return proc

    def terminate(self, proc: subprocess.Popen | None) -> None:
        if proc is None or proc.poll() is not None:
            return
        for sig, delay in [(signal.SIGINT, 1.5), (signal.SIGTERM, 1.0), (signal.SIGKILL, 0)]:
            if proc.poll() is not None:
                break
            try:
                os.killpg(os.getpgid(proc.pid), sig)
                if delay:
                    time.sleep(delay)
            except Exception:
                break

    def cleanup(self) -> None:
        for proc in reversed(self._procs):
            self.terminate(proc)
        self._procs.clear()
        for handle in self._logs:
            try:
                handle.flush()
                handle.close()
            except Exception:
                pass

    def remove_dead(self) -> None:
        self._procs = [proc for proc in self._procs if proc.poll() is None]


def validate_workspace() -> None:
    if not (WORKSPACE / "src").is_dir():
        raise RuntimeError(
            f"Kein src/-Ordner in {WORKSPACE}.\n"
            "Lege den Starter bitte in den Root-Ordner deines ROS-2-Workspaces."
        )


def build_workspace(pm: ProcessManager, status: StatusFn | None = None) -> None:
    cmd = f"source /opt/ros/{ROS_DISTRO}/setup.bash && colcon build --symlink-install"
    if pm.run_blocking(cmd, "build", status) != 0:
        raise RuntimeError(f"Build fehlgeschlagen. Logs unter: {LOG_DIR}")


def build_project_package(pm: ProcessManager, status: StatusFn | None = None) -> bool:
    cmd = (
        f"source /opt/ros/{ROS_DISTRO}/setup.bash && "
        f"colcon build --packages-select {PACKAGE_NAME} --symlink-install"
    )
    return pm.run_blocking(cmd, "build_package", status) == 0


def get_executables(status: StatusFn | None = None) -> list[str]:
    if status:
        status(f"Lese Executables aus Paket {PACKAGE_NAME} ...")
    result = subprocess.run(
        ["bash", "-lc", ros_cmd(f"ros2 pkg executables {PACKAGE_NAME}")],
        cwd=WORKSPACE,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"Executables nicht lesbar:\n{result.stderr}")
    exes = sorted(
        {
            parts[1]
            for line in result.stdout.splitlines()
            if (parts := line.split()) and len(parts) >= 2 and parts[0] == PACKAGE_NAME
        }
    )
    if not exes:
        raise RuntimeError(f"Keine Executables in {PACKAGE_NAME} gefunden.")
    return exes


def launch_program(
    pm: ProcessManager,
    exe: str,
    log_prefix: str,
    status: StatusFn | None = None,
) -> subprocess.Popen:
    command = f"ros2 run {PACKAGE_NAME} {exe} --ros-args -r __ns:={NAMESPACE}"
    log, path = pm._open_log(f"{log_prefix}_{exe}")
    if status:
        status(f"Starte Programm: {exe}\nLog: {path}")
    proc = subprocess.Popen(
        ["bash", "-lc", ros_cmd(command)],
        stdout=log,
        stderr=subprocess.STDOUT,
        cwd=WORKSPACE,
        preexec_fn=os.setsid,
        text=True,
    )
    pm._procs.append(proc)
    return proc


def is_running(proc: subprocess.Popen | None) -> bool:
    return proc is not None and proc.poll() is None


def start_mock_system(pm: ProcessManager, status: StatusFn | None = None) -> tuple[subprocess.Popen, subprocess.Popen]:
    mock_proc = pm.start_background(
        f"ros2 launch lbr_bringup mock.launch.py model:={ROBOT_MODEL}",
        "mock",
        status,
    )
    time.sleep(WAIT_AFTER_MOCK)
    moveit_proc = pm.start_background(
        f"ros2 launch lbr_bringup move_group.launch.py model:={ROBOT_MODEL} mode:=mock rviz:=true",
        "moveit_rviz",
        status,
    )
    time.sleep(WAIT_AFTER_MOVEIT)
    return mock_proc, moveit_proc


def hardware_launch_command() -> str:
    namespace = NAMESPACE.strip("/")
    return (
        f"ros2 launch lbr_bringup hardware.launch.py "
        f"ctrl:={HARDWARE_CTRL} model:={ROBOT_MODEL} "
        f"robot_name:={namespace} namespace:={namespace}"
    )


def hardware_move_group_command(rviz: bool = False) -> str:
    return (
        f"ros2 launch lbr_bringup move_group.launch.py "
        f"model:={ROBOT_MODEL} mode:=hardware rviz:={'true' if rviz else 'false'}"
    )


def wait_for_lbr_state(pm: ProcessManager, status: StatusFn | None = None) -> bool:
    topic = f"{NAMESPACE}/lbr_state"
    script = textwrap.dedent(
        f"""
        import sys
        import time

        import rclpy
        from lbr_fri_idl.msg import LBRState

        topic = {topic!r}
        timeout = {HARDWARE_READY_TIMEOUT!r}
        stable_seconds = {HARDWARE_STABLE_SECONDS!r}
        session_names = {{
            0: "IDLE",
            1: "MONITORING_WAIT",
            2: "MONITORING_READY",
            3: "COMMANDING_WAIT",
            4: "COMMANDING_ACTIVE",
        }}
        quality_names = {{0: "POOR", 1: "FAIR", 2: "GOOD", 3: "EXCELLENT"}}
        last_msg = None
        stable_since = None
        last_print = 0.0

        def callback(msg):
            global last_msg, stable_since
            last_msg = msg
            ready = msg.session_state >= 3 and msg.connection_quality >= 2
            if ready and stable_since is None:
                stable_since = time.monotonic()
            elif not ready:
                stable_since = None

        rclpy.init()
        node = rclpy.create_node("wait_for_lbr_hardware_ready")
        node.create_subscription(LBRState, topic, callback, 10)

        deadline = time.monotonic() + timeout
        print(f"Waiting for {{topic}}: session >= COMMANDING_WAIT and quality >= GOOD")
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if last_msg is None:
                if now - last_print >= 1.0:
                    print("No LBRState yet. Start LBRServer on smartPAD and keep hardware.launch running.")
                    last_print = now
                continue

            if now - last_print >= 1.0:
                session = session_names.get(int(last_msg.session_state), str(last_msg.session_state))
                quality = quality_names.get(int(last_msg.connection_quality), str(last_msg.connection_quality))
                print(f"state={{session}} quality={{quality}} sample_time={{last_msg.sample_time:.4f}}s")
                last_print = now

            if stable_since is not None and now - stable_since >= stable_seconds:
                session = session_names.get(int(last_msg.session_state), str(last_msg.session_state))
                quality = quality_names.get(int(last_msg.connection_quality), str(last_msg.connection_quality))
                print(f"READY: state={{session}} quality={{quality}}")
                node.destroy_node()
                rclpy.shutdown()
                sys.exit(0)

        if last_msg is None:
            print(f"ERROR: No messages on {{topic}} within {{timeout}}s.")
        else:
            session = session_names.get(int(last_msg.session_state), str(last_msg.session_state))
            quality = quality_names.get(int(last_msg.connection_quality), str(last_msg.connection_quality))
            print(f"ERROR: Not ready. Last state={{session}} quality={{quality}}.")
            if int(last_msg.session_state) == 0:
                print("Robot is IDLE. Restart LBRServer on the smartPAD, then restart hardware.launch.py.")

        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
        """
    ).strip()
    rc, _ = pm.run_ros_capture(
        f"python3 -u -c {shlex.quote(script)}",
        "wait_lbr_state",
        HARDWARE_READY_TIMEOUT + 10,
        status,
    )
    return rc == 0


def wait_for_controller_active(
    controller: str = HARDWARE_CTRL,
    status: StatusFn | None = None,
) -> bool:
    controller_manager = f"{NAMESPACE}/controller_manager"
    required = {"joint_state_broadcaster", "lbr_state_broadcaster", controller}
    deadline = time.time() + 30
    last_output = ""
    while time.time() < deadline:
        result = subprocess.run(
            ["bash", "-lc", ros_cmd(f"ros2 control list_controllers -c {controller_manager}")],
            cwd=WORKSPACE,
            capture_output=True,
            text=True,
        )
        last_output = (result.stdout or result.stderr or "").strip()
        if result.returncode == 0:
            states: dict[str, list[str]] = {}
            for line in result.stdout.splitlines():
                parts = line.split()
                if parts:
                    states[parts[0]] = parts
            if all(name in states and "active" in states[name] for name in required):
                if status:
                    status(f"Controller aktiv: {', '.join(sorted(required))}")
                return True
        time.sleep(1)

    if status:
        status("Controller wurden nicht rechtzeitig aktiv.")
        if last_output:
            status(last_output)
    return False


def wait_for_node(expected_node: str, timeout: float, status: StatusFn | None = None) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        result = subprocess.run(
            ["bash", "-lc", ros_cmd("ros2 node list")],
            cwd=WORKSPACE,
            capture_output=True,
            text=True,
        )
        if result.returncode == 0 and expected_node in result.stdout.splitlines():
            if status:
                status(f"Node bereit: {expected_node}")
            return True
        time.sleep(1)
    if status:
        status(f"Node nicht rechtzeitig gefunden: {expected_node}")
    return False
