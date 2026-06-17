#!/usr/bin/env python3
"""Shared, simple helpers for the robotics starters."""

from __future__ import annotations

import os
import shlex
import shutil
import signal
import subprocess
import tempfile
import textwrap
import time
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
VENV_ACTIVATE = WORKSPACE / ".venv" / "bin" / "activate"
COLCON_PYTHON = "python" if VENV_ACTIVATE.exists() else "python3"

StatusFn = Callable[[str], None]


def ros_cmd(command: str) -> str:
    setup_ws = WORKSPACE / "install" / "setup.bash"
    return " && ".join(
        [
            f"cd {shlex.quote(str(WORKSPACE))}",
            "set +u",
            f"source {shlex.quote(f'/opt/ros/{ROS_DISTRO}/setup.bash')}",
            f"if [ -f {shlex.quote(str(VENV_ACTIVATE))} ]; then source {shlex.quote(str(VENV_ACTIVATE))}; fi",
            f"if [ -f {shlex.quote(str(setup_ws))} ]; then source {shlex.quote(str(setup_ws))}; fi",
            "set -u",
            command,
        ]
    )


def _terminal_command(title: str, script: str) -> list[str]:
    terminal = (
        shutil.which("gnome-terminal")
        or shutil.which("xterm")
        or shutil.which("konsole")
        or shutil.which("xfce4-terminal")
        or shutil.which("x-terminal-emulator")
    )
    if not terminal:
        raise RuntimeError(
            "No terminal emulator found. Install gnome-terminal or xterm, for example."
        )

    name = Path(terminal).name
    if name == "gnome-terminal":
        return [terminal, "--wait", "--title", title, "--", "bash", "-lc", script]
    if name == "xterm":
        return [terminal, "-T", title, "-e", "bash", "-lc", script]
    if name == "konsole":
        return [terminal, "--new-tab", "-p", f"tabtitle={title}", "-e", "bash", "-lc", script]
    if name == "xfce4-terminal":
        return [terminal, "--disable-server", "--title", title, "--command", f"bash -lc {shlex.quote(script)}"]
    return [terminal, "-T", title, "-e", "bash", "-lc", script]


def _terminal_env() -> dict[str, str]:
    env = os.environ.copy()
    for key in list(env):
        if key == "SNAP" or key.startswith("SNAP_"):
            env.pop(key, None)
    for key in (
        "GTK_PATH",
        "GIO_MODULE_DIR",
        "GDK_PIXBUF_MODULE_FILE",
        "GDK_PIXBUF_MODULEDIR",
    ):
        env.pop(key, None)
    return env


def _terminal_script(
    command: str,
    title: str,
    exit_file: Path | None = None,
    pid_file: Path | None = None,
    keep_open: bool = True,
) -> str:
    exit_write = ""
    if exit_file is not None:
        exit_write = f"printf '%s\\n' \"$rc\" > {shlex.quote(str(exit_file))}"

    pid_write = ""
    if pid_file is not None:
        pid_write = f"printf '%s\\n' \"$$\" > {shlex.quote(str(pid_file))}"

    wait_line = ""
    if keep_open:
        wait_line = "read -r -p 'Press Enter to close...'"
    else:
        wait_line = "if [ \"$rc\" -ne 0 ]; then read -r -p 'Error. Press Enter to close...'; fi"

    return textwrap.dedent(
        f"""
        trap 'exit 130' INT TERM
        {pid_write}
        {ros_cmd(':')}
        clear
        echo '== {title} =='
        echo {shlex.quote(command)}
        echo
        {command}
        rc=$?
        echo
        echo '== {title} finished with code' "$rc" '=='
        {exit_write}
        {wait_line}
        exit "$rc"
        """
    ).strip()


class ProcessManager:
    def __init__(self):
        self._procs: list[subprocess.Popen] = []
        self._terminal_shells: dict[subprocess.Popen, Path] = {}

    def start_terminal(
        self,
        command: str,
        title: str,
        status: StatusFn | None = None,
    ) -> subprocess.Popen:
        if status:
            status(f"Terminal: {title}\n{command}")
        pid_tmp = tempfile.NamedTemporaryFile(prefix="robotik_shell_", delete=False)
        pid_path = Path(pid_tmp.name)
        pid_tmp.close()
        script = _terminal_script(command, title, pid_file=pid_path, keep_open=True)
        proc = subprocess.Popen(
            _terminal_command(title, script),
            cwd=WORKSPACE,
            env=_terminal_env(),
            preexec_fn=os.setsid,
            text=True,
        )
        self._procs.append(proc)
        self._terminal_shells[proc] = pid_path
        return proc

    def run_terminal(
        self,
        command: str,
        title: str,
        status: StatusFn | None = None,
    ) -> int:
        if status:
            status(f"Starting: {title}\n{command}")
        tmp = tempfile.NamedTemporaryFile(prefix="robotik_exit_", delete=False)
        tmp_path = Path(tmp.name)
        tmp.close()
        pid_tmp = tempfile.NamedTemporaryFile(prefix="robotik_shell_", delete=False)
        pid_path = Path(pid_tmp.name)
        pid_tmp.close()
        script = _terminal_script(
            command,
            title,
            exit_file=tmp_path,
            pid_file=pid_path,
            keep_open=False,
        )
        proc = subprocess.Popen(
            _terminal_command(title, script),
            cwd=WORKSPACE,
            env=_terminal_env(),
            preexec_fn=os.setsid,
            text=True,
        )
        self._procs.append(proc)
        self._terminal_shells[proc] = pid_path
        proc.wait()
        if proc in self._procs:
            self._procs.remove(proc)
        self._remove_shell_pid(proc)

        rc = proc.returncode
        try:
            content = tmp_path.read_text(encoding="utf-8").strip()
            if content:
                rc = int(content)
        finally:
            try:
                tmp_path.unlink()
            except OSError:
                pass

        if status:
            status(f"Finished with code {rc}: {title}")
        return rc

    def run_ros_capture(
        self,
        command: str,
        timeout: float,
        status: StatusFn | None = None,
    ) -> tuple[int, str]:
        if status:
            status(f"Checking: {command}")
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

        output = output or ""
        if status and output:
            for line in output.strip().splitlines()[-8:]:
                status(line)
        return rc, output

    def terminate(self, proc: subprocess.Popen | None) -> None:
        if proc is None:
            return

        self._terminate_terminal_shell(proc)
        if proc.poll() is not None:
            self._remove_shell_pid(proc)
            return

        for sig, delay in [(signal.SIGINT, 1.5), (signal.SIGTERM, 1.0), (signal.SIGKILL, 0)]:
            if proc.poll() is not None:
                break
            try:
                os.killpg(os.getpgid(proc.pid), sig)
                if delay:
                    time.sleep(delay)
            except OSError:
                break
        self._remove_shell_pid(proc)

    def cleanup(self) -> None:
        for proc in reversed(self._procs):
            self.terminate(proc)
        self._procs.clear()

    def remove_dead(self) -> None:
        dead = [proc for proc in self._procs if proc.poll() is not None]
        for proc in dead:
            self._remove_shell_pid(proc)
        self._procs = [proc for proc in self._procs if proc.poll() is None]

    def _terminate_terminal_shell(self, proc: subprocess.Popen) -> None:
        pid_path = self._terminal_shells.get(proc)
        if pid_path is None or not pid_path.exists():
            return
        try:
            shell_pid = int(pid_path.read_text(encoding="utf-8").strip())
        except (OSError, ValueError):
            return

        for sig, delay in [(signal.SIGINT, 0.7), (signal.SIGTERM, 0.7), (signal.SIGKILL, 0)]:
            try:
                os.killpg(os.getpgid(shell_pid), sig)
            except OSError:
                try:
                    os.kill(shell_pid, sig)
                except OSError:
                    break
            if delay:
                time.sleep(delay)

    def _remove_shell_pid(self, proc: subprocess.Popen) -> None:
        pid_path = self._terminal_shells.pop(proc, None)
        if pid_path is None:
            return
        try:
            pid_path.unlink()
        except OSError:
            pass


def is_running(proc: subprocess.Popen | None) -> bool:
    return proc is not None and proc.poll() is None


def validate_workspace() -> None:
    if not (WORKSPACE / "src").is_dir():
        raise RuntimeError(
            f"No src/ folder found in {WORKSPACE}.\n"
            "Place the starter in the root folder of your ROS 2 workspace."
        )


def build_workspace(pm: ProcessManager, status: StatusFn | None = None) -> None:
    rc = pm.run_terminal(f"{COLCON_PYTHON} -m colcon build --symlink-install", "Build Workspace", status)
    if rc != 0:
        raise RuntimeError("Build failed.")


def build_project_package(pm: ProcessManager, status: StatusFn | None = None) -> bool:
    command = f"{COLCON_PYTHON} -m colcon build --packages-select {PACKAGE_NAME} --symlink-install"
    return pm.run_terminal(command, f"Build {PACKAGE_NAME}", status) == 0


def get_executables(status: StatusFn | None = None) -> list[str]:
    if status:
        status(f"Reading executables from package {PACKAGE_NAME} ...")
    result = subprocess.run(
        ["bash", "-lc", ros_cmd(f"ros2 pkg executables {PACKAGE_NAME}")],
        cwd=WORKSPACE,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"Could not read executables:\n{result.stderr}")
    exes = sorted(
        {
            parts[1]
            for line in result.stdout.splitlines()
            if (parts := line.split()) and len(parts) >= 2 and parts[0] == PACKAGE_NAME
        }
    )
    if not exes:
        raise RuntimeError(f"No executables found in {PACKAGE_NAME}.")
    return exes


def launch_program(
    pm: ProcessManager,
    exe: str,
    title_prefix: str,
    status: StatusFn | None = None,
) -> subprocess.Popen:
    command = f"ros2 run {PACKAGE_NAME} {exe} --ros-args -r __ns:={NAMESPACE}"
    return pm.start_terminal(command, f"{title_prefix}: {exe}", status)


def start_mock_system(pm: ProcessManager, status: StatusFn | None = None) -> tuple[subprocess.Popen, subprocess.Popen]:
    mock_proc = pm.start_terminal(
        f"ros2 launch lbr_bringup mock.launch.py model:={ROBOT_MODEL}",
        "Mock",
        status,
    )
    time.sleep(WAIT_AFTER_MOCK)
    moveit_proc = pm.start_terminal(
        f"ros2 launch lbr_bringup move_group.launch.py model:={ROBOT_MODEL} mode:=mock rviz:=true",
        "MoveIt Mock + RViz",
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
                    status(f"Controllers active: {', '.join(sorted(required))}")
                return True
        time.sleep(1)

    if status:
        status("Controllers did not become active in time.")
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
                status(f"Node ready: {expected_node}")
            return True
        time.sleep(1)
    if status:
        status(f"Node not found in time: {expected_node}")
    return False
