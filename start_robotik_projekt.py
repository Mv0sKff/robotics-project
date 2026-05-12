#!/usr/bin/env python3
"""
GUI-Starter fuer das Robotik-Projekt.

Ablage: robotics-project/start_robotik_projekt.py
Start:  ./start_robotik_projekt.py
"""

from __future__ import annotations

import os
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path
import tkinter as tk
from tkinter import messagebox
from tkinter.scrolledtext import ScrolledText

# -----------------------------------------------------------------------------
# Einstellungen
# -----------------------------------------------------------------------------

ROS_DISTRO    = "jazzy"
PACKAGE_NAME  = "robotik_projekt"
ROBOT_MODEL   = "iiwa14"
NAMESPACE     = "/lbr"

WAIT_AFTER_MOCK     = 5   # Sekunden nach Mock-Start
WAIT_AFTER_MOVEIT   = 8   # Sekunden nach MoveIt-Start
WAIT_AFTER_HARDWARE = 8   # Sekunden nach Hardware-Launch

# Controller fuer den echten Roboter (laut Doku: joint_trajectory_controller
# oder lbr_joint_position_command_controller)
HARDWARE_CTRL = "joint_trajectory_controller"

# -----------------------------------------------------------------------------
# Pfade
# -----------------------------------------------------------------------------

WORKSPACE = Path(__file__).resolve().parent
LOG_DIR   = WORKSPACE / "log" / "starter"

# -----------------------------------------------------------------------------
# Prozessverwaltung
# -----------------------------------------------------------------------------

class ProcessManager:
    """Verwaltet alle gestarteten Hintergrundprozesse und Log-Dateien."""

    def __init__(self):
        self._procs: list[subprocess.Popen] = []
        self._logs:  list                   = []

    # -- Log-Hilfsmethode -----------------------------------------------------

    def _open_log(self, name: str):
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        path = LOG_DIR / f"{time.strftime('%Y%m%d_%H%M%S')}_{name}.log"
        f = open(path, "w", encoding="utf-8")
        self._logs.append(f)
        return f, path

    # -- Prozesse starten -----------------------------------------------------

    def run_blocking(self, command: str, log_name: str, status=None) -> int:
        """Fuehrt einen Shell-Befehl blockierend aus."""
        log, path = self._open_log(log_name)
        if status:
            status(f"Starte: {command}\nLog: {path}")
        proc = subprocess.Popen(
            ["bash", "-lc", command],
            stdout=log, stderr=subprocess.STDOUT,
            cwd=WORKSPACE, preexec_fn=os.setsid, text=True,
        )
        self._procs.append(proc)
        rc = proc.wait()
        if proc in self._procs:
            self._procs.remove(proc)
        if status:
            status(f"Befehl beendet (Code {rc}): {command}")
        return rc

    def start_background(self, command: str, log_name: str, status=None) -> subprocess.Popen:
        """Startet einen ROS-Befehl im Hintergrund."""
        log, path = self._open_log(log_name)
        if status:
            status(f"Hintergrund: {command}\nLog: {path}")
        proc = subprocess.Popen(
            ["bash", "-lc", _ros_cmd(command)],
            stdout=log, stderr=subprocess.STDOUT,
            cwd=WORKSPACE, preexec_fn=os.setsid, text=True,
        )
        self._procs.append(proc)
        return proc

    # -- Prozesse beenden -----------------------------------------------------

    def terminate(self, proc: subprocess.Popen | None) -> None:
        """Beendet einen Prozess (SIGINT → SIGTERM → SIGKILL)."""
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
        """Beendet alle verwalteten Prozesse und schliesst Log-Dateien."""
        for proc in reversed(self._procs):
            self.terminate(proc)
        self._procs.clear()
        for f in self._logs:
            try:
                f.flush(); f.close()
            except Exception:
                pass

    def remove_dead(self) -> None:
        self._procs = [p for p in self._procs if p.poll() is None]


# Singleton
pm = ProcessManager()


def _ros_cmd(command: str) -> str:
    """Erzeugt einen Bash-Befehl mit ROS- und Workspace-Sourcing."""
    setup_ws = WORKSPACE / "install" / "setup.bash"
    source_ws = f"[ -f {shlex.quote(str(setup_ws))} ] && source {shlex.quote(str(setup_ws))}"
    return (
        f"set +u && "
        f"source {shlex.quote(f'/opt/ros/{ROS_DISTRO}/setup.bash')} && "
        f"{source_ws} && "
        f"set -u && exec {command}"
    )


# -----------------------------------------------------------------------------
# Vorbereitungsschritte
# -----------------------------------------------------------------------------

def validate_workspace() -> None:
    if not (WORKSPACE / "src").is_dir():
        raise RuntimeError(
            f"Kein src/-Ordner in {WORKSPACE}.\n"
            "Lege dieses Skript bitte in den Root-Ordner deines ROS-2-Workspaces."
        )


def build_workspace(status=None) -> None:
    cmd = f"source /opt/ros/{ROS_DISTRO}/setup.bash && colcon build --symlink-install"
    if pm.run_blocking(cmd, "build", status) != 0:
        raise RuntimeError(f"Build fehlgeschlagen. Logs unter: {LOG_DIR}")


def get_executables(status=None) -> list[str]:
    if status:
        status(f"Lese Executables aus Paket {PACKAGE_NAME} ...")
    result = subprocess.run(
        ["bash", "-lc", _ros_cmd(f"ros2 pkg executables {PACKAGE_NAME}")],
        cwd=WORKSPACE, capture_output=True, text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"Executables nicht lesbar:\n{result.stderr}")
    exes = sorted({
        parts[1]
        for line in result.stdout.splitlines()
        if (parts := line.split()) and len(parts) >= 2 and parts[0] == PACKAGE_NAME
    })
    if not exes:
        raise RuntimeError(f"Keine Executables in {PACKAGE_NAME} gefunden.")
    return exes


def start_mock_system(status=None) -> None:
    """Startet Mock + MoveIt + RViz (Simulation)."""
    pm.start_background(
        f"ros2 launch lbr_bringup mock.launch.py model:={ROBOT_MODEL}",
        "mock", status,
    )
    time.sleep(WAIT_AFTER_MOCK)
    pm.start_background(
        f"ros2 launch lbr_bringup move_group.launch.py model:={ROBOT_MODEL} mode:=mock rviz:=true",
        "moveit_rviz", status,
    )
    time.sleep(WAIT_AFTER_MOVEIT)


# -----------------------------------------------------------------------------
# GUI
# -----------------------------------------------------------------------------

class RobotikStarterGUI:
    def __init__(self, root: tk.Tk, executables: list[str]):
        self.root        = root
        self.executables = executables
        self._robot_proc: subprocess.Popen | None = None
        self._robot_name: str | None = None
        self._hw_proc:    subprocess.Popen | None = None  # hardware.launch.py

        root.title("Robotik Projekt Starter")
        root.geometry("850x560")
        root.protocol("WM_DELETE_WINDOW", self.on_exit)

        self._build_ui()
        self._setup_konami()
        self._poll_dead_procs()
        self.log(f"Logs: {LOG_DIR}")
        self.log("Mock, MoveIt und RViz laufen. Wähle ein Roboterprogramm.")

    # -- UI aufbauen ----------------------------------------------------------

    def _build_ui(self):
        tk.Label(
            self.root,
            text="Robotik-Projekt: Programm auswählen und starten",
            font=("Arial", 16, "bold"),
        ).pack(padx=12, pady=(12, 4), anchor="w")

        tk.Label(
            self.root,
            text=f"Workspace: {WORKSPACE}   |   Paket: {PACKAGE_NAME}   |   Modell: {ROBOT_MODEL}   |   NS: {NAMESPACE}",
            justify="left",
        ).pack(padx=12, pady=(0, 8), anchor="w")

        main = tk.Frame(self.root)
        main.pack(fill="both", expand=True, padx=12, pady=4)

        # Linke Spalte: Programmliste
        left = tk.Frame(main)
        left.pack(side="left", fill="both")
        tk.Label(left, text="Roboterprogramme").pack(anchor="w")
        self.listbox = tk.Listbox(left, width=42, height=16)
        self.listbox.pack(fill="y")
        for exe in self.executables:
            self.listbox.insert(tk.END, exe)
        if self.executables:
            self.listbox.selection_set(0)

        # Rechte Spalte: Statuslog
        right = tk.Frame(main)
        right.pack(side="right", fill="both", expand=True, padx=(14, 0))
        tk.Label(right, text="Status / Hinweise").pack(anchor="w")
        self.status_text = ScrolledText(right, height=16, state="disabled")
        self.status_text.pack(fill="both", expand=True)

        # Untere Leiste
        bottom = tk.Frame(self.root)
        bottom.pack(fill="x", padx=12, pady=12)

        # Links: Beenden
        tk.Button(
            bottom, text="Alles beenden", command=self.on_exit,
            bg="#b00020", fg="white", width=18, height=2,
        ).pack(side="left")

        # Rechts: Status-Label + Buttons
        self.hw_server_btn = tk.Button(
            bottom,
            text=f"▶  Hardware-Server starten\n(ctrl: {HARDWARE_CTRL})",
            command=self.start_hardware_server,
            bg="#8a3d00", fg="white", width=26, height=2,
        )
        self.hw_server_btn.pack(side="right", padx=(8, 0))

        self.hw_btn = tk.Button(
            bottom,
            text="▶  Auf Hardware starten\n(pick_place_node)",
            command=self.start_selected_hardware,
            bg="#b85c00", fg="white", width=26, height=2,
        )
        self.hw_btn.pack(side="right", padx=(8, 0))

        self.mock_btn = tk.Button(
            bottom,
            text="▶  Auf Mock starten",
            command=self.start_selected_mock,
            bg="#006400", fg="white", width=20, height=2,
        )
        self.mock_btn.pack(side="right", padx=(8, 0))

        self.status_label = tk.Label(bottom, text="Bereit")
        self.status_label.pack(side="right", padx=(12, 0))

    # -- Logging --------------------------------------------------------------

    def log(self, text: str):
        self.status_text.configure(state="normal")
        self.status_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {text}\n")
        self.status_text.see(tk.END)
        self.status_text.configure(state="disabled")

    # -- Gemeinsame Hilfsmethoden ---------------------------------------------

    def _get_selected_exe(self) -> str | None:
        """Gibt das gewählte Executable zurück oder None bei Fehler."""
        if self._robot_proc and self._robot_proc.poll() is None:
            messagebox.showinfo("Läuft bereits", "Warte bis das aktuelle Programm beendet ist.")
            return None
        sel = self.listbox.curselection()
        if not sel:
            messagebox.showwarning("Keine Auswahl", "Bitte wähle zuerst ein Programm aus.")
            return None
        return self.listbox.get(sel[0])

    def _launch_program(self, exe: str):
        """Startet das Executable und beginnt den Monitor-Loop."""
        cmd = f"ros2 run {PACKAGE_NAME} {exe} --ros-args -r __ns:={NAMESPACE}"
        log, path = pm._open_log(f"program_{exe}")
        self.log(f"Starte Programm: {exe}\nLog: {path}")
        self._robot_proc = subprocess.Popen(
            ["bash", "-lc", _ros_cmd(cmd)],
            stdout=log, stderr=subprocess.STDOUT,
            cwd=WORKSPACE, preexec_fn=os.setsid, text=True,
        )
        pm._procs.append(self._robot_proc)
        self._robot_name = exe
        self.root.after(500, self._monitor)

    def _set_buttons(self, state: str):
        self.mock_btn.configure(state=state)
        self.hw_btn.configure(state=state)
        self.hw_server_btn.configure(state=state)

    def _reset_buttons(self):
        self._set_buttons("normal")
        self.status_label.configure(text="Bereit")

    # -- Mock-Start -----------------------------------------------------------

    def start_selected_mock(self):
        exe = self._get_selected_exe()
        if not exe:
            return
        self.status_label.configure(text=f"Mock: {exe}")
        self._set_buttons("disabled")
        self.log(f"[Mock] Starte '{exe}' auf Simulation.")
        self._launch_program(exe)

    # -- Hardware-Start -------------------------------------------------------

    def start_hardware_server(self):
        if self._hw_proc and self._hw_proc.poll() is None:
            messagebox.showinfo("Läuft bereits", "Der Hardware-Server läuft bereits.")
            return

        self.status_label.configure(text="Hardware-Server")
        self._set_buttons("disabled")

        hw_cmd = (
            f"ros2 launch lbr_bringup hardware.launch.py "
            f"ctrl:={HARDWARE_CTRL} model:={ROBOT_MODEL}"
        )
        self.log(f"[Hardware] Starte Server: {hw_cmd}")
        self._hw_proc = pm.start_background(hw_cmd, "hardware_launch", self.log)
        self._reset_buttons()

    def start_selected_hardware(self):
        # Sicherheitswarnung (laut Doku: immer zuerst T1-Modus!)
        confirmed = messagebox.askyesno(
            "⚠  Hardware – Sicherheitshinweis",
            "Du startest auf dem Roboter!\n\n"
            "Fortfahren?",
            icon="warning",
        )
        if not confirmed:
            return

        self.status_label.configure(text="Hardware: pick_place_node")
        self._set_buttons("disabled")

        build_cmd = (
            f"source /opt/ros/{ROS_DISTRO}/setup.bash && "
            f"colcon build --packages-select {PACKAGE_NAME}"
        )
        self.log(f"[Hardware] Baue Paket: {PACKAGE_NAME}")
        if pm.run_blocking(build_cmd, "build_hardware", self.log) != 0:
            self.log("[Hardware] Build fehlgeschlagen")
            self._reset_buttons()
            return

        self.log("[Hardware] Starte pick_place_node")
        self._launch_program("pick_place_node")

    # -- Prozess-Monitor ------------------------------------------------------

    def _monitor(self):
        if self._robot_proc is None:
            self._reset_buttons()
            return
        rc = self._robot_proc.poll()
        if rc is None:
            self.root.after(1000, self._monitor)
            return
        self.log(f"Programm beendet: {self._robot_name} (Exit {rc})")
        if self._robot_proc in pm._procs:
            pm._procs.remove(self._robot_proc)
        self._robot_proc = None
        self._robot_name = None
        self._reset_buttons()

    # -- Hintergrundprozesse prüfen ------------------------------------------

    def _poll_dead_procs(self):
        pm.remove_dead()
        self.root.after(2000, self._poll_dead_procs)

    # -- Beenden --------------------------------------------------------------

    def on_exit(self):
        if messagebox.askyesno("Alles beenden", "Mock, MoveIt, RViz und alle Programme beenden?"):
            self.log("Beende alle Prozesse ...")
            pm.cleanup()
            self.root.destroy()

    # -- Konami-Code ----------------------------------------------------------

    def _setup_konami(self):
        self._konami = ["Up","Up","Down","Down","Left","Right","Left","Right","a","b"]
        self._buf: list[str] = []
        self.root.bind_all("<KeyPress>", self._on_key)

    def _on_key(self, event):
        key = event.keysym.lower() if event.keysym.lower() in ("a", "b") else event.keysym
        self._buf = (self._buf + [key])[-len(self._konami):]
        if self._buf == self._konami:
            self._buf.clear()
            self._praise_the_sun()

    def _praise_the_sun(self):
        import random
        msgs = ["Praise the Sun", "☀ Praise the Sun ☀", "\\[T]/"]
        labels = [
            tk.Label(self.root, text=random.choice(msgs),
                     font=("Arial", random.randint(12, 28), "bold"), bg="yellow", fg="black")
            for _ in range(77)
        ]
        for lbl in labels:
            lbl.place(
                x=random.randint(0, max(1, self.root.winfo_width() - 180)),
                y=random.randint(0, max(1, self.root.winfo_height() - 40)),
            )
        self.root.after(10_000, lambda: [lbl.destroy() for lbl in labels])


# -----------------------------------------------------------------------------
# Start
# -----------------------------------------------------------------------------

def main() -> int:
    signal.signal(signal.SIGINT,  lambda *_: (pm.cleanup(), sys.exit(0)))
    signal.signal(signal.SIGTERM, lambda *_: (pm.cleanup(), sys.exit(0)))

    try:
        validate_workspace()

        # Vorbereitungsfenster
        prep = tk.Tk()
        prep.title("Robotik Starter – Vorbereitung")
        prep.geometry("760x360")
        log_widget = ScrolledText(prep, height=18, state="disabled")
        log_widget.pack(fill="both", expand=True, padx=12, pady=12)

        def prep_log(msg: str):
            log_widget.configure(state="normal")
            log_widget.insert(tk.END, f"{msg}\n")
            log_widget.see(tk.END)
            log_widget.configure(state="disabled")
            prep.update()

        prep_log(f"Workspace : {WORKSPACE}")
        prep_log(f"ROS-Distro: {ROS_DISTRO}  |  Paket: {PACKAGE_NAME}")
        prep_log("Baue Workspace ...")
        build_workspace(prep_log)

        prep_log("Lese Executables ...")
        executables = get_executables(prep_log)

        prep_log("Starte Mock, MoveIt und RViz ...")
        start_mock_system(prep_log)

        prep_log("Bereit – öffne Hauptfenster ...")
        time.sleep(0.5)
        prep.destroy()

        root = tk.Tk(className="Robotik Projekt Launcher")
        RobotikStarterGUI(root, executables)
        root.mainloop()

    except Exception as exc:
        pm.cleanup()
        try:
            messagebox.showerror("Fehler", str(exc))
        except Exception:
            print(f"Fehler: {exc}", file=sys.stderr)
        return 1
    finally:
        pm.cleanup()

    return 0


if __name__ == "__main__":
    sys.exit(main())
