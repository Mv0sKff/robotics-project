#!/usr/bin/env python3
"""Starter nur fuer die echte Roboterverbindung."""

from __future__ import annotations

import signal
import sys
import time
import tkinter as tk
from tkinter import messagebox
from tkinter.scrolledtext import ScrolledText

from starter_common import (
    HARDWARE_CTRL,
    LOG_DIR,
    MOVE_GROUP_READY_TIMEOUT,
    NAMESPACE,
    PACKAGE_NAME,
    ROBOT_MODEL,
    ROS_DISTRO,
    WAIT_AFTER_HARDWARE,
    WORKSPACE,
    ProcessManager,
    build_project_package,
    build_workspace,
    get_executables,
    hardware_launch_command,
    hardware_move_group_command,
    is_running,
    launch_program,
    validate_workspace,
    wait_for_controller_active,
    wait_for_lbr_state,
    wait_for_node,
)

pm = ProcessManager()


class HardwareStarterGUI:
    def __init__(self, root: tk.Tk, executables: list[str]):
        self.root = root
        self.executables = executables
        self._hw_proc = None
        self._moveit_proc = None
        self._robot_proc = None
        self._robot_name: str | None = None

        root.title("Robotik Hardware Starter")
        root.geometry("900x600")
        root.protocol("WM_DELETE_WINDOW", self.on_exit)

        self._build_ui()
        self._poll_dead_procs()
        self.log(f"Logs: {LOG_DIR}")
        self.log("Hardware-Starter bereit. Es wird kein Mock gestartet.")

    def _build_ui(self):
        tk.Label(
            self.root,
            text="Robotik-Projekt: Echte Hardware",
            font=("Arial", 16, "bold"),
        ).pack(padx=12, pady=(12, 4), anchor="w")

        tk.Label(
            self.root,
            text=f"Workspace: {WORKSPACE}   |   Paket: {PACKAGE_NAME}   |   Modell: {ROBOT_MODEL}   |   NS: {NAMESPACE}   |   Ctrl: {HARDWARE_CTRL}",
            justify="left",
        ).pack(padx=12, pady=(0, 8), anchor="w")

        checklist = (
            "smartPAD: LBRServer starten | T1-Modus | FRI send period 10 ms | "
            "FRI control mode POSITION_CONTROL | FRI client command mode POSITION"
        )
        tk.Label(self.root, text=checklist, justify="left", fg="#8a3d00").pack(
            padx=12, pady=(0, 8), anchor="w"
        )

        main = tk.Frame(self.root)
        main.pack(fill="both", expand=True, padx=12, pady=4)

        left = tk.Frame(main)
        left.pack(side="left", fill="both")
        tk.Label(left, text="Roboterprogramme").pack(anchor="w")
        self.listbox = tk.Listbox(left, width=42, height=16)
        self.listbox.pack(fill="y")
        for exe in self.executables:
            self.listbox.insert(tk.END, exe)
        if "pick_place_iiwa14" in self.executables:
            self.listbox.selection_set(self.executables.index("pick_place_iiwa14"))
        elif self.executables:
            self.listbox.selection_set(0)

        right = tk.Frame(main)
        right.pack(side="right", fill="both", expand=True, padx=(14, 0))
        tk.Label(right, text="Status / Hinweise").pack(anchor="w")
        self.status_text = ScrolledText(right, height=16, state="disabled")
        self.status_text.pack(fill="both", expand=True)

        bottom = tk.Frame(self.root)
        bottom.pack(fill="x", padx=12, pady=12)

        tk.Button(
            bottom,
            text="Alles beenden",
            command=self.on_exit,
            bg="#b00020",
            fg="white",
            width=18,
            height=2,
        ).pack(side="left")

        self.start_btn = tk.Button(
            bottom,
            text="Programm starten",
            command=self.start_selected,
            bg="#b85c00",
            fg="white",
            width=22,
            height=2,
        )
        self.start_btn.pack(side="right", padx=(8, 0))

        self.server_btn = tk.Button(
            bottom,
            text="Hardware verbinden",
            command=self.start_hardware_server,
            bg="#8a3d00",
            fg="white",
            width=22,
            height=2,
        )
        self.server_btn.pack(side="right", padx=(8, 0))

        self.stop_btn = tk.Button(
            bottom,
            text="Hardware stoppen",
            command=self.stop_hardware,
            width=22,
            height=2,
        )
        self.stop_btn.pack(side="right", padx=(8, 0))

        self.status_label = tk.Label(bottom, text="Nicht verbunden")
        self.status_label.pack(side="right", padx=(12, 0))

    def log(self, text: str):
        self.status_text.configure(state="normal")
        self.status_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {text}\n")
        self.status_text.see(tk.END)
        self.status_text.configure(state="disabled")
        try:
            self.root.update_idletasks()
        except tk.TclError:
            pass

    def _set_buttons(self, state: str):
        self.start_btn.configure(state=state)
        self.server_btn.configure(state=state)
        self.stop_btn.configure(state=state)

    def _get_selected_exe(self) -> str | None:
        if is_running(self._robot_proc):
            messagebox.showinfo("Laeuft bereits", "Warte bis das aktuelle Programm beendet ist.")
            return None
        sel = self.listbox.curselection()
        if not sel:
            messagebox.showwarning("Keine Auswahl", "Bitte waehle zuerst ein Programm aus.")
            return None
        return self.listbox.get(sel[0])

    def _confirm_hardware(self) -> bool:
        return messagebox.askyesno(
            "Hardware verbinden",
            "Bitte auf dem smartPAD zuerst LBRServer starten.\n\n"
            "Einstellungen laut Doku:\n"
            "- FRI send period: 10 ms\n"
            "- FRI control mode: POSITION_CONTROL\n"
            "- FRI client command mode: POSITION\n\n"
            "Danach wartet dieser Starter auf eine stabile Verbindung.",
            icon="warning",
        )

    def start_hardware_server(self):
        if is_running(self._hw_proc):
            self.log("[Hardware] Server laeuft bereits. Pruefe Verbindung ...")
            self._set_buttons("disabled")
            ready = self._wait_until_ready()
            self.status_label.configure(text="Verbunden" if ready else "Nicht bereit")
            self._set_buttons("normal")
            return

        if not self._confirm_hardware():
            return

        self._set_buttons("disabled")
        self.status_label.configure(text="Verbinde ...")
        command = hardware_launch_command()
        self.log(f"[Hardware] Starte: {command}")
        self._hw_proc = pm.start_background(command, "hardware_launch", self.log)
        time.sleep(WAIT_AFTER_HARDWARE)

        ready = self._wait_until_ready()
        self.status_label.configure(text="Verbunden" if ready else "Nicht bereit")
        self._set_buttons("normal")

    def _wait_until_ready(self) -> bool:
        if not wait_for_lbr_state(pm, self.log):
            self.log("[Hardware] Keine stabile FRI-Verbindung. LBRServer pruefen oder neu starten.")
            return False
        if not wait_for_controller_active(HARDWARE_CTRL, self.log):
            self.log("[Hardware] Controller sind nicht aktiv.")
            return False
        self.log("[Hardware] Verbindung stabil. Programme koennen gestartet werden.")
        return True

    def _ensure_hardware_ready(self) -> bool:
        if not is_running(self._hw_proc):
            if not self._confirm_hardware():
                return False
            self.status_label.configure(text="Verbinde ...")
            command = hardware_launch_command()
            self.log(f"[Hardware] Starte: {command}")
            self._hw_proc = pm.start_background(command, "hardware_launch", self.log)
            time.sleep(WAIT_AFTER_HARDWARE)
        return self._wait_until_ready()

    def _ensure_move_group(self) -> bool:
        if is_running(self._moveit_proc):
            return wait_for_node(f"{NAMESPACE}/move_group", MOVE_GROUP_READY_TIMEOUT, self.log)

        command = hardware_move_group_command(rviz=False)
        self.log(f"[MoveIt] Starte fuer Hardware: {command}")
        self._moveit_proc = pm.start_background(command, "move_group_hardware", self.log)
        return wait_for_node(f"{NAMESPACE}/move_group", MOVE_GROUP_READY_TIMEOUT, self.log)

    def start_selected(self):
        exe = self._get_selected_exe()
        if not exe:
            return

        confirmed = messagebox.askyesno(
            "Hardware - Sicherheitshinweis",
            "Du startest auf dem echten Roboter.\n\n"
            "Bitte nur in T1 testen, Arbeitsraum frei halten und Not-Halt griffbereit haben.\n\n"
            "Fortfahren?",
            icon="warning",
        )
        if not confirmed:
            return

        self._set_buttons("disabled")
        self.status_label.configure(text=f"Starte {exe}")

        if not self._ensure_hardware_ready():
            self._set_buttons("normal")
            return

        self.log(f"[Hardware] Baue Paket: {PACKAGE_NAME}")
        if not build_project_package(pm, self.log):
            self.log("[Hardware] Build fehlgeschlagen.")
            self.status_label.configure(text="Build fehlgeschlagen")
            self._set_buttons("normal")
            return

        if exe == "pick_place_iiwa14" and not self._ensure_move_group():
            self.status_label.configure(text="MoveIt nicht bereit")
            self._set_buttons("normal")
            return

        if not wait_for_lbr_state(pm, self.log):
            self.log("[Hardware] Verbindung ist vor Programmstart nicht stabil.")
            self.status_label.configure(text="Verbindung instabil")
            self._set_buttons("normal")
            return

        self.log(f"[Hardware] Starte {exe}.")
        self._robot_proc = launch_program(pm, exe, "hardware_program", self.log)
        self._robot_name = exe
        self.root.after(500, self._monitor)

    def stop_hardware(self):
        if is_running(self._robot_proc):
            pm.terminate(self._robot_proc)
            self._robot_proc = None
            self._robot_name = None
        if is_running(self._moveit_proc):
            self.log("Stoppe MoveIt (Hardware) ...")
            pm.terminate(self._moveit_proc)
        self._moveit_proc = None
        if is_running(self._hw_proc):
            self.log("Stoppe Hardware-Server ...")
            pm.terminate(self._hw_proc)
        self._hw_proc = None
        self.status_label.configure(text="Nicht verbunden")

    def _monitor(self):
        if self._robot_proc is None:
            self._set_buttons("normal")
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
        self.status_label.configure(text="Verbunden" if is_running(self._hw_proc) else "Nicht verbunden")
        self._set_buttons("normal")

    def _poll_dead_procs(self):
        if self._hw_proc is not None and self._hw_proc.poll() is not None:
            self.log(f"[Hardware] Server beendet (Exit {self._hw_proc.returncode}). LBRServer ggf. neu starten.")
            self._hw_proc = None
            self.status_label.configure(text="Nicht verbunden")
        if self._moveit_proc is not None and self._moveit_proc.poll() is not None:
            self.log(f"[MoveIt] Prozess beendet (Exit {self._moveit_proc.returncode}).")
            self._moveit_proc = None
        pm.remove_dead()
        self.root.after(2000, self._poll_dead_procs)

    def on_exit(self):
        if messagebox.askyesno("Alles beenden", "Hardware-Verbindung und alle Programme beenden?"):
            self.log("Beende alle Prozesse ...")
            pm.cleanup()
            self.root.destroy()


def main() -> int:
    signal.signal(signal.SIGINT, lambda *_: (pm.cleanup(), sys.exit(0)))
    signal.signal(signal.SIGTERM, lambda *_: (pm.cleanup(), sys.exit(0)))

    try:
        validate_workspace()

        prep = tk.Tk()
        prep.title("Hardware Starter - Vorbereitung")
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
        build_workspace(pm, prep_log)
        prep_log("Lese Executables ...")
        executables = get_executables(prep_log)
        prep_log("Bereit - oeffne Hardware-Fenster ...")
        time.sleep(0.5)
        prep.destroy()

        root = tk.Tk(className="Robotik Hardware Launcher")
        HardwareStarterGUI(root, executables)
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
