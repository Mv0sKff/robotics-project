#!/usr/bin/env python3
"""Starter nur fuer Simulation/Mock."""

from __future__ import annotations

import signal
import sys
import time
import tkinter as tk
from tkinter import messagebox
from tkinter.scrolledtext import ScrolledText

from starter_common import (
    LOG_DIR,
    NAMESPACE,
    PACKAGE_NAME,
    ROBOT_MODEL,
    ROS_DISTRO,
    WORKSPACE,
    ProcessManager,
    build_workspace,
    get_executables,
    is_running,
    launch_program,
    start_mock_system,
    validate_workspace,
)

pm = ProcessManager()


class SimulationStarterGUI:
    def __init__(
        self,
        root: tk.Tk,
        executables: list[str],
        mock_proc,
        moveit_proc,
    ):
        self.root = root
        self.executables = executables
        self._robot_proc = None
        self._robot_name: str | None = None
        self._mock_proc = mock_proc
        self._moveit_proc = moveit_proc

        root.title("Robotik Simulation Starter")
        root.geometry("850x560")
        root.protocol("WM_DELETE_WINDOW", self.on_exit)

        self._build_ui()
        self._poll_dead_procs()
        self.log(f"Logs: {LOG_DIR}")
        self.log("Simulation laeuft: Mock, MoveIt und RViz sind gestartet.")

    def _build_ui(self):
        tk.Label(
            self.root,
            text="Robotik-Projekt: Simulation",
            font=("Arial", 16, "bold"),
        ).pack(padx=12, pady=(12, 4), anchor="w")

        tk.Label(
            self.root,
            text=f"Workspace: {WORKSPACE}   |   Paket: {PACKAGE_NAME}   |   Modell: {ROBOT_MODEL}   |   NS: {NAMESPACE}",
            justify="left",
        ).pack(padx=12, pady=(0, 8), anchor="w")

        main = tk.Frame(self.root)
        main.pack(fill="both", expand=True, padx=12, pady=4)

        left = tk.Frame(main)
        left.pack(side="left", fill="both")
        tk.Label(left, text="Roboterprogramme").pack(anchor="w")
        self.listbox = tk.Listbox(left, width=42, height=16)
        self.listbox.pack(fill="y")
        for exe in self.executables:
            self.listbox.insert(tk.END, exe)
        if self.executables:
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
            text="Auf Simulation starten",
            command=self.start_selected,
            bg="#006400",
            fg="white",
            width=24,
            height=2,
        )
        self.start_btn.pack(side="right", padx=(8, 0))

        self.restart_btn = tk.Button(
            bottom,
            text="Simulation neu starten",
            command=self.restart_simulation,
            width=24,
            height=2,
        )
        self.restart_btn.pack(side="right", padx=(8, 0))

        self.status_label = tk.Label(bottom, text="Bereit")
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
        self.restart_btn.configure(state=state)

    def _get_selected_exe(self) -> str | None:
        if is_running(self._robot_proc):
            messagebox.showinfo("Laeuft bereits", "Warte bis das aktuelle Programm beendet ist.")
            return None
        sel = self.listbox.curselection()
        if not sel:
            messagebox.showwarning("Keine Auswahl", "Bitte waehle zuerst ein Programm aus.")
            return None
        return self.listbox.get(sel[0])

    def start_selected(self):
        exe = self._get_selected_exe()
        if not exe:
            return
        if not is_running(self._mock_proc) or not is_running(self._moveit_proc):
            self.restart_simulation(confirm=False)

        self.status_label.configure(text=f"Simulation: {exe}")
        self._set_buttons("disabled")
        self.log(f"[Simulation] Starte '{exe}'.")
        self._robot_proc = launch_program(pm, exe, "simulation_program", self.log)
        self._robot_name = exe
        self.root.after(500, self._monitor)

    def restart_simulation(self, confirm: bool = True):
        if confirm and not messagebox.askyesno("Simulation neu starten", "Mock, MoveIt und RViz neu starten?"):
            return
        self._set_buttons("disabled")
        self.log("[Simulation] Stoppe laufende Simulation ...")
        pm.terminate(self._moveit_proc)
        pm.terminate(self._mock_proc)
        self.log("[Simulation] Starte Mock, MoveIt und RViz ...")
        self._mock_proc, self._moveit_proc = start_mock_system(pm, self.log)
        self.status_label.configure(text="Bereit")
        self._set_buttons("normal")

    def _monitor(self):
        if self._robot_proc is None:
            self._set_buttons("normal")
            self.status_label.configure(text="Bereit")
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
        self.status_label.configure(text="Bereit")
        self._set_buttons("normal")

    def _poll_dead_procs(self):
        if self._mock_proc is not None and self._mock_proc.poll() is not None:
            self.log(f"[Simulation] Mock beendet (Exit {self._mock_proc.returncode}).")
            self._mock_proc = None
        if self._moveit_proc is not None and self._moveit_proc.poll() is not None:
            self.log(f"[Simulation] MoveIt/RViz beendet (Exit {self._moveit_proc.returncode}).")
            self._moveit_proc = None
        pm.remove_dead()
        self.root.after(2000, self._poll_dead_procs)

    def on_exit(self):
        if messagebox.askyesno("Alles beenden", "Simulation und alle Programme beenden?"):
            self.log("Beende alle Prozesse ...")
            pm.cleanup()
            self.root.destroy()


def main() -> int:
    signal.signal(signal.SIGINT, lambda *_: (pm.cleanup(), sys.exit(0)))
    signal.signal(signal.SIGTERM, lambda *_: (pm.cleanup(), sys.exit(0)))

    try:
        validate_workspace()

        prep = tk.Tk()
        prep.title("Simulation Starter - Vorbereitung")
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
        prep_log("Starte Mock, MoveIt und RViz ...")
        mock_proc, moveit_proc = start_mock_system(pm, prep_log)
        prep_log("Bereit - oeffne Hauptfenster ...")
        time.sleep(0.5)
        prep.destroy()

        root = tk.Tk(className="Robotik Simulation Launcher")
        SimulationStarterGUI(root, executables, mock_proc, moveit_proc)
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
