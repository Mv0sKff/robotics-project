#!/usr/bin/env python3
"""Launcher for simulation/mock mode."""

from __future__ import annotations

import signal
import sys
import time
import tkinter as tk
from tkinter import messagebox
from tkinter.scrolledtext import ScrolledText

from starter_common import (
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

        root.title("Robotics Simulation Launcher")
        root.geometry("850x560")
        root.protocol("WM_DELETE_WINDOW", self.on_exit)

        self._setup_konami()
        self._build_ui()
        self._poll_dead_procs()
        self.log("Simulation is running. ROS commands open in visible terminal windows.")

    def _build_ui(self):
        tk.Label(
            self.root,
            text="Robotics Project: Simulation",
            font=("Arial", 16, "bold"),
        ).pack(padx=12, pady=(12, 4), anchor="w")

        tk.Label(
            self.root,
            text=f"Workspace: {WORKSPACE}   |   Package: {PACKAGE_NAME}   |   Model: {ROBOT_MODEL}   |   NS: {NAMESPACE}",
            justify="left",
        ).pack(padx=12, pady=(0, 8), anchor="w")

        main = tk.Frame(self.root)
        main.pack(fill="both", expand=True, padx=12, pady=4)

        left = tk.Frame(main)
        left.pack(side="left", fill="both")
        tk.Label(left, text="Robot programs").pack(anchor="w")
        self.listbox = tk.Listbox(left, width=42, height=16)
        self.listbox.pack(fill="y")
        for exe in self.executables:
            self.listbox.insert(tk.END, exe)
        if self.executables:
            self.listbox.selection_set(0)

        right = tk.Frame(main)
        right.pack(side="right", fill="both", expand=True, padx=(14, 0))
        tk.Label(right, text="Status / Notes").pack(anchor="w")
        self.status_text = ScrolledText(right, height=16, state="disabled")
        self.status_text.pack(fill="both", expand=True)

        bottom = tk.Frame(self.root)
        bottom.pack(fill="x", padx=12, pady=12)

        tk.Button(
            bottom,
            text="Stop all",
            command=self.on_exit,
            bg="#b00020",
            fg="white",
            width=18,
            height=2,
        ).pack(side="left")

        self.start_btn = tk.Button(
            bottom,
            text="Start on simulation",
            command=self.start_selected,
            bg="#006400",
            fg="white",
            width=24,
            height=2,
        )
        self.start_btn.pack(side="right", padx=(8, 0))

        self.restart_btn = tk.Button(
            bottom,
            text="Restart simulation",
            command=self.restart_simulation,
            width=24,
            height=2,
        )
        self.restart_btn.pack(side="right", padx=(8, 0))

        self.status_label = tk.Label(bottom, text="Ready")
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

    def _setup_konami(self):
        self._konami = ["Up", "Up", "Down", "Down", "Left", "Right", "Left", "Right", "a", "b"]
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
            tk.Label(
                self.root,
                text=random.choice(msgs),
                font=("Arial", random.randint(12, 28), "bold"),
                bg="yellow",
                fg="black",
            )
            for _ in range(77)
        ]
        for lbl in labels:
            lbl.place(
                x=random.randint(0, max(1, self.root.winfo_width() - 180)),
                y=random.randint(0, max(1, self.root.winfo_height() - 40)),
            )
        self.root.after(10_000, lambda: [lbl.destroy() for lbl in labels])

    def _get_selected_exe(self) -> str | None:
        if is_running(self._robot_proc):
            messagebox.showinfo("Already running", "Wait until the current program has finished.")
            return None
        sel = self.listbox.curselection()
        if not sel:
            messagebox.showwarning("No selection", "Please select a program first.")
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
        self.log(f"[Simulation] Starting '{exe}' in a terminal window.")
        self._robot_proc = launch_program(pm, exe, "Simulation Program", self.log)
        self._robot_name = exe
        self.root.after(500, self._monitor)

    def restart_simulation(self, confirm: bool = True):
        if confirm and not messagebox.askyesno("Restart simulation", "Restart Mock, MoveIt, and RViz?"):
            return
        self._set_buttons("disabled")
        self.log("[Simulation] Stopping running simulation ...")
        pm.terminate(self._moveit_proc)
        pm.terminate(self._mock_proc)
        self.log("[Simulation] Starting Mock, MoveIt, and RViz ...")
        self._mock_proc, self._moveit_proc = start_mock_system(pm, self.log)
        self.status_label.configure(text="Ready")
        self._set_buttons("normal")

    def _monitor(self):
        if self._robot_proc is None:
            self._set_buttons("normal")
            self.status_label.configure(text="Ready")
            return
        rc = self._robot_proc.poll()
        if rc is None:
            self.root.after(1000, self._monitor)
            return
        self.log(f"Program finished: {self._robot_name} (exit {rc})")
        if self._robot_proc in pm._procs:
            pm._procs.remove(self._robot_proc)
        self._robot_proc = None
        self._robot_name = None
        self.status_label.configure(text="Ready")
        self._set_buttons("normal")

    def _poll_dead_procs(self):
        if self._mock_proc is not None and self._mock_proc.poll() is not None:
            self.log(f"[Simulation] Mock exited (exit {self._mock_proc.returncode}).")
            self._mock_proc = None
        if self._moveit_proc is not None and self._moveit_proc.poll() is not None:
            self.log(f"[Simulation] MoveIt/RViz exited (exit {self._moveit_proc.returncode}).")
            self._moveit_proc = None
        pm.remove_dead()
        self.root.after(2000, self._poll_dead_procs)

    def on_exit(self):
        if messagebox.askyesno("Stop all", "Stop simulation and all programs?"):
            self.log("Stopping all processes ...")
            pm.cleanup()
            self.root.destroy()


def main() -> int:
    signal.signal(signal.SIGINT, lambda *_: (pm.cleanup(), sys.exit(0)))
    signal.signal(signal.SIGTERM, lambda *_: (pm.cleanup(), sys.exit(0)))

    try:
        validate_workspace()

        prep = tk.Tk()
        prep.title("Simulation Launcher - Preparation")
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
        prep_log(f"ROS distro: {ROS_DISTRO}  |  Package: {PACKAGE_NAME}")
        prep_log("Building workspace ...")
        build_workspace(pm, prep_log)
        prep_log("Reading executables ...")
        executables = get_executables(prep_log)
        prep_log("Starting Mock, MoveIt, and RViz ...")
        mock_proc, moveit_proc = start_mock_system(pm, prep_log)
        prep_log("Ready - opening main window ...")
        time.sleep(0.5)
        prep.destroy()

        root = tk.Tk(className="Robotics Simulation Launcher")
        SimulationStarterGUI(root, executables, mock_proc, moveit_proc)
        root.mainloop()
    except Exception as exc:
        pm.cleanup()
        try:
            messagebox.showerror("Error", str(exc))
        except Exception:
            print(f"Error: {exc}", file=sys.stderr)
        return 1
    finally:
        pm.cleanup()
    return 0


if __name__ == "__main__":
    sys.exit(main())
