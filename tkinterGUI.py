import tkinter as tk
from tkinter import ttk
_early_root = tk.Tk()
_early_root.withdraw()
import math
from spatialmath import SE3
from math import pi
import threading, time

class JointControlUI:
    def __init__(self, botSystem: 'newRobotSystem'):
        # CONTAINS ALL ROBOT INFO 
        self.robotSystem = botSystem
        self.active_robot = botSystem.rebel  # Default active robot
        # Reuse early root
        global _early_root
        self.root = _early_root
        self.root.deiconify()

        # --- Threading info ---
        self.stop_event = threading.Event()
        self.run_thread = None

        # --- Scheduled UI update handle (for debouncing slider events) ---
        self._update_job = None      # after() id
        self._debounce_ms = 30       # tweak if you want snappier/smoother

        # --- Main window ---
        self.root.title("Industrial Robotics Control")
        self.root.geometry("600x500")
        self.root.configure(bg="#0F1115")

        # --- Global style ---
        style = ttk.Style()
        style.theme_use("default")
        style.configure("TFrame", background="#171A21")
        style.configure("TLabel", foreground="#E8EDF2", background="#171A21", font=("Segoe UI", 10))
        style.configure("TButton", font=("Segoe UI", 10, "bold"), padding=8)
        style.configure("Title.TLabel", font=("Segoe UI", 12, "bold"))
        style.configure("TScale", background="#171A21")
        style.configure("Run.TButton", foreground="#0F1115", background="#77E0A6")
        style.map("Run.TButton", background=[("active", "#60C893")])
        style.configure("Reset.TButton", foreground="#0F1115", background="#FFB454")
        style.map("Reset.TButton", background=[("active", "#E5A24B")])
        style.configure("EStop.TButton", foreground="white", background="#FF3B30")
        style.map("EStop.TButton", background=[("active", "#E2362B")])

        # --- Layout structure ---
        self.root.rowconfigure(0, weight=0)
        self.root.rowconfigure(1, weight=1)
        self.root.columnconfigure(0, weight=1)

        # --- Top control bar ---
        top = ttk.Frame(self.root, padding=12)
        top.grid(row=0, column=0, sticky="ew")
        for i in range(3):
            top.columnconfigure(i, weight=1)

        ttk.Button(top, text="Run", style="Run.TButton", command=self.on_run).grid(row=0, column=0, sticky="ew", padx=5)
        ttk.Button(top, text="Reset", style="Reset.TButton", command=self.on_reset).grid(row=0, column=1, sticky="ew", padx=5)
        ttk.Button(top, text="Emergency Stop", style="EStop.TButton", command=self.on_estop).grid(row=0, column=2, sticky="ew", padx=5)

        # -- Dropdown menu --
        self.robot_map = {}
        self.robot_map["ReBeL"] = self.robotSystem.rebel
        self.robot_map["UR3"] = self.robotSystem.ur3
        self.robot_map["GP4"] = self.robotSystem.gp4


        sel_row = ttk.Frame(self.root, padding=(12, 0, 12, 0))
        sel_row.grid(row=0, column=0, sticky="e", pady=(52, 0))  # sits under buttons
        ttk.Label(sel_row, text="Robot:", style="TLabel").grid(row=0, column=0, padx=(0, 6))
        self.robot_choice = tk.StringVar(value=next(iter(self.robot_map)) if self.robot_map else "")
        self.robot_select = ttk.Combobox(
            sel_row,
            textvariable=self.robot_choice,
            values=list(self.robot_map.keys()),
            state="readonly",
            width=10
        )
        self.robot_select.grid(row=0, column=1, sticky="w")
        self.robot_select.bind("<<ComboboxSelected>>", lambda _e: self._load_robot(self.robot_choice.get()))
        # --- Bottom panel (sliders) ---
        bottom = ttk.Frame(self.root, padding=15)
        bottom.grid(row=1, column=0, sticky="nsew")
        bottom.columnconfigure(0, weight=1)
        bottom.columnconfigure(1, weight=1)

        ttk.Label(bottom, text="Teach Panel — Joint Angles (°)", style="Title.TLabel").grid(row=0, column=0, columnspan=2, pady=(0, 10))

        self.slider_vars = []
        self.value_strs = []
        self.scales = [] 
        # Create 6 sliders, 2 columns
        for i in range(6):
            col = i % 2
            row = (i // 2) * 2 + 1

            # container for "Joint X | value"
            joint_box = ttk.Frame(bottom)
            joint_box.grid(row=row, column=col, sticky="ew", pady=(0, 2))
            joint_box.columnconfigure(0, weight=1)
            joint_box.columnconfigure(1, weight=0)

            ttk.Label(joint_box, text=f"Joint {i+1}").grid(row=0, column=0, sticky="w")

            var = tk.DoubleVar(master=self.root, value=0.0)
            sv  = tk.StringVar(master=self.root, value="0.0°")
            ttk.Label(joint_box, textvariable=sv).grid(row=0, column=1, sticky="e")

            # Each slider calls self._on_slider_move continuously while dragging
            scale = ttk.Scale(
                bottom,
                from_=-180, to=180,
                orient="horizontal",
                variable=var,
                command=lambda v, idx=i, s=sv: self._on_slider_move(idx, v, s)
            )
            scale.grid(row=row+1, column=col, sticky="ew", pady=(0, 10))

            self.slider_vars.append(var)
            self.value_strs.append(sv)
            self.scales.append(scale)

        # Initial push to robot so UI=robot
        self._apply_current_angles()

        self.root.mainloop()

    # --- Button callbacks ---
    def degrees_to_radians(self, angle_list):
        return [math.radians(angle) for angle in angle_list]

    def on_run(self):
        if (self.run_thread and self.run_thread.is_alive()) or self.stop_event.is_set():
            return
        self.stop_event.clear()
        self.run_thread = threading.Thread(target=self._run_motion, daemon=True)
        self.run_thread.start()

    def on_reset(self):
        # Set all sliders back to zero; this will auto-apply via debounce
        for v in self.slider_vars:
            v.set(0.0)
        print("Reset pressed. All joints set to 0.")
        self.stop_event.clear()

    def on_estop(self):
        print("!!! EMERGENCY STOP ACTIVATED !!!")
        self.stop_event.set()

    # --- Slider handling (continuous updates) ---
    def _on_slider_move(self, idx: int, val_str: str, label_var: tk.StringVar):
        """Called by every slider while it moves. Updates label and schedules a debounced robot update."""
        try:
            val = float(val_str)
        except ValueError:
            return
        label_var.set(f"{val:.1f}°")
        self._schedule_apply()

    def _schedule_apply(self):
        """Debounce: cancel any pending apply and schedule a fresh one very soon."""
        if self._update_job is not None:
            try:
                self.root.after_cancel(self._update_job)
            except Exception:
                pass
        self._update_job = self.root.after(self._debounce_ms, self._apply_current_angles)

    def _apply_current_angles(self):
        self._update_job = None
        angles_deg = self.current_angles()
        angles_rad = [math.radians(a) for a in angles_deg]

        try:
            # Set on the currently selected robot
            self.active_robot.q = angles_rad
        except Exception as e:
            print(f"Set q warn: {e}")

        try:
            self.robotSystem.env.step()
        except Exception as e:
            print(f"env.step() warn: {e}")

        # Optional EE pose update (uncomment if you want the same behavior)
        self._update_ee_pose()

    def _run_motion(self):
        # Example: RMRC-like stepping loop (runs in thread)
        self.robotSystem.simulation()
        self.root.after(0, lambda: print("Motion finished or stopped."))

    # --- Helper ---
    def current_angles(self):
        """Return all slider values in degrees as a list [j1..j6]."""
        return [float(var.get()) for var in self.slider_vars]
        
    def _load_robot(self, name: str):
        """Switch active robot, update slider ranges from qlim, and sync values from current q."""
        self.active_name = name
        self.active_robot = self.robot_map[name]

            # Configure slider ranges from the robot's qlim (if present) in degrees
        self._set_slider_range_from_qlim(self.active_robot)

            # Sync sliders to the robot's current q (in deg)
        current_q_deg = self._q_to_degrees(getattr(self.active_robot, "q", [0]*6))
        for i, v in enumerate(current_q_deg[:6]):
            self.slider_vars[i].set(v)  # this will trigger the debounced apply

    def _set_slider_range_from_qlim(self, robot):
        """Set each slider's from_/to to match the robot's joint limits (degrees), else default +/-180."""
        qlim = getattr(robot, "qlim", None)
        for i, scale in enumerate(self.scales):
            if (qlim is not None) and (i < qlim.shape[1]):
                lo_deg = math.degrees(float(qlim[0, i]))
                hi_deg = math.degrees(float(qlim[1, i]))
                # Ensure lo <= hi
                lo, hi = (min(lo_deg, hi_deg), max(lo_deg, hi_deg))
            else:
                lo, hi = -180.0, 180.0
            scale.configure(from_=lo, to=hi)

    def _q_to_degrees(self, q_rad):
        out = []
        for i in range(6):
            try:
                out.append(math.degrees(float(q_rad[i])))
            except Exception:
                out.append(0.0)
        return out

    def _update_ee_pose(self):
        """Try to keep EE pose logic consistent across robots (optional)."""
        if self.active_robot.name == 'rebel':
            self.robotSystem.rebelEE.T  = self.robotSystem.rebel.fkine(self.robotSystem.rebel.q).A @ SE3.Ry(pi/2).A
        elif self.active_robot.name == 'UR3':
            self.robotSystem.ur3EE.T  = self.robotSystem.ur3.fkine(self.robotSystem.ur3.q).A
        elif self.active_robot.name == 'GP4':
            self.robotSystem.topperEE.T = self.robotSystem.gp4.fkine(self.robotSystem.gp4.q).A 
        else:
            print("No EE update logic for this robot.")
