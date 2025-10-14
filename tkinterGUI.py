import tkinter as tk
from tkinter import ttk
_early_root = tk.Tk()
_early_root.withdraw()
import math
from spatialmath import SE3
from robotSystem import newRobotSystem
from math import pi
import threading, time
import numpy as np

class JointControlUI:
    def __init__(self, botSystem: 'newRobotSystem', stop_event: threading.Event):
        # CONTAINS ALL ROBOT INFO 
        self.robotSystem = botSystem
        self.active_robot = botSystem.rebel  # Default active robot

        # Reuse early root
        global _early_root
        self.root = _early_root
        self.root.deiconify()

        # --- Threading info ---
        self.stop_event = stop_event
        self.stop_event.clear()
        self.run_thread = None

        # --- Scheduled UI update handle (for debouncing slider events) ---
        self._update_job = None      # after() id
        self._debounce_ms = 30       # tweak if you want snappier/smoother

        # --- Main window ---
        self.root.title("Industrial Robotics Control")
        self.root.geometry("820x620")
        self.root.configure(bg="#0F1115")

        # --- Global style ---
        style = ttk.Style()
        style.theme_use("default")
        style.configure("TFrame", background="#171A21")
        style.configure("TLabel", foreground="#E8EDF2", background="#171A21", font=("Segoe UI", 10))
        style.configure("TButton", font=("Segoe UI", 10, "bold"), padding=8)
        style.configure("Title.TLabel", font=("Segoe UI", 12, "bold"))
        style.configure("SubTitle.TLabel", font=("Segoe UI", 10, "bold"))
        style.configure("TScale", background="#171A21")
        style.configure("Run.TButton", foreground="#0F1115", background="#77E0A6")
        style.map("Run.TButton", background=[("active", "#60C893")])
        style.configure("Reset.TButton", foreground="#0F1115", background="#FFB454")
        style.map("Reset.TButton", background=[("active", "#E5A24B")])
        style.configure("EStop.TButton", foreground="white", background="#FF3B30")
        style.map("EStop.TButton", background=[("active", "#E2362B")])

        # --- Layout structure ---
        self.root.rowconfigure(0, weight=0)  # top controls
        self.root.rowconfigure(1, weight=1)  # bottom panels
        self.root.columnconfigure(0, weight=1)

        # --- Top control bar ---
        top = ttk.Frame(self.root, padding=12)
        top.grid(row=0, column=0, sticky="ew")
        for i in range(4):
            top.columnconfigure(i, weight=1)

        ttk.Button(top, text="Run", style="Run.TButton", command=self.on_run).grid(row=0, column=0, sticky="ew", padx=5)
        ttk.Button(top, text="Reset", style="Reset.TButton", command=self.on_reset).grid(row=0, column=1, sticky="ew", padx=5)
        ttk.Button(top, text="Emergency Stop", style="EStop.TButton", command=self.on_estop).grid(row=0, column=2, sticky="ew", padx=5)

        # -- Robot dropdown --
        self.robot_map = {
            "ReBeL": self.robotSystem.rebel,
            "UR3": self.robotSystem.ur3,
            "GP4": self.robotSystem.gp4
        }

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

        # --- Bottom container ---
        bottom = ttk.Frame(self.root, padding=15)
        bottom.grid(row=1, column=0, sticky="nsew")
        bottom.columnconfigure(0, weight=1)

        # === Panel Mode Toggle (Joint / Cartesian / Simulation Control) ===
        toggle_bar = ttk.Frame(bottom)
        toggle_bar.grid(row=0, column=0, sticky="ew", pady=(0, 12))
        for i in range(4):
            toggle_bar.columnconfigure(i, weight=1)

        ttk.Label(toggle_bar, text="Panel:", style="SubTitle.TLabel").grid(row=0, column=0, sticky="w")

        self.mode_var = tk.StringVar(value="joint")
        self.btn_joint = ttk.Radiobutton(toggle_bar, text="Joint Angles", value="joint", variable=self.mode_var,
                                         command=lambda: self._show_mode("joint"))
        self.btn_cart  = ttk.Radiobutton(toggle_bar, text="Cartesian XYZ", value="cart", variable=self.mode_var,
                                         command=lambda: self._show_mode("cart"))
        self.btn_sim   = ttk.Radiobutton(toggle_bar, text="Simulation Control", value="sim", variable=self.mode_var,
                                         command=lambda: self._show_mode("sim"))
        self.btn_joint.grid(row=0, column=1, sticky="e", padx=(8, 4))
        self.btn_cart.grid(row=0, column=2, sticky="w", padx=(4, 8))
        self.btn_sim.grid(row=0, column=3, sticky="w", padx=(4, 8))

        # === Panels stack ===
        self.panel_stack = ttk.Frame(bottom)
        self.panel_stack.grid(row=1, column=0, sticky="nsew")
        self.panel_stack.columnconfigure(0, weight=1)
        self.panel_stack.rowconfigure(0, weight=1)

        # Build all panels
        self._build_joint_panel(parent=self.panel_stack)
        self._build_cartesian_panel(parent=self.panel_stack)
        self._build_sim_panel(parent=self.panel_stack)

        # Show initial mode
        self._show_mode("joint")

        # Initial push to robot so UI=robot
        self._apply_current_angles()

        # Start feedback loop for position readouts
        self._start_feedback_loop()

        self.root.mainloop()

    # ---------- Build Panels ----------
    def _build_joint_panel(self, parent):
        self.joint_frame = ttk.Frame(parent)
        self.joint_frame.grid(row=0, column=0, sticky="nsew")
        self.joint_frame.columnconfigure(0, weight=1)
        self.joint_frame.columnconfigure(1, weight=1)

        ttk.Label(self.joint_frame, text="Teach Panel — Joint Angles (°)", style="Title.TLabel")\
            .grid(row=0, column=0, columnspan=2, pady=(0, 10))

        self.slider_vars = []
        self.value_strs = []
        self.scales = []

        # Create 6 sliders, 2 columns
        for i in range(6):
            col = i % 2
            row = (i // 2) * 2 + 1

            joint_box = ttk.Frame(self.joint_frame)
            joint_box.grid(row=row, column=col, sticky="ew", pady=(0, 2))
            joint_box.columnconfigure(0, weight=1)
            joint_box.columnconfigure(1, weight=0)

            ttk.Label(joint_box, text=f"Joint {i+1}").grid(row=0, column=0, sticky="w")

            # Default joint angles (deg): [0, 90, 90, 0, 0, 90]
            default_angles = [0, 90, 90, 0, 0, 90]

# Inside the for-loop for i in range(6):
            initial_val = default_angles[i]
            var = tk.DoubleVar(master=self.root, value=initial_val)
            sv  = tk.StringVar(master=self.root, value=f"{initial_val:.1f}°")



            ttk.Label(joint_box, textvariable=sv).grid(row=0, column=1, sticky="e")

            scale = ttk.Scale(
                self.joint_frame,
                from_=-180, to=180,
                orient="horizontal",
                variable=var,
                command=lambda v, idx=i, s=sv: self._on_slider_move(idx, v, s)
            )
            scale.grid(row=row+1, column=col, sticky="ew", pady=(0, 10))

            self.slider_vars.append(var)
            self.value_strs.append(sv)
            self.scales.append(scale)

    def _build_cartesian_panel(self, parent):
        """Button-driven XYZ jogging (front-end only)."""
        self.cart_frame = ttk.Frame(parent)
        # shown via _show_mode()
        self.cart_frame.columnconfigure(0, weight=1)
        self.cart_frame.columnconfigure(1, weight=1)
        self.cart_frame.columnconfigure(2, weight=1)

        ttk.Label(self.cart_frame, text="Teach Panel — Cartesian (X, Y, Z)", style="Title.TLabel")\
            .grid(row=0, column=0, columnspan=3, pady=(0, 10), sticky="w")

        # XYZ state (front-end only)
        self.cart_vars = {"X": tk.DoubleVar(value=0.00),
                          "Y": tk.DoubleVar(value=0.00),
                          "Z": tk.DoubleVar(value=0.20)}
        self.cart_val_strs = {k: tk.StringVar(value=f"{self.cart_vars[k].get():.3f} m") for k in self.cart_vars}

        # Step size selector (meters)
        step_row = ttk.Frame(self.cart_frame)
        step_row.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(0, 8))
        ttk.Label(step_row, text="Step:", style="SubTitle.TLabel").grid(row=0, column=0, padx=(0, 8))
        self.step_var_m = tk.StringVar(value="0.010")  # 10 mm default
        self.step_select = ttk.Combobox(step_row, textvariable=self.step_var_m,
                                        values=["0.001", "0.005", "0.010", "0.020", "0.050", "0.100"],
                                        width=6, state="readonly")
        self.step_select.grid(row=0, column=1, sticky="w")

        # Readout
        readout = ttk.Frame(self.cart_frame)
        readout.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(0, 12))
        for i in range(6):
            readout.columnconfigure(i, weight=1)

        ttk.Label(readout, text="X:").grid(row=0, column=0, sticky="e")
        ttk.Label(readout, textvariable=self.cart_val_strs["X"]).grid(row=0, column=1, sticky="w")
        ttk.Label(readout, text="Y:").grid(row=0, column=2, sticky="e")
        ttk.Label(readout, textvariable=self.cart_val_strs["Y"]).grid(row=0, column=3, sticky="w")
        ttk.Label(readout, text="Z:").grid(row=0, column=4, sticky="e")
        ttk.Label(readout, textvariable=self.cart_val_strs["Z"]).grid(row=0, column=5, sticky="w")

        # XY pad and Z column
        pad = ttk.Frame(self.cart_frame)
        pad.grid(row=3, column=0, columnspan=3, sticky="nsew")
        for c in range(5):
            pad.columnconfigure(c, weight=1)
        for r in range(3):
            pad.rowconfigure(r, weight=1)

        ttk.Button(pad, text="Y +", command=lambda: self._cart_step("Y", +1)).grid(row=0, column=1, sticky="nsew", padx=4, pady=4)
        ttk.Button(pad, text="X −", command=lambda: self._cart_step("X", -1)).grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        ttk.Label(pad, text="XY", style="SubTitle.TLabel").grid(row=1, column=1, sticky="nsew", padx=4, pady=4)
        ttk.Button(pad, text="X +", command=lambda: self._cart_step("X", +1)).grid(row=1, column=2, sticky="nsew", padx=4, pady=4)
        ttk.Button(pad, text="Y −", command=lambda: self._cart_step("Y", -1)).grid(row=2, column=1, sticky="nsew", padx=4, pady=4)

        ttk.Button(pad, text="Z +", command=lambda: self._cart_step("Z", +1)).grid(row=0, column=4, sticky="nsew", padx=4, pady=4)
        ttk.Label(pad, text="Z", style="SubTitle.TLabel").grid(row=1, column=4, sticky="nsew", padx=4, pady=4)
        ttk.Button(pad, text="Z −", command=lambda: self._cart_step("Z", -1)).grid(row=2, column=4, sticky="nsew", padx=4, pady=4)

        # Quick actions (still front-end only)
        qa = ttk.Frame(self.cart_frame)
        qa.grid(row=6, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        qa.columnconfigure(0, weight=1)
        qa.columnconfigure(1, weight=1)
        ttk.Button(qa, text="Zero XY (stub)", command=lambda: self._cart_set(x=0.0, y=0.0, keep="Z")).grid(row=0, column=0, sticky="ew", padx=(0, 6))
        ttk.Button(qa, text="Home Z=0.20 (stub)", command=lambda: self._cart_set(z=0.20, keep="XY")).grid(row=0, column=1, sticky="ew", padx=(6, 0))

    def _build_sim_panel(self, parent):
        """Simulation Control panel: test-tube specimen choices + live XYZ feedback per robot."""
        self.sim_frame = ttk.Frame(parent)
        # shown via _show_mode()
        self.sim_frame.columnconfigure(0, weight=1)
        self.sim_frame.columnconfigure(1, weight=1)

        ttk.Label(self.sim_frame, text="Simulation Control", style="Title.TLabel")\
            .grid(row=0, column=0, columnspan=2, pady=(0, 10), sticky="w")

        # --- Specimen selection for 6 test tubes ---
        ttk.Label(self.sim_frame, text="Test Tubes — Specimen Selection", style="SubTitle.TLabel")\
            .grid(row=1, column=0, columnspan=2, sticky="w", pady=(0, 6))

        tube_box = ttk.Frame(self.sim_frame)
        tube_box.grid(row=2, column=0, columnspan=2, sticky="ew")
        for c in range(3):
            tube_box.columnconfigure(c, weight=1)

        self.specimen_vars = []  # list of StringVars "1" or "2"
        for i in range(6):
            col = i % 3
            row = i // 3
            cell = ttk.Labelframe(tube_box, text=f"Tube {i+1}")
            cell.grid(row=row, column=col, padx=6, pady=6, sticky="nsew")
            cell.columnconfigure(0, weight=1)
            v = tk.StringVar(value="1")
            self.specimen_vars.append(v)
            ttk.Radiobutton(cell, text="Specimen 1", value="1", variable=v).grid(row=0, column=0, sticky="w", padx=6, pady=2)
            ttk.Radiobutton(cell, text="Specimen 2", value="2", variable=v).grid(row=1, column=0, sticky="w", padx=6, pady=2)

        # --- Live XYZ feedback per robot ---
        ttk.Label(self.sim_frame, text="Robot Pose Feedback (EE XYZ, m)", style="SubTitle.TLabel")\
            .grid(row=3, column=0, columnspan=2, sticky="w", pady=(12, 6))

        fb = ttk.Frame(self.sim_frame)
        fb.grid(row=4, column=0, columnspan=2, sticky="ew")
        for c in range(7):
            fb.columnconfigure(c, weight=1)

        # Headers
        ttk.Label(fb, text="Robot").grid(row=0, column=0, sticky="w")
        ttk.Label(fb, text="X").grid(row=0, column=2, sticky="w")
        ttk.Label(fb, text="Y").grid(row=0, column=4, sticky="w")
        ttk.Label(fb, text="Z").grid(row=0, column=6, sticky="w")

        # Rows for each robot
        self.pose_labels = {}  # {"ReBeL": (xVar,yVar,zVar), ...}
        row = 1
        for name in ["ReBeL", "UR3", "GP4"]:
            ttk.Label(fb, text=name).grid(row=row, column=0, sticky="w")
            xsv = tk.StringVar(value="0.000")
            ysv = tk.StringVar(value="0.000")
            zsv = tk.StringVar(value="0.000")
            ttk.Label(fb, text="X:").grid(row=row, column=1, sticky="e")
            ttk.Label(fb, textvariable=xsv).grid(row=row, column=2, sticky="w")
            ttk.Label(fb, text="Y:").grid(row=row, column=3, sticky="e")
            ttk.Label(fb, textvariable=ysv).grid(row=row, column=4, sticky="w")
            ttk.Label(fb, text="Z:").grid(row=row, column=5, sticky="e")
            ttk.Label(fb, textvariable=zsv).grid(row=row, column=6, sticky="w")
            self.pose_labels[name] = (xsv, ysv, zsv)
            row += 1

        # Spacer / placeholder for future sim buttons
        ttk.Frame(self.sim_frame).grid(row=5, column=0, columnspan=2, sticky="nsew", pady=(10,0))

    # ---------- Mode switching ----------
    def _show_mode(self, mode: str):
        """Show one panel and hide the others."""
        # Hide all
        for f in [getattr(self, 'joint_frame', None),
                  getattr(self, 'cart_frame', None),
                  getattr(self, 'sim_frame', None)]:
            if f is not None:
                f.grid_remove()

        if mode == "joint":
            self.joint_frame.grid(row=0, column=0, sticky="nsew")
        elif mode == "cart":
            self.cart_frame.grid(row=0, column=0, sticky="nsew")
        else:
            self.sim_frame.grid(row=0, column=0, sticky="nsew")

    # ---------- Button callbacks ----------
    def degrees_to_radians(self, angle_list):
        return [math.radians(angle) for angle in angle_list]

    def on_run(self):
        if (self.run_thread and self.run_thread.is_alive()) or self.stop_event.is_set():
            return
        self.stop_event.clear()
        self.run_thread = threading.Thread(target=self._run_motion, daemon=True)
        self.run_thread.start()

    def on_reset(self):
        # Reset joint sliders
        for v in getattr(self, "slider_vars", []):
            v.set(0.0)
        # Reset Cartesian readout to defaults
        if hasattr(self, "cart_vars"):
            self.cart_vars["X"].set(0.0)
            self.cart_vars["Y"].set(0.0)
            self.cart_vars["Z"].set(0.20)
            for k in self.cart_vars:
                self.cart_val_strs[k].set(f"{self.cart_vars[k].get():.3f} m")
            if hasattr(self, "step_var_m"):
                self.step_var_m.set("0.010")  # 10 mm
        # Reset specimens to Specimen 1
        for v in getattr(self, "specimen_vars", []):
            v.set("1")

        print("Reset pressed. Joint=0°, Cartesian defaults, Specimens=1.")
        self.stop_event.clear()

    def on_estop(self):
        print("!!! EMERGENCY STOP ACTIVATED !!!")
        self.stop_event.set()

    # ---------- Joint slider handling ----------
    def _on_slider_move(self, idx: int, val_str: str, label_var: tk.StringVar):
        try:
            val = float(val_str)
        except ValueError:
            return
        label_var.set(f"{val:.1f}°")
        self._schedule_apply()

    def _schedule_apply(self):
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
            self.active_robot.q = angles_rad
        except Exception as e:
            print(f"Set q warn: {e}")

        try:
            self.robotSystem.env.step()
        except Exception as e:
            print(f"env.step() warn: {e}")

        self._update_ee_pose()

    def _run_motion(self):
        # Example: RMRC-like stepping loop (runs in thread)
        print("SIMULATION STARTED")
        self.robotSystem.simulation()
        self.root.after(0, lambda: print("Motion finished or stopped."))

    # ---------- Cartesian jogging (front-end only) ----------
    def _cart_step(self, axis: str, direction: int):
        try:
            step = float(self.step_var_m.get()) if hasattr(self, "step_var_m") else 0.01
        except ValueError:
            step = 0.01
        current = self.cart_vars[axis].get()
        new_val = current + (direction * step)
        self.cart_vars[axis].set(new_val)
        self.cart_val_strs[axis].set(f"{new_val:.3f} m")

    def _cart_set(self, x=None, y=None, z=None, keep=""):
        if x is not None and "X" not in keep:
            self.cart_vars["X"].set(x); self.cart_val_strs["X"].set(f"{x:.3f} m")
        if y is not None and "Y" not in keep:
            self.cart_vars["Y"].set(y); self.cart_val_strs["Y"].set(f"{y:.3f} m")
        if z is not None and "Z" not in keep:
            self.cart_vars["Z"].set(z); self.cart_val_strs["Z"].set(f"{z:.3f} m")

    # ---------- Helpers ----------
    def current_angles(self):
        return [float(var.get()) for var in getattr(self, "slider_vars", [])]

    def _load_robot(self, name: str):
        self.active_name = name
        self.active_robot = self.robot_map[name]
        self._set_slider_range_from_qlim(self.active_robot)
        current_q_deg = self._q_to_degrees(getattr(self.active_robot, "q", [0]*6))
        for i, v in enumerate(current_q_deg[:6]):
            self.slider_vars[i].set(v)

    def _set_slider_range_from_qlim(self, robot):
        qlim = getattr(robot, "qlim", None)
        for i, scale in enumerate(self.scales):
            if (qlim is not None) and (i < qlim.shape[1]):
                lo_deg = math.degrees(float(qlim[0, i]))
                hi_deg = math.degrees(float(qlim[1, i]))
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
        """Keep EE pose visuals consistent across robots."""
        if self.active_robot.name == 'rebel':
            self.robotSystem.rebelEE.T  = self.robotSystem.rebel.fkine(self.robotSystem.rebel.q).A @ SE3.Ry(pi/2).A
        elif self.active_robot.name == 'UR3':
            self.robotSystem.ur3EE.T  = self.robotSystem.ur3.fkine(self.robotSystem.ur3.q).A @ self.robotSystem.ur3eeToolOffset
        elif self.active_robot.name == 'GP4':
            self.robotSystem.gp4EE.T = self.robotSystem.gp4.fkine(self.robotSystem.gp4.q).A @ self.robotSystem.gp4eeToolOffset
        else:
            print("No EE update logic for this robot.")

    # ---------- Live feedback loop ----------
    def _start_feedback_loop(self):
        """Refresh the XYZ readouts periodically."""
        # create an after() loop
        self._update_feedback_positions()
        self.root.after(200, self._start_feedback_loop)

    def _update_feedback_positions(self):
        """Reads fkine(q) for each robot and updates stringvars. Front-end only."""
        # Helper to safely compute X,Y,Z from robot fkine
        def xyz_of(robot):
            try:
                T = robot.fkine(robot.q)
                # spatialmath SE3: T.t is (3,), T.A[:3,3] also works
                if hasattr(T, 't'):
                    xyz = np.array(T.t, dtype=float).reshape(3)
                else:
                    xyz = np.array(T.A[:3, 3], dtype=float).reshape(3)
                return float(xyz[0]), float(xyz[1]), float(xyz[2])
            except Exception:
                return 0.0, 0.0, 0.0

        if hasattr(self, "pose_labels"):
            for name, rob in [("ReBeL", self.robotSystem.rebel),
                              ("UR3", self.robotSystem.ur3),
                              ("GP4", self.robotSystem.gp4)]:
                x, y, z = xyz_of(rob)
                xsv, ysv, zsv = self.pose_labels[name]
                xsv.set(f"{x:.3f}")
                ysv.set(f"{y:.3f}")
                zsv.set(f"{z:.3f}")
