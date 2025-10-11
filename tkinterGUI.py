import tkinter as tk
from tkinter import ttk

# Create the Tk app *first*
_early_root = tk.Tk()
_early_root.withdraw()  # hide until your UI class sets it up

from robotSystem import newRobotSystem
import math


# Require libraries
from spatialmath import SE3
from spatialmath.base import *
from math import pi
from customUr3e import UR3e
import swift
from spatialgeometry import Mesh, Sphere
import time
from testTubes import testTubePython
from toppers import topperPython
from math import pi
import roboticstoolbox as rtb
from roboticstoolbox import DHLink, DHRobot, trapezoidal
from ir_support import CylindricalDHRobotPlot
from gp4 import newGP4
import numpy as np
from robotSystem import newRobotSystem
from Grippers import gripperObj
from IGUS_testcode import ReBeL


class JointControlUI:
    def __init__(self, botSystem: 'newRobotSystem'):
        # CONTAINS ALL ROBOT INFO 
        self.robotSystem = botSystem
        global _early_root
        self.root = _early_root  # use the one we created early
        self.root.deiconify()    # show it now
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

        # --- Bottom panel (sliders) ---
        bottom = ttk.Frame(self.root, padding=15)
        bottom.grid(row=1, column=0, sticky="nsew")
        bottom.columnconfigure(0, weight=1)
        bottom.columnconfigure(1, weight=1)

        ttk.Label(bottom, text="Teach Panel — Joint Angles (°)", style="Title.TLabel").grid(row=0, column=0, columnspan=2, pady=(0, 10))

        self.slider_vars = []
        self.value_strs = []

        #Create 6 sliders, 2 columns
        for i in range(6):
            col = i % 2
            row = (i // 2) * 2 + 1

            # container for "Joint X | value"
            joint_box = ttk.Frame(bottom)
            joint_box.grid(row=row, column=col, sticky="ew", pady=(0, 2))
            joint_box.columnconfigure(0, weight=1)
            joint_box.columnconfigure(1, weight=0)

            ttk.Label(joint_box, text=f"Joint {i+1}").grid(row=0, column=0, sticky="w")

            # Tk variables explicitly bound to the same root
            var = tk.DoubleVar(master=self.root, value=0.0)
            sv  = tk.StringVar(master=self.root, value="0.0°")

            ttk.Label(joint_box, textvariable=sv).grid(row=0, column=1, sticky="e")

            # Scale bound to var; command updates the label
            ttk.Scale(
                bottom,
                from_=-180, to=180,
                orient="horizontal",
                variable=var,
                command=lambda v, s=sv: s.set(f"{float(v):.1f}°")
            ).grid(row=row+1, column=col, sticky="ew", pady=(0, 10))

            self.slider_vars.append(var)
            self.value_strs.append(sv)

            # Apply button under both columns
            ttk.Button(bottom, text="Apply Angles", command=self.on_apply).grid(row=7, column=0, columnspan=2, sticky="ew", pady=(10, 0))

        self.root.mainloop()

    # --- Button callbacks ---
    def degrees_to_radians(self, angle_list):
        return [math.radians(angle) for angle in angle_list]
    
    def on_run(self):
        print("Run pressed. Current angles:", self.current_angles())
        self.robotSystem.simulation()

    def on_reset(self):
        print("Reset pressed. All joints set to 0.")
        for var in self.slider_vars:
            var.set(0.0)

    def on_estop(self):
        print("!!! EMERGENCY STOP ACTIVATED !!!")
        

    def on_apply(self):
        '''Apply the current slider values to the robot.
        Make this function update automatically on slider change value instead of using a button. 
        '''
        print("Applied joint angles:", self.current_angles())
        # Apply the current joint configuration to the robot 
        angles_deg = self.current_angles()
        angles_rad = self.degrees_to_radians(angles_deg)
        self.robotSystem.rebel.q = angles_rad
        self.robotSystem.env.step()
        self.robotSystem.rebelEE.T = self.robotSystem.rebel.fkine(self.robotSystem.rebel.q).A @ SE3.Ry(math.pi/2 ).A
    def getCurrentAngles():
        #NEEDS TO BE A FUNCTION THAT GETS THE VALUES OF THE robot to update the
        pass
    # --- Helper ---
    def current_angles(self):
        """Return all slider values in degrees as a list [j1..j6]."""
        return [float(var.get()) for var in self.slider_vars]

# Run the UI

