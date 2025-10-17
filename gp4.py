#  @file
#  @brief Motoman GP4 defined by standard DH parameters with 3D model (Swift-ready)
#  @author You
#  @date Oct 1, 2025

import os
import time
from math import pi
from pathlib import Path

import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
import swift

from roboticstoolbox import DHRobot, DHLink
from ir_support import CylindricalDHRobotPlot

from ir_support.robots.DHRobot3D import DHRobot3D  # adjust if your path differs


class newGP4(DHRobot3D):
    """
    Motoman GP4 robot using DHRobot3D + 3D meshes.

    Example:
        >>> from yourpkg import MotomanGP4
        >>> import swift
        >>> r = MotomanGP4()
        >>> q = [0, 0, 0, 0, 0, 0]
        >>> r.q = q
        >>> q_goal = [r.q[i] - pi/6 for i in range(r.n)]
        >>> env = swift.Swift()
        >>> env.launch(realtime=True)
        >>> r.add_to_env(env)
        >>> qtraj = rtb.jtraj(r.q, q_goal, 50).q
        >>> for qk in qtraj:
        ...     r.q = qk
        ...     env.step(0.02)
    """

    def __init__(self):
        # --- DH links (see _create_DH for details) ---
        links = self._create_DH()
        self.dhRobot = self.createDHRobotForCollision()

        # --- Mesh names for each link (stripped of package paths, just basenames) ---
        # Make sure these filenames match the .dae/.stl files you've already renamed.
        link3d_names = dict(
            link0 ="base_link",
            link1 ="link_1_s",
            link2 ="link_2_l",
            link3 ="link_3_u",
            link4 ="link_4_r",
            link5 ="link_5_b",
            link6 ="link_6_t"
        )

        # --- Reference pose that aligns meshes to the DH frames ---
        # We use the URDF zero pose: all joint angles = 0
        qtest = [0, pi/2, 0, 0, 0, 0]

        # The transforms below place the CAD meshes so they visually match the URDF zero.
        # They’re derived from your Xacro origins and typical CAD frame flips (Z-up -> DH frames).
        # If your meshes already sit perfectly with identity transforms, you can simplify these.
        qtest_transforms = [
            # base_link (world)
            spb.transl(0, 0, 0),
            spb.transl(0, 0, 0.33),
            spb.transl(0, 0, 0.33),
            spb.transl(0, 0, 0.59),
            spb.transl(0.29, 0, 0.605),
            spb.transl(0.29, 0, 0.605),
            spb.transl(0.29, 0, 0.605)
        ]

        # Robust current path (works in notebooks where __file__ may be missing)
        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3d_names, name = 'GP4', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        self.q = qtest

        # Expose the ROS-Industrial flange/tool as in your Xacro
        # flange at +X 0.072 from link_6_t, tool0 rotated (180, -90, 0) deg
        self.tool = SE3(0.072, 0, 0)  # TCP offset from wrist
        # If you want the exact 'tool0' frame (Rz(π) * Ry(-π/2)), uncomment:
        # self.tool = SE3(0.072, 0, 0) * SE3.RPY([pi, -pi/2, 0], order="xyz")

    # -----------------------------------------------------------------------------------#
    def _create_DH(self):
        """
        Create robot's standard DH model (approx. from URDF joint origins).
        Notes:
          - d1 = 0.330 (base to J1)
          - d3 = 0.260 (J2 to J3 along the forearm vertical)
          - small wrist offset d4 ≈ 0.015
          - a3 ≈ 0.290 (reach to J4)
          - final flange offset (0.072) modeled via self.tool, not DH
        These are set to be simulation-friendly and to align with the meshes at q = 0.
        """
        # --- Standard DH (alpha about x, a along x, d along z, theta about z) ---
        # Axis pattern from URDF: z, +y, -y, -x, -y, -x
        # We encode that with 90° twists between joint axes.
        a =   [0.0, 0.260, 0.015, 0.0, 0.0, 0.0]
        d =   [0.330, 0.0, 0.0, 0.290, 0.0, 0.078]
        alpha = [pi/2, 0,  pi/2, -pi/2,  pi/2, 0.0]

        # Joint limits from your Xacro (converted to radians); J6 unlimited-ish is clamped to ±(455°)
        qlim = [
            [(-170) * pi / 180, (170) * pi / 180],
            [(-110) * pi / 180, (130) * pi / 180],
            [(-65)  * pi / 180, (200) * pi / 180],
            [(-200) * pi / 180, (200) * pi / 180],
            [(-123) * pi / 180, (123) * pi / 180],
            [(-455) * pi / 180, (455) * pi / 180],
        ]

        links = [
            rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim=qlim[i]) for i in range(6)
        ]
        return links

    # -----------------------------------------------------------------------------------#
    def test(self):
        """
        Open a Swift window, drop the robot in, and do a small joint move.
        """
        env = swift.Swift()
        env.launch(realtime=True)

        # Known alignment pose
        self.q = getattr(self, "_qtest", self.q)
        self.base = SE3(0.5, 0.5, 0.0)  # nudge off origin for a nicer view
        self.add_to_env(env)

        q_goal = [self.q[i] - pi / 6 for i in range(self.n)]
        qtraj = rtb.jtraj(self.q, q_goal, 60).q
        for qk in qtraj:
            self.q = qk
            env.step(0.02)
        time.sleep(2)

    def createDHRobotForCollision(self):
        a =   [0.0, 0.260, 0.015, 0.0, 0.0, 0.0]
        d =   [0.330, 0.0, 0.0, 0.290, 0.0, 0.2]
        alpha = [pi/2, 0,  pi/2, -pi/2,  pi/2, 0.0]

        # Joint limits from your Xacro (converted to radians); J6 unlimited-ish is clamped to ±(455°)
        qlim = [
            [(-170) * pi / 180, (170) * pi / 180],
            [(-110) * pi / 180, (130) * pi / 180],
            [(-65)  * pi / 180, (200) * pi / 180],
            [(-200) * pi / 180, (200) * pi / 180],
            [(-123) * pi / 180, (123) * pi / 180],
            [(-455) * pi / 180, (455) * pi / 180],
        ]
        link1 = DHLink(d=d[0], a=a[0], alpha=alpha[0], qlim=qlim[0])
        link2 = DHLink(d=d[1], a=a[1], alpha=alpha[1], qlim=qlim[1])
        link3 = DHLink(d=d[2], a=a[2], alpha=alpha[2], qlim=qlim[2])
        link4 = DHLink(d=d[3], a=a[3], alpha=alpha[3], qlim=qlim[3])
        link5 = DHLink(d=d[4], a=a[4], alpha=alpha[4], qlim=qlim[4])
        link6 = DHLink(d=d[5], a=a[5], alpha=alpha[5], qlim=qlim[5])

        robot = DHRobot([link1, link2, link3, link4, link5, link6], name='gp4_DH')

        cyl_viz = CylindricalDHRobotPlot(robot, cylinder_radius=0.0000001, color="#3478f604")
        robot = cyl_viz.create_cylinders()

        return robot
