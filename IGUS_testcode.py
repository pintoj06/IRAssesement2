##  @file
#   @brief igus ReBeL 6-DoF robot defined by **Modified DH** parameters with optional 3D model
#   @author <you>
#   @date Sep 25, 2025

import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os
import numpy as np
from math import pi

# -----------------------------------------------------------------------------------#
class ReBeL(DHRobot3D):
    def __init__(self):
        """
            igus ReBeL by DHRobot3D class (MDH)

            Example:
            >>> r = ReBeL()
            >>> env = swift.Swift(); env.launch(realtime=True)
            >>> r.add_to_env(env); r.q = [0,-pi/4,pi/4,0,0,0]
            >>> for q in rtb.jtraj(r.q, [0,-pi/2,pi/2,0,0,0], 60).q:
            ...     r.q = q; env.step(0.02)
        """
        links = self._create_DH()

        
        link3D_names = dict(
            link0='J0',
            link1='J1',
            link2='J2',
            link3='J3',
            link4='J4',
            link5='J5',
            link6='J6'
        )

      
        qtest = [0, 0, 0, 0, 0, 0]
        dz = [0.0,0.103, 0.103+0.149, 0.103+0.149+0.237, 0.103+0.149+0.237+0.127,
              0.103+0.149+0.237+0.127+0.170, 0.103+0.149+0.237+0.127+0.170+0.126]
        
        qtest_transforms = [
            spb.transl(0, 0, dz[0]),                                 # base (Joint0)
            spb.transl(0, 0, dz[1]),                                 # link1
            spb.transl(0, 0, dz[2]),                                 # link2
            spb.transl(0, 0, dz[3]),                                 # link3
            spb.transl(0, 0, dz[4]),                                 # link4
            spb.transl(0, 0, dz[5]),                                 # link5
            spb.transl(0, 0, dz[6]) @ SE3.Ry(-pi/2).A # flange (joint7 fixed Ry(-pi/2))
        ]

        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(
            links,
            link3D_names,
            name='igus ReBeL 6DoF (MDH)',
            link3d_dir= r"C:\Users\jayde\OneDrive - UTS\2025\Sem2\Industrial Robotics\VS Code\A2",
            qtest=qtest,
            qtest_transforms=qtest_transforms
        )
        self.q = qtest

    # -----------------------------------------------------------------------------------#
    def _create_DH(self):
        """
        Create robot's **Modified DH** model (from igus XACRO):
          J1: Rz, d=0.103,  α=-pi/2
          J2: Ry, d=0.149,  α= 0
          J3: Ry, d=0.237,  α=+pi/2
          J4: Rz, d=0.127,  α=-pi/2
          J5: Ry, d=0.170,  α=+pi/2
          J6: Rz, d=0.126,  α= 0
        All a_i = 0. Tool has an extra fixed Ry(-pi/2) (set on self.tool).
        """
        # MDH parameters,adjust
        alphas = [-np.pi/2, 0.0, +np.pi/2, -np.pi/2, +np.pi/2, 0.0]
        a      = [0, 0, 0, 0, 0, 0]
        d      = [0.103, 0.149, 0.237, 0.127, 0.170, 0.126]
       

        # Joint limits 
        deg = np.pi/180
        qlim = [
            [-179*deg,  179*deg],   # J1
            [ -80*deg,  140*deg],   # J2
            [ -80*deg,  140*deg],   # J3
            [-179*deg,  179*deg],   # J4
            [ -90*deg,   90*deg],   # J5
            [-179*deg,  179*deg],   # J6
        ]

        # Build RevoluteMDH links (MDH!)
        flip_flags = [False, False, True, False, True, False]  # flip J2, J3, J5 Notttt working!!, might need to change the orientation in blender 

        links = [
            rtb.RevoluteMDH(alpha=alphas[i], a=a[i], d=d[i], qlim=qlim[i], flip=flip_flags[i])
            for i in range(6)
        ]

        
        return links

    # -----------------------------------------------------------------------------------#
    def add_to_env(self, env):
        """Override to ensure tool transform is applied before adding to Swift."""
        # Set tool per XACRO joint7 (fixed Ry(-pi/2))
        self.tool = SE3.Ry(-np.pi/2)
        return super().add_to_env(env)

    # -----------------------------------------------------------------------------------#
    def test(self):
        """
        Quick self-test: add to Swift and move a little.
        """
        env = swift.Swift()
        env.launch(realtime=True)
        self.base = SE3(0.4, 0.0, 0.0)
        self.add_to_env(env)

       # q_goal = [0, -pi/2, +pi/2, 0, 0, 0]
        q_goal= [np.pi/4, -np.pi/3, np.pi/4, 0, np.pi/4, 0]
        qtraj = rtb.jtraj(self.q, q_goal, 60).q
        for q in qtraj:
            self.q = q
            env.step(0.02)
        time.sleep(7.0)
        env.hold()

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    r = ReBeL()
    r.test()
    
