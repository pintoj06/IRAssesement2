##  @file
#   @brief UR3 Robot defined by standard DH parameters with 3D model
#   @author Ho Minh Quang Ngo
#   @date Jul 20, 2023

import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os

from roboticstoolbox import DHRobot, DHLink
from ir_support import CylindricalDHRobotPlot

# Useful variables
from math import pi

# -----------------------------------------------------------------------------------#
class UR3e(DHRobot3D):
    def __init__(self):
        """
            UR3 Robot based on DHRobot3D class

            USES THE ORIGINAL UR3 CLASS HOWEVER ALTERED THE QLIM VALUES OF THE ROBOT SO IT
            DOES NOT PASS THROUGH THE FLOOR

            Example usage:
            >>> from ir-support import UR3
            >>> import swift

            >>> r = UR3()
            >>> q = [0,-pi/2,pi/4,0,0,0]r
            >>> r.q = q
            >>> q_goal = [r.q[i]-pi/4 for i in range(r.n)]
            >>> env = swift.Swift()
            >>> env.launch(realtime= True)
            >>> r.add_to_env(env)
            >>> qtraj = rtb.jtraj(r.q, q_goal, 50).q
            >>> for q in qtraj:r
            >>>    r.q = q
            >>>    env.step(0.02)
        """
        # DH links
        links = self._create_DH()
        self.dhRobot = self.createDHRobotForCollision()

        # Names of the robot link files in the directory
        link3D_names = dict(link0 = 'base_ur3',
                            link1 = 'shoulder_ur3',
                            link2 = 'upperarm_ur3',
                            link3 = 'forearm_ur3',
                            link4 = 'wrist1_ur3',
                            link5 = 'wrist2_ur3',
                            link6 = 'wrist3_ur3')

        # A joint config and the 3D object transforms to match that config
        qtest = [0,-pi/2,0,0,0,0]
        qtest_transforms = [spb.transl(0,0,0),
                            spb.transl(0,0,0.15239) @ spb.trotz(pi),
                            spb.transl(0,-0.12,0.1524) @ spb.trotz(pi),
                            spb.transl(0,-0.027115,0.39583) @ spb.trotz(pi),
                            spb.transl(0,-0.027316,0.60903) @ spb.rpy2tr(0,-pi/2,pi, order = 'xyz'),
                            spb.transl(0.000389,-0.11253,0.60902) @ spb.rpy2tr(0,-pi/2,pi, order= 'xyz'),
                            spb.transl(-0.083765,-0.11333,0.61096) @ spb.trotz(pi)]

        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name = 'UR3', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        self.q = qtest

    # -----------------------------------------------------------------------------------#
    def _create_DH(self):
        """
        Create robot's standard DH model
        """
        a = [0, -0.24365, -0.21325, 0, 0, 0]
        d = [0.1519, 0, 0, 0.11235, 0.08535, 0.0819]
        alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]
        qlim = [[-pi, 2*pi] for _ in range(6)] 
        qlim[1] = [-pi, 0] #Changled limiting angle of second joint so robot does not spin through ground
        links = []
        for i in range(6):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim= qlim[i])
            links.append(link)
        return links

    # -----------------------------------------------------------------------------------#
    def test(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """
        env = swift.Swift()
        env.launch(realtime= True)
        self.q = self._qtest
        self.base = SE3(0.5,0.5,0)
        self.add_to_env(env)

        q_goal = [self.q[i]-pi/3 for i in range(self.n)]
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        # fig = self.plot(self.q)
        for q in qtraj:
            self.q = q
            env.step(0.02)
            # fig.step(0.01)
        time.sleep(3)
        # env.hold()

    def createDHRobotForCollision(self):
        a = [0, -0.24365, -0.21325, 0, 0, 0]
        d = [0.1519, 0, 0, 0.11235, 0.08535, 0.0819]
        alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]
        qlim = [[-pi, 2*pi] for _ in range(6)] 
        qlim[1] = [-pi, 0] #Changled limiting angle of second joint so robot does not spin through ground
        
        link1 = DHLink(d=d[0], a=a[0], alpha=alpha[0], qlim=qlim[0])
        link2 = DHLink(d=d[1], a=a[1], alpha=alpha[1], qlim=qlim[1])
        link3 = DHLink(d=d[2], a=a[2], alpha=alpha[2], qlim=qlim[2])
        link4 = DHLink(d=d[3], a=a[3], alpha=alpha[3], qlim=qlim[3])
        link5 = DHLink(d=d[4], a=a[4], alpha=alpha[4], qlim=qlim[4])
        link6 = DHLink(d=d[5], a=a[5], alpha=alpha[5], qlim=qlim[5])

        robot = DHRobot([link1, link2, link3, link4, link5, link6], name='gp4_DH')

        cyl_viz = CylindricalDHRobotPlot(robot, cylinder_radius=0.000005, color="#3478f604")
        robot = cyl_viz.create_cylinders()

        return robot
# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    r = UR3e()
    r.test()

