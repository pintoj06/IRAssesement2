import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
from ir_support import CylindricalDHRobotPlot
import time
import os
import numpy as np
from math import pi
from roboticstoolbox import DHRobot, DHLink
from spatialgeometry import Mesh, Sphere
from spatialmath.base import *
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
        self.dhRobot = self.createDHRobotForCollision() #cylinders for collision detection
        
        link3D_names = dict(
            link0='J0',
            link1='J1',
            link2='J2',
            link3='J3',
            link4='J4',
            link5='J5',
            link6='J6'
        )

      # 0, 90, 90 
      #qtest = [pi/2, 0, 0, 0, pi/4, 0]
        qtest = [0, pi/2, pi/2,0,0,pi/2]
        dz = [0.0,0.103, 0.103+0.149, 0.103+0.149+0.237, 0.103+0.149+0.237+0.127,
              0.103+0.149+0.237+0.127+0.170, 0.103+0.149+0.237+0.127+0.170+0.126]
        
        qtest_transforms = [
            spb.transl(0, 0, dz[0]) ,                                 # base (Joint0)
            spb.transl(0, 0, dz[1]) ,                                  # link1
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
            #link3d_dir= r"C:\Users\jayde\OneDrive - UTS\2025\Sem2\Industrial Robotics\VS Code\A2", # FOR JAYDEN
            link3d_dir= os.path.abspath(os.path.dirname(__file__)), # FOR HARRY
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
        #ORIGINAL
   #     alphas = [-np.pi/2, 0.0, +np.pi/2, -np.pi/2, +np.pi/2, 0.0]
    #    a      = [0, 0, 0, 0, 0, 0]
     #   d     = [0.103, 0.149, 0.237, 0.127, 0.170, 0.126]
#WITH JOHN
        # alphas = [-np.pi/2, 0, -np.pi/2, -np.pi/2, +np.pi/2, 0.0]
        # a      = [0, 0.149, 0, 0, 0, 0]
        # d      = [0.103, 0,0 , 0.127, 0.170, 0.126]
# OTHER
     #   alphas = [-np.pi/2, 0.0, +np.pi/2, -np.pi/2, +np.pi/2, 0.0]
      #  a      = [0, 0, 0, 0, 0, 0]
       # d     = [0.103, 0.149, 0.237, 0.127, 0.170, 0.126]
#CHATGPT PART 2
        # alphas = [ 0, 0.0,  1.6301956900972687, +pi/2, +pi/2, 0.0 ]
        # a     = [ 0.0,   0.23945825494215744,      0.0,          0.0,   0.0,   0.0 ]
        # d     = [ 0.153, 0.0,  0.032256047714453125, 0.143,       0.029, 0.0  ]
#CHATGPR PT3.
        alphas = [pi/2, 0 , pi/2, -pi/2, pi/2, 0 ]
        a     = [0, 0.237, 0,0,0,0]
        d     = [0.252, 0, 0, 0.297, 0, 0.126]
        offset = [0,0,0,0,0, 0]

        # Joint limits 
        deg = np.pi/180
        qlim = [
            [ -2*pi,  2*pi],   # J1
            [ -2*pi,  2*pi],   # J2
            [ -2*pi,  2*pi],   # J3
            [ -2*pi,  2*pi],   # J4
            [ -2*pi,  2*pi],   # J5
            [ -2*pi,  2*pi],   # J6
        ]

        # Build RevoluteMDH links (MDH!)
        flip_flags = [False, False, False, False, False, False]  # flip J2, J3, J5 Notttt working!!, might need to change the orientation in blender 

        links = [
            rtb.RevoluteDH(alpha=alphas[i], a=a[i], d=d[i], qlim=qlim[i], flip=flip_flags[i], offset=offset[i])
            for i in range(6)
        ]

        
        return links
    
    #function that creates robot for the purpose of collision detection
    def createDHRobotForCollision(self):
        alphas = [pi/2, 0 , pi/2, -pi/2, pi/2,0 ]
        a     = [0, 0.237, 0,0,0,0]
        d     = [0.252, 0, 0, 0.297, 0, 0.126]
        deg = np.pi/180
        qlim = [
            [ -2*pi,  2*pi],   # J1
            [ -2*pi,  2*pi],   # J2
            [ -2*pi,  2*pi],   # J3
            [ -2*pi,  2*pi],   # J4
            [ -2*pi,  2*pi],   # J5
            [ -2*pi,  2*pi],   # J6
        ]

        link1 = DHLink(d=d[0], a=a[0], alpha=alphas[0], qlim=qlim[0])
        link2 = DHLink(d=d[1], a=a[1], alpha=alphas[1], qlim=qlim[1])
        link3 = DHLink(d=d[2], a=a[2], alpha=alphas[2], qlim=qlim[2])
        link4 = DHLink(d=d[3], a=a[3], alpha=alphas[3], qlim=qlim[3])
        link5 = DHLink(d=d[4], a=a[4], alpha=alphas[4], qlim=qlim[4])
        link6 = DHLink(d=d[5], a=a[5], alpha=alphas[5], qlim=qlim[5])

        r = DHRobot([link1, link2, link3, link4, link5, link6], name='rebel_DH')

        cyl_viz = CylindricalDHRobotPlot(r, cylinder_radius=0.000005, color="#fc2929ff")
        robot = cyl_viz.create_cylinders()
        return robot


    # -----------------------------------------------------------------------------------#
    def add_to_env(self, env):
        """Override to ensure tool transform is applied before adding to Swift."""
        # Set tool per XACRO joint7 (fixed Ry(-pi/2))
        self.tool = SE3.Ry(-np.pi/2)
        return super().add_to_env(env)

    # -----------------------------------------------------------------------------------#
    def test(self):
        """
        Quick self-test: add to Swift a nd move a little.
        """
        env = swift.Swift()
        env.launch(realtime=True)
        self.base = SE3(0, 0.0, 0.0)
        self.add_to_env(env)

       # q_goal = [0, -pi/2, +pi/2, 0, 0, 0]
        time.sleep(5)
        q_goal= [np.pi/4, -np.pi/3, np.pi/4, 0, np.pi/4, 0]
        qtraj = rtb.jtraj(self.q, q_goal, 60).q
        for q in qtraj:
            self.q = q
            env.step(0.02)
        time.sleep(7.0)
        env.hold()
        
# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    alphas = [pi/2, 0 , pi/2, -pi/2, pi/2,0 ]
    a     = [0, 0.237, 0,0,0,0]
    d     = [0.252, 0, 0, 0.297, 0, 0.126]
    deg = np.pi/180
    qlim = [
            [ -2*pi,  2*pi],   # J1
            [ -2*pi,  2*pi],   # J2
            [ -2*pi,  2*pi],   # J3
            [ -2*pi,  2*pi],   # J4
            [ -2*pi,  2*pi],   # J5
            [ -2*pi,  2*pi],   # J6
        ]
    
    env = swift.Swift()
    env.launch(realtime=True)
    

    link1 = DHLink(d=d[0], a=a[0], alpha=alphas[0], qlim=qlim[0])
    link2 = DHLink(d=d[1], a=a[1], alpha=alphas[1], qlim=qlim[1])
    link3 = DHLink(d=d[2], a=a[2], alpha=alphas[2], qlim=qlim[2])
    link4 = DHLink(d=d[3], a=a[3], alpha=alphas[3], qlim=qlim[3])
    link5 = DHLink(d=d[4], a=a[4], alpha=alphas[4], qlim=qlim[4])
    link6 = DHLink(d=d[5], a=a[5], alpha=alphas[5], qlim=qlim[5])


    r = ReBeL()
    #robot = DHRobot([link1, link2, link3, link4, link5, link6], name='gp4_DH')
 #   cyl_viz = CylindricalDHRobotPlot(robot, cylinder_radius=0.01, color="#00FF4C")
   # robot = cyl_viz.create_cylinders()    
  #  robot.q=[ np.pi/4, -np.pi/2, np.pi/2, 0, np.pi/4, 0]

#   env.add(robot)
  #  r.q = [ 1.40546703, -0.63328708,  1.26858756,
  # r.base
  #       0.72299413,  0.48662232, 0.27487697]
    r.base = SE3(0.5, 0, 0) 
    r.add_to_env(env)
 #   robot.base = SE3(0.5, 0, 0).A
#    robot.q = [0, pi/2, pi/2,0,0,0]
    env.step(0.05)
    pose = SE3(0.2, 0.2, 0.2).A @ troty(pi/2) 

# Create a blue sphere with 3 cm radius
    sphere = Sphere(radius=0.03, pose=pose, color=[0.0, 0.2, 1.0, 1.0])

# Add to the environment
    env.add(sphere)
    
    t_world = pose @ trotx(pi)  #ply tool offset
    

    sol = r.ik_LM(t_world)#add .A for orientation
# array([ 1.40546703, -0.63328708,  1.26858756,  0.72299413,  0.48662232, 0.27487697])
#    print(sol.success, sol.reason, sol.residual, sol.iterations, sol.searches)
    q_goal = sol[0]
    qtraj = rtb.jtraj(r.q, q_goal, 60).q
    for q in qtraj:
        r.q = q
        env.step(0.02)
    t_we = r.fkine(q)
    x, y, z = t_we.t
    print(x, y, z)
    print(f"x={x:.3f}, y={y:.3f}, z={z:.3f}")
    print("Final q:", np.array_str(r.q, precision=3, suppress_small=True))
    time.sleep(7.0)
    env.hold()
    
    
    
    
    
