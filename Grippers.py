import swift
from math import pi
from roboticstoolbox import DHLink, DHRobot
from ir_support import CylindricalDHRobotPlot
import roboticstoolbox as rtb


# ---------------------------------------------------------------------------------------#
class gripperObj:
    """
    This class initialises the grippers required for the task and also holds 
    the relevant functions required to control the opening and closing of the grippers
    """
    def __init__(self, envrionment: 'swift.Swift'):
        #Define DH parameters for gripper
        link1G1 = DHLink(d=0, a=0.09, alpha=0, qlim=[-pi, pi])
        link2G1 = DHLink(d=0, a=0.05, alpha=0, qlim=[-pi, pi])
        
        link1G2 = DHLink(d=0, a=0.09, alpha=0, qlim=[-pi, pi])
        link2G2 = DHLink(d=0, a=0.05, alpha=0, qlim=[-pi, pi])
        
        #Create the 2 DH gripper fingers which make up the gripper
        self.gripFing1 = DHRobot([link1G1, link2G1], name='gripper1')
        self.gripFing2 = DHRobot([link1G2, link2G2], name='gripper2')

        #Surround the DH grippers with a cylindrical mesh to be displayed in the swift environment
        cyl_viz = CylindricalDHRobotPlot(self.gripFing1, cylinder_radius=0.01, color="#00FF4C")
        self.gripFing1 = cyl_viz.create_cylinders()
        cyl_viz = CylindricalDHRobotPlot(self.gripFing2, cylinder_radius=0.01, color="#00FF4C")
        self.gripFing2 = cyl_viz.create_cylinders()

        #Define the joint angles for the closed configuration
        self.qCloseG1 = [pi/6, -pi/3]
        self.qCloseG2 = [-pi/6, pi/3]
        
        #Define the joint angles for the open configuration
        self.qOpenG1 = [pi/3, -pi/3]
        self.qOpenG2 = [-pi/3, pi/3]

        self.steps = 50
        self.env = envrionment

        #Defualt state of grippers is open
        self.gripFing1.q = self.qOpenG1
        self.gripFing2.q = self.qOpenG2
    
    # -----------------------------------------------------------------------------------#
    def openGripper(self):
        """
        Open the grippers using jtraj function to path the minimum jerk trajectory of angle 
        changes. Loop through these values to animate the change in the swift environment 
        """
        qNewG1 = rtb.jtraj(self.gripFing1.q, self.qOpenG1, self.steps).q #Path of finger 1
        qNewG2 = rtb.jtraj(self.gripFing2.q, self.qOpenG2, self.steps).q #Path of finger 2
        #Update q vals with loop to animate
        for i in range(self.steps):
            self.gripFing1.q = qNewG1[i] 
            self.gripFing2.q = qNewG2[i]
            self.env.step(0.03)
    

    # -----------------------------------------------------------------------------------#
    def closeGripper(self):
        """
        Close the grippers using jtraj function to path the minimum jerk trajectory of angle 
        changes. Loop through these values to animate the change in the swift environment 
        """
        qNewG1 = rtb.jtraj(self.gripFing1.q, self.qCloseG1, self.steps).q #Path of finger 1
        qNewG2 = rtb.jtraj(self.gripFing2.q, self.qCloseG2, self.steps).q #Path of finger 2
        #Update q vals with loop to animate
        for i in range(self.steps):
            self.gripFing1.q = qNewG1[i]
            self.gripFing2.q = qNewG2[i]
            self.env.step(0.03)
