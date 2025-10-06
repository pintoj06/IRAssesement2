"""
This class is the main movment one. It's made of the UR3 and GP4 so far but needs to add
the ibus to it. So far it's just a class with the functions to move the GP4 and the test tube
toppers in the envrionment.
"""

from gp4 import newGP4
from ir_support import UR3
import roboticstoolbox as rtb
import swift
from spatialmath import SE3
from spatialmath.base import *
from math import pi
from toppers import topperPython
from customUr3e import UR3e

class newRobotSystem:
    def __init__(self, ur3: 'UR3e', gp4: 'newGP4', ttList: 'list', topList: 'list', env: 'swift.Swift', ee: 'list' = [], eeToolOffset: 'list' = []):
        self.ur3 = ur3
        self.ttList = ttList
        self.topList = topList
        self.env = env

        self.ur3 = ur3
        self.ur3eeToolOffset = eeToolOffset[0]
        self.ur3EE = ee[0]
        self.ur3RMRCoffset = SE3(0, 0, 0.25).A

        self.gp4 = gp4
        self.gp4Offset = SE3(0.07, 0, 0).A
        self.gp4eeToolOffset = eeToolOffset[1]
        self.gp4EE = ee[1]
        self.gp4RMRCoffset = SE3(0, 0, 0.3).A

        self.steps = 150


    def simulation(self):
        #self.fillTubes()
        self.placeToppers()
        self.moveToCentrifuge()
        #self.collectTubes()
    
    def placeToppers(self):
        for i in range(len(self.ttList)):
            self.jtrajMovegp4(self.gp4.ik_LM(self.topList[i].startPos @ self.gp4Offset @ self.gp4RMRCoffset @ trotx(pi))[0])
            self.jtrajMovegp4(self.gp4.ik_LM(self.ttList[i].startPos @ self.gp4Offset @ self.gp4RMRCoffset @ trotx(pi))[0], moveObj=[self.topList[i]], objOffset=[self.topList[i].offset])
        self.jtrajMovegp4([0, pi/2, 0, 0, 0, 0])
        self.jtrajMovegp4([-pi, pi/2, 0, 0, 0, 0])
    
    def moveToCentrifuge(self):
        for i in range(len(self.ttList)):
            self.jtrajMoveur3(self.ur3.ik_LM(self.ttList[i].startPos @ self.ur3RMRCoffset @ trotx(pi) @ trotz(pi/2))[0])
            self.jtrajMoveur3(self.ur3.ik_LM(self.ttList[i].endPos @ self.ur3RMRCoffset @ trotx(pi))[0], moveObj = [self.topList[i], self.ttList[i]], objOffset=[self.topList[i].ttoffset, self.ttList[i].offset])

    
    def jtrajMovegp4(self, endq, moveObj:'list'=None, objOffset:'list'=None):
        """
        This is specifically for moving the gp4 using jtraj
        It can be copied and used for the other robots just change the EE and offset

        moveObj is a list of the objects that need to be moved with at the end effector
        It's originally at None so the robot can still move without anything in the end effector
        Also the function relies on the object that is moved being called meshObj in the class
            EXAMPLE IN TOPPERS.PY:
                self.meshObj = Mesh(filename = self.file)
                self.offset = SE3(-0.07, 0, 0.143).A

        """

        qMatrix = rtb.jtraj(self.gp4.q, endq, self.steps).q
        for x in range(self.steps):
            self.gp4.q = qMatrix[x]
            self.gp4EE.T = self.gp4.fkine(self.gp4.q).A @ self.gp4eeToolOffset
            if moveObj is not None:
                for y in range(len(moveObj)):
                    moveObj[y].meshObj.T = self.gp4.fkine(self.gp4.q).A @ objOffset[y]
            self.env.step(0.015)

    def jtrajMoveur3(self, endq, moveObj:'list'=None, objOffset:'list'=None):
        qMatrix = rtb.jtraj(self.ur3.q, endq, self.steps).q
        for x in range(self.steps):
            self.ur3.q = qMatrix[x]
            self.ur3EE.gripFing1.base = self.ur3.fkine(self.ur3.q).A @ self.ur3eeToolOffset
            self.ur3EE.gripFing2.base = self.ur3.fkine(self.ur3.q).A @ self.ur3eeToolOffset
            if moveObj is not None:
                for y in range(len(moveObj)):
                    moveObj[y].meshObj.T = self.ur3.fkine(self.ur3.q).A @ objOffset[y]
            self.env.step(0.015)
