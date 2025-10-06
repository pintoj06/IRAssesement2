from gp4 import newGP4
from ir_support import UR3
import roboticstoolbox as rtb
import swift
from spatialmath import SE3
from spatialmath.base import *
from math import pi
from toppers import topperPython

class newRobotSystem:
    def __init__(self, ur3: 'UR3', gp4: 'newGP4', ttList: 'list', topList: 'list', env: 'swift.Swift', ee: 'list' = [], eeToolOffset: 'list' = []):
        self.ur3 = ur3
        self.gp4 = gp4
        self.ttList = ttList
        self.topList = topList
        self.env = env

        self.gp4Offset = SE3(0.07, 0, 0).A
        self.gp4eeToolOffset = eeToolOffset[0]
        self.gp4EE = ee[0]
        self.gp4RMRCoffset = SE3(0, 0, 0.3).A

        self.steps = 50


    def simulation(self):
        #self.fillTubes()
        self.placeToppers()
        #self.moveToCentrifuge()
        #self.collectTubes()
    
    def placeToppers(self):
        for i in range(len(self.ttList)):
            self.jtrajMovegp4(self.gp4.ik_LM(self.topList[i].startPos @ self.gp4Offset @ self.gp4RMRCoffset @ trotx(pi))[0])
            self.jtrajMovegp4(self.gp4.ik_LM(self.ttList[i].startPos @ self.gp4Offset @ self.gp4RMRCoffset @ trotx(pi))[0], moveObj = [self.topList[i]])

    def jtrajMovegp4(self, endq, moveObj:'list'=None):
        qMatrix = rtb.jtraj(self.gp4.q, endq, self.steps).q
        for x in range(self.steps):
            self.gp4.q = qMatrix[x]
            self.gp4EE.T = self.gp4.fkine(self.gp4.q).A @ self.gp4eeToolOffset
            if moveObj is not None:
                for y in moveObj:
                    y.meshObj.T = self.gp4.fkine(self.gp4.q).A @ y.offset
            self.env.step(0.05)