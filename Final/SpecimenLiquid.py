import swift
from spatialgeometry import Mesh
from spatialmath.base import *
from spatialmath import SE3
from math import pi
import time
class specimenLiquid:
    #Contins 3 objects, bottom one is globl
    def __init__(self, filename, startPos, env: 'swift.Swift'):
        self.startPos = startPos

        self.file = filename
        self.env = env
        self.Zlength = -0.0351 
        self.offset = SE3(0, 0, self.Zlength).A
        self.inverseOffset = SE3(0, 0, -self.Zlength).A
        #first mesh
        self.mesh1 = Mesh(filename = self.file, scale = (0.6,0.6,1))
        self.mesh1.T = self.startPos
        #second mesh
        self.mesh2 = Mesh(filename = self.file, scale = (0.6,0.6,1))
        self.mesh2.T = self.startPos @ self.offset
        #third mesh
        self.mesh3 = Mesh(filename = self.file, scale = (0.6,0.6,1))
        self.mesh3.T = self.startPos @ self.offset @ self.offset

        self.meshes = [self.mesh1, self.mesh2, self.mesh3]
        for mesh in self.meshes:
            self.env.add(mesh)

    def updatePos(self,):
        reference = self.mesh1.T
        self.mesh2.T = reference @self.inverseOffset
        self.mesh3.T = reference @ self.inverseOffset @ self.inverseOffset
    
    def attachToRobot(self, robot, env):
        '''Once the robots end affector is in position, move each mesh individually onto the robots end affector with the offset'''
        self.mesh1.T =  robot.fkine(robot.q).A @ troty(pi/2)
        env.step(.015)
        time.sleep(.5)
        self.mesh2.T =  self.mesh1.T @ self.inverseOffset
        env.step(.015)
        time.sleep(.5)
        self.mesh3.T =  self.mesh1.T @ self.inverseOffset @ self.inverseOffset
        env.step(.015)
        time.sleep(.5)
    def attachToTestTube(self, testTubePos, env):
        '''Once the robots end affector is in position, move each mesh individually onto the robots end affector with the offset'''
        self.mesh1.T =  testTubePos @ SE3(0,0, -0.05).A
        env.step(.015)
        time.sleep(.5)
        self.mesh2.T =  self.mesh1.T @ self.inverseOffset
        env.step(.015)
        time.sleep(.5)
        self.mesh3.T =  self.mesh1.T @ self.inverseOffset @ self.inverseOffset
        env.step(.015)
        
