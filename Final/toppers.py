import swift
from spatialgeometry import Mesh
from spatialmath import SE3
from spatialmath.base import *
from math import pi

class topperPython:
    def __init__(self, filename, startPos, env: 'swift.Swift'):
        self.startPos = startPos # Start position of the topper
        self.file = filename # filename of the topper mesh
        self.env = env # Environment used for the simulation
        self.offset = SE3(-0.07, 0, 0.16).A @ trotx(pi) # Offset to attach the topper to the gripper of the GP4
        self.ttoffset = SE3(0.0, 0, 0.14).A @ trotx(pi) # Offset to attach thr topper to the gripper of the UR3

        self.meshObj = Mesh(filename = self.file) # Create the object
        self.meshObj.T = self.startPos # Move the object to the start position

        self.env.add(self.meshObj) # Add the object to the environment
