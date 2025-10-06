import swift
from spatialgeometry import Mesh
from spatialmath.base import *
from spatialmath import SE3
from math import pi

class testTubePython:
    def __init__(self, filename, startPos, endPos, env: 'swift.Swift'):
        self.startPos = startPos
        self.endPos = endPos
        self.file = filename
        self.env = env
        self.offset = SE3(0, 0, 0.1).A @ trotx(pi)

        self.meshObj = Mesh(filename = self.file)
        self.meshObj.T = self.startPos

        self.env.add(self.meshObj)
