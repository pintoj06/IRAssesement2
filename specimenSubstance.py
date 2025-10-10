import swift
from spatialgeometry import Mesh
from spatialmath import SE3
from spatialmath.base import *
from math import pi

class specimenSubstance:
    def __init__(self, filename, startPos, endPos, env: 'swift.Swift'):
        self.startPos = startPos
        self.endPos = endPos
        self.file = filename
        self.env = env


        self.meshObj = Mesh(filename = self.file)
        self.meshObj.T = self.startPos

        self.env.add(self.meshObj)
