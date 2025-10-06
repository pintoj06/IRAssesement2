import swift
from spatialgeometry import Mesh
from spatialmath import SE3

class topperPython:
    def __init__(self, filename, startPos, endPos, env: 'swift.Swift'):
        self.startPos = startPos
        self.endPos = endPos
        self.file = filename
        self.env = env
        self.offset = SE3(-0.07, 0, 0.143).A

        self.meshObj = Mesh(filename = self.file)
        self.meshObj.T = self.startPos

        self.env.add(self.meshObj)