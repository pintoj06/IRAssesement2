import swift
from spatialgeometry import Mesh

class testTubePython:
    def __init__(self, filename, startPos, endPos, env: 'swift.Swift'):
        self.startPos = startPos
        self.endPos = endPos
        self.file = filename
        self.env = env

        self.addTube()

    def addTube(self):
        tt = Mesh(filename = self.file)
        tt.T = self.startPos

        self.env.add(tt)