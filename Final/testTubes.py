import swift
from spatialgeometry import Mesh
from spatialmath.base import *
from spatialmath import SE3
from math import pi
from SpecimenLiquid import specimenLiquid
class testTubePython:
    def __init__(self, filename, startPos, endPos, env: 'swift.Swift'):
        self.startPos = startPos # Start position of the test tube
        self.endPos = endPos # Final position of the test tube
        self.file = filename # File path of the test tube mesh
        self.env = env # Environment used for the simulation
        self.offset = SE3(0, 0, 0.2).A @ trotx(pi) # Offset to attach the test to the gripper
        self.specimen = None
        self.meshObj = Mesh(filename = self.file) # Create the object
        self.meshObj.T = self.startPos # Set the start position of the object
        self.specimenFull = None
        self.env.add(self.meshObj) # Add the object to the environment

    def attachSpecimen(self, specimen: specimenLiquid):
        self.specimenFull = specimen
        
        