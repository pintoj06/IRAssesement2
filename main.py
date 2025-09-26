# Require libraries
from spatialmath import SE3
from spatialmath.base import *
from math import pi
from ir_support import UR3
import swift
from spatialgeometry import Mesh
import time
from testTubes import testTubePython

# ---------------------------------------------------------------------------------------#
def initialise():
    # Add room
    envFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/CentrifugeEnvironment.dae'
    envRoom = Mesh(filename = envFile)
    env.add(envRoom)

    # Add UR3 and Move to starting position
    ur3 = UR3()
    ur3.base = SE3(0.47, 3.5, 0.585).A @ trotz(-pi/2)
    ur3.add_to_env(env)

    # Add centrifuge base and move to starting position
    cenBaseFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/CentrifugeBottom.dae'
    cenBase = Mesh(filename = cenBaseFile)
    cenBase.T = SE3(0.6, 4, 0.7).A @ trotz(pi/2)
    env.add(cenBase)

    # Add centrifuge top and move to starting position
    cenTopFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/CentrifugeTop.dae'
    cenTop = Mesh(filename = cenTopFile)
    cenTop.T = SE3(0.46, 4, 0.7395).A @ trotz(pi/2) @ trotx(-4*pi/5)
    env.add(cenTop)

    # Add test tube Rack and Move to starting position
    testTubeRackFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/TesttubeRack.dae'
    testTubeRack = Mesh(filename = testTubeRackFile)
    testTubeRack.T = SE3(0.6, 3.15, 0.585).A
    env.add(testTubeRack)

    camera_position = (SE3.Trans(4, 4, 1.4).A)[:3, 3]
    env.set_camera_pose(position=camera_position, look_at = ur3.base.A[:3, 3])

    # Add Test Tubes to Rack
    testTubeFileName = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/TestTube.dae'
    tt1 = testTubePython(testTubeFileName, SE3(0.598, 3.26, 0.65), SE3(0, 0, 0), env)
    tt2 = testTubePython(testTubeFileName, SE3(0.598, 3.215, 0.65), SE3(0, 0, 0), env)
    tt3 = testTubePython(testTubeFileName, SE3(0.598, 3.17, 0.65), SE3(0, 0, 0), env)
    tt4 = testTubePython(testTubeFileName, SE3(0.598, 3.123, 0.65), SE3(0, 0, 0), env)
    tt5 = testTubePython(testTubeFileName, SE3(0.598, 3.077, 0.65), SE3(0, 0, 0), env)
    

    ttList = [tt1, tt2, tt3]

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__": 
    #Launch swift   
    env = swift.Swift() 
    env.launch(realtime=True)   

    initialise()

    env.hold()
