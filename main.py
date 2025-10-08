"""
THIS IS THE MAIN FILE
ALL I'VE DONE IS MAKE AN ENVIRONMENT THAT WE CAN WORK OFF
"""

# Require libraries
from spatialmath import SE3
from spatialmath.base import *
from math import pi
from customUr3e import UR3e
import swift
from spatialgeometry import Mesh
import time
from testTubes import testTubePython
from toppers import topperPython
from math import pi
import roboticstoolbox as rtb
from roboticstoolbox import DHLink, DHRobot, trapezoidal
from ir_support import CylindricalDHRobotPlot
from gp4 import newGP4
import numpy as np
from robotSystem import newRobotSystem
from Grippers import gripperObj


# ---------------------------------------------------------------------------------------#
def initialise():
    # Add room
    envFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/CentrifugeEnvironment1.dae'
    envRoom = Mesh(filename = envFile)
    env.add(envRoom)

    # Add UR3 and Move to starting position
    ur3.base = SE3(1.5, 4.0, 0.585).A 
    ur3.add_to_env(env)
    env.add(ur3.dhRobot)
    ur3.dhRobot.base = SE3(1.5, 4.0, 0.585).A 
    ur3.dhRobot.q = ur3.q

    # Add GP4 and Move to starting position
    gp4.base = SE3(1.5, 3.1, 0.585).A @ trotz(pi/2)
    gp4.add_to_env(env)
    env.add(gp4.dhRobot)
    gp4.dhRobot.base = SE3(1.5, 3.1, 0.585).A @ trotz(pi/2)
    gp4.dhRobot.q = gp4.q

    # Add centrifuge base and move to starting position
    cenBaseFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/CentrifugeBottom.dae'
    cenBase = Mesh(filename = cenBaseFile)
    cenBase.T = SE3(1.8, 4.2, 0.7).A #1.1, 4.2
    env.add(cenBase)

    # Add centrifuge top and move to starting position
    cenTopFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/CentrifugeTop.dae'
    cenTop = Mesh(filename = cenTopFile)
    cenTop.T = cenBase.T @ SE3(-0.003, 0.138, 0.0372).A @ trotx(-4*pi/5) #0.46
    env.add(cenTop)

    # Add test tube Rack and Move to starting position
    testTubeRackFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/TesttubeRack.dae'
    testTubeRack = Mesh(filename = testTubeRackFile)
    testTubeRack.T = SE3(1.51, 3.55, 0.585).A @ trotz(pi/2) #1.36
    env.add(testTubeRack)

    """
    dropperFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/dropper.dae'
    dropper = Mesh(filename = dropperFile)
    dropperOffset = SE3(-0.05, 0, 0).A @ trotz(pi)
    dropper.T = gp4.fkine(gp4.q).A @ dropperOffset
    env.add(dropper)
    """

    topperEEFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/topperEE.dae'
    topperEE = Mesh(filename = topperEEFile)
    topperEEOffset = trotx(pi) @ SE3(-0.07, 0, 0).A
    topperEE.T = gp4.fkine(gp4.q).A @ topperEEOffset
    env.add(topperEE)

    topperFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/Topper.dae'
    topPlanePoint = SE3(1.7, 3, 0.6).A
    topxOff = SE3(0.15, 0, 0).A
    topyOff = SE3(0, 0.15, 0).A

    grippers = gripperObj(env)
    env.add(grippers.gripFing1)
    env.add(grippers.gripFing2)
    gripperOffset = troty(-pi/2) @ trotx(pi/2)
    grippers.gripFing1.base = ur3.fkine(ur3.q).A @ gripperOffset
    grippers.gripFing2.base = ur3.fkine(ur3.q).A @ gripperOffset


    top1 = topperPython(topperFile, topPlanePoint, SE3(0, 0, 0), env)
    top2 = topperPython(topperFile, topPlanePoint @ topyOff, SE3(0, 0, 0), env)
    top3 = topperPython(topperFile, topPlanePoint @ topyOff @ topyOff, SE3(0, 0, 0), env)
    top4 = topperPython(topperFile, topPlanePoint @ topxOff, SE3(0, 0, 0), env)
    top5 = topperPython(topperFile, topPlanePoint @ topxOff @ topyOff, SE3(0, 0, 0), env)
    
    topList = [top1, top2, top3, top4, top5]

    camera_position = (SE3.Trans(4, 4, 1.4).A)[:3, 3]
    env.set_camera_pose(position=camera_position, look_at = ur3.base.A[:3, 3])

    # Add Test Tubes to Rack
    testTubeFileName = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/TestTube.dae'
    tt1 = testTubePython(testTubeFileName, SE3(testTubeRack.T[0, 3] + 0.075, 3.548, 0.65).A, SE3(1.8, 4.13, 0.68).A, env)
    tt2 = testTubePython(testTubeFileName, SE3(testTubeRack.T[0, 3] + 0.029, 3.548, 0.65).A, SE3(1.735, 4.175, 0.68).A, env)
    tt3 = testTubePython(testTubeFileName, SE3(testTubeRack.T[0, 3] - 0.018, 3.548, 0.65).A, SE3(1.866, 4.172, 0.68).A, env)
    tt4 = testTubePython(testTubeFileName, SE3(testTubeRack.T[0, 3] - 0.063, 3.548, 0.65).A, SE3(1.85, 4.256, 0.68).A, env)
    tt5 = testTubePython(testTubeFileName, SE3(testTubeRack.T[0, 3] - 0.109, 3.548, 0.65).A, SE3(1.75, 4.256, 0.68).A, env)
    
    ttList = [tt1, tt2, tt3, tt4, tt5]

    botSystem = newRobotSystem(ur3, gp4, ttList, topList, env, [grippers, topperEE, None], [gripperOffset, topperEEOffset, None])

    botSystem.simulation()

    #rmrc(gp4, topperEEOffset, SE3(0.07, 0, 0).A)


# ---------------------------------------------------------------------------------------#
if __name__ == "__main__": 
    #Launch swift   
    env = swift.Swift() 
    env.launch(realtime=True)

    ur3 = UR3e()
    gp4 = newGP4()

    initialise()

    #q_matrix = rtb.jtraj(gp4.q, (pi/2,pi/4,pi/8,pi/16,pi/32,pi/64), 100).q

    #for i in range(100):
        #gp4.q = q_matrix[i]
        #robot.q = q_matrix[i]
        #env.step(0.05)

    env.step(1)

    print("FINISHED")
    env.hold()
