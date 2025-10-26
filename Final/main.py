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
from spatialgeometry import Mesh, Sphere
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
from IGUS_testcode import ReBeL
from tkinterGUI import JointControlUI
from SpecimenLiquid import specimenLiquid 
import threading
import serial

stop_event = None
restart_event = None

# ---------------------------------------------------------------------------------------#
def initialise():
    # Add room
    #A2-GithunTotalCode-8-10-25\IRAssesement2-main\enivornmentFiles\CentrifugeEnvironment.dae

    # 1) Create Environment
    envFile = r'enivornmentFiles\CentrifugeEnvironment.dae' # FOR JAYDEN
    envFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/EnivornmentFiles/CentrifugeEnvironment.dae' # FOR HARRY
    envRoom = Mesh(filename = envFile) 
    env.add(envRoom)
    
    # 2) Add Specimen Liquid 1
    specimenLiquidFile = r'enivornmentFiles\3DSPECIMENLIQUID.dae' # FOR JAYDEN
    specimenLiquidFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/3DSPECIMENLIQUID.dae' # FOR HARRY
    specimen1LiquidList = []
    for i in range (1, 6): #change to 6 for 6 specimens after testing
        s1 = specimenLiquid(specimenLiquidFile, SE3(1.0, 3.1, 0.685).A, env)
        specimen1LiquidList.append(s1)
    
    # Add specimen Liquid 2
    specimenLiquidFile2 = r'3DSPECIMENLIQUIDGREEN.dae' # FOR JAYDEN
    specimenLiquidFile2 = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/3DSPECIMENLIQUIDGREEN.dae' # FOR HARRY
    specimen2LiquidList = []
    for i in range (1, 6): #change to 6 for 6 specimens after testing
        s1 = specimenLiquid(specimenLiquidFile2, SE3(1.0, 3.9, 0.685).A, env)
        specimen2LiquidList.append(s1)
    
                                  
    # Add UR3 and Move to starting position
    ur3.base = SE3(1.5, 4.0, 0.585).A 
    ur3.q = [pi,-pi/2,0,0,0,0]
    ur3.add_to_env(env)
    env.add(ur3.dhRobot)
    ur3.dhRobot.base = SE3(1.5, 4.0, 0.585).A 
    ur3.dhRobot.q = ur3.q

    # Add GP4 and Move to starting position
    gp4.base = SE3(1.5, 3.1, 0.585).A @ trotz(pi/2)
    gp4.q = [-pi, pi/4, pi/4, 0, 0, 0]
    gp4.add_to_env(env)
    env.add(gp4.dhRobot)
    gp4.dhRobot.base = SE3(1.5, 3.1, 0.585).A @ trotz(pi/2)
    gp4.dhRobot.q = gp4.q

    # Add REBEL and Move to starting Position
    rebel.base = SE3(1.2, 3.5, 0.585).A 
    rebel.add_to_env(env)
    env.add(rebel.dhRobot)
    rebel.dhRobot.base = SE3(1.2, 3.5, 0.585).A 
    rebel.dhRobot.q = rebel.q
    

    # Add centrifuge base and move to starting position
    cenBaseFile = r'enivornmentFiles\CentrifugeBottom.dae'
    cenBaseFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/EnivornmentFiles/CentrifugeBottom.dae' # FOR HARRY'
    cenBase = Mesh(filename = cenBaseFile)
    cenBase.T = SE3(1.8, 4.2, 0.68).A #1.1, 4.2
    env.add(cenBase)

    # Add centrifuge top and move to starting position
    cenTopFile = r'enivornmentFiles\CentrifugeTop.dae' # FOR JAYDEN
    cenTopFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/CentrifugeTop.dae' # FOR HARRY
    cenTop = Mesh(filename = cenTopFile)
    cenTop.T = cenBase.T @ SE3(-0.003, 0.138, 0.0372).A @ trotx(-4*pi/5) #0.46
    env.add(cenTop)

    # Add test tube Rack and Move to starting position
    testTubeRackFile = r'enivornmentFiles\TesttubeRack.dae' # FOR JAYDEN
    testTubeRackFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/EnivornmentFiles/TesttubeRack.dae' # FOR HARRY
    testTubeRack = Mesh(filename = testTubeRackFile)
    testTubeRack.T = SE3(1.51, 3.55, 0.585).A @ trotz(pi/2) #1.36
    env.add(testTubeRack)

    # Add specimens to extract liquid from 
    specimen1File = r'enivornmentFiles\Specimen1.dae' # FOR JAYDEN
    specimen1File = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/EnivornmentFiles/Specimen1.dae' # FOR HARRY
    specimen1 = Mesh(filename = specimen1File)
    specimen1.T = SE3(1.0, 3.1, 0.65).A @ trotz(pi/2) #1.36
    env.add(specimen1)

    specimen2File = r'enivornmentFiles\Specimen2.dae' # FOR JAYDEN
    specimen2File = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/EnivornmentFiles/Specimen2.dae' # FOR HARRY
    specimen2 = Mesh(filename = specimen2File)
    specimen2.T = SE3(1.0, 3.9, 0.685).A @ trotz(pi/2) #1.36
    env.add(specimen2)

    # Add topper end effector and attach to the GP4
    topperEEFile = r'enivornmentFiles\topperEE.dae' # FOR JAYDEN
    topperEEFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/EnivornmentFiles/topperEE.dae' # FOR HARRY
    topperEE = Mesh(filename = topperEEFile)
    topperEEOffset = trotx(pi) @ SE3(-0.07, 0, 0).A
    topperEE.T = gp4.fkine(gp4.q).A @ topperEEOffset
    env.add(topperEE)

    # Add the toppers to the environment
    topperFile = r'enivornmentFiles\Topper.dae' # FOR JAYDEN
    topperFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/EnivornmentFiles/Topper.dae' # FOR HARRY
    topPlanePoint = SE3(1.92, 2.82, 0.6).A
    topyOff = SE3(0, 0.05, 0).A

    top1 = topperPython(topperFile, topPlanePoint, env)
    top2 = topperPython(topperFile, topPlanePoint @ topyOff, env)
    top3 = topperPython(topperFile, topPlanePoint @ topyOff @ topyOff, env)
    top4 = topperPython(topperFile, topPlanePoint @ topyOff @ topyOff @ topyOff, env)
    top5 = topperPython(topperFile, topPlanePoint @ topyOff @ topyOff @ topyOff @ topyOff, env)
    
    topList = [top1, top2, top3, top4, top5]

    # Add grippers to the UR3
    grippers = gripperObj(env)
    gripperOffset = troty(-pi/2) @ trotx(pi/2)
    grippers.gripFing1.base = ur3.fkine(ur3.q).A @ gripperOffset
    grippers.gripFing2.base = ur3.fkine(ur3.q).A @ gripperOffset
    env.add(grippers.gripFing1)
    env.add(grippers.gripFing2)
    

    #Rebel gripper 
    pippetteEEFile = r'enivornmentFiles\pippettev2.dae' # FOR JAYDEN
    pippetteEEFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/EnivornmentFiles/pippette.dae' # FOR HARRY
    pipetteEE = Mesh(filename = pippetteEEFile)
    pipetteEEOffset = SE3(-0.16243,0,0)
    pipetteEE.T = rebel.fkine(rebel.q).A @ troty(pi/2)  
    env.add(pipetteEE)

    camera_position = (SE3.Trans(4, 4, 1.4).A)[:3, 3]
    env.set_camera_pose(position=camera_position, look_at = ur3.base.A[:3, 3])

    # Add Test Tubes to Rack
    testTubeFileName = r'enivornmentFiles\TestTube.dae' # FOR JAYDEN
    testTubeFileName = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/environmentFiles/TestTube.dae' # FOR HARRY
    tt1 = testTubePython(testTubeFileName, SE3(testTubeRack.T[0, 3] + 0.075, 3.548, 0.65).A, SE3(1.8, 4.13, 0.68).A, env)
    tt2 = testTubePython(testTubeFileName, SE3(testTubeRack.T[0, 3] + 0.029, 3.548, 0.65).A, SE3(1.736, 4.175, 0.68).A, env)
    tt3 = testTubePython(testTubeFileName, SE3(testTubeRack.T[0, 3] - 0.018, 3.548, 0.65).A, SE3(1.866, 4.172, 0.68).A, env)
    tt4 = testTubePython(testTubeFileName, SE3(testTubeRack.T[0, 3] - 0.063, 3.548, 0.65).A, SE3(1.85, 4.256, 0.68).A, env)
    tt5 = testTubePython(testTubeFileName, SE3(testTubeRack.T[0, 3] - 0.109, 3.548, 0.65).A, SE3(1.75, 4.256, 0.68).A, env)
    
    ttList = [tt1, tt2, tt3, tt4, tt5]

    dummyFile = '/Users/harrymentis/Documents/SensorsAndControls/dummy.dae' # For Jayden
    dummyFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/EnivornmentFiles/dummy.dae' # For Harry
    dummy = Mesh(filename = dummyFile)
    dummy.T = SE3(1.5, 1.8, 0.1).A @ trotz(pi)
    env.add(dummy)
    dummyMoveThread = threading.Thread(target=moveDummy(dummy))

    dummyMoveThread.start()
    env.step(0.1)

    botSystem = newRobotSystem(ur3, gp4, rebel, cenTop, stop_event, restart_event, ttList, topList, env, [grippers, topperEE, pipetteEE], [gripperOffset, topperEEOffset, pipetteEEOffset], specimen1LiquidList, specimen2LiquidList)
    GUI = JointControlUI(botSystem, stop_event, restart_event)
    print("GUI launched")
    

  
def checkForPress():
    arduino_port = 'COM3' # FOR JAYDEN
    arduino_port = '/dev/cu.usbmodem34B7DA63709C2' # FOR HARRY
    baud_rate = 9600

    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)

    try:
        while True:
            if ser.in_waiting >0:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    stop_event.set()

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("Serial port closed.")
    pass

def moveDummy(dummy):
    print("IN")
    rot1 = np.linspace(pi, pi/2, 100, endpoint=True) #First step front arm rotation
    for i in rot1:
        dummy.T = SE3(1.5, 1.8, 0.1).A @ trotz(i)
        env.step(0.05)
    trans = np.linspace(1.5, 2.2, 100, endpoint=True)
    for i in trans:
        dummy.T = SE3(i, 1.8, 0.1).A @ trotz(pi/2)
        env.step(0.05)
    stop_event.set()



# ---------------------------------------------------------------------------------------#
if __name__ == "__main__": 
    #Launch swift   
    env = swift.Swift() 
    env.launch(realtime=True)

    ur3 = UR3e()
    rebel = ReBeL()
    gp4 = newGP4()

    stop_event = threading.Event()
    restart_event = threading.Event()
    buttonPressThread = threading.Thread(target=checkForPress)
    buttonPressThread.start()

    initialise()

    env.step(1)

    print("FINISHED")
    env.hold()
