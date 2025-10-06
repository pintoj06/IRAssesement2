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

    # Add GP4 and Move to starting position
    gp4.base = SE3(1.5, 3.1, 0.585).A @ trotz(pi/2)
    gp4.add_to_env(env)

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

    #rmrc(gp4, topperOffset, eeOffset)

def rmrc(robot: 'newGP4', offsetTop: 'SE3', eeOffset: 'SE3'):
    # 1.1) Set parameters for the simulation                 
    t = 10                                     # Total time (s)
    delta_t = 0.02                             # Control frequency
    steps = int(t/delta_t)                     # No. of steps for simulation
    delta = 2*pi/steps                         # Small angle change
    epsilon = 0.1                              # Threshold value for manipulability/Damped Least Squares
    W = np.diag([1, 1, 1, 0, 0, 0])      # Weighting matrix for the velocity vector

    # 1.2) Allocate array data
    m = np.zeros([steps,1])                    # Array for Measure of Manipulability
    q_matrix = np.zeros([steps,6])             # Array for joint anglesR
    qdot = np.zeros([steps,6])                 # Array for joint velocities
    theta = np.zeros([3,steps])                # Array for roll-pitch-yaw angles
    x = np.zeros([3,steps])                    # Array for x-y-z trajectory
    position_error = np.zeros([3,steps])       # For plotting trajectory error
    angle_error = np.zeros([3,steps])          # For plotting trajectory error

    # 1.3) Set up trajectory, initial pose
    s = trapezoidal(0,1,steps).q
    currentPos = gp4.fkine(gp4.q).A @ eeOffset             # Trapezoidal trajectory scalar
    for i in range(steps):
        x[0,i] = currentPos[0,3]   # Points in x
        x[1,i] = currentPos[1,3]                # Points in y
        x[2,i] = currentPos[2,3] - 0.05 * s[i]               # Points in z
        theta[0,i] = 0                          # Roll angle 
        theta[1,i] = 0                          # Pitch angle
        theta[2,i] = 0                          # Yaw angle
    
    T = transl(x[:,0]) @ rpy2tr(theta[0,0], theta[1,0], theta[2,0])       # Create transformation of first point and angle     
    q0 = np.zeros([1,6])                                                  # Initial guess for joint angles
    q_matrix[0,:] = gp4.ikine_LM(T, q0).q                                # Solve joint angles to achieve first waypoint
    qlim = np.transpose(gp4.qlim)
    
    # 1.4) Track the trajectory with RMRC
    for i in range(steps-1):
        T = gp4.fkine(q_matrix[i,:]).A                                   # Get forward transformation at current joint state
        delta_x = x[:,i+1] - T[:3,3]                                      # Get position error from next waypoint
        Rd = rpy2r(theta[0,i+1], theta[1,i+1], theta[2,i+1])              # Get next RPY angles, convert to rotation matrix
        Ra = T[:3,:3]                                                     # Current end-effector rotation matrix
        Rdot = (1/delta_t)*(Rd - Ra)                                      # Calculate rotation matrix error
        S = Rdot @ Ra.T                                                   # Skew symmetric!
        linear_velocity = (1/delta_t)*delta_x
        angular_velocity = np.array([S[2,1], S[0,2], S[1,0]])             # Check the structure of Skew Symmetric matrix!!
        delta_theta = tr2rpy(Rd @ Ra.T, order='xyz')                      # Convert rotation matrix to RPY angles
        xdot = W @ np.vstack((linear_velocity.reshape(3,1), 
                              angular_velocity.reshape(3,1)))             # Calculate end-effector velocity to reach next waypoint.
        J = gp4.jacob0(q_matrix[i,:])                                    # Get Jacobian at current joint state
        m[i] = np.sqrt(np.linalg.det(J @ J.T))
        if m[i] < epsilon:                                                # If manipulability is less than given threshold
            m_lambda = (1 - m[i]/epsilon) * 0.05
            print("SINUGLARITY")
        else:
            m_lambda = 0
        inv_j = np.linalg.inv(J.T @ J + m_lambda * np.eye(6)) @ J.T          # DLS Inverse
        qdot[i,:] = (inv_j @ xdot).T                                      # Solve the RMRC equation (you may need to transpose the         vector)
        for j in range(6):                                                # Loop through joints 1 to 6
            if q_matrix[i,j] + delta_t*qdot[i,j] < qlim[j,0]:             # If next joint angle is lower than joint limit...
                qdot[i,j] = 0 # Stop the motor
                print("LOWER")
            elif q_matrix[i,j] + delta_t*qdot[i,j] > qlim[j,1]:           # If next joint angle is greater than joint limit ...
                qdot[i,j] = 0 # Stop the motor
                print("HIGHER")
            
        q_matrix[i+1,:] = q_matrix[i,:] + delta_t*qdot[i,:]               # Update next joint state based on joint velocities
        
        
        position_error[:,i] = x[:,i+1] - T[:3,3]                          # For plotting
        angle_error[:,i] = delta_theta                                    # For plotting

    # 1.5) Plot the results
    for q in q_matrix:
        gp4.q = q
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

    env.hold()
