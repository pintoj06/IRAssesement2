"""
This class is the main movment one. It's made of the UR3 and GP4 so far but needs to add
the ibus to it. So far it's just a class with the functions to move the GP4 and the test tube
toppers in the envrionment.
"""

from gp4 import newGP4
import roboticstoolbox as rtb
from roboticstoolbox import trapezoidal
import swift
from spatialmath import SE3
from spatialmath.base import *
from math import pi
from toppers import topperPython
from customUr3e import UR3e
from collisionDet import collisions
import numpy as np
import random

class newRobotSystem:
    def __init__(self, ur3: 'UR3e', gp4: 'newGP4', ttList: 'list', topList: 'list', env: 'swift.Swift', ee: 'list' = [], eeToolOffset: 'list' = []):
        self.ur3 = ur3
        self.ttList = ttList
        self.topList = topList
        self.env = env

        self.ur3 = ur3
        self.ur3eeToolOffset = eeToolOffset[0]
        self.ur3EE = ee[0]
        self.ur3RMRCoffset = SE3(0, 0, 0.25).A

        self.gp4 = gp4
        self.gp4Offset = SE3(0.07, 0, 0).A
        self.gp4eeToolOffset = eeToolOffset[1]
        self.gp4EE = ee[1]
        self.gp4RMRCoffset = SE3(0, 0, 0.3).A

        self.steps = 150
        self.collisionDet = collisions(self.ur3, self.gp4, self.env)
        self.useRRT = False


    def simulation(self):
        #self.rmrc(self.gp4, self.gp4eeToolOffset,  self.gp4Offset)
        #self.testRRTPastObj()


        #self.fillTubes()
        self.moveToppers()
        self.pickupToppers()
        self.moveToCentrifuge()
        
        #FOR FORCED COLLISION TESTING:
        self.jtrajMovegp4([-pi, -pi/2, pi/2, 0, 0, 0])
        #self.jtrajMoveur3([-pi, 0, pi/2, 0, 0, 0])

        #self.collectTubes()
    
    def moveToppers(self):
        if self.useRRT:
            for i in range(len(self.ttList)):
                self.primRRTgp4(self.gp4.ik_LM(self.topList[i].startPos @ self.gp4Offset @ self.gp4RMRCoffset @ trotx(pi))[0], [self.collisionDet.testRRt, self.collisionDet.table])
                self.primRRTgp4(self.gp4.ik_LM(self.ttList[i].startPos @ self.gp4Offset @ self.gp4RMRCoffset @ trotx(pi))[0], [self.collisionDet.testRRt, self.collisionDet.table], moveObj=[self.topList[i]], objOffset=[self.topList[i].offset])
            self.jtrajMovegp4([0, pi/2, 0, 0, 0, 0])
            self.jtrajMovegp4([-pi, pi/2, 0, 0, 0, 0])
        else:
            for i in range(len(self.ttList)):
                self.jtrajMovegp4(self.gp4.ik_LM(self.topList[i].startPos @ self.gp4Offset @ self.gp4RMRCoffset @ trotx(pi))[0])
                self.jtrajMovegp4(self.gp4.ik_LM(self.ttList[i].startPos @ self.gp4Offset @ self.gp4RMRCoffset @ trotx(pi))[0], moveObj=[self.topList[i]], objOffset=[self.topList[i].offset])
            self.jtrajMovegp4([0, pi/2, 0, 0, 0, 0])
            self.jtrajMovegp4([-pi, pi/2, 0, 0, 0, 0])
    
    def pickupToppers(self):
        self.rmrc(self.gp4eeToolOffset, self.gp4Offset)

    def moveToCentrifuge(self):
        if self.useRRT:
            for i in range(len(self.ttList)):
                self.primRRTur3(self.ur3.ik_LM(self.ttList[i].startPos @ self.ur3RMRCoffset @ trotx(pi) @ trotz(pi/2))[0], [self.collisionDet.testRRt, self.collisionDet.table])
                self.primRRTur3(self.ur3.ik_LM(self.ttList[i].endPos @ self.ur3RMRCoffset @ trotx(pi))[0], [self.collisionDet.testRRt, self.collisionDet.table], moveObj = [self.topList[i], self.ttList[i]], objOffset=[self.topList[i].ttoffset, self.ttList[i].offset])
        else:
            for i in range(len(self.ttList)):
                self.jtrajMoveur3(self.ur3.ik_LM(self.ttList[i].startPos @ self.ur3RMRCoffset @ trotx(pi) @ trotz(pi/2))[0])
                self.jtrajMoveur3(self.ur3.ik_LM(self.ttList[i].endPos @ self.ur3RMRCoffset @ trotx(pi))[0], moveObj = [self.topList[i], self.ttList[i]], objOffset=[self.topList[i].ttoffset, self.ttList[i].offset])

    def testRRTPastObj(self):
        self.collisionDet.testRRTgp4()
        self.useRRT = True
    
    def jtrajMovegp4(self, endq, moveObj:'list'=None, objOffset:'list'=None):
        """
        This is specifically for moving the gp4 using jtraj
        It can be copied and used for the other robots just change the EE and offset

        moveObj is a list of the objects that need to be moved with at the end effector
        It's originally at None so the robot can still move without anything in the end effector
        Also the function relies on the object that is moved being called meshObj in the class
            EXAMPLE IN TOPPERS.PY:
                self.meshObj = Mesh(filename = self.file)
                self.offset = SE3(-0.07, 0, 0.143).A

        """

        qMatrix = rtb.jtraj(self.gp4.q, endq, self.steps).q
        for x in range(self.steps):
            self.gp4.q = qMatrix[x]
            self.gp4EE.T = self.gp4.fkine(self.gp4.q).A @ self.gp4eeToolOffset
            if self.collisionDet.collisionCheck(self.gp4):
                print("COLLISION DETECTED")
                while True:
                    pass
            if moveObj is not None:
                for y in range(len(moveObj)):
                    moveObj[y].meshObj.T = self.gp4.fkine(self.gp4.q).A @ objOffset[y]
            self.env.step(0.015)

    def jtrajMoveur3(self, endq, moveObj:'list'=None, objOffset:'list'=None):
        qMatrix = rtb.jtraj(self.ur3.q, endq, self.steps).q
        for x in range(self.steps):
            self.ur3.q = qMatrix[x]
            self.ur3EE.gripFing1.base = self.ur3.fkine(self.ur3.q).A @ self.ur3eeToolOffset
            self.ur3EE.gripFing2.base = self.ur3.fkine(self.ur3.q).A @ self.ur3eeToolOffset
            self.collisionDet.collisionCheck(self.ur3)
            if moveObj is not None:
                for y in range(len(moveObj)):
                    moveObj[y].meshObj.T = self.ur3.fkine(self.ur3.q).A @ objOffset[y]
            self.env.step(0.015)

    def primRRTgp4(self, endq, myObj:'list', moveObj:'list'=None, objOffset:'list'=None):
        collisions = []
        q1 = self.gp4.q
        q2 = endq
        q_waypoints = np.array([q1, q2]) # Start and end positions
        is_collision_check = True
        checked_till_waypoint = 0 # How many times the function tries to find a way past
        q_matrix = []
        while is_collision_check:
            start_waypoint = checked_till_waypoint
            for i in range(start_waypoint, len(q_waypoints)-1):
                q_matrix_join = self.collisionDet.interpolate_waypoints_radians([q_waypoints[i], q_waypoints[i+1]], np.deg2rad(10))

                if not self.collisionDet.is_collision(self.gp4.dhRobot, q_matrix_join, myObj[0].faces, myObj[0].vertices, myObj[0].face_normals, collisions, return_once_found=True) and not self.collisionDet.is_collision(self.gp4.dhRobot, q_matrix_join, myObj[1].faces, myObj[1].vertices, myObj[1].face_normals, collisions, return_once_found=True):
                    q_matrix.extend(q_matrix_join)
                    for q in q_matrix_join:
                        self.gp4.q = q
                        self.gp4EE.T = self.gp4.fkine(self.gp4.q).A @ self.gp4eeToolOffset
                        if moveObj is not None:
                            for y in range(len(moveObj)):
                                moveObj[y].meshObj.T = self.gp4.fkine(self.gp4.q).A @ objOffset[y]
                        self.env.step(0.03)
                    is_collision_check = False
                    checked_till_waypoint = i+1
                    if checked_till_waypoint >= 30:
                        print('No Path Found')
                        return

                    # Now try to join to the final goal q2
                    q_matrix_join = self.collisionDet.interpolate_waypoints_radians([q_matrix[-1], q2], np.deg2rad(10))
                    if not self.collisionDet.is_collision(self.gp4.dhRobot, q_matrix_join, myObj[0].faces, myObj[0].vertices, myObj[0].face_normals, collisions, return_once_found=True) and not self.collisionDet.is_collision(self.gp4.dhRobot, q_matrix_join, myObj[1].faces, myObj[1].vertices, myObj[1].face_normals, collisions, return_once_found=True):
                        q_matrix.extend(q_matrix_join)
                        for q in q_matrix_join:
                            self.gp4.q = q
                            self.gp4EE.T = self.gp4.fkine(self.gp4.q).A @ self.gp4eeToolOffset
                            if moveObj is not None:
                                for y in range(len(moveObj)):
                                    moveObj[y].meshObj.T = self.gp4.fkine(self.gp4.q).A @ objOffset[y]
                            self.env.step(0.03)
                        # Reached goal without collision, so break out
                        print("reached goal")
                        break
                else:
                    # Randomly pick a pose that is not in collision
                    q_rand = np.zeros(6)
                    q_rand[0] = random.uniform(-17/18, 17/18) * np.pi
                    q_rand[1] = random.uniform(-11/18, 13/18) * np.pi
                    q_rand[2] = random.uniform(-65/180, 20/18) * np.pi
                    q_rand[3] = random.uniform(-20/18, 20/18) * np.pi
                    q_rand[4] = random.uniform(-123/180, 123/180) * np.pi
                    q_rand[5] = random.uniform(-455/180, 455/180) * np.pi
                    q_rand = q_rand.tolist()  # Convert to a 3-element list

                    while self.collisionDet.is_collision(self.gp4.dhRobot, [q_rand], myObj[0].faces, myObj[0].vertices, myObj[0].face_normals, collisions, return_once_found=True) and self.collisionDet.is_collision(self.gp4.dhRobot, [q_rand], myObj[1].faces, myObj[1].vertices, myObj[1].face_normals, collisions, return_once_found=True):
                        q_rand = np.zeros(6)
                        q_rand[0] = random.uniform(-17/18, 17/18) * np.pi
                        q_rand[1] = random.uniform(-11/18, 13/18) * np.pi
                        q_rand[2] = random.uniform(-65/180, 20/18) * np.pi
                        q_rand[3] = random.uniform(-20/18, 20/18) * np.pi
                        q_rand[4] = random.uniform(-123/180, 123/180) * np.pi
                        q_rand[5] = random.uniform(-455/180, 455/180) * np.pi
                        q_rand = q_rand.tolist()  # Convert to a 3-element list
                    q_waypoints = np.concatenate((q_waypoints[:i+1], [q_rand], q_waypoints[i+1:]), axis=0)
                    is_collision_check = True
                    break

        # Check again
        if self.collisionDet.is_collision(self.gp4.dhRobot, q_matrix_join, myObj[0].faces, myObj[0].vertices, myObj[0].face_normals, collisions, return_once_found=True) and self.collisionDet.is_collision(self.gp4.dhRobot, q_matrix_join, myObj[1].faces, myObj[1].vertices, myObj[1].face_normals, collisions, return_once_found=True):
            print('Collision detected!')
        else:
            print('No collision found')

    def rmrc(self, offsetTop: 'SE3', eeOffset: 'SE3'):
        # 1.1) Set parameters for the simulation
        #                # Load robot model
        t = 3                                     # Total time (s)
        delta_t = 0.02                             # Control frequency
        steps = int(t/delta_t)                     # No. of steps for simulation
        delta = 2*pi/steps                         # Small angle change
        epsilon = 0.1                              # Threshold value for manipulability/Damped Least Squares
        W = np.diag([1, 1, 1, 0.1, 0.1, 1])      # Weighting matrix for the velocity vector

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
        currentPos = self.gp4.fkine(self.gp4.q).A @ eeOffset             # Trapezoidal trajectory scalar
        for i in range(steps):
            x[0,i] = currentPos[0,3]   # Points in x
            x[1,i] = currentPos[1,3]                # Points in y
            x[2,i] = currentPos[2,3] - 0.05 * s[i]               # Points in z
            theta[0,i] = 0                          # Roll angle 
            theta[1,i] = pi                          # Pitch angle
            theta[2,i] = 0                          # Yaw angle
        
        T = transl(x[:,0]) @ rpy2tr(theta[0,0], theta[1,0], theta[2,0])       # Create transformation of first point and angle     
        q0 = np.zeros([1,6])                                                  # Initial guess for joint angles
        q_matrix[0,:] = self.gp4.ikine_LM(T, q0).q                                # Solve joint angles to achieve first waypoint
        qlim = np.transpose(self.gp4.qlim)
        
        # 1.4) Track the trajectory with RMRC
        for i in range(steps-1):
            T = self.gp4.fkine(q_matrix[i,:]).A                                   # Get forward transformation at current joint state
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
            J = self.gp4.jacob0(q_matrix[i,:])                                    # Get Jacobian at current joint state
            m[i] = np.sqrt(np.linalg.det(J @ J.T))
            if m[i] < epsilon:                                                # If manipulability is less than given threshold
                m_lambda = (1 - m[i]/epsilon) * 0.05
            else:
                m_lambda = 0
            inv_j = np.linalg.inv(J.T @ J + m_lambda * np.eye(6)) @ J.T          # DLS Inverse
            qdot[i,:] = (inv_j @ xdot).T                                      # Solve the RMRC equation (you may need to transpose the         vector)
            for j in range(6):                                                # Loop through joints 1 to 6
                if q_matrix[i,j] + delta_t*qdot[i,j] < qlim[j,0]:             # If next joint angle is lower than joint limit...
                    qdot[i,j] = 0 # Stop the motor
                elif q_matrix[i,j] + delta_t*qdot[i,j] > qlim[j,1]:           # If next joint angle is greater than joint limit ...
                    qdot[i,j] = 0 # Stop the motor
                
            q_matrix[i+1,:] = q_matrix[i,:] + delta_t*qdot[i,:]               # Update next joint state based on joint velocities
            
            
            position_error[:,i] = x[:,i+1] - T[:3,3]                          # For plotting
            angle_error[:,i] = delta_theta                                    # For plotting

        # 1.5) Plot the results
        for q in q_matrix:
            self.gp4.q = q
            self.gp4EE.T = self.gp4.fkine(self.gp4.q).A @ self.gp4eeToolOffset
            pos = self.gp4.fkine(q).A[:3,3] 
            self.env.step(0.05)
        
    def primRRTur3(self, endq, myObj, moveObj:'list'=None, objOffset:'list'=None):
        collisions = []
        q1 = self.ur3.q
        q2 = endq
        q_waypoints = np.array([q1, q2]) # Start and end positions
        is_collision_check = True
        checked_till_waypoint = 0 # How many times the function tries to find a way past
        q_matrix = []
        while is_collision_check:
            start_waypoint = checked_till_waypoint
            for i in range(start_waypoint, len(q_waypoints)-1):
                q_matrix_join = self.collisionDet.interpolate_waypoints_radians([q_waypoints[i], q_waypoints[i+1]], np.deg2rad(10))

                if not self.collisionDet.is_collision(self.ur3.dhRobot, q_matrix_join, myObj[0].faces, myObj[0].vertices, myObj[0].face_normals, collisions, return_once_found=True) and not self.collisionDet.is_collision(self.ur3.dhRobot, q_matrix_join, myObj[1].faces, myObj[1].vertices, myObj[1].face_normals, collisions, return_once_found=True):
                    q_matrix.extend(q_matrix_join)
                    for q in q_matrix_join:
                        self.ur3.q = q
                        self.ur3EE.gripFing1.base = self.ur3.fkine(self.ur3.q).A @ self.ur3eeToolOffset
                        self.ur3EE.gripFing2.base = self.ur3.fkine(self.ur3.q).A @ self.ur3eeToolOffset
                        if moveObj is not None:
                            for y in range(len(moveObj)):
                                moveObj[y].meshObj.T = self.ur3.fkine(self.ur3.q).A @ objOffset[y]
                        self.env.step(0.03)
                    is_collision_check = False
                    checked_till_waypoint = i+1
                    if checked_till_waypoint >= 30:
                        print('No Path Found')
                        return

                    # Now try to join to the final goal q2
                    q_matrix_join = self.collisionDet.interpolate_waypoints_radians([q_matrix[-1], q2], np.deg2rad(10))
                    if not self.collisionDet.is_collision(self.ur3.dhRobot, q_matrix_join, myObj[0].faces, myObj[0].vertices, myObj[0].face_normals, collisions, return_once_found=True) and not self.collisionDet.is_collision(self.ur3.dhRobot, q_matrix_join, myObj[1].faces, myObj[1].vertices, myObj[1].face_normals, collisions, return_once_found=True):
                        q_matrix.extend(q_matrix_join)
                        for q in q_matrix_join:
                            self.ur3.q = q
                            self.ur3EE.gripFing1.base = self.ur3.fkine(self.ur3.q).A @ self.ur3eeToolOffset
                            self.ur3EE.gripFing2.base = self.ur3.fkine(self.ur3.q).A @ self.ur3eeToolOffset
                            if moveObj is not None:
                                for y in range(len(moveObj)):
                                    moveObj[y].meshObj.T = self.ur3.fkine(self.ur3.q).A @ objOffset[y]
                            self.env.step(0.03)
                        # Reached goal without collision, so break out
                        break
                else:
                    # Randomly pick a pose that is not in collision
                    q_rand = (2 * np.random.rand(1, 6) - 1) * np.pi
                    q_rand = q_rand.tolist()[0]  # Convert to a 6-element list

                    while self.collisionDet.is_collision(self.ur3.dhRobot, [q_rand], myObj[0].faces, myObj[0].vertices, myObj[0].face_normals, collisions, return_once_found=True) and self.collisionDet.is_collision(self.ur3.dhRobot, [q_rand], myObj[1].faces, myObj[1].vertices, myObj[1].face_normals, collisions, return_once_found=True):
                        q_rand = (2 * np.random.rand(1, 6) - 1) * np.pi
                        q_rand = q_rand.tolist()[0]  # Convert to a 3-element list
                    q_waypoints = np.concatenate((q_waypoints[:i+1], [q_rand], q_waypoints[i+1:]), axis=0)
                    is_collision_check = True
                    break

        # Check again
        if self.collisionDet.is_collision(self.ur3.dhRobot, q_matrix_join, myObj[0].faces, myObj[0].vertices, myObj[0].face_normals, collisions, return_once_found=True) and self.collisionDet.is_collision(self.ur3.dhRobot, q_matrix_join, myObj[1].faces, myObj[1].vertices, myObj[1].face_normals, collisions, return_once_found=True):
            print('Collision detected!')
        else:
            print('No collision found')
