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
from IGUS_testcode import ReBeL
import numpy as np
import random
from spatialgeometry import Mesh, Sphere
import time
from SpecimenLiquid import specimenLiquid
class newRobotSystem:
    def __init__(self, ur3: 'UR3e', gp4: 'newGP4', rebel: 'ReBeL', cenTop, ttList: 'list', topList: 'list', env: 'swift.Swift', ee: 'list' = [], eeToolOffset: 'list' = [], specimen1LiquidList: 'list' = []):
        self.ttList = ttList # List of test tubes
        self.topList = topList # List of toppers
        self.env = env # Environment
        self.cenTop = cenTop # Centrifuge Lid
 
        self.ur3 = ur3 # Ur3 Robot
        self.ur3eeToolOffset = eeToolOffset[0] # Offset for the grippers from first element of list
        self.ur3EE = ee[0] # UR3 Gripper from first element of gripper list
        self.ur3RMRCoffsetOne = SE3(0, 0, 0.25).A # Offset for RMRC Movement 1
        self.ur3RMRCoffsetTwo = SE3(0, 0, 0.28).A # Offset for RMRC Movement 1

        self.gp4 = gp4 # GP4 Robot
        self.gp4Offset = SE3(0.07, 0, 0).A # Offset for the gp4 to account for error
        self.gp4eeToolOffset = eeToolOffset[1] # Offset for the end effector from second element of list
        self.gp4EE = ee[1] # GP4 end effector from second element of gripper list
        self.gp4RMRCoffset = SE3(0, 0, 0.3).A # Offset to account for the RMRC movement

        self.rebel = rebel # ReBel Robot
        self.rebeleetoolOffset = eeToolOffset[2] # Offset for the end effector from third element of list
        self.rebelEE = ee[2] # ReBel end effector from third element of gripper list

        self.steps = 150 # Nnumber of steps for jtraj
        self.collisionDet = collisions(self.ur3, self.gp4, self.rebel, self.env) # Create collision detection objects
        self.specimen1LiquidList = specimen1LiquidList # List of specimen liquids
        self.useRRT = False # Boolean to declare the use of RRT in the movment functions
        self.running = True
        self.boolSpecimen1 = True

        self.alarmhide = SE3(10, 10, 10).A # Position to hide the alarms (removing them caused an error)
        self.alarmshow = SE3(2.53, 2.18, 1.5).A # Position of alarm on wall

        greenAlarmFile = '' # FOR JAYDEN TO ADD
        greenAlarmFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/environmentFiles/greenAlarm.dae' # FOR HARRY
        self.greenAlarm = Mesh(filename = greenAlarmFile) # Create the green alarm object
        self.greenAlarm.T = self.alarmshow # Start with the green alarm showing
        self.env.add(self.greenAlarm) # Add the green alarm to the environment

        redAlarmFile = '' # FOR JAYDEN TO ADD
        redAlarmFile = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/environmentFiles/redAlarm.dae' # FOR HARRY
        self.redAlarm = Mesh(filename = redAlarmFile) # Create the red alarm object
        self.redAlarm.T = self.alarmhide # Start with the grredeen alarm showing
        self.env.add(self.redAlarm) # Add the red alarm to the environment




    def simulation(self):
        self.greenAlarm.T = self.alarmhide # Hide green alarm
        self.redAlarm.T = self.alarmshow # Show red alarm

        #self.testRRTPastObj()

        self.fillTubes() # Fill the test tubes with sample
        self.moveToppers() # Move the toppers to the test tubes
        self.moveToCentrifuge() # Move the test tubes to the centrifuge
        self.closeCentrifugeLid() # Close the centrifuge
        self.openCentrifugeLid() # Open the centrifuge
        self.moveToRack() # Move the test tubes to the rack
        
        #FOR FORCED COLLISION TESTING:
        #self.jtrajMovegp4([-pi, -pi/2, pi/2, 0, 0, 0])

        self.greenAlarm.T = self.alarmshow # Show the green alarm
        self.redAlarm.T = self.alarmhide # Hide the red alarm
        self.env.step(0.1)

    def fillTubes(self):
        #test sphere
        
        '''
          Jayden needs to fix, not sure why jtraj not working
        '''

        self.env.step(0.05)
        if self.useRRT:
            pass #RRT NOT SET UP FOR THIS YET
        else:
            for i in range(0, len(self.ttList)): # Repeat for each test tube
                #Go to designated specimen 
                if self.boolSpecimen1:
                    specimenLocation = self.specimen1LiquidList[0].startPos
                    q1 = specimenLocation @ SE3.Ry(pi/2).A
                    q2 = q1 @ SE3(-0.17,0,0).A
                    q_goal= self.rebel.ik_LM(q2, q0 = [3.027, 1.101, 0.087, 0.105, -1.264, 0.0])[0]#change to Specimen 1 location
                else:
                    pass 
                    #q_goal= self.rebel.ik_LM(SE3(1.0, 3.1, 0.885) @ SE3(-0.16243,0,0),  q0 = [-1.303, 1.303, -0.410, 0.311, -0.583, 0.000])[0]# change to Specimen 2 location
                self.jtrajMoverebel(q_goal) 
                self.specimen1LiquidList[i].attachToRobot(self.rebel, self.env)
               


                #move to the indexed test tube
                q_goal = self.rebel.ik_LM(self.ttList[i].startPos @ self.ttList[i].offset @ troty(-pi/2) @ SE3(-0.2,0,0).A, q0 = [-1.303, 1.303, -0.410, 0.311, -0.583, 0.000])[0]
                self.jtrajMoverebel(q_goal, self.specimen1LiquidList[i])

                self.rmrcrebel(-0.2, 3, [self.specimen1LiquidList[i]], objOffset=[SE3(0,0,0).A]) #move down to fill tube

                self.specimen1LiquidList[i].attachToTestTube(self.ttList[i].startPos, self.env)
            self.jtrajMoverebel([0, pi/2, pi/2,0,0,pi/2])
                                                                   
    def moveToppers(self):
        """
        This function moves the toppers from the plane (starting position) to the top of the test tubes where they are placed
        It uses a mix of regular path planning using JTRAJ, however also accomidates RMRC for the vertical components and can use RRT when an obstacle is in the way
        """
        if self.useRRT:
            self.jtrajMovegp4([0, pi/2, 0, 0, 0, 0]) # Move to robot starting position
            for i in range(len(self.ttList)): # Repeat the following for each test tube
                self.primRRTgp4(self.gp4.ik_LM(self.topList[i].startPos @ self.gp4Offset @ self.gp4RMRCoffset @ trotx(pi))[0], [self.collisionDet.testRRt, self.collisionDet.table]) # Move to topper using RRT
                self.rmrcgp4(-0.14, 3) # Use RMRC to move directly down 0.14mm to pick up the topper 
                self.primRRTgp4(self.gp4.ik_LM(self.ttList[i].startPos @ self.gp4Offset @ self.gp4RMRCoffset @ trotx(pi))[0], [self.collisionDet.testRRt, self.collisionDet.table], moveObj=[self.topList[i]], objOffset=[self.topList[i].offset]) #Move directly to the test tube using RRT
                self.rmrcgp4(-0.08, 3, moveObj=[self.topList[i]], objOffset=[self.topList[i].offset]) # Use RMRC to move the topper directly on top of the test tube
            self.jtrajMovegp4([0, pi/2, 0, 0, 0, 0]) # Move back to starting position
            self.jtrajMovegp4([-pi, pi/2, 0, 0, 0, 0]) # Rotate 180 degrees to be out of the way of the UR3
        else:
            self.jtrajMovegp4([0, pi/2, 0, 0, 0, 0]) # Move to robot starting position
            for i in range(len(self.ttList)): # Repeat the following for each test tube
                self.jtrajMovegp4(self.gp4.ik_LM(self.topList[i].startPos @ self.gp4Offset @ self.gp4RMRCoffset @ trotx(pi))[0]) # Move directly above the test topper
                self.rmrcgp4(-0.14, 3) # Use RMRC to move directly down 0.14mm to pick up the topper 
                self.jtrajMovegp4(self.gp4.ik_LM(self.ttList[i].startPos @ self.gp4Offset @ self.gp4RMRCoffset @ trotx(pi))[0], moveObj=[self.topList[i]], objOffset=[self.topList[i].offset]) # Move directly above the test tube
                self.rmrcgp4(-0.08, 3, moveObj=[self.topList[i]], objOffset=[self.topList[i].offset]) # Use RMRC to move the topper directly on top of the test tube
            self.jtrajMovegp4([0, pi/2, 0, 0, 0, 0]) # Move back to starting position
            self.jtrajMovegp4([-pi, pi/2, 0, 0, 0, 0]) # Rotate 180 degrees to be out of the way of the UR3

    def moveToCentrifuge(self):
            self.jtrajMoveur3([0,-pi/2,0,0,0,0]) # Move to robot starting position
            for i in range(len(self.ttList)): # Repeat the following for each test tube
                self.jtrajMoveur3(self.ur3.ik_LM(self.ttList[i].startPos @ self.ur3RMRCoffsetOne @ trotx(pi) @ trotz(pi/2))[0]) # Move directly above the test tube
                self.rmrcur3(-0.05, 2) # Use RMRC to move down to the exact position for pickup
                self.ur3EE.closeGripper() # Close the gripper to pick up the test tube
                self.rmrcur3(0.11, 2, [self.topList[i], self.ttList[i], self.specimen1LiquidList[i]], objOffset=[self.topList[i].ttoffset, self.ttList[i].offset,SE3(0,0,0.13).A]) # Move the end effector directly up to remove the test tube from the holder
                self.jtrajMoveur3(self.ur3.ik_LM(self.ttList[i].endPos @ self.ur3RMRCoffsetTwo @ trotx(pi))[0], moveObj = [self.topList[i], self.ttList[i], self.specimen1LiquidList[i]], objOffset=[self.topList[i].ttoffset, self.ttList[i].offset, SE3(0,0,0.13).A]) # Move directly to the test tubes final position located in the centrifuge # Move directly down to place the test tube in the centrifuge
                self.rmrcur3(-0.1, 2, [self.topList[i], self.ttList[i],  self.specimen1LiquidList[i]], objOffset=[self.topList[i].ttoffset, self.ttList[i].offset, SE3(0,0,0.13).A]) # Move directly down to place the test tube in the centrifuge
                self.ur3EE.openGripper() # Open the gripper to release the test tube
            self.jtrajMoveur3([0,-pi/2,0,0,0,0]) # Move back to the starting position
    
    def moveToRack(self):
            for i in range(len(self.ttList)): # Repeat the following for each test tube
                self.jtrajMoveur3(self.ur3.ik_LM(self.ttList[i].endPos @ self.ur3RMRCoffsetTwo @ trotx(pi))[0]) # Move diretly above the test tube
                self.rmrcur3(-0.1, 2) # Move directly down to a safe distance above the test tube
                self.ur3EE.closeGripper() # Close the gripper to pick u pthe test tube
                self.rmrcur3(0.1, 2, [self.topList[i], self.ttList[i], self.specimen1LiquidList[i] ], objOffset=[self.topList[i].ttoffset, self.ttList[i].offset, SE3(0,0,0.13).A]) # Move the end effector directly up to remove the test tube from the centrifuge
                self.jtrajMoveur3(self.ur3.ik_LM(self.ttList[i].startPos @ SE3(0,0,0.28).A @ trotx(pi) @ trotz(pi/2))[0], [self.topList[i], self.ttList[i], self.specimen1LiquidList[i]], objOffset=[self.topList[i].ttoffset, self.ttList[i].offset, SE3(0,0,0.13).A]) # Move the test tube directly above the rack
                self.rmrcur3(-0.085, 2, moveObj = [self.topList[i], self.ttList[i] , self.specimen1LiquidList[i]], objOffset=[self.topList[i].ttoffset, self.ttList[i].offset, SE3(0,0,0.13).A]) # Move the end effector directly down to place the test tube in the rack
                self.ur3EE.openGripper() # Open the gripper to release the test tube
            self.jtrajMoveur3([0,-pi/2,0,0,0,0]) # Move back to the starting position

    def testRRTPastObj(self):
        self.collisionDet.testRRTgp4() # Add the object that the robot needs to avoid to the environment
        self.useRRT = True # Boolean used to declare the use of RRT in the movement functions
    
    def closeCentrifugeLid(self): # Closes the centrifuge lip by rotating it along the x direction
        for i in range(30): 
            self.cenTop.T = self.cenTop.T @ trotx(2*pi/75)
            self.env.step(0.03)
        
        self.env.step(5)

    def openCentrifugeLid(self): # Opens the centrifuge lip by rotating it along the x direction
        for i in range(30):
            self.cenTop.T = self.cenTop.T @ trotx(-2*pi/75)
            self.env.step(0.03)

    def jtrajMovegp4(self, endq, moveObj:'list'=None, objOffset:'list'=None):
        """
        This is specifically for moving the gp4 using jtraj

        moveObj is a list of the objects that need to be moved with at the end effector
        It's originally at None so the robot can still move without anything in the end effector
        Also the function relies on the object that is moved being called meshObj in the class
            EXAMPLE IN TOPPERS.PY:
                self.meshObj = Mesh(filename = self.file)
                self.offset = SE3(-0.07, 0, 0.143).A

        """

        qMatrix = rtb.jtraj(self.gp4.q, endq, self.steps).q # Create a matrix of joint positions to move the gp4 to the target position
        for x in range(self.steps): # For each position in the matrix
            self.gp4.q = qMatrix[x] # Update the current joint position of the gp4
            self.gp4EE.T = self.gp4.fkine(self.gp4.q).A @ self.gp4eeToolOffset # Attach the tool to the end effector of the gp4
            if self.collisionDet.collisionCheck(self.gp4): # Check for collisions at each step
                print("COLLISION DETECTED") 
                while True: # Catch the collision and stop the program from running
                    pass
            if moveObj is not None: # If there is an object passed in the function
                for y in range(len(moveObj)): # For each object in the list
                    moveObj[y].meshObj.T = self.gp4.fkine(self.gp4.q).A @ objOffset[y] # Attach the object to the end effector of the gp4 using the offset provided
            self.env.step(0.01) # Update the environment at each step

    def jtrajMoverebel(self, endq, moveObj: 'specimenLiquid' =None, objOffset:'list'=None):
        qMatrix = rtb.jtraj(self.rebel.q, endq, self.steps).q # Create a matrix of joint postions for the ReBel to move to the target position
        for x in range(self.steps): # Repeat for each postion in the matrix
            self.rebel.q = qMatrix[x] # Update the current joint position of the ReBel
            self.rebelEE.T = self.rebel.fkine(self.rebel.q).A @ troty(pi/2) # Attach the tool to the end effector of the ReBel
            if self.collisionDet.collisionCheck(self.rebel): # Check for collsions at each step
                print("COLLISION DETECTED")
                while True:
                    pass
            if moveObj is not None:
                #move mesh1 and update other meshes accordingly
                moveObj.mesh1.T = self.rebel.fkine(self.rebel.q).A @ troty(pi/2)
                moveObj.updatePos()
            self.env.step(0.015)

    def jtrajMoveur3(self, endq, moveObj:'list'=None, objOffset:'list'=None):
        qMatrix = rtb.jtraj(self.ur3.q, endq, self.steps).q # Create a matrix of joint positions to move the gp4 to the target position
        for x in range(self.steps): # For each position in the matrix
            self.ur3.q = qMatrix[x] # Update the current joint position of the ur3
            self.ur3EE.gripFing1.base = self.ur3.fkine(self.ur3.q).A @ self.ur3eeToolOffset # Attach the first gripper finger to the end effector of the ur3 with its offset
            self.ur3EE.gripFing2.base = self.ur3.fkine(self.ur3.q).A @ self.ur3eeToolOffset # Attach the second gripper finger to the end effector of the ur3 with its offset
            if self.collisionDet.collisionCheck(self.ur3): # Check for collisions at each step
                print("COLLISION DETECTED") 
                while True: # Catch the collision and stop the program from running
                    pass
            if moveObj is not None: # If there is an object passed in the function
                for y in range(len(moveObj)): # For each object in the list
                    try: # Try moving the meshObj, if it can not be found presume it's a specimenLiquid
                        moveObj[y].meshObj.T = self.ur3.fkine(self.ur3.q).A @ objOffset[y] # Attach the object to the end effector of the ur3
                    except: 
                        print("couldnt access meshObj, presumed to be specimenLiquid")
                        moveObj[y].mesh1.T = self.ur3.fkine(self.ur3.q).A @ objOffset[y] # Attach the specimineLiquid to the end effector of the ur3
                        moveObj[y].updatePos() #Update the position of the specimenLiquid

            self.env.step(0.01) #Update the environment at each step

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
                                    try:
                                        moveObj[y].meshObj.T = self.ur3.fkine(self.ur3.q).A @ objOffset[y]
                                    except:
                                        print("couldnt access meshObj, presumed to be specimenLiquid")
                                        moveObj[y].mesh1.T = self.ur3.fkine(self.ur3.q).A @ objOffset[y]
                                        moveObj[y].updatePos()

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

    def rmrcur3(self, movedown, time, moveObj:'list'=None, objOffset:'list'=None):
        # 1.1) Set parameters for the simulation
        #                # Load robot model
        t = time                                     # Total time (s)
        delta_t = 0.02                             # Control frequency
        steps = int(t/delta_t)                     # No. of steps for simulation
        delta = 2*pi/steps                         # Small angle change
        epsilon = 0.1                              # Threshold value for manipulability/Damped Least Squares
        W = np.diag([1, 1, 1, 0.1, 1, 1])      # Weighting matrix for the velocity vector

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
        currentPos = self.ur3.fkine(self.ur3.q).A          # Trapezoidal trajectory scalar
        currentRot = self.ur3.fkine(self.ur3.q).R
        for i in range(steps):
            x[0,i] = currentPos[0,3] # Points in x
            x[1,i] = currentPos[1,3]                # Points in y
            x[2,i] = currentPos[2,3] + movedown * s[i]               # Points in z
            theta[0,i] = 0                          # Roll angle 
            theta[1,i] = pi                         # Pitch angle
            theta[2,i] = np.arctan2(currentRot[1][0], currentRot[0][0])                          # Yaw angle
        
        T = transl(x[:,0]) @ rpy2tr(theta[0,0], theta[1,0], theta[2,0])       # Create transformation of first point and angle     
        q0 = self.ur3.q                                                  # Initial guess for joint angles
        q_matrix[0,:] = self.ur3.ikine_LM(T, q0).q                                # Solve joint angles to achieve first waypoint
        qlim = np.transpose(self.ur3.qlim)
        
        # 1.4) Track the trajectory with RMRC
        for i in range(steps-1):
            T = self.ur3.fkine(q_matrix[i,:]).A                                   # Get forward transformation at current joint state
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
            J = self.ur3.jacob0(q_matrix[i,:])                                    # Get Jacobian at current joint state
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
            self.ur3.q = q
            if moveObj is not None:
                for y in range(len(moveObj)):
                    try:
                        moveObj[y].meshObj.T = self.ur3.fkine(self.ur3.q).A @ objOffset[y]
                    except:
                        print("couldnt access meshObj, presumed to be specimenLiquid")
                        moveObj[y].mesh1.T = self.ur3.fkine(self.ur3.q).A @ objOffset[y]
                        moveObj[y].updatePos()
                
            self.ur3EE.gripFing1.base = self.ur3.fkine(self.ur3.q).A @ self.ur3eeToolOffset
            self.ur3EE.gripFing2.base = self.ur3.fkine(self.ur3.q).A @ self.ur3eeToolOffset
            self.env.step(0.01)

    def rmrcgp4(self, movedown, time, moveObj:'list'=None, objOffset:'list'=None):
        # 1.1) Set parameters for the simulation
        #                # Load robot model
        t = time                                     # Total time (s)
        delta_t = 0.02                             # Control frequency
        steps = int(t/delta_t)                     # No. of steps for simulation
        delta = 2*pi/steps                         # Small angle change
        epsilon = 0.1                              # Threshold value for manipulability/Damped Least Squares
        W = np.diag([1, 1, 1, 0.1, 1, 0.1])      # Weighting matrix for the velocity vector

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
        currentPos = self.gp4.fkine(self.gp4.q).A          # Trapezoidal trajectory scalar
        for i in range(steps):
            x[0,i] = currentPos[0,3] - 0.14 # Points in x
            x[1,i] = currentPos[1,3]                # Points in y
            x[2,i] = currentPos[2,3] + movedown * s[i]               # Points in z
            theta[0,i] = 0                          # Roll angle 
            theta[1,i] = pi                          # Pitch angle
            theta[2,i] = 0                          # Yaw angle
        
        T = transl(x[:,0]) @ rpy2tr(theta[0,0], theta[1,0], theta[2,0])       # Create transformation of first point and angle     
        q0 = self.gp4.q                                                  # Initial guess for joint angles
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
            angle_error[:,i] = delta_theta

        for q in q_matrix:
            self.gp4.q = q
            if moveObj is not None:
                for y in range(len(moveObj)):
                    moveObj[y].meshObj.T = self.gp4.fkine(self.gp4.q).A @ objOffset[y]
            self.gp4EE.T = self.gp4.fkine(self.gp4.q).A @ self.gp4eeToolOffset
            pos = self.gp4.fkine(q).A[:3,3] 
            self.env.step(0.01)

    def rmrcrebel(self, movedown, time, moveObj:'specimenLiquid'=None, objOffset:'list'=None):
    # 1.1) Set parameters for the simulation
        t = time                                     # Total time (s)
        delta_t = 0.02                               # Control frequency
        steps = int(t/delta_t)                       # No. of steps for simulation
        delta = 2*pi/steps                           # Small angle change
        epsilon = 0.1                                # Threshold value for manipulability/Damped Least Squares
        W = np.diag([1, 1, 1, 0.1, 1, 0.1])          # Weighting matrix for the velocity vector

        # 1.2) Allocate array data
        m = np.zeros([steps,1])                      # Array for Measure of Manipulability
        q_matrix = np.zeros([steps,6])               # Array for joint angles
        qdot = np.zeros([steps,6])                   # Array for joint velocities
        theta = np.zeros([3,steps])                  # Array for roll-pitch-yaw angles
        x = np.zeros([3,steps])                      # Array for x-y-z trajectory
        position_error = np.zeros([3,steps])         # For plotting trajectory error
        angle_error = np.zeros([3,steps])            # For plotting trajectory error

        # 1.3) Set up trajectory, initial pose
        s = trapezoidal(0,1,steps).q
        currentPos = self.rebel.fkine(self.rebel.q).A    # Trapezoidal trajectory scalar
        for i in range(steps):
            x[0,i] = currentPos[0,3]             # Points in x
            x[1,i] = currentPos[1,3]                     # Points in y
            x[2,i] = currentPos[2,3] + movedown * s[i]   # Points in z
            theta[0,i] = 0                               # Roll angle 
            theta[1,i] = pi/2                              # Pitch angle
            theta[2,i] = 0                               # Yaw angle
        
        T = transl(x[:,0]) @ rpy2tr(theta[0,0], theta[1,0], theta[2,0])  # First pose
        q0 = self.rebel.q                                                # Initial guess
        q_matrix[0,:] = self.rebel.ikine_LM(T, q0).q                     # First waypoint IK
        qlim = np.transpose(self.rebel.qlim)

        # 1.4) Track the trajectory with RMRC
        for i in range(steps-1):
            T = self.rebel.fkine(q_matrix[i,:]).A                        # FK at current state
            delta_x = x[:,i+1] - T[:3,3]                                 # Position error
            Rd = rpy2r(theta[0,i+1], theta[1,i+1], theta[2,i+1])         # Desired rotation
            Ra = T[:3,:3]                                                # Actual rotation
            Rdot = (1/delta_t)*(Rd - Ra)                                 # Rotation error rate
            S = Rdot @ Ra.T                                              # Skew-symmetric
            linear_velocity = (1/delta_t)*delta_x
            angular_velocity = np.array([S[2,1], S[0,2], S[1,0]])        # From skew
            delta_theta = tr2rpy(Rd @ Ra.T, order='xyz')                 # Angle error
            xdot = W @ np.vstack((linear_velocity.reshape(3,1),
                                angular_velocity.reshape(3,1)))        # EE velocity
            J = self.rebel.jacob0(q_matrix[i,:])                         # Jacobian
            m[i] = np.sqrt(np.linalg.det(J @ J.T))
            if m[i] < epsilon:                                           # DLS damping
                m_lambda = (1 - m[i]/epsilon) * 0.05
            else:
                m_lambda = 0
            inv_j = np.linalg.inv(J.T @ J + m_lambda * np.eye(6)) @ J.T  # DLS inverse
            qdot[i,:] = (inv_j @ xdot).T                                 # Joint velocities

            for j in range(6):                                           # Joint limits
                if q_matrix[i,j] + delta_t*qdot[i,j] < qlim[j,0]:
                    qdot[i,j] = 0
                elif q_matrix[i,j] + delta_t*qdot[i,j] > qlim[j,1]:
                    qdot[i,j] = 0
            
            q_matrix[i+1,:] = q_matrix[i,:] + delta_t*qdot[i,:]          # Integrate

            position_error[:,i] = x[:,i+1] - T[:3,3]                     # For plotting
            angle_error[:,i] = delta_theta
        # 1.5) Plot the results
        for q in q_matrix:
            self.rebel.q = q
            if moveObj is not None:
                for y in range(len(moveObj)):
                    try: # Try moving the meshObj, if it can not be found presume it's a specimenLiquid
                        moveObj[y].meshObj.T = self.rebel.fkine(self.rebel.q).A @ objOffset[y] # Attach the object to the end effector of the ur3
                    except: 
                        print("couldnt access meshObj, presumed to be specimenLiquid")
                        moveObj[y].mesh1.T = self.rebel.fkine(self.rebel.q).A @ troty(pi/2) # Attach the specimineLiquid to the end effector of the ur3
                        moveObj[y].updatePos() #Update the position of the specimenLiquid
            self.rebelEE.T = self.rebel.fkine(self.rebel.q).A @troty(pi/2) #
            pos = self.rebel.fkine(q).A[:3,3] 
            self.env.step(0.01)
