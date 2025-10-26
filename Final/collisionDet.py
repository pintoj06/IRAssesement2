import swift
from spatialmath import SE3
from spatialmath.base import *
from spatialgeometry import Cuboid, Sphere, Mesh
from ir_support import RectangularPrism, line_plane_intersection
from customUr3e import UR3e
from gp4 import newGP4
import numpy as np
from roboticstoolbox import DHRobot, jtraj
from typing import List
from itertools import combinations
from IGUS_testcode import ReBeL


class collisions:
    def __init__(self, ur3: 'UR3e', gp4: 'newGP4', rebel: 'ReBeL',  environment: 'swift'):
        self.env = environment # Enviroment used for the simulation
        self.ur3 = ur3 # UR3
        self.gp4 = gp4 # GP4
        self.rebel = rebel #ReBeL robot
        self.table = collisionObj([1.17, 1.86, 0.55], [1.36, 3.69, 0.3], self.env) # Collision object for the table
        self.centrifugeBase = collisionObj([0.24, 0.24, 0.14], [1.8, 4.2, 0.63], self.env) # Collision object for the centrifuge base
        self.ttHolder = collisionObj([0.26, 0.08, 0.11], [1.49, 3.56, 0.64], self.env) # Collision object for the test tube rack
        self.specimen2 = collisionObj([1.0, 3.9, 0.685], [0.3, 0.3, 0.25], self.env) # Collision object for specimen 2
        self.testRRt = collisionObj([0.08, 0.33, 0.45], [1.9, 3.33, 0.72], self.env) # Collision object for the RRT testing object
        self.testRRtrebel = collisionObj([0.08, 0.33, 0.45], [1.3, 3.1, 0.725], self.env) # Collision object for the RRT testing object for rebel
        self.avoidRebel = collisionObj([0.13, 0.13, 0.9], [1.2, 3.5, 1.275], self.env) # Collision object for the RRT testing object

      #  self.specimen1 = collisionObj([0.3, 0.3, 0.25 ], [1.0, 3.1, 0.65], self.env) # Collision object for specimen 1
     #   self.specimen2 = collisionObj([0.3, 0.3,  0.25], [1.0, 3.9, 0.685], self.env) # Collision object for specimen 2

        self.collisionObjList = [self.table, self.centrifugeBase, self.ttHolder,] # List of all collision objects in the environment
    
    def collisionCheck(self, robot):
        collisions = []
        q = robot.q
        robot.dhRobot.q = robot.q
        #Check if any of the lines created between the joints is intersecting with any of the faces of the mesh
        #This result function will provide at each step if there is a collision
        for y in self.collisionObjList:
            result = self.is_collision(robot.dhRobot, [q], y.faces, y.vertices, y.face_normals, collisions, env=self.env, return_once_found=True)
            if result:
                return True
        return False
    
    def is_intersection_point_inside_triangle(self, intersect_p, triangle_verts):
        u = triangle_verts[1, :] - triangle_verts[0, :]
        v = triangle_verts[2, :] - triangle_verts[0, :]

        uu = np.dot(u, u)
        uv = np.dot(u, v)
        vv = np.dot(v, v)

        w = intersect_p - triangle_verts[0, :]
        wu = np.dot(w, u)
        wv = np.dot(w, v)

        D = uv * uv - uu * vv

        # Get and test parametric coords (s and t)
        s = (uv * wv - vv * wu) / D
        if s < 0.0 or s > 1.0:  # intersect_p is outside Triangle
            return 0

        t = (uv * wu - uu * wv) / D
        if t < 0.0 or (s + t) > 1.0:  # intersect_p is outside Triangle
            return False

        return True  # intersect_p is in Triangle


    def is_collision(self, robot, q_matrix, faces, vertex, face_normals, collisions=[], env=None, return_once_found=True):
        """
        This is based upon the output of questions 2.5 and 2.6
        Given a robot model (robot), and trajectory (i.e. joint state vector) (q_matrix)
        and triangle obstacles in the environment (faces,vertex,face_normals)
        """
        result = False
        for i, q in enumerate(q_matrix):
            # Get the transform of every joint (i.e. start and end of every link)
            tr = self.get_link_poses(robot,q)
            
            # Go through each link and also each triangle face
            for i in range(6):
                for j, face in enumerate(faces):
                    vert_on_plane = vertex[face][0]
                    intersect_p, check = line_plane_intersection(face_normals[j], 
                                                                vert_on_plane, 
                                                                tr[i][:3,3], 
                                                                tr[i+1][:3,3])
                    # list of all triangle combination in a face
                    triangle_list  = np.array(list(combinations(face,3)),dtype= int)
                    if check == 1:
                        for triangle in triangle_list:
                            if self.is_intersection_point_inside_triangle(intersect_p, vertex[triangle]):
                                # Create a red sphere in Swift at the intersection point IF environment passed - if lagging, reduce radius
                                if env is not None:
                                    new_collision = Sphere(radius=0.05, color=[1.0, 0.0, 0.0, 1.0])
                                    new_collision.T = transl(intersect_p[0], intersect_p[1], intersect_p[2])
                                    env.add(new_collision)
                                    collisions.append(new_collision)
                                    
                                result = True
                                if return_once_found:
                                    return result
                                break
        return result
    
    def get_link_poses(self, robot:DHRobot,q=None)->List[np.ndarray]|np.ndarray:
        """
        :param q robot joint angles
        :param robot -  seriallink robot model
        :param transforms - list of transforms
        """
        if q is None:
            return robot.fkine_all().A
        return robot.fkine_all(q).A
    
        
    def fine_interpolation(self, q1, q2, max_step_radians = np.deg2rad(1))->np.ndarray:
        """
        Use results from Q2.6 to keep calling jtraj until all step sizes are
        smaller than a given max steps size
        """
        steps = 4
        while np.any(max_step_radians < np.abs(np.diff(jtraj(q1,q2,steps).q, axis= 0))):
            steps+=1
        return jtraj(q1,q2,steps).q

    def interpolate_waypoints_radians(self, waypoint_radians, max_step_radians = np.deg2rad(1))->np.ndarray:
        """
        Given a set of waypoints, finely intepolate them
        """
        q_matrix = []
        for i in range(np.size(waypoint_radians,0)-1):
            for q in self.fine_interpolation(waypoint_radians[i], waypoint_radians[i+1], max_step_radians):
                q_matrix.append(q)
        return q_matrix

    def testRRTgp4(self):
        # Create the collision object for the RRT testing
        #objRRT_file = 'CollisionObj.dae' # FOR JAYDEN TO FILL IN
        objRRT_file = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/CollisionObj.dae' # FOR HARRY
        objRRT = Mesh(filename = objRRT_file)
        self.env.add(objRRT)
        objRRT.T = SE3(1.87, 3.34, 0.65).A
    
    def testRRTrebel(self):
        # Create the collision object for the RRT testing
        #objRRT_file = 'CollisionObj.dae' # FOR JAYDEN 
        objRRT_file = '/Users/harrymentis/Documents/SensorsAndControls/Assignment2/CollisionObj.dae' # FOR HARRY
        objRRTrebel = Mesh(filename = objRRT_file)
        self.env.add(objRRTrebel)
        objRRTrebel.T = SE3(1.3, 3.1, 0.65).A


class collisionObj:
    def __init__(self, size: 'list', centre: 'SE3', environment: 'swift'):
        self.lwh = size # Python list containing the desired length (X), width (Y), height (Z) of the cuboid object
        self.centre = centre  # Python list containing XYZ centre of cuboid
        self.pose = transl(self.centre)       # Define the pose of the centre of the cuboid
        # Create prism/cuboid and set desired pose
        self.collisionObj = Cuboid(scale=self.lwh, color=[0.0, 1.0, 0.0, 0.00001])   # Set colour to green, but with some transparency to see through it (RGBA)
        self.collisionObj.T = self.pose # Set the pose of the cuboid
        self.env = environment # Enviorment used for the simulation
        self.env.add(self.collisionObj) # add the collision cuboid to the environment

        self.vertices, self.faces, self.face_normals = RectangularPrism(self.lwh[0], self.lwh[1], self.lwh[2], center=self.centre, color="#29fc34ca").get_data() # Get the vertecies, faces and face_normals of the collision object
