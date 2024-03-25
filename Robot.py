import itertools
import numpy as np
from matplotlib import pyplot as plt
from numpy.core.fromnumeric import size
from shapely.geometry import Point, LineString
import math

class Robot(object):
    
    def __init__(self):

        # define robot properties
        self.links = np.array([80.0,70.0,40.0,40.0])
        self.dim = len(self.links)

        # robot field of view (FOV) for inspecting points, from [-np.pi/6, np.pi/6]
        self.ee_fov = np.pi/3

        # visibility distance for the robot's end-effector. Farther than that, the robot won't see any points.
        self.vis_dist = 60.0

    def compute_distance(self, prev_config, next_config):
        '''
        Compute the euclidean distance betweeen two given configurations.
        @param prev_config Previous configuration.
        @param next_config Next configuration.
        '''
        # TODO: Task 2.2
        coords_prev = self.compute_forward_kinematics(prev_config)
        coords_next = self.compute_forward_kinematics(next_config)
        distance = np.linalg.norm(coords_prev - coords_next)#TODO - check if it works good

        return distance

    def compute_forward_kinematics(self, given_config):
        '''
        Compute the 2D position (x,y) of each one of the links 
        (including end-effector) and return.
        @param given_config Given configuration.
        '''
        # TODO: Task 2.2

        link_base = [0, 0]
        pos_vec = []
        prev_angle = 0
        for l in range(len(given_config)):
            new_angle = self.compute_link_angle(prev_angle, given_config[l])
            x = self.links[l] * np.cos(new_angle) + link_base[0]
            y = self.links[l] * np.sin(new_angle) + link_base[1]
            link_base = [x, y]
            pos_vec.append(link_base)
            prev_angle = new_angle
        return np.array(pos_vec)

        # link_length = self.links
        # curr_angle = 0.0 #angles are in radians and ranged in [−π, π]
        # curr_x = 0.0
        # curr_y = 0.0
        #
        # coords = {}
        # for i in range(self.dim):
        #     angle = self.compute_link_angle(curr_angle, given_config[i])
        #     x = curr_x + (math.cos(angle) * link_length[i])
        #     y = curr_y + (math.sin(angle) * link_length[i])
        #     coords[i] = (x, y)
        #     curr_x = x
        #     curr_y = y
        #     curr_angle = given_config[i]
        #
        # #should i use compute_ee_angle here?
        #
        # coords_array = [coords[key] for key in coords.keys()]
        # return np.array(coords_array)
  
    def compute_ee_angle(self, given_config):
        '''
        Compute the 1D orientation of the end-effector w.r.t. world origin (or first joint)
        @param given_config Given configuration.
        '''
        ee_angle = given_config[0]
        for i in range(1,len(given_config)):
            ee_angle = self.compute_link_angle(ee_angle, given_config[i])

        return ee_angle

    def compute_link_angle(self, link_angle, given_angle):
        '''
        Compute the 1D orientation of a link given the previous link and the current joint angle.
        @param link_angle previous link angle.
        @param given_angle Given joint angle.
        '''
        if link_angle + given_angle > np.pi:
            return link_angle + given_angle - 2*np.pi
        elif link_angle + given_angle < -np.pi:
            return link_angle + given_angle + 2*np.pi
        else:
            return link_angle + given_angle
        
    def validate_robot(self, robot_positions):
        '''
        Verify that the given set of links positions does not contain self collisions.
        @param robot_positions Given links positions.
        '''
        # TODO: Task 2.2

        arm = LineString(robot_positions)
        return arm.is_simple

        #
        # links_lines = {}
        # for i in range (0, len(robot_positions) - 1):
        #     link_line = LineString([Point(robot_positions[i]), Point(robot_positions[i + 1])])
        #     links_lines[i] = link_line
        #
        # #convert to dictionary?
        # for i in range(len(links_lines)):
        #     for j in range(i + 1, len(links_lines)):
        #         intersectionPoints = list(links_lines[i].intersection(links_lines[j]).coords)
        #         if len(intersectionPoints) > 1:
        #             return False
        #
        #     """collisions = [links_lines[i].crosses(x) for x in links_lines]
        #     if any(collisions):
        #         return False"""
        #
        # return True
    