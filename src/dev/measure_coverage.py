#!/usr/bin/env python

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, Point, Quaternion
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose

class MeasureAreaCovered:
    link_name = 'turtlebot3_burger::base_footprint'
    tool_radius = 0.2
    current_pose = Pose()
    last_pose = Pose()
    area_covered = 0
    area_duplicated = 0
    dist_traveled = 0
    first = True
    block_constraints = []
    average_speed = 0
    time = 0

    def __init__(self):
        pass
        self.link_name_rectified = self.link_name.replace("::", "_")

        if not self.link_name:
            raise ValueError("'link_name' is an empty string")

        self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)

    def callback(self, data):
        try:
            ind = data.name.index(self.link_name)
        except ValueError:
            return 1
        self.current_pose = data.pose[ind]
        if self.first == True:
            self.first = False
            self.last_pose = self.current_pose
            return 1

        # self.area_covered += self.polygon_area(self.calculate_corners(self.current_pose, self.last_pose, 0.85))
        self.dist_traveled += self.euclidean_dist(self.current_pose.position.x, self.current_pose.position.y, self.last_pose.position.x, self.last_pose.position.y)
        self.time = rospy.get_time() #get time as float secs
        self.average_speed = self.dist_traveled / self.time
        
        self.last_pose = self.current_pose

    def euclidean_dist(self, x1, y1, x2, y2):
        return math.sqrt(abs(x1 - x2) ** 2 + abs(y1-y2) ** 2)

    def disp(self):
        print("dist", self.dist_traveled, "time", self.time, "speed", self.average_speed)

    # def calculate_corners(self, pose, last_pose, radius):
    #     corners = []
    #     current_normal_angle = self.convert_quat_to_heading(pose.orientation) + math.pi / 2
    #     last_normal_angle = self.convert_quat_to_heading(last_pose.orientation) + math.pi / 2
    #     lines = []

    #     corners.append((pose.position.x + radius * math.cos(current_normal_angle), pose.position.y + radius * math.sin(current_normal_angle)))
    #     corners.append((pose.position.x + radius * math.cos(current_normal_angle + math.pi), pose.position.y + radius * math.sin(current_normal_angle + math.pi)))
    #     self.lines.append(self.convert_to_linear_inequality(self.convert_to_linear_function(corners[-1], corners[-2]))
    #     corners.append((last_pose.position.x + radius * math.cos(last_normal_angle + math.pi), last_pose.position.y + radius * math.sin(last_normal_angle + math.pi)))
    #     self.lines.append(self.convert_to_linear_inequality(self.convert_to_linear_function(corners[-1], corners[-2]))
    #     corners.append((last_pose.position.x + radius * math.cos(last_normal_angle), last_pose.position.y + radius * math.sin(last_normal_angle)))
    #     self.lines.append(self.convert_to_linear_inequality(self.convert_to_linear_function(corners[-1], corners[-2]))
    #     self.lines.append(self.convert_to_linear_inequality(self.convert_to_linear_function(corners[-4], corners[-1]))
    #     self.block_constraints.append(lines)


    #     return corners

    # # shoelace method
    # def polygon_area(self, corners):
    #     n = len(corners) # of corners
    #     area = 0.0
    #     for i in range(n):
    #         j = (i + 1) % n
    #         area += corners[i][0] * corners[j][1]
    #         area -= corners[j][0] * corners[i][1]
    #     area = abs(area) / 2.0
    #     return area

    # # linear function format: mx + b (m, b)
    # def convert_to_linear_function(self, point1, point2):
    #     m = (point2[1] - point1[1]) / (point2[0] - point1[0])
    #     b = point1[1] - m * point1[0]
    #     return (m, b)

    # def solve_linear_equation(self, linear_function, x):
    #     return linear_function[0] * x + linear_function[1]

    # # linear inequality format: (m, b, constrain for greater than or not, inclusive or not )
    # def convert_to_linear_inequality(self,function, greater_than, inclusive):
    #     return function + (greater_than,) + (inclusive,)

    # def is_in_bound(self, inequalities, point):
    #     for line in inequalities:
    #         if line[2] == True:
    #             if line[3] == True:
    #                 if point[1] < solve_linear_equation(line, point[0]):
    #                     return False
    #             else:
    #                 if point[1] <= solve_linear_equation(line, point[0]):
    #                     return False
    #         else:
    #             if line[3] == True:
    #                 if point[1] > solve_linear_equation(line, point[0]):
    #                     return False
    #             else:
    #                 if point[1] >= solve_linear_equation(line, point[0]):
    #                     return False
    #     return True

    # def convert_quat_to_heading(self, quat):
    #     return euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]