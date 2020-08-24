#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np


class GoToPose():
    def __init__(self):
        # rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.position = Point()
        self.move_cmd = Twist()
        self.r = rospy.Rate(1000)
        self.tf_listener = tf.TransformListener()
        self.odom_frame= 'odom'

        self.r.sleep()
        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(30.0))
            self.base_frame = 'base_footprint'
            print("base frame tf listener initialized")
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                print("trying to transform")
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(30.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                print("tfException")
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        print("all initialized")

    def goto(self, point, angle):
        (goal_x, goal_y, goal_z) = (0, 0, 0)
        goal_x = point['x']
        goal_y = point['y']
        goal_z = angle

        (self.position, rotation) = self.get_odom()
        last_rotation = 0
        linear_speed = 0.1
        angular_speed = 2

        goal_distance = sqrt(pow(goal_x - self.position.x, 2) + pow(goal_y - self.position.y, 2))
        distance = goal_distance
        old_distance = distance

        print("goal", goal_x, goal_y, goal_z)

        while distance > 0.05:
            (self.position, rotation) = self.get_odom()
            x_start = self.position.x
            y_start = self.position.y

            path_angle = atan2(goal_y - y_start, goal_x- x_start)
            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            z_dist = min(abs(rotation - path_angle), abs(2 * pi - abs(rotation - path_angle)))

            # detect and halt open loop behaviorr
            if distance > old_distance + 0.1:
                break

            # proportional control for lienear speed
            self.move_cmd.linear.x = min(linear_speed * distance ** 1.1 + 0.1, linear_speed)

            # proportional control for heading
            if path_angle >= 0:
                if rotation <= path_angle and rotation >= path_angle - pi:
                    self.move_cmd.angular.z = angular_speed * z_dist / (abs(self.move_cmd.linear.x) + 1)
                else:
                    self.move_cmd.angular.z = -1 * angular_speed * z_dist / (abs(self.move_cmd.linear.x) + 1)
            else:
                if rotation <= path_angle + pi and rotation > path_angle: 
                    self.move_cmd.angular.z = -1 * angular_speed * z_dist / (abs(self.move_cmd.linear.x) + 1)
                else:
                    self.move_cmd.angular.z = angular_speed * z_dist / (abs(self.move_cmd.linear.x) + 1)

            old_distance = distance
            
            self.cmd_vel.publish(self.move_cmd)
            
        (self.position, rotation) = self.get_odom()

        if abs(goal_z) > pi / 2:
            while abs(rotation - goal_z) > 0.01:
                (self.position, rotation) = self.get_odom()
                if goal_z >= 0:
                    if rotation <= goal_z and rotation >= goal_z - pi:
                        self.move_cmd.linear.x = 0.00
                        self.move_cmd.angular.z = min(1.5 * abs(rotation - goal_z), 1.5)
                    else:
                        self.move_cmd.linear.x = 0.00
                        self.move_cmd.angular.z = -min(1.5 * abs(rotation - goal_z), 1.5)
                else:
                    if rotation <= goal_z + pi and rotation > goal_z:
                        self.move_cmd.linear.x = 0.00
                        self.move_cmd.angular.z = -min(1.5 * abs(rotation - goal_z), 1.5)
                    else:
                        self.move_cmd.linear.x = 0.00
                        self.move_cmd.angular.z = min(1.5 * abs(rotation - goal_z), 1.5)
                self.cmd_vel.publish(self.move_cmd)

        rospy.loginfo("point reached")
        self.cmd_vel.publish(Twist())

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])


    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
