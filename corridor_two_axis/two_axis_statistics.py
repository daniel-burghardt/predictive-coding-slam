#! /usr/bin/env python
import curses
import math
import rospy
import tf
import time
import json
import numpy as np
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan


class SimpleKeyTeleop():
    def __init__(self):
        self.set_coordinates = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self._pub_cmd = rospy.Publisher('key_vel', Twist)

        self._hz = rospy.get_param('~hz', 10)
        self.rate = rospy.Rate(self._hz)   

        self._forward_rate = 0.8
        self._backward_rate = 0.5
        self._rotation_rate = 0.7
        self._angular = 0
        self._linear = 0

        self.inferred_poses = []

    movement_bindings = {
        curses.KEY_UP:    ( 1,  0),
        curses.KEY_DOWN:  (-1,  0),
        curses.KEY_LEFT:  ( 0,  1),
        curses.KEY_RIGHT: ( 0, -1),
    }

    def run(self):
        with open('inferred_poses.json', 'r') as outfile:
            self.inferred_poses = json.load(outfile)

        self.align_rotation();
        self.read_pose();
        self.save_poses();


    def align_rotation(self):
        try:
            true_pose = self.get_coordinates('pmb2', '')
            true_yaw = self.get_yaw(true_pose.pose.orientation)

            while math.pi - abs(true_yaw) > 0.1:                
                print(math.pi - abs(true_yaw))

                self.step(curses.KEY_RIGHT, 1)
                time.sleep(1)                
                true_pose = self.get_coordinates('pmb2', '')
                true_yaw = self.get_yaw(true_pose.pose.orientation)

        except Exception as e:
            print(e)
            
    def save_poses(self):
        with open('inferred_poses.json', 'w') as outfile:
            json.dump(self.inferred_poses, outfile)
        print("- saved -")

    def read_pose(self):
        try:
            laser = rospy.wait_for_message('/scan_raw', LaserScan, timeout=5)
            amcl_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=5)
            amcl_yaw = self.get_yaw(amcl_pose.pose.pose.orientation)
            true_pose = self.get_coordinates('pmb2', '')
            true_yaw = self.get_yaw(true_pose.pose.orientation)

            self.inferred_poses.append({
                "amcl": {
                    "x": round(amcl_pose.pose.pose.position.x, 3) + 3,
                    "y": round(amcl_pose.pose.pose.position.y, 3),
                    "yaw": round(amcl_yaw, 3),
                },
                "true": {
                    "x": round(true_pose.pose.position.x, 3),
                    "y": round(true_pose.pose.position.y, 3),
                    "yaw": round(true_yaw, 3),
                },
                "laser": np.around(laser.ranges, 3).tolist()
            })

        except Exception as e:
            print(e)

    def get_yaw(self, orientation):
        get_quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(get_quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        return yaw

    def step(self, mov, steps):
        for i in range(steps):
            self._set_velocity(mov)
            self._publish()
            self.rate.sleep()

    def set_state(self, x, y, yaw):
        set_quaternion = tf.transformations.quaternion_from_euler(0., 0., yaw)
        next_state = ModelState()
        next_state.model_name = 'pmb2'
        next_state.pose.position.x = x
        next_state.pose.position.y = y
        next_state.pose.position.z = 0.001
        next_state.pose.orientation.x = set_quaternion[0]
        next_state.pose.orientation.y = set_quaternion[1]
        next_state.pose.orientation.z = set_quaternion[2]
        next_state.pose.orientation.w = set_quaternion[3]
        resp_set = self.set_coordinates(next_state)


    def _get_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _set_velocity(self, key):
        linear, angular = self.movement_bindings[key]
        if linear > 0:
            linear = linear * self._forward_rate
        else:
            linear = linear * self._backward_rate
        angular = angular * self._rotation_rate
        self._angular = angular
        self._linear = linear

    def _publish(self):
        twist = self._get_twist(self._linear, self._angular)
        self._pub_cmd.publish(twist)


def main(stdscr):
    rospy.init_node('key_teleop')
    app = SimpleKeyTeleop()
    app.run()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass