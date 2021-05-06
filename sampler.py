#! /usr/bin/env python

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from sensor_msgs.msg import LaserScan
from random import uniform
import numpy as np
import rospy
import tf
import json
import time
import math


def has_tumbled(roll, pitch):
    return abs(roll - 0.0) > 0.001 or abs(pitch - 0.0) > 0.001


if __name__ == '__main__':
    # Setup
    amount_samples = 6000
    data = []
    rospy.init_node('sampler_node')
    get_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_coordinates = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    i = 1
    while not rospy.is_shutdown():
        print("iteration: " + str(i))


        # Set new robot's position
        next_yaw = math.pi  # uniform(-math.pi, math.pi) # from -pi to pi  (radians)
        next_x = 4.32  # uniform(-4.62, 4.62)
        next_y = uniform(-8.5, 8.5) # uniform(-12.63, 1.62)
        set_quaternion = tf.transformations.quaternion_from_euler(0., 0., next_yaw)

        next_state = ModelState()
        next_state.model_name = 'pmb2'
        next_state.pose.position.x = next_x
        next_state.pose.position.y = next_y
        next_state.pose.position.z = 0.001
        next_state.pose.orientation.x = set_quaternion[0]
        next_state.pose.orientation.y = set_quaternion[1]
        next_state.pose.orientation.z = set_quaternion[2]
        next_state.pose.orientation.w = set_quaternion[3]
        resp_set = set_coordinates(next_state)

        if (resp_set.success != True):
            raise Exception("Set position failed");



        # Get robot's position
        resp_get = get_coordinates('pmb2', '')

        if (resp_get.success != True):
            raise Exception("Get position failed");

        get_quaternion = (
            resp_get.pose.orientation.x,
            resp_get.pose.orientation.y,
            resp_get.pose.orientation.z,
            resp_get.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(get_quaternion)
        get_roll = euler[0]
        get_pitch = euler[1]
        get_yaw = euler[2]


        # Gives time for the robot to "land" on the environment
        # If it has hit a wall and tumbled, skip this iteration
        time.sleep(0.5)

        if has_tumbled(get_roll, get_pitch):
            print("- TUMBLED -")
            continue



        # Get laser sensor measurements
        try:
            resp_laser = rospy.wait_for_message('/scan_raw', LaserScan, timeout=5)
        except:
            print("Laser reading failed")



        # Adds data to array
        data.append({
            "position": { "x": round(resp_get.pose.position.x, 3), "y": round(resp_get.pose.position.y, 3) },
            "angle": round(get_yaw, 3),
            "laser_ranges": np.around(resp_laser.ranges, 3).tolist()
        })



        # Every 50 iterations, save data
        if (i % amount_samples == 0):
            with open('data.json', 'w') as outfile:
                json.dump(data, outfile)
            print("- saved -")


        i = i + 1;

        if i > amount_samples:
        	break