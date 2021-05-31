#! /usr/bin/env python
import curses
import math
import rospy
import tf
import time
import json
import numpy as np
from random import uniform
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


class SimpleKeyTeleop():
    def __init__(self):
        self.set_coordinates = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.global_localization = rospy.ServiceProxy('global_localization', Empty)
        self.state_x = 4
        self.state_y = 8

        self.inferred_poses = []
        self.amcl_iterations = 120

        self.global_localization()

    def run(self):
        self.set_state(self.state_x, self.state_y, math.pi)
        self.amcl_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callbackPose)



    def callbackPose(self, data):
        print('Iter: {x}'.format(x = len(self.inferred_poses)))

        amcl_yaw = self.get_yaw(data.pose.pose.orientation)
        laser = rospy.wait_for_message('/scan_raw', LaserScan, timeout=5)
        true_pose = self.get_coordinates('pmb2', '')
        true_yaw = self.get_yaw(true_pose.pose.orientation)

        self.inferred_poses.append({
            "amcl": {
                "x": round(data.pose.pose.position.x, 3) + 3,
                "y": round(data.pose.pose.position.y, 3),
                "yaw": round(amcl_yaw, 3),
            },
            "true": {
                "x": round(true_pose.pose.position.x, 3),
                "y": round(true_pose.pose.position.y, 3),
                "yaw": round(true_yaw, 3),
            },
            "laser": np.around(laser.ranges, 3).tolist()
        })

        if (len(self.inferred_poses) >= self.amcl_iterations):
            self.amcl_subscriber.unregister();
            self.save_poses();
            print('finished')
        elif len(self.inferred_poses) % 3 == 0:
            self.move();

    def move(self):
        self.state_y = self.state_y - 0.2
        self.set_state(self.state_x, self.state_y, math.pi)

    def save_poses(self):
        with open('two_axis_move_stats_data.json', 'w') as outfile:
            json.dump(self.inferred_poses, outfile)

        print("- saved -")

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


def main(stdscr):
    rospy.init_node('listener', anonymous=True)
    app = SimpleKeyTeleop()
    app.run()
    # time.sleep(200)
    rospy.spin()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass