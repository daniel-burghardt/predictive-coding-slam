#! /usr/bin/env python
import curses
import math
import rospy
import tf
import time
import json
import numpy as np
from random import uniform
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


class SimpleKeyTeleop():
    def __init__(self):
        self.set_coordinates = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.global_localization = rospy.ServiceProxy('global_localization', Empty)

        self.samples = 100
        self.inferred_poses = []
        self.amcl_poses = []
        self.amcl_iterations = 50

    def run(self):
        next_x = uniform(2.3, 4.5)
        next_y = uniform(-8.8, 8.8)
        self.set_state(next_x, next_y, math.pi)
        self.global_localization()
        self.first_event = True;
        self.amcl_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callbackPose)

    def callbackPose(self, data):
        # event from previous iteration
        if self.first_event: #len(self.amcl_poses) == 0 and (abs(data.pose.pose.position.x - 0) > 0.25 or abs(data.pose.pose.position.y - 0) > 0.25):
            self.first_event = False;
            return

        # if len(self.amcl_poses) == 0:
        #     print('X: {x} Y: {y}'.format(x = data.pose.pose.position.x, y = data.pose.pose.position.y))

        print('Pos: {p} Iter: {x}'.format(p = len(self.inferred_poses), x = len(self.amcl_poses) + 1))

        amcl_yaw = self.get_yaw(data.pose.pose.orientation)
        self.amcl_poses.append({
            'x': data.pose.pose.position.x,
            'y': data.pose.pose.position.y,
            'yaw': amcl_yaw,
        })

        if (len(self.amcl_poses) == self.amcl_iterations):
            self.amcl_subscriber.unregister();
            self.finalize_batch();

    def finalize_batch(self):
        try:
            laser = rospy.wait_for_message('/scan_raw', LaserScan, timeout=5)
            true_pose = self.get_coordinates('pmb2', '')
            true_yaw = self.get_yaw(true_pose.pose.orientation)

            self.inferred_poses.append(map(lambda amcl: {
                "amcl": {
                    "x": round(amcl['x'], 3) + 3,
                    "y": round(amcl['y'], 3),
                    "yaw": round(amcl['yaw'], 3),
                },
                "true": {
                    "x": round(true_pose.pose.position.x, 3),
                    "y": round(true_pose.pose.position.y, 3),
                    "yaw": round(true_yaw, 3),
                },
                "laser": np.around(laser.ranges, 3).tolist()
            }, self.amcl_poses))

            self.amcl_poses = []
            if len(self.inferred_poses) == self.samples:
                self.save_poses();
                print('finished')
            else:
                self.run();

        except Exception as e:
            print(e)

    def save_poses(self):
        with open('two_axis_static_stats_data.json', 'w') as outfile:
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