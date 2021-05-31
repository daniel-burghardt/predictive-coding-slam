#! /usr/bin/env python
import sys
import curses
import math
import rospy
import time
import json
import numpy as np
import torch
import torch.nn as nn
from torch.autograd import Variable
from random import uniform
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan


class ConvTransposeNet(nn.Module):
    def __init__(self):
        super(ConvTransposeNet, self).__init__()
        
        self.layers = nn.Sequential(
            nn.Linear(2, 99),
            nn.ReLU(),
            nn.Linear(99, 198),
            nn.ReLU(),
            
            nn.ConvTranspose1d(in_channels=1, out_channels = 1, kernel_size=4, stride=1, padding=0, bias=False),
            nn.ReLU(),
            nn.Conv1d(in_channels=1, out_channels = 1, kernel_size=1, stride=1, padding=0, bias=False),
            nn.ReLU(),
            nn.ConvTranspose1d(in_channels=1, out_channels = 1, kernel_size=22, stride=3, padding=0, bias=False),
            nn.ReLU(),
            nn.Conv1d(in_channels=1, out_channels = 1, kernel_size=1, stride=1, padding=0, bias=False),
            nn.ReLU(),
        )
        
        
    def forward(self, x):     
        out = self.layers(x)
        
        return out


class NavigationFEP():
    def __init__(self):
        # normalization constants from training
        self.ranges_max = 18.409
        self.ranges_min = 0.271
        self.positions_max = [4.5, 8.8]
        self.positions_min = [2.3, -8.799]

        self.ranges_trim = 22
        self.iterations = 100
        self.alpha = 0.005
        self.dt = 0.005
        self.belief = torch.tensor([[ self.normalize_pos([4, 8]) ]])
        self.goal = torch.tensor([[ self.normalize_pos([2.5, -7]) ]])

        self.model = ConvTransposeNet()
        self.model.load_state_dict(torch.load("model_corridor"))
        self.model.eval()

        self.set_coordinates = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)


    def run_navigation(self):
        start_state = self.unnormalize_pos(self.belief.detach().numpy()[0][0])
        self.set_state(start_state[0], start_state[1])
        time.sleep(1)
        
        for i in range(self.iterations):
            laser_ranges = torch.tensor([[ self.normalize_ranges(self.get_current_laser()) ]])

            belief_dot = self.dF_dg(self.belief, laser_ranges)
            attractor_dot = self.dF_dg_attractor(self.belief, self.goal)
            
            # Action to minimize diff current position and desired position (belief + attractor)
            move_dot = belief_dot.detach().numpy()[0][0] * -1
            # Moving belief in direction of the goal
            belief_dot = torch.add(belief_dot, attractor_dot)
            self.belief = torch.add(self.belief, belief_dot, alpha=self.alpha) # alpha = learning rate

            current_state = self.get_current_state()
            move_to = np.array([current_state.x, current_state.y]) + np.array(self.unnormalize_pos(move_dot)) * self.dt
            self.set_state(move_to[0], move_to[1])

            print('New State: {x}'.format(x = move_to))
            time.sleep(0.5)


    def run_localization(self):
        true_location = self.unnormalize_pos(self.goal.detach().numpy()[0][0])
        self.set_state(true_location[0], true_location[1])
        true_ranges = torch.tensor([[ self.normalize_ranges(self.get_current_laser()) ]])

        start_state = self.unnormalize_pos(self.belief.detach().numpy()[0][0])
        self.set_state(start_state[0], start_state[1])

        time.sleep(1)
        
        for i in range(self.iterations):
            laser_ranges = torch.tensor([[ self.normalize_ranges(self.get_current_laser()) ]])

            belief_dot = self.dF_dg(self.belief, true_ranges)
            self.belief = torch.add(self.belief, belief_dot, alpha=self.alpha) # alpha = learning rate
            
            updated_belief = self.unnormalize_pos(self.belief.detach().numpy()[0][0])
            self.set_state(updated_belief[0], updated_belief[1])

            print('Updated belief: {x}'.format(x = updated_belief))
            time.sleep(0.5)

    def get_current_laser(self):
        try:
            ranges = rospy.wait_for_message('/scan_raw', LaserScan, timeout=5)
            ranges = np.array(ranges.ranges)
            ranges[ranges == np.inf] = 15
            ranges = ranges[self.ranges_trim : len(ranges) - self.ranges_trim]
            return ranges

        except Exception as e:
            print(e)


    def get_current_state(self):
        try:
            state = self.get_coordinates('pmb2', '')
            return state.pose.position

        except Exception as e:
            print(e)


    def set_state(self, x, y):
        next_state = ModelState()
        next_state.model_name = 'pmb2'
        next_state.pose.position.x = x
        next_state.pose.position.y = y
        next_state.pose.position.z = 0.001
        next_state.pose.orientation.x = 0
        next_state.pose.orientation.y = 0
        next_state.pose.orientation.z = 1
        next_state.pose.orientation.w = 0
        resp_set = self.set_coordinates(next_state)

    def dF_dg(self, belief_pos, real_ranges):
        input = Variable(belief_pos, requires_grad=True)

        pred_ranges = self.model(input)
        pred_error = real_ranges - pred_ranges   # torch.Size([1, 1, 622])
        
        sigma = torch.tensor([1] * 622)   # torch.Size([622])
        dF_dg = (1/sigma) * pred_error  # torch.Size([1, 1, 622])

        input.grad=torch.zeros(input.size())
        pred_ranges.backward(torch.ones(pred_ranges.shape) * dF_dg, retain_graph=True)

        return input.grad
    

    def dF_dg_attractor(self, belief_pos, goal_pos):
        input = Variable(belief_pos, requires_grad=True)

        pred_ranges = self.model(input)
        goal_ranges = self.model(goal_pos)

        goal_error = (goal_ranges - pred_ranges) * 1
        
        sigma = torch.tensor([1] * 622)   # torch.Size([622]) 
        dF_dg = (1/sigma) * goal_error  # torch.Size([1, 1, 622])

        input.grad=torch.zeros(input.size())
        pred_ranges.backward(torch.ones(pred_ranges.shape) * dF_dg, retain_graph=True)

        return input.grad


    def normalize_ranges(self, ranges):
        return (ranges - self.ranges_min) / (self.ranges_max - self.ranges_min)
        
    def normalize_pos(self, position):
        x = (position[0] - self.positions_min[0]) / (self.positions_max[0] - self.positions_min[0])
        y = (position[1] - self.positions_min[1]) / (self.positions_max[1] - self.positions_min[1])
        return [x, y]

    def unormalize_ranges(self, ranges):
        return ranges * (self.ranges_max - self.ranges_min) + self.ranges_min;

    def unnormalize_pos(self, position):
        x = position[0] * (self.positions_max[0] - self.positions_min[0]) + self.positions_min[0]
        y = position[1] * (self.positions_max[1] - self.positions_min[1]) + self.positions_min[1]
        return [x, y]


def main(stdscr):
    rospy.init_node('nav_fep', anonymous=True)
    app = NavigationFEP()
    
    if sys.argv[1] == 'loc':
        app.run_localization()
    elif sys.argv[1] == 'nav':
        app.run_navigation()
    else:
        raise Exception("Algorithm required as parameter ['loc', 'nav']")

    rospy.spin()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass