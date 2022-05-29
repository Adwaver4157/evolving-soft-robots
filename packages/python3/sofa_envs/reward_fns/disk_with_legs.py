"""Define reward functions here.

The interface for a reward function is:
Input:
    previous_observation, action, observation
Output:
    float
"""
import numpy as np 


def forward_distance(prev_ob, action, ob):
    return (ob['center'][0] - prev_ob['center'][0]) / 10.  # cm

def goal_distance(prev_ob, goal, ob):
    return np.linalg.norm(ob['center'][:2] - goal[:2]) / 10.  # cm


def zero_reward(prev_ob, action, ob):
    return 0.0
