#!/usr/bin/env python

# See README.md for more info.

import rospy
from tejas.srv import EnvStep, EnvStepRequest, EnvStepResponse
import robot_stepper_client
from collections import Counter
import random


print("Example returns for each state*action combination and Terminating model")
for state in range(15):
    for action in range(3):
        s_, r, t, i = robot_stepper_client.step(state, action, "True")
        print("S: {} Ar: {} F:{} Ac: {} S_: {} R: {} T: {}".format(state, state//3, state%3, action, s_, r, t))

print("")


print("Example returns for each state*action combination and non-Terminating model")
for state in range(15):
    for action in range(3):
        s_, r, t, i = robot_stepper_client.step(state, action, "False")
        print("S: {} Ar: {} F:{} Ac: {} S_: {} R: {} T: {}".format(state, state//3, state%3, action, s_, r, t))

s_list = []

print("Random probability distribution test for 1000 runs:")
for x in range (1000):
    if x % 100 == 0:  print("Runs completed:",x)
    s_, r, t, i = robot_stepper_client.step(0, 1, "False")
    s_list.append(s_)

print(Counter(s_list))
