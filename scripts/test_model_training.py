#!/usr/bin/env python

# See README.md for more info.

import random
import env
import robot_stepper_client
import csv_interface
import numpy as np
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam
from keras.models import model_from_json
import csv


def single_input():
    states = range(30)
    states = np.array(states)
    print(states)

    q_table = csv_interface.readQTableFromCsv("Q_table.csv",(30,3))
    print(q_table)


    model = Sequential()
    model.add(Dense(1, 
                    input_dim=1, # 1D = nS
                    activation="relu"))
    model.add(Dense(300, activation="relu"))
    model.add(Dense(300, activation="relu"))
    model.add(Dense(3,activation='selu'))
    model.compile(loss="mse", optimizer=Adam(),metrics=['accuracy'])

    print(" ")

    print("Training with single input")

    model.fit(np.reshape(states,[30,1]),np.reshape(q_table,[30,3]),epochs=200 ,batch_size=4,verbose=2)

    print("Training done, resulting  table and policy :")
    policy = []
    q_table=[]
    for state in range(30):
        state = np.reshape(state,[1,1])
        q_values = model.predict(state)
        print("S:{} Q:{}".format(state,q_values[0]))
        policy.append(np.argmax(q_values[0]))
        q_table.append(q_values)

    print(policy)

def vector_input():
    print(" ")
    print(" ")
    print(" ")
    print("Training with vector input")


    states = range(30)
    states = np.array(states)
    print(states)

    q_table = csv_interface.readQTableFromCsv("Q_table.csv",(30,3))
    print(q_table)

    model2 = Sequential()
    model2.add(Dense(24, 
                    input_dim=30, # 1D = nS
                    activation="relu"))
    model2.add(Dense(300, activation="relu"))
    model2.add(Dense(300, activation="relu"))
    model2.add(Dense(3,activation='selu'))
    model2.compile(loss="mse", optimizer=Adam(),metrics=['accuracy'])

    state_matrix = np.zeros((30,30))
    for state in states:
        state_vector = np.zeros(30)
        state_vector[state] = 1
        state_matrix[state] = (state_vector)


    state_matrix = np.reshape(state_matrix,[30,30])

    model2.fit(state_matrix,np.reshape(q_table,[30,3]),epochs=35 ,batch_size=4,verbose=2)

    print("Training done, resulting  table and policy :")
    policy = []
    q_table=[]
    for state in range(30):
        state_vector = np.zeros(30)
        state_vector[state] = 1
        q_values = model2.predict(np.reshape(state_vector,[1,30]))
        print("S:{} Q:{}".format(state,q_values[0]))
        policy.append(np.argmax(q_values[0]))
        q_table.append(q_values)

    print(policy)


# single_input()
vector_input()
