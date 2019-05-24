#!/usr/bin/env python

# See README.md for more info.
# Training done here using vector as input.
# Overall structure of this code adopted from an online blog, source is unfortunately lost.

import sys
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
import time
import numpy as np

GAMMA = 0.1 # Discount factor
LEARNING_RATE = 0.001

# Memory replay
MEMORY_SIZE = 1000000
BATCH_SIZE = 20

# Exploration vs exploitation
EXPLORATION_MAX = 1.0
EXPLORATION_MIN = 0.01
EXPLORATION_DECAY = 0.1

# Incremental change tracker for convergence check
DELTA = 0

class DQNSolver:

    def __init__(self, observation_space, action_space):
        self.exploration_rate = EXPLORATION_MAX

        self.action_space = action_space
        self.memory = deque(maxlen=MEMORY_SIZE)
        self.observation_space = observation_space
    
    def build_fresh(self):
        self.model = Sequential()
        self.model.add(Dense(24, 
                        input_dim=self.observation_space, # 1D = nS
                        activation="relu"))
        self.model.add(Dense(24, activation="relu"))
        self.model.add(Dense(24, activation="relu"))
        self.model.add(Dense(self.action_space, activation="linear"))
        self.model.compile(loss="mse", optimizer=Adam(lr=LEARNING_RATE))
        self.train_mode = True

#Save to memory
    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
    
    def clearMemory(self):
        self.memory = deque(maxlen=MEMORY_SIZE)

    def act(self, state):
        if np.random.rand() < self.exploration_rate and self.train_mode:
            return random.randrange(self.action_space)
        q_values = self.model.predict(state)
        return np.argmax(q_values[0])

# Performs the actual Q value updates by reading memory.
    def experience_replay(self, episode):
        if len(self.memory) < BATCH_SIZE:
            return
        batch = random.sample(list(self.memory), BATCH_SIZE)
        for state, action, reward, state_next, terminal in batch:
            q_update = reward
            if not terminal:
                q_update = (reward + GAMMA * np.amax(self.model.predict(state_next)[0]))
            q_values = self.model.predict(state)
            q_variation = q_values[0][action] - q_update
            global DELTA
            DELTA = min(DELTA, abs(q_variation))
            q_values[0][action] = q_update
            self.model.fit(state, q_values, verbose=0)
        self.exploration_rate = EXPLORATION_MIN + (EXPLORATION_MAX - EXPLORATION_MIN)*np.exp(-EXPLORATION_DECAY*episode) 
        return q_variation**2
        # return abs(q_variation)

    def save_model(self, json_file, weights_file):
        model_json = self.model.to_json()
        json_file_name = json_file
        with open(json_file,"w") as json_file:
            json_file.write(model_json)
        self.model.save_weights(weights_file)
        print("Saved model to {}".format(json_file_name))
        print("Saved weights to {}".format(weights_file))

    def load_model(self):
        json_file = open('feeder_dqn.json', 'r')
        loaded_model_json = json_file.read()
        json_file.close()
        loaded_model = model_from_json(loaded_model_json)
        # load weights into new model
        loaded_model.load_weights("feeder_dqn.h5")
        print("Loaded model from disk")
        self.model = loaded_model
        self.model.compile(loss="mse", optimizer=Adam(lr=LEARNING_RATE))
        self.train_mode = False
        return loaded_model

# Once model is trained, use this to extract Q table.
    def getQTable(self, verbose=False):
        policy = []
        q_table=[]
        for state in range(env.nS):
            state_vector = np.zeros(self.observation_space)
            state_vector[state] = 1
            state_vector = np.reshape(state_vector,[1,self.observation_space])
            q_values = self.model.predict(state_vector)
            if verbose:
                print("S:{} Q:{}".format(state,q_values[0]))
            policy.append(np.argmax(q_values[0]))
            q_table.append(q_values)
               
        if verbose:
            print("Optimum policy: ")
            print(policy)
        return q_table, policy

    
    def saveResults(self, policy_file, json_file, weights_file, performance_file):
        qtable, policy = self.getQTable(verbose=False)
        self.save_model(json_file,weights_file)
        print("Saving DQN policy to : {}".format(policy_file))
        csv_interface.writePolicyArrayToCsv(policy_file, policy)
        print("Saving DQN performance to: {}".format(performance_file))

# Main function
def feeder():
    # Load params from launch file, or defaults
    if len(sys.argv) == 10:
        dqn_policy_file = sys.argv[1]
        dqn_json_file = sys.argv[2]
        dqn_weights_file = sys.argv[3]
        STEPS_PER_EPISODE = int(sys.argv[5])
        total_episodes = int(sys.argv[4])
        loss_threshold = float(sys.argv[6])
        dqn_performance_file = sys.argv[7]
    else:
        dqn_policy_file = "DQN_policy.csv"
        dqn_json_file = "DQN_model.json"
        dqn_weights_file = "DQN_model.h5"
        STEPS_PER_EPISODE = 50
        total_episodes = 10 * STEPS_PER_EPISODE
        loss_threshold= 0.005
        dqn_performance_file = "DQN_performance.csv"
    
    print("")
    print("Save space:")
    print(" Policy - {}".format(dqn_policy_file))
    print(" Model.json - {}".format(dqn_json_file))
    print(" Model.weights - {}".format(dqn_weights_file))
    print("Running for {} episodes until loss of {}".format(total_episodes, loss_threshold))
    time.sleep(1)

    # Define model
    observation_space = env.nS
    action_space = env.nA
    state_list = range(0, observation_space)
    action_list = range(0, action_space)

    # Setup NN
    dqn_solver = DQNSolver(observation_space, action_space)
    dqn_solver.build_fresh()
    states_visited = np.zeros(dqn_solver.observation_space)
    episode = 0
    
    csv_interface.writeQTableToCsv(dqn_performance_file, np.array([("episode","reward","avg_loss","epsilon")]) )
    # Start training
    for episode in range(total_episodes):
        # episode += 1
        state = random.choice(state_list)
        
        run = 0
        total_reward = 0
        avg_loss = 0.0
        for run in range(STEPS_PER_EPISODE):

            # Input shaping is done here. Instead of having a single input neuron with 30 values (for 30 states), a vector of size 30 is used (ie 30 neurons).
            # ONE of the indices of the vecotor has value 1, indicating the actual input state corresponding to the index, other indices are 0
            # Example: Vector = [0, 0, 1, 0, 0, ... 0] = State 2 & Vector = [0, 0, 0, 1, 0, ... 0] = State 3 and so on.
            # This is done because the dataset for training the NN is small and uncorrelated. Without input shaping, NN never trains.
            state_vector = np.zeros(dqn_solver.observation_space)
            state_vector[state] = 1
            state_vector = np.reshape(state_vector,[1,dqn_solver.observation_space])

            states_visited[state] += 1
            action = dqn_solver.act(state_vector)
            state_next, reward, terminal, info = robot_stepper_client.step(state, action, "False")
            state_next = np.reshape(state_next,[1,])
            
            if dqn_solver.train_mode:
                state_next_vector = np.zeros(dqn_solver.observation_space)
                state_next_vector[state_next] = 1
                state_next_vector = np.reshape(state_next_vector,[1,dqn_solver.observation_space])
                dqn_solver.remember(state_vector, action, reward, state_next_vector, terminal)
            state = state_next
            total_reward += reward
            if dqn_solver.train_mode:
                loss = dqn_solver.experience_replay(episode)
                if loss>0:
                    avg_loss = (avg_loss+loss)/2

        csv_interface.appendPerformaceData(dqn_performance_file, np.array([(episode,total_reward,avg_loss,dqn_solver.exploration_rate)]) )   
        print("")

        table, policy = dqn_solver.getQTable(verbose=False)
        # print("Ep: {} eps:{} avg_loss {} total_reward: {}".format((episode+1)*STEPS_PER_EPISODE, dqn_solver.exploration_rate, avg_loss, total_reward))
        print("Ep: {} eps:{} avg_loss {} total_reward: {}".format(episode, dqn_solver.exploration_rate, avg_loss, total_reward))
        print("Optimum policy: {}".format(policy))

        if avg_loss <= loss_threshold:
            print("Loss threshold satisfied, breaking at ep : {}!!".format(episode))
            break
        # if total_reward == 300:
        #     print("Max reward obtained, breaking!! @ ep : {}".format(episode))
        #     break

        # print("States visited :", states_visited)

    print("")
    print("Q Table: ")
    dqn_solver.getQTable(verbose=True)

    print("")
    if dqn_solver.train_mode:
        dqn_solver.saveResults(dqn_policy_file,dqn_json_file,dqn_weights_file,dqn_performance_file)
        print("")

if __name__=="__main__":
    feeder()

        


