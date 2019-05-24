#!/usr/bin/env python
""" 
SOURCE: https://www.kaggle.com/angps95/intro-to-reinforcement-learning-with-openai-gym

REQUIRES:
env.py in the same directory

OUTPUTS:
The optimal policies as calculated by the PI and VI algorithms on screen and as .csv files
If run through launch file, .csv files go to tejas/resources directory, else .csv files go to same directory as script
"""

import sys
import numpy as np 
import env
import csv
import csv_interface

def policy_eval(policy, discount_factor=1.0, theta=0.00001):
    """
    Evaluate a policy given an environment and a full description of the environment's dynamics.
    
    Args:
        policy: [S, A] shaped matrix representing the policy.
        env: OpenAI env. env.P represents the transition probabilities of the environment.
            env.P[s][a] is a list of transition tuples (prob, next_state, reward, done).
            env.nS is a number of states in the environment. 
            env.nA is a number of actions in the environment.
        theta: We stop evaluation once our value function change is less than theta for all states.
        discount_factor: Gamma discount factor.
    
    Returns:
        Vector of length env.nS representing the value function.
    """
    # Start with a random (all 0) value function
    V = np.zeros(env.nS)
    while True:
        # TODO: Implement!
        delta = 0  #delta = change in value of state from one iteration to next
       
        for state in range(env.nS):  #for all states
            val = 0  #initiate value as 0
            
            for action,act_prob in enumerate(policy[state]): #for all actions/action probabilities
                for prob,next_state,reward,_ in env.P[state][action]:  #transition probabilities,state,rewards of each action
                    val += act_prob * prob * (reward + discount_factor * V[next_state])  #eqn to calculate
            delta = max(delta, np.abs(val-V[state]))
            V[state] = val
        if delta < theta:  #break if the change in value is less than the threshold (theta)
            break
    return np.array(V)

def policy_iteration(policy_eval_fn=policy_eval, discount_factor=1.0):
    """
    Policy Improvement Algorithm. Iteratively evaluates and improves a policy
    until an optimal policy is found.
    
    Args:
        env: The OpenAI envrionment.
        policy_eval_fn: Policy Evaluation function that takes 3 arguments:
            policy, env, discount_factor.
        discount_factor: gamma discount factor.
        
    Returns:
        A tuple (policy, V). 
        policy is the optimal policy, a matrix of shape [S, A] where each state s
        contains a valid probability distribution over actions.
        V is the value function for the optimal policy.
        
    """
    def one_step_lookahead(state, V):
        """
        Helper function to calculate the value for all action in a given state.
        
        Args:
            state: The state to consider (int)
            V: The value to use as an estimator, Vector of length env.nS
        
        Returns:
            A vector of length env.nA containing the expected value of each action.
        """
        A = np.zeros(env.nA)
        for a in range(env.nA):
            for prob, next_state, reward, _ in env.P[state][a]:
                A[a] += prob * (reward + discount_factor * V[next_state])
        return A
    # Start with a random policy
    policy = np.ones([env.nS, env.nA]) / env.nA

    steps=0
    while True:
        steps+=1
        # Implement this!
        curr_pol_val = policy_eval_fn(policy, discount_factor)  #eval current policy
        policy_stable = True  #Check if policy did improve (Set it as True first)
        for state in range(env.nS):  #for each states
            chosen_act = np.argmax(policy[state])  #best action (Highest prob) under current policy
            act_values = one_step_lookahead(state,curr_pol_val)  #use one step lookahead to find action values
            best_act = np.argmax(act_values) #find best action
            if chosen_act != best_act:
                policy_stable = False  #Greedily find best action
            policy[state] = np.eye(env.nA)[best_act]  #update 
        if policy_stable:
            print("Policy iteration converged in {} steps".format(steps))
            return policy, curr_pol_val
    
    return policy, np.zeros(env.nS)

def value_iteration(theta=0.0001, discount_factor=1.0):
    """
    Value Iteration Algorithm.
    
    Args:
        env: OpenAI env. env.P represents the transition probabilities of the environment.
            env.P[s][a] is a list of transition tuples (prob, next_state, reward, done).
            env.nS is a number of states in the environment. 
            env.nA is a number of actions in the environment.
        theta: We stop evaluation once our value function change is less than theta for all states.
        discount_factor: Gamma discount factor.
        
    Returns:
        A tuple (policy, V) of the optimal policy and the optimal value function.        
    """
    
    def one_step_lookahead(state, V):
        """
        Helper function to calculate the value for all action in a given state.
        
        Args:
            state: The state to consider (int)
            V: The value to use as an estimator, Vector of length env.nS
        
        Returns:
            A vector of length env.nA containing the expected value of each action.
        """
        A = np.zeros(env.nA)
        for act in range(env.nA):
            for prob, next_state, reward, _ in env.P[state][act]:
                A[act] += prob * (reward + discount_factor*V[next_state])
        return A
    
    V = np.zeros(env.nS)
    steps=0
    while True:
        steps+=1
        delta = 0  #checker for improvements across states
        for state in range(env.nS):
            act_values = one_step_lookahead(state,V)  #lookahead one step
            best_act_value = np.max(act_values) #get best action value
            delta = max(delta,np.abs(best_act_value - V[state]))  #find max delta across all states
            V[state] = best_act_value  #update value to best action value
            # print("s: {}, V: {}, delta: {}".format(state,V[state], delta) )
        if delta < theta:  #if max improvement less than threshold
            print("Value iteration converged in {} steps".format(steps))
            break
    policy = np.zeros([env.nS, env.nA])
    for state in range(env.nS):  #for all states, create deterministic policy
        act_val = one_step_lookahead(state,V)
        best_action = np.argmax(act_val)
        policy[state][best_action] = 1
        
    
    # Implement!
    return policy, V


def policyMatrixToVector(policy):
    policy_vector = []
    for array in policy:
        policy_vector.append(np.argmax(array))
    return policy_vector
    
# def writePolicyArrayToCsv(path, policy):
#     print("Writing policy to", path)
#     with open(path, mode='w') as policy_file:
#         writer = csv.writer(policy_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
#         writer.writerow(policy)

# def readPolicyFromCsv(path):
#     arrays = []
#     with open(path) as policy_file:
#         reader = csv.reader(policy_file, delimiter=',')
#         for row in reader:
#             arrays.append(row)
    
    # intarrays = []
    # for row in arrays:
    #     for val in row:
    #         intarrays.append(int(val))
    # return intarrays

env.buildP(terminal_state=False)

val_iter_policy = value_iteration(discount_factor=0.1)
pol_iter_policy = policy_iteration(policy_eval,discount_factor=0.1)

vi_pi_star = policyMatrixToVector(val_iter_policy[0])
pi_pi_star = policyMatrixToVector(pol_iter_policy[0])

random_policy = np.ones([env.nS, env.nA]) / env.nA
print("Random pi eval: ")
print(policy_eval(random_policy,discount_factor=0.1))
print("VI pi* eval: ")
print(policy_eval(val_iter_policy[0],discount_factor=0.1))
print("PI pi* eval: ")
print(policy_eval(pol_iter_policy[0],discount_factor=0.1))

print("VI pi*: ")
print(vi_pi_star) # This is the output policy
print("PI pi*: ")
print(pi_pi_star) # This is the output policy



if len(sys.argv) == 5:
    VI_pi_star_file = sys.argv[1]
    PI_pi_star_file = sys.argv[2]
    print("Saving VI_pi_star to : {}".format(VI_pi_star_file))
    print("Saving PI_pi_star to : {}".format(PI_pi_star_file))
else:
    VI_pi_star_file = "VI_pi_star.csv"
    PI_pi_star_file = "PI_pi_star.csv"        
    print("Saving VI_pi_star to : {}".format(VI_pi_star_file))
    print("Saving PI_pi_star to : {}".format(PI_pi_star_file))



csv_interface.writePolicyArrayToCsv(VI_pi_star_file, vi_pi_star)
csv_interface.writePolicyArrayToCsv(PI_pi_star_file, pi_pi_star)
