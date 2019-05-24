#!/usr/bin/env python

""" 

See README.md for more info

1. Import this script
2. Invoke env.buildP()
3. Then use P as required
4. Invoke env.printP() to print a nice readable format
5. When done using P, invoke env.scrubP() to empty it so that it can be rebuilt


TERMINATING MODEL (terminal_state == True)
Terminating states
    Reward GOAL_REWARD_POSITIVE for reaching or staying in final arm pose (for the correct action) 
Pre-terminating states
    Reward GOAL_REWARD_POSITIVE for reaching the final arm pose (action up) 
Starting states
    Reward ACTION_REWARD_POSITIVE for staying or leaving initial arm pose (for the correct action) 
Other states
    Reward ACTION_REWARD_POSITIVE for action matching force, else ACTION_REWARD_NEGATIVE

NON-TERMINATING MODEL (terminal_state == False)
Final arm pose states
    Reward ACTION_REWARD_POSITIVE for staying (action up or stay) in or leaving final arm pose (action down) 
Initial arm state
    Reward ACTION_REWARD_POSITIVE for staying in (action down or stay) or leaving initial arm pose (action up) 
Other states
    Reward ACTION_REWARD_POSITIVE for action matching force, else ACTION_REWARD_NEGATIVE
"""

import numpy as np
import pandas as pd 

nS = 30
nA = 3

ACTION_DOWN = 0
ACTION_STAY = 1
ACTION_UP = 2

FORCE_SAYS_DOWN = 0
FORCE_SAYS_STAY = 1
FORCE_SAYS_UP = 2

ACTION_REWARD_POSITIVE = -1
ACTION_REWARD_NEGATIVE = -3
GOAL_REWARD_POSITIVE = 10


TERMINAL_STATES=[nS-1, nS-2, nS-3]

P = {s : {a : [] for a in range(nA)} for s in range(nS)}

# Need to build the model when invoking this script in others before using it.
# @param terminal_state. If the model is terminating or non terminating type.
def buildP(terminal_state=False):
    global P
    for s in range(nS):
        for a in range(nA):
            force = s%3
            arm = int(s/3)
            s_=r1=r2=r3=1
            #General cases
            
            if     (force == FORCE_SAYS_DOWN and a == ACTION_DOWN) \
                or (force == FORCE_SAYS_STAY and a == ACTION_STAY) \
                or (force == FORCE_SAYS_UP and a == ACTION_UP):
                r = ACTION_REWARD_POSITIVE
            else:
                r = ACTION_REWARD_NEGATIVE

            if (a == ACTION_UP):
                s_ = (s - force) + 3
            elif (a == ACTION_STAY):
                s_ = s - force
            else:
                s_ = (s - force) - 3


            #Border cases

            #Arm in lowest poistion
            if (arm == 0):
                s_ = 0 if (a == ACTION_DOWN or a == ACTION_STAY) else 3
                # r = ACTION_REWARD_POSITIVE if a == DO_NOTHING else ACTION_REWARD_NEGATIVE  
                if (s == 0 and a == ACTION_STAY) or (s == 1 and a == ACTION_STAY) or (s == 2 and a == ACTION_UP) : 
                    r = ACTION_REWARD_POSITIVE
                else:
                    r = ACTION_REWARD_NEGATIVE

            #Arm in final position
            if (arm ==  int((nS/3))-1):
                if (a == ACTION_UP):
                    s_ = s - force
                elif (a == ACTION_STAY):
                    s_ = s - force
                else:
                    s_ = (s - force) - 3
                    
                if terminal_state:
                    r = GOAL_REWARD_POSITIVE if a == ACTION_STAY else ACTION_REWARD_NEGATIVE
                else:
                    if(s == nS-3 and a == ACTION_DOWN) or ( (s == nS-1 or s == nS-2) and a == ACTION_STAY )  :
                        r = ACTION_REWARD_POSITIVE
                    else:
                        r = ACTION_REWARD_NEGATIVE

            
            r1 = r2 = r3 = r

            # Arm in penultimate position and input to go up
            if (arm ==  int((nS/3))-2 and force == FORCE_SAYS_UP and terminal_state):
                if s_ in TERMINAL_STATES: r1 = GOAL_REWARD_POSITIVE
                if s_+1 in TERMINAL_STATES: r2 = GOAL_REWARD_POSITIVE
                if s_+2 in TERMINAL_STATES: r3 = GOAL_REWARD_POSITIVE

            P[s][a].append(  (1.0/3.0, s_,   r1, True if (s_   in TERMINAL_STATES or s in TERMINAL_STATES) and terminal_state else False)  )
            P[s][a].append(  (1.0/3.0, s_+1, r2, True if (s_+1   in TERMINAL_STATES or s in TERMINAL_STATES) and terminal_state else False)  )
            P[s][a].append(  (1.0/3.0, s_+2, r3, True if (s_+2   in TERMINAL_STATES or s in TERMINAL_STATES) and terminal_state else False)  )

# Since model is built explicitly, successive buildP() calls keep stacking the arrays in size. scrubP() before each buildP() ensures the model size remains constant.
def scrubP():
    global P
    P = {s : {a : [] for a in range(nA)} for s in range(nS)}

# Nice readable format
def printP():
    for s in range(nS):
        for a in P[s]:
            force = s%3
            arm = int(s/3)
            for x in P[s][a]:
                print("s:",s, 
                " a:",  "UP" if a == ACTION_UP else ("DOWN" if a == ACTION_DOWN else "STAY")  , 
                " :", x, 
                "Force :",  "UP" if force==FORCE_SAYS_UP else ("DOWN" if force==FORCE_SAYS_DOWN else "STAY")  , 
                "Arm : ", arm)
            print()
        print("----------------------")

# Save model to text file
def printPtoTxtFile(path):
    for s in range(nS):
        for a in P[s]:
            force = s%3
            arm = int(s/3)
            for x in P[s][a]:
                print("s:",s, 
                " a:",  "UP" if a == ACTION_UP else ("DOWN" if a == ACTION_DOWN else "STAY")  , 
                " :", x, 
                "Force :",  "UP" if force==FORCE_SAYS_UP else ("DOWN" if force==FORCE_SAYS_DOWN else "STAY")  , 
                "Arm : ", arm, open(path,"a"))
            print(" ", open(path,"a"))
        print("----------------------", open(path,"a"))
