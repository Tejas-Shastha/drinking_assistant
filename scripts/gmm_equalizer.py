#!/usr/bin/env python

# See README.md for more info

import numpy as np
import random
import csv

### Requires @param{number_of_trajectories} number of files named data%d.txt in path @param{path}

path ="/home/tejas/data/gmm/"
number_of_trajectories = 4
reshaped_name = "_reshaped.txt"

trajectories = [] # create an empty array
size = []

print("Trajectories acquired : ")
for i in  range(number_of_trajectories):
    trajectories.append([])                 # Add another dimension each time another trajectory is to be loaded
    file_name = path+"data"+str(i)+".txt"

    with open(file_name) as csv_file:    
        csv_reader = csv.reader(csv_file, delimiter='\t')
        for row in csv_reader:
            trajectories[i].append(row)

    print("{} has shape : {}".format(i,np.asarray(trajectories[i]).shape))  
    size.append  (np.asarray(trajectories[i]).shape[0])

min_size = min(size)
print("{} is the minimum size between all trajectories, reshaping to this".format(min_size))

difference = 0
reshaped_trajectories = []
for i in range(number_of_trajectories):
    reshaped_trajectories.append([])  # Add another dimension to this array
    
    difference = size[i] - min_size   # Create a list of UNIQUE random indices, with size equal to the difference of current trajectory from minimum trajectory
    random_list = random.sample(range(1,size[i]) , difference)
    random_list.sort()

    for j,row in enumerate(trajectories[i]):    # Drop samples at these random indices
        if j in random_list:
            continue
        else:
            reshaped_trajectories[i].append(row)
    print("{} has shape : {}".format(i,np.asarray(reshaped_trajectories[i]).shape)) 

# Write the reshaped trajectories back to file
for i in range(number_of_trajectories):  
    file_name = path+"data"+str(i)+reshaped_name
    with open(file_name, mode='w') as writer_file:
        writer = csv.writer(writer_file, delimiter='\t', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        for row in reshaped_trajectories[i]:
            writer.writerow(row)








    





