#!/usr/bin/env python

# See README.md for more info

import numpy
import csv

# Used to specifically write policy arrays to file
def writePolicyArrayToCsv(path, policy):
    with open(path, mode='w') as policy_file:
        writer = csv.writer(policy_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(policy)

# Used to specificalyl read policy arrays from csv
def readPolicyFromCsv(path):
    arrays = []
    with open(path) as policy_file:
        reader = csv.reader(policy_file, delimiter=',')
        for row in reader:
            arrays.append(row)
    
    intarrays = []
    for row in arrays:
        for val in row:
            intarrays.append(int(val))
    return intarrays

# Used to specifically write Q Tables arrays to file
def writeQTableToCsv(path, qtable):
    with open(path, mode='w') as q_file:
        writer = csv.writer(q_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        for row in qtable:
            writer.writerow(row)

# Used to specifically read Q Tables arrays from file
def readQTableFromCsv(path, shape):
    qtable = numpy.empty(shape)
    with open(path) as q_file:
        reader = csv.reader(q_file, delimiter=',')
        for i,row in enumerate(reader):
            qtable[i]=row
    return qtable

# Used while computing performance metrics to compare the different algorithms
def appendPerformaceData(path, data):
    with open(path, mode='a') as p_file:
        writer = csv.writer(p_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        for row in data:
            writer.writerow(row)


# Used only while executing this script to test other functions
def test():
    array = [1, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 1]
    qtable = numpy.random.rand(15,3)

    writePolicyArrayToCsv("test.csv", array)
    arrays = readPolicyFromCsv("test.csv")
    if array==arrays: print("Policy csv Same")
    print(arrays)

    writeQTableToCsv("qtable.csv",qtable)
    acquired = readQTableFromCsv("qtable.csv",[15,3])
    if (qtable==acquired).all: print("Qtables csv same")
    print(acquired)

if __name__=="__main__":
    test()

