# THE DRINKING ASSISTANT PROJECT
This package contains all the nodes required to run the drinking assistant using the Kinova JACO arm. 

## BRINGUP
* `robot_bringup.launch` : This launch file can be used to start the Kinova ROS driver and the Robot Control interface.   
* `sensors.launch` : This starts the force sensors i.e. rosserial.
* `feeder_gmm.launch` : Starts the force controller using GMR trajectory that is available from the `./resources` directory.
* `feeder_sideways.launch` : This launches the sideways drinking force control node. Optionally one may choose to record rosbags.   
* `feeder_straight.launch` : Similar as previous, but for forward feeding approach.
> Note that the paths of the rosbags are hardcoded. If not recording, comment out the whole thing.
* `feeder_policy_drive` : This starts the Reinforcement Learning based controller with forward feeding.
* `rl_xx_training.launch` : This set of launch files is used to start the training process for the particular algorithm. This also starts the emulator service for training.   

**You generally want to start `robot_bringup.launch` and `sensors.launch` in order to use any other nodes.**   

### Launch Parameters (wherever applicable)
1. `port` - Enable and use this in the `sensors.launch` to specify the full port name like /dev/tty_USB1 if usb port naming is not done.   
2. `usb_port` - Currently used in `sensors.launch` instead of `port`. Requires the USB device to be configured with the correct name.
3. `user` - User number. Used for categorizing. Required.   
4. `name` - Name of the user/experiment. Used for subcategorizing. Required.  
5. `algorithm` - Select the algorithm whose optimum policy is to be loaded. Valied choices are - `pi, vi, q, sarsa`. These are case sensitive.
> DQN policy is not included in this list. This is still a TODO.
6. `episodes` - The number of episodes to run the training for.   
7. `steps` - The number of steps per episode.   
8. `loss` - Threshold value for the loss function to converge to.   

## CPP NODES
* `data_extractor.cpp`   

This node is used to extract data from recorded rosbags (for force and policy controller data) into ODS files. The path to save and filename need to be  carefully supplied as **2 command line arguments.** `arg_1` is the user number and `arg_2` is the user name. Start the rosbag separately from console and configure where the extracted data is to be saved using the two arguments. For example : 
```
rosrun drinking_assistant 1 force_water_1
```
This will extract the configured data from whatever rosbag is playing currently and save it under the directory structure `/home/tejas/data/extracted_data/1/force_water_1_synchronised.ods`.    
Edit the hardcoded save path as required.

* `data_extractor_gmm.cpp`   

This functions similarly as the previous one but works with bags that recorded gmm trajectory recording data. Generates a TXT and a CSV file. Requires **only 1 argument** which is the file name to save under.

* `feeder_gmm.cpp`   

This node starts the force controller using the GMR trajectory. You can pass a command line argument with the full path to the GMR trajectory file but it is advisable to use launch file which has it pre-configured.   

* `feeder_policy_drive.cpp`   
 
This is the Policy Drive Controller. Suggested to use the launch file instead.   

* `feeder_position_control.cpp`
*NOT IN USE*    

This is the Force Controller Mk I using Position Controller instead of Velocity Controller. Technically works, but since PC is so bad, this was not further developed and is no longer used. Still left here for reference or future use. 

* `feeder_sideways.cpp`
*NOT IN USE*

This is the Force Controller Mk I with sideways feeding approach. Deprecated since the smart cup was changed to single sensor.

* `feeder_sideways_1D.cpp`   

This is the Force Controller Mk II with sidewys feeding approach. Start with launch file if recording, or simply run this node if only testing.

* `feeder_straight.cpp`
*NOT IN USE*

This is the Force Controller Mk I with forward feeding approach. Deprecated since the smart cup was changed to single sensor.

* `feeder_straight_1D.cpp`

This is the Force Controller Mk II with forward feeding approach. Start with launch file if recording, or simply run this node if only testing.

* `ik_tests.cpp`
*NOT IN USE*   

This is used to test TRAC and KDL kinematic solvers. They do not work so far as could be seen. Left here for future development.

* `position_drive.cpp`
*NOT IN USE*   

This was one of the earlier nodes written to test out the position controller from Kinova. Used for development purposes only.

* `quaternion_tester.cpp`
*NOT IN USE*   

This was one of the earlier nodes written to test out quaternions on the JACO. Used for development purposes only.

* `robot_control.cpp`

This node serves as an inerface to the controllers supplide by Kinova. Developed by *Felix Goldau*, it is started along with the actual Kinova drivers from the `robot_bringup.launch`.    


* `tf_caster.cpp`
*NOT IN USE*   

This was one of the earlier nodes written to test out TF on the JACO. Used for development purposes only.


* `velocity_drive.cpp` and `velocity_test.cpp`
*NOT IN USE*   

This was one of the earlier nodes written to test out the velocity controller by Kinova and the targeted velocity drive used for most other nodes. Used for development purposes only.


## PYTHON SCRIPTS

These scripts serve mostly the Reinforcement Learning part and so do not actually run with the robot. They are used for development only.

* `csv_interface.py`   

Serves as a library used by other scripts to read/write CSV files on the disk.

* `env.py`   

This is the Markov Model of the drinking assistant robot. Does not run directly but is invoked by other scripts. Modeled to function similarly as the OpenAI Gym models.

* `feeder_xx_training.py`    

This set of scripts contains the training program for the respective algorithm. Make use of the model, csv interface and the robot emulator. Suggested to use the launch files instead of running these scripts.

* `gmm_equalizer.py`   

This script is used to prepare the data files for training the GMM.

* `robot_emulator_server.py`

The main part of the emulator. This script runs as the server providing the emulation service.

* `robot_emulator_client.py`

This script provides the interface of making use of the amulator. Is invoked by other scripts and its functions are made use of by the training scripts.

* `test_xx.py`

These scripts can be used to test the training procedure before actually starting it as well as the validity of model and emulator.

## RESOURCES

the `./resources` directory contains some useful files for other nodes to function such as the optimum policies, the DQN model, the performance metrics etc...





