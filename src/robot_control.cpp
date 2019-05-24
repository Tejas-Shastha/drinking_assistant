/**
  * See README.md for more info.
  */
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinova_msgs/PoseVelocity.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/ArmPoseGoal.h>
#include <kinova_msgs/ArmJointAnglesAction.h>
#include <kinova_msgs/ArmJointAnglesGoal.h>
#include <mutex>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>

#define POSE_CONTROL_CLIENT_SERVER_ADDRESS "/j2s7s300_driver/pose_action/tool_pose"
#define JOINT_CONTROL_CLIENT_SERVER_ADDRESS "/j2s7s300_driver/joints_action/joint_angles"
#define POSE_CONTROL_CLIENT_SERVER_TIMEOUT 3// s
#define JOINT_CONTROL_CLIENT_SERVER_TIMEOUT 3// s
#define STANDART_VEL_CMD_TIMEOUT  (0.5) // s

// State of robots control
typedef enum State_{
  VELOCITY_CONTROL,
  POSITION_CONTROL,
  JOINT_CONTROL,
  STOPPED,
  NO_STATE,
} State;

kinova_msgs::PoseVelocity poseVelCmd; // Latest velocity command
State currentState = STOPPED; // Current State of the Robot Control
ros::Time poseVel_time; // time of the latest velocity command OR set to now-timeout to deactivate velocity commands.
ros::Duration poseVel_timeout(STANDART_VEL_CMD_TIMEOUT); // timeout for velocity commands
std::mutex poseVel_lock;  // Lock to make all global variables thread safe
actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> *poseControlClient_ptr; // pointer to position control actionclient
actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction> *jointControlClient_ptr; // pointer to joint control actionclient


/**
 * @brief Checks if velocity command is up to date
 * @return false if velocity command has timed out or none set.
 */
inline bool velCmd_upToDate(){
  return (ros::Time::now() - poseVel_time) < poseVel_timeout;
}

/**
 * @brief Callback function for velocity command subscriber. Stores the command, update latest command time and cancel a position controls
 * @param twist velocity command
 */
void poseVelocity_rawCallback(geometry_msgs::TwistStamped twist){
  poseVel_lock.lock();
  poseVel_time = ros::Time::now();
  poseVelCmd.twist_linear_x = twist.twist.linear.x;
  poseVelCmd.twist_linear_y = twist.twist.linear.y;
  poseVelCmd.twist_linear_z = twist.twist.linear.z;
  poseVelCmd.twist_angular_x = twist.twist.angular.x;
  poseVelCmd.twist_angular_y = twist.twist.angular.y;
  poseVelCmd.twist_angular_z = twist.twist.angular.z;
  if(currentState == POSITION_CONTROL || currentState==JOINT_CONTROL)
  {
    jointControlClient_ptr->cancelAllGoals();
    poseControlClient_ptr->cancelAllGoals();
  }
  if(!poseVelCmd.twist_linear_x && !poseVelCmd.twist_linear_y && !poseVelCmd.twist_linear_z && !poseVelCmd.twist_angular_x && !poseVelCmd.twist_angular_y && !poseVelCmd.twist_angular_z)
    currentState = STOPPED; // If all velocities are set to zero, set current State to Stopped
  else
    currentState = VELOCITY_CONTROL;
  poseVel_lock.unlock();
}

/**
 * @brief Callback function the Pose Control Client sending pose goals. This function sets the current state to stopped if it was set to position control before.
 */
void goalDoneCallback(actionlib::SimpleClientGoalState state, kinova_msgs::ArmPoseResultConstPtr result){
  poseVel_lock.lock();
  if(currentState == POSITION_CONTROL)
    currentState = STOPPED;
  poseVel_lock.unlock();
}

void jointGoalDoneCallback(actionlib::SimpleClientGoalState state, kinova_msgs::ArmJointAnglesActionGoal result){
  poseVel_lock.lock();
  if(currentState == JOINT_CONTROL)
    currentState = STOPPED;
  poseVel_lock.unlock();
}

/**
 * @brief Callback function for position control subscriber. Clears latest velocity command and sends position command
 * @param pose position command
 */
void poseGoal_rawCallback(geometry_msgs::PoseStamped pose){
  // Create goal to send to system
  kinova_msgs::ArmPoseGoal goal;
  goal.pose.header = std_msgs::Header();
  goal.pose.header.frame_id = "j2s7s300_link_base"; // define frame to be the robot's base
  geometry_msgs::Point pos;
  pos.x = pose.pose.position.x;
  pos.y = pose.pose.position.y;
  pos.z = pose.pose.position.z;
  geometry_msgs::Quaternion ori;
  ori.x = pose.pose.orientation.x;
  ori.y = pose.pose.orientation.y;
  ori.z = pose.pose.orientation.z;
  ori.w = pose.pose.orientation.w;
  goal.pose.pose.position=pos;
  goal.pose.pose.orientation=ori;
  poseVel_lock.lock();
  poseControlClient_ptr->sendGoal(goal, goalDoneCallback);// // send new position command
  currentState = POSITION_CONTROL;
  poseVel_lock.unlock();
}

void jointGoal_rawCallback(kinova_msgs::ArmJointAnglesActionGoal goal){
  // Create goal to send to system
  goal.header = std_msgs::Header();
  goal.header.frame_id = "j2s7s300_link_base"; // define frame to be the robot's base
  goal.header.stamp = ros::Time::now();
  poseVel_lock.lock();
//  jointControlClient_ptr->sendGoal(goal, jointGoalDoneCallback);// // send new joint command
  currentState = JOINT_CONTROL;
  poseVel_lock.unlock();
}

/**
 * @brief Publishes the state of current control if it changed. Currently publishes via ROS_INFO
 * @param status_pub The publisher to send data with
 */
void publishState(const ros::Publisher &status_pub){
  static State lastState = NO_STATE;
  const std::map<State, std::string> publishData = {
    {VELOCITY_CONTROL, "Velocity Control"},
    {POSITION_CONTROL, "Position Control"},
    {JOINT_CONTROL, "Joint Control"},
    {STOPPED, "Stopped"}
  };
  if(lastState != currentState){
    lastState = currentState;
    std_msgs::String msg;
    msg.data = publishData.at(currentState);
    status_pub.publish(msg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_control");
  ros::NodeHandle nh;

  ROS_INFO("Robot Control initialising");

  if(argc == 2){  // define timeout if set via function call
    double timeout = atof(argv[1]);
    if(timeout != 0)
      poseVel_timeout = ros::Duration(timeout);
  }

  actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> poseControlClient(POSE_CONTROL_CLIENT_SERVER_ADDRESS);
  poseControlClient_ptr = &poseControlClient;
  if(!poseControlClient.waitForServer(ros::Duration(POSE_CONTROL_CLIENT_SERVER_TIMEOUT))){
    ROS_ERROR("Robot Control failing to connect to actionlib client for position control. Aborting");
    return 0;
  }

  actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction> jointControlClient(JOINT_CONTROL_CLIENT_SERVER_ADDRESS);
  jointControlClient_ptr = &jointControlClient;
  if(!jointControlClient.waitForServer(ros::Duration(JOINT_CONTROL_CLIENT_SERVER_TIMEOUT))){
    ROS_ERROR("Robot Control failing to connect to actionlib client for joint control. Aborting");
    return 0;
  }


  ros::Subscriber poseVel_sub = nh.subscribe("/RobotControl/VelocityControl",1, poseVelocity_rawCallback);
  ros::Subscriber poseGoal_sub = nh.subscribe("/RobotControl/PoseControl",1, poseGoal_rawCallback);
  ros::Subscriber jointGoal_sub = nh.subscribe("/RobotControl/JointControl",1, jointGoal_rawCallback);
  ros::Publisher vel_cmd_pub = nh.advertise<kinova_msgs::PoseVelocity>("/j2s7s300_driver/in/cartesian_velocity",10);
  ros::Publisher status_pub = nh.advertise<std_msgs::String>("/RobotControl/Status",1, true);

  ROS_INFO("Robot Control Running");
  // loop at 100 hz for velocity control to kick in
  ros::Rate loop_rate(100);
  while(ros::ok()){
    poseVel_lock.lock();
    if(currentState == VELOCITY_CONTROL){
      // don't publish old data
      if(velCmd_upToDate()){
        vel_cmd_pub.publish(poseVelCmd);
      }
    }
    publishState(status_pub);
    poseVel_lock.unlock();
    // standart ros loop end
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("Robot Control ending");
  poseControlClient.cancelAllGoals(); // Cancel all goals so the robot does not move unchecked
}
