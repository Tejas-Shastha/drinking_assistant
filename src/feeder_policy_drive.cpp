///
/// See README.md for more info
///
/// if force € [0, FORCE_F_1_2_THRESH] then state 1 : lower cup
/// if force € (FORCE_F_1_2_THRESH, FORCE_F_2_3_THRESH] then state 2 : do nothing
/// if force € (FORCE_F_2_3_THRESH, MAX] then state 3 : raise cup
///
/// Fallback velocity functions
///	Vel.x				                                  Vel.y 
/// -180 to -90	  y =  0.004444444*x + 0.8			  -180 to   0	y =  0.004444444*x + 0.4
///  -90 to  90	  y = -0.004444444*x 			           0 to 180	y = -0.004444444*x + 0.4
///   90 to 180	  y =  0.004444444*x - 0.8				
///
///
///
/// TODO: IMPLEMENT SAFETY ABOVE 1.5N

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "mutex"
#include "angles/angles.h"
#include "std_msgs/Int32.h"
#include "audio_emergency/AudioMessage.h"


#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"

#include <hri_package/Sens_Force.h>
#include <kinova_msgs/PoseVelocity.h>

#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>


#define FORCE_F_1_2_THRESH 0.05 /** Threshold between force regions 1 and 2*/
#define FORCE_F_2_3_THRESH 0.5  /** Threshold between force regions 2 and 3*/
#define NUMBER_OF_ARM_SUB_STATES 10 /** Must be same as the model used to train the RL algorithms */
#define UPPER_FEED_ANGLE_THRESH 176.00 /** Force safety trigger threshold*/
#define FORCE_SAFETY 5

/** These are the encoded values for the different movement patterns for the gripper*/
#define ARC_DOWN 1
#define ARC_UP 2
#define TRANSLATE_BACK 3
#define RAISE_CUP 4
#define LOWER_CUP 5
#define TRANSLATE_UP 6
#define TRANSLATE_DOWN 7
#define TRANSLATE_FRONT 8
#define END_EFF_FRAME "j2s7s300_end_effector"
#define BASE_FRAME "j2s7s300_link_base"
#define SENSOR_FRAME "forcesensor"

/** Parameters to define the velocity commands*/
#define MAX_STEPS 200
#define VEL_LIN_MAX 0.04
#define VEL_ANG_MAX 0.4
#define VEL_CMD_DURATION 0.8

#define ACTION_DOWN 0
#define ACTION_STAY 1
#define ACTION_UP 2


double lower_angle_thresh = 85;
float thresh_lin = 0.01;
float thresh_ang = 0.05;
double rotation_step;
bool emergency = false;

int step_count;

std::string result;
std::mutex lock_pose;
std::mutex lock_status;
std::mutex lock_proc;
std::mutex lock_force;
std::mutex lock_emerg;
double force_f, force_b;
bool processing=false;
ros::Publisher cmd_pos;
ros::Publisher cmd_vel;
geometry_msgs::PoseStamped current_pose, initial_pose;

/**
 * @brief Converts a quaternion message type into euler XYZ- Roll, Pitch, Yaw angles in radians
 * @param orientation. geometry_msgs::Quaterion
 * @param roll
 * @param pitch
 * @param yaw
 */
void getRPYFromQuaternionMSG(geometry_msgs::Quaternion orientation, double& roll,double& pitch, double& yaw)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(orientation,quat);
  quat.normalize();
  tf::Matrix3x3 mat(quat);
  mat.getRPY(roll, pitch,yaw);
}

/**
 * @brief Once a position control command is given, we need to wait until action server finished this command or aborts.
 */
void waitForActionCompleted()
{
  std::string temp_res="";
  while(ros::ok())
  {
    ros::spinOnce();
    lock_status.lock();
    temp_res = result;
    lock_status.unlock();

    if (temp_res=="Stopped")
    {
      ROS_INFO_STREAM("Status : " << temp_res);
      //Wait a little extra for arm to settle due to inertia.
      ros::Time start = ros::Time::now();
      while(ros::Time::now()-start < ros::Duration(0.2)) ros::spinOnce();
      break;
    }
  }
}

/**
 * @brief Used in position controller. Based on the desired movement pattern and current pose, calculate target pose.
 * @param direction. Encoded movement pattern.
 * @param start_pose. Starting pose, manipulated into target pose in call be reference model.
 * @param distance. Custom distance to travel in given pattern.
 */
void setPoseForDirection(int direction, geometry_msgs::PoseStamped& start_pose,double  distance)
{
  geometry_msgs::Quaternion quat;
  switch(direction)
  {
  /// END EFFECTOR FRAME  Z-forward-blue, Y-up-green, X-left-red
  case ARC_DOWN :
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(rotation_step),angles::from_degrees(0),angles::from_degrees(0)); //Turn down
    start_pose.header.frame_id=END_EFF_FRAME;
    start_pose.pose.position.x=0;
    start_pose.pose.position.y=0.01;  // Slightly up
    start_pose.pose.position.z=-0.01; // Slightly back
    start_pose.pose.orientation = quat;
    break;
  case ARC_UP :
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(-rotation_step),angles::from_degrees(0),angles::from_degrees(0));
    start_pose.header.frame_id=END_EFF_FRAME;
    start_pose.pose.position.x=0;
    start_pose.pose.position.y=-0.01;
    start_pose.pose.position.z=0.01;
    start_pose.pose.orientation = quat;
    break;
  case TRANSLATE_BACK :
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(0),angles::from_degrees(0),angles::from_degrees(0));
    start_pose.header.frame_id=END_EFF_FRAME;
    start_pose.pose.position.x=0;
    start_pose.pose.position.y=0;
    start_pose.pose.position.z=-distance;
    start_pose.pose.orientation = quat;
    break;
  case TRANSLATE_FRONT :
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(0),angles::from_degrees(0),angles::from_degrees(0));
    start_pose.header.frame_id=END_EFF_FRAME;
    start_pose.pose.position.x=0;
    start_pose.pose.position.y=0;
    start_pose.pose.position.z=distance;
    start_pose.pose.orientation = quat;
    break;

  /// SENSORFRAME  X-forward-red, Y-left-green, Z-up-blue
  }
}

/**
 * @brief Used in velocity control. Given the requested movement pattern, return the necessary corresponding twist values for the velocity command.
 * @param direction
 * @return The resultant twist.
 */
geometry_msgs::TwistStamped getTwistForDirection(int direction)
{
  geometry_msgs::TwistStamped twist;
  twist.header.stamp=ros::Time::now();
  twist.header.frame_id="j2s7s300_link_7";
  switch(direction)
  {
    case ARC_DOWN:
      ROS_INFO("Arc down");
      twist.twist.linear.z=VEL_LIN_MAX;
      twist.twist.angular.x=VEL_ANG_MAX;
    break;

    case ARC_UP:
      ROS_INFO("Arc up");
      twist.twist.linear.z=-VEL_LIN_MAX;
      twist.twist.angular.x=-VEL_ANG_MAX;
    break;

    case TRANSLATE_BACK:
      ROS_INFO("Translate back");
      twist.twist.linear.x=-VEL_LIN_MAX;
    break;

    case RAISE_CUP:
      ROS_INFO("Raise Cup");
      twist.twist.angular.x=VEL_ANG_MAX;
    break;

    case LOWER_CUP:
      ROS_INFO("Lower Cup");
      twist.twist.angular.x=-VEL_ANG_MAX;
    break;

    case TRANSLATE_UP:
      ROS_INFO("Translate up");
      twist.twist.linear.z=VEL_LIN_MAX;
    break;

    case TRANSLATE_DOWN:
      ROS_INFO("Translate down");
      twist.twist.linear.z=-VEL_LIN_MAX;
    break;

  }
  return twist;
}

/**
 * @brief Check if max upper angle threshold has been reached.
 * @return Boolean result.
 */
bool checkUpperAngleThreshold();

/**
 * @brief Check if max lower angle has been reached
 * @return Boolean result.
 */
bool checkLowerAngleThreshold();

/**
 * @brief Function to actually publish the calculated velocity command
 * @param twist_msg. Command to publish
 * @param duration. Duration to publish it for.
 */
void publishTwistForDuration(geometry_msgs::TwistStamped twist_msg, double duration)
{
  ros::Time time_start = ros::Time::now();
  while (ros::Time::now() - time_start < ros::Duration(duration))
  {
    ros::spinOnce();
    cmd_vel.publish(twist_msg);
  }
}

/**
 * @brief Main wrapper for position control.
 * @param direction. Movement pattern requested.
 * @param distance. Custom distance to move in this pattern.
 */
void positionControlDriveForDirection(int direction, double distance)
{
  geometry_msgs::PoseStamped start_pose, pose_in_base;
  tf::TransformListener tf_listener;
  //Wait for TF buffer to fill up
  ros::Duration(0.25).sleep();
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    try
    {
      lock_pose.lock();
      start_pose = current_pose;
      lock_pose.unlock();

      /** Calculate the actual pose to send to the position control action server*/
      setPoseForDirection(direction, start_pose, distance);

      tf_listener.transformPose(BASE_FRAME,start_pose,pose_in_base);
      break;

    }
    catch (tf::ExtrapolationException e)
    {
     /* ROS_INFO_STREAM("Waiting for transform "
                      //<< e.what()
                      );*/
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  cmd_pos.publish(pose_in_base);
}

/**
 * @brief Check if force sensors report max force safety threshold has been reached
 * @return Boolean result.
 */
bool isForceSafe()
{
  double local_force_f;
  lock_force.lock();
  local_force_f=force_f;
  lock_force.unlock();

  if (local_force_f >= FORCE_SAFETY)
  {
    ROS_WARN_STREAM("PAIN THRESHOLD BREACHED!!");
    return false;
  }
  else
    return true;
}


/**
 * @brief Check if the audio sensor node detects an emergency input from the user.
 * @return Boolean result.
 */
bool isAudioSafe()
{
  lock_emerg.lock();
  bool emerg = emergency;
  lock_emerg.unlock();

  if (emerg) ROS_WARN_STREAM("AUDIO EMERGENCY DETECTED!!");
  return !emerg;
}

/**
 * @brief Wrapper for the fallback sequence.
 * @param emerg. Is it an emergency fallback or a timed fallback. Movement differs based on this.
 */
void fallback(bool emerg=false);


/**
 * @brief Wrapper for the actual targeted velocity controller. Manipulated Roll angle only.
 * @param direction. Movement pattern requested.
 */
void driveToRollGoalWithVelocity(int direction)
{
  if(direction == RAISE_CUP)
    ROS_INFO_STREAM("Raise cup");
  else if (direction == LOWER_CUP)
    ROS_INFO_STREAM("Lower cup");
  else
  {
    ROS_ERROR_STREAM("Wrong direction given!!");
    return;
  }

  geometry_msgs::PoseStamped start_pose, goal_pose, temp_pose;

  lock_pose.lock();
  start_pose=current_pose;
  lock_pose.unlock();

  double temp_rol, temp_pit, temp_yaw;
  double goal_rol, goal_pit, goal_yaw;
  double start_rol, start_pit, start_yaw;
  double del_rol;

  goal_pose = start_pose;

  /** Calculate first the quaternion for how much roll angle to change */
  tf::Quaternion q_rot = tf::createQuaternionFromRPY(angles::from_degrees( direction==RAISE_CUP?rotation_step:-rotation_step),angles::from_degrees(0),angles::from_degrees(0)); // Rotate about x by 20 degrees
  tf::Quaternion q_start; tf::quaternionMsgToTF(start_pose.pose.orientation, q_start);
  /** Multiply starting quat with the rotation quat to get the resultant goal quat*/
  tf::Quaternion q_goal = q_start*q_rot;

  /** Convert the resultant quat to msg type*/
  tf::quaternionTFToMsg(q_goal,goal_pose.pose.orientation);

  /** The proportional controller*/
  ros::Rate loop_rate(100);
  //ros::Time start = ros::Time::now();
  while(ros::ok())
  {
    lock_pose.lock();
    temp_pose = current_pose;
    lock_pose.unlock();

    getRPYFromQuaternionMSG(goal_pose.pose.orientation, goal_rol, goal_pit, goal_yaw);
    getRPYFromQuaternionMSG(temp_pose.pose.orientation, temp_rol, temp_pit, temp_yaw);
    getRPYFromQuaternionMSG(start_pose.pose.orientation, start_rol, start_pit, start_yaw);

    /** Rescale from -180 to 180 into 0 to 360*/
    if (goal_rol < 0 ) goal_rol = goal_rol + angles::from_degrees(360);
    if (temp_rol < 0 ) temp_rol = temp_rol + angles::from_degrees(360);

    del_rol = goal_rol - temp_rol;

    if (std::fabs(del_rol)>=thresh_ang)
    {
      float linear_vel = 0.02;
      geometry_msgs::TwistStamped twist_msg;
      twist_msg.twist.linear.x = 0;
      twist_msg.twist.linear.y = 0;
      // Comment out next line ONLY if using end-effector mode ie: Mode 1 on jostick is active and 4th blue LED on far right is active.
//      twist_msg.twist.linear.z = direction==RAISE_CUP ? linear_vel : -linear_vel;
      twist_msg.twist.angular.x= del_rol>0?VEL_ANG_MAX:-VEL_ANG_MAX;
      twist_msg.twist.angular.y = 0;
      twist_msg.twist.angular.z= 0;

      if(isForceSafe() && isAudioSafe())
      {
        cmd_vel.publish(twist_msg);
      }
      else
      {
        ROS_WARN("ABORTING SEQUENCE!!!");
        fallback(true);
        ros::shutdown();
      }
    }
    else
    {
      //ROS_INFO_STREAM("Goal reached, breaking loop");
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

}

/**
 * @brief moveCup. Hybrid wrapper to chose between position or velocity controller based on requested pattern.
 * @param direction. Requested movement pattern.
 * @param duration
 * @param distance
 */
void moveCup(int direction, double duration=VEL_CMD_DURATION, double distance=0.1)
{
  if (direction==TRANSLATE_BACK || direction==TRANSLATE_FRONT)
  {
    positionControlDriveForDirection(direction, distance);
    return;
  }
  else
  {
    geometry_msgs::TwistStamped twist_msg = getTwistForDirection(direction);
    publishTwistForDuration(twist_msg, duration);

  }
  return;
}

/**
 * @brief poseGrabber. Callback for reading pose values. Store pose in a mutexed global variable.
 * @param pose
 */
void poseGrabber(geometry_msgs::PoseStamped pose)
{
  lock_pose.lock();
  current_pose=pose;
  lock_pose.unlock();
}

/**
 * @brief forceGrabber. Callback to read force values from rosserial
 * @param msg
 */
void forceGrabber(const hri_package::Sens_Force::ConstPtr msg)
{
  lock_force.lock();
  force_f=msg->forceF;
  force_b=msg->forceB;
  lock_force.unlock();
}

/**
 * @brief statusGrabber. Read robot status from RobotControl node.
 * @param status
 */
void statusGrabber(std_msgs::String::ConstPtr status)
{
  lock_status.lock();
  result=status->data;
  lock_status.unlock();
}

/**
 * @brief waitForPoseDataAvailable. When initialising robot, wait until all topics publish good data before starting the controllers.
 */
void waitForPoseDataAvailable()
{
  // This loop to wait until subscriber starts returning valid poses.
  ros::Rate loop_rate(9);
  geometry_msgs::PoseStamped temp_pose;
  while(ros::ok())
  {
    lock_pose.lock();
    temp_pose = current_pose;
    lock_pose.unlock();

    ros::spinOnce();
    loop_rate.sleep();

    if (temp_pose.pose.position.x != 0) break;
  }
  ROS_INFO("Pose data available");
}

bool checkUpperAngleThreshold()
{
  geometry_msgs::PoseStamped temp_pose;

  lock_pose.lock();
  temp_pose=current_pose;
  lock_pose.unlock();

  double temp_roll, temp_pitch, temp_yaw;
  getRPYFromQuaternionMSG(temp_pose.pose.orientation,temp_roll, temp_pitch, temp_yaw);

  if ( angles::to_degrees(temp_roll) > UPPER_FEED_ANGLE_THRESH )
  {
    ROS_WARN_STREAM("MAX UPPER FEED ANGLE REACHED");
    return false;
  }
  else
  {
   return true;
  }
}

bool checkLowerAngleThreshold()
{
  geometry_msgs::PoseStamped temp_pose;

  lock_pose.lock();
  temp_pose=current_pose;
  lock_pose.unlock();

  double temp_roll, temp_pitch, temp_yaw;
  getRPYFromQuaternionMSG(temp_pose.pose.orientation,temp_roll, temp_pitch, temp_yaw);

  if ((int)angles::to_degrees(temp_roll) - (int)angles::to_degrees(lower_angle_thresh) < rotation_step/2 && temp_roll > 0)
  {
    //ROS_WARN_STREAM("MAX LOWER ANGLE REACHED. LIMIT: " << angles::to_degrees(lower_angle_thresh) << " CURRENT: " << angles::to_degrees(temp_roll));
    return false;
  }
  else
  {
//   ROS_INFO_STREAM("Current roll: " << sangles::to_degrees(temp_roll));
   return true;
  }
}

/**
 * @brief getCurrentRoll. Get the current roll angle only.
 * @return Roll in degrees, rescaled from -180:180 to 0:360
 */
double getCurrentRoll()
{
  lock_pose.lock();
  geometry_msgs::PoseStamped temp_pose = current_pose;
  lock_pose.unlock();

  double temp_rol, temp_pitch, temp_yaw;
  getRPYFromQuaternionMSG(temp_pose.pose.orientation, temp_rol, temp_pitch, temp_yaw);


  return temp_rol<0?temp_rol+angles::from_degrees(360):temp_rol;
}

/**
 * @brief callFallbackTimer. Start the timer when robot reaches starting pose.
 * @param duration. Duration of timer.
 */
void callFallbackTimer(double duration)
{

  ros::Time start = ros::Time::now();
  ros::Duration run_for(duration);
  ros::Rate loop_rate(10);
  ROS_WARN_STREAM("Starting fallback timeout for: " << duration << "s.");
  while(ros::Time::now() - start <= run_for)
  {
    lock_force.lock();
    double force = force_f;
    lock_force.unlock();

    if (force>= FORCE_F_2_3_THRESH)
    {
      ROS_INFO_STREAM("Force detected, breaking out of timer");
      return;
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

  ROS_WARN_STREAM("Time out! Initialising fallback");
 // moveCup(TRANSLATE_BACK);
  fallback();
  sleep(1);
  ros::shutdown();
}

void fallback(bool emerg)
{
  lock_pose.lock();
  geometry_msgs::PoseStamped  start_pose=current_pose;
  lock_pose.unlock();

  double roll, pitch, yaw;
  getRPYFromQuaternionMSG(start_pose.pose.orientation, roll, pitch, yaw);

  int yaw_degrees = angles::to_degrees(yaw);

  // ROS_INFO_STREAM("Current yaw float: " << angles::to_degrees(yaw) << " int " << yaw_degrees);

  geometry_msgs::TwistStamped twist_cmd;

  /** Fallback is position controller based, so it is in world coordinates.
      Calculate the approximate trajectory in world coordinates based in current pose.
      These calculations were made separately by hand.
      Because fallback has to always happen AWAY from the users face
    */
  // y = mx + c
  double m = 0.004444444;
  double velx_c = 0.8;
  double vely_c = 0.4;


  /// Fallback velocity functions
  ///	Vel.x				                                  Vel.y 
  /// -180 to -90	  y =  0.004444444*x + 0.8			  -180 to   0	y =  0.004444444*x + 0.4
  ///  -90 to  90	  y = -0.004444444*x 			           0 to 180	y = -0.004444444*x + 0.4
  ///   90 to 180	  y =  0.004444444*x - 0.8				

  if (yaw_degrees >= -180 && yaw_degrees < -90)
  {
    //ROS_INFO_STREAM("Sector 1");
    twist_cmd.twist.linear.x = m * yaw_degrees + velx_c;
    twist_cmd.twist.linear.y = m * yaw_degrees + vely_c;
  }  
  else if (yaw_degrees >= -90 &&  yaw_degrees <0)
  {
    //ROS_INFO_STREAM("Sector 2");
    twist_cmd.twist.linear.x = -m * yaw_degrees;
    twist_cmd.twist.linear.y = m * yaw_degrees + vely_c;
  }
  else if (yaw_degrees >= 0 &&  yaw_degrees <90)
  {
    //ROS_INFO_STREAM("Sector 3");
    twist_cmd.twist.linear.x = -m * yaw_degrees;
    twist_cmd.twist.linear.y = -m * yaw_degrees + vely_c;
  }
  else if (yaw_degrees >= 90 &&  yaw_degrees <=180)
  {
    //ROS_INFO_STREAM("Sector 4");
    twist_cmd.twist.linear.x = m * yaw_degrees - velx_c;
    twist_cmd.twist.linear.y = -m * yaw_degrees + vely_c;
  }
  //ROS_INFO_STREAM("Publishing twist : ");
  //std::cout << (twist_cmd) << std::endl;
  if (emerg==true) twist_cmd.twist.angular.x = -1;
  publishTwistForDuration(twist_cmd,1);
}

/**
 * @brief The CSVReader class. Wraps a method to read data from a CSV file on disk.
 */
class CSVReader
{
    std::string fileName;
    std::string delimeter;

public:
    CSVReader(std::string filename, std::string delm = ",") :
        fileName(filename), delimeter(delm)
    { }

    // Function to fetch data from a CSV File
    std::vector<std::vector<std::string> > getData();
};

/**
 * @brief CSVReader::getData. Get generic data from a CSV file on disk.
 * @return
 */
std::vector<std::vector<std::string> > CSVReader::getData()
{
    std::ifstream file(fileName);

    std::vector<std::vector<std::string> > dataList;

    std::string line = "";
    // Iterate through each line and split the content using delimeter
    while (getline(file, line))
    {
        std::vector<std::string> vec;
        boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
        dataList.push_back(vec);
    }
    // Close the File
    file.close();

    return dataList;
}

/**
 * @brief getPolicyFromCsv. get specifically the policy data from a CSV file on disk.
 * @param file
 * @return
 */
std::vector<int> getPolicyFromCsv(std::string file)
{
    std::vector<int> policy;
    /*
  * A class to read data from a csv file.
  */
   CSVReader reader(file);

   std::vector<std::vector<std::string> > dataList = reader.getData();

   // Print the content of row by row on screen
   for(std::vector<std::string> vec : dataList)
   {
     for(std::string data : vec)
     {
      // std::cout<<data << " , ";
       policy.push_back( std::stoi(data));
     }
     std::cout<<std::endl;
   }
    return policy;
}

/**
 * @brief calculateCurrentState. Calculate the current state based on force and step count
 * @param force. Obained from force sensor.
 * @param step. Internally tracked.
 * @return The state.
 */
int calculateCurrentState(double force, int step)
{
  int force_state;

  if (force >= 0 && force < FORCE_F_1_2_THRESH) force_state = 0;
  else if (force >= FORCE_F_1_2_THRESH && force < FORCE_F_2_3_THRESH) force_state = 1;
  else force_state = 2;

  int current_state = (3 * step) + force_state;

  return current_state;
}

/**
 * @brief selectActivePolicy. Select the policy to use based on the chosen algorithm.
 * @param algorithm. Contains the name of selected algorithm.
 * @param argv. Contains the path to all policy files.
 * @return
 */
std::vector<int> selectActivePolicy(std::string algorithm, char** argv)
{
  if(algorithm == "pi")
  {
    ROS_INFO_STREAM("Using optimal policy for " << algorithm << " from file " << argv[1]);
    return getPolicyFromCsv(argv[1]);
  }
  else if(algorithm == "vi")
  {
    ROS_INFO_STREAM("Using optimal policy for " << algorithm << " from file " << argv[2]);
    return getPolicyFromCsv(argv[2]);
  }
  else if(algorithm == "q")
  {
    ROS_INFO_STREAM("Using optimal policy for " << algorithm << " from file " << argv[3]);
    return getPolicyFromCsv(argv[3]);
  }
  else if(algorithm == "sarsa")
  {
    ROS_INFO_STREAM("Using optimal policy for " << algorithm << " from file " << argv[4]);
    return getPolicyFromCsv(argv[4]);
  }
  else
  {
    ROS_ERROR_STREAM("Wrong algorithm selected"<< algorithm <<". Choices: pi, vi, q sarsa (case sensitive)");
    ros::shutdown();
  }
}

/**
 * @brief audioGrabber. Get the result of the audio_emergency node.
 * @param msg
 */
void audioGrabber(audio_emergency::AudioMessage::ConstPtr msg)
{
  lock_emerg.lock();
  if (msg->result == "talk") emergency = true;
  lock_emerg.unlock();
}

/**
 * @brief safePauseFor. Function used to puase for a duration, while still maintianing all the safety checks.
 * @param duration
 */
void safePauseFor(float duration)
{
  ros::Time start = ros::Time::now();
  ros::Rate loop_rate(10);
  while(ros::Time::now()-start < ros::Duration(duration))
  {
    ros::spinOnce();
    if (!isAudioSafe() || !isForceSafe())
    {
      ROS_WARN_STREAM("ABORTING SEQUENCE!!");
      fallback(true);
      ros::shutdown();
    }
    loop_rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feeder_policy_drive");
  ros::NodeHandle nh;

  tf::TransformListener tf_listener;


  ros::Subscriber sub = nh.subscribe("/force_values", 1000, forceGrabber);
  cmd_vel = nh.advertise<geometry_msgs::TwistStamped>("/RobotControl/VelocityControl", 1000);
  cmd_pos = nh.advertise<geometry_msgs::PoseStamped>("/RobotControl/PoseControl", 1000);
  ros::Subscriber tool_pose = nh.subscribe("/j2s7s300_driver/out/tool_pose",1000, poseGrabber );
  ros::Subscriber control_status = nh.subscribe("/RobotControl/Status",1000, statusGrabber );
  ros::Publisher arm_pose_pub = nh.advertise<std_msgs::Int32>("/arm_state", 1000);
  ros::Subscriber audio_emerg = nh.subscribe("/audio_emergency",1000, audioGrabber );

  /** Important to wait here until all topics have valid data*/
  waitForPoseDataAvailable();

  double local_force_f;
  bool print_once_only=true;

  /** Initial rotation to ensure the user can comfortably press the force sensor*/
  ROS_INFO_STREAM("Initial rotation from " << angles::to_degrees(getCurrentRoll()));
  moveCup(RAISE_CUP, VEL_CMD_DURATION);
  ros::Duration(1.0).sleep(); // Allow inertial settlement
  ros::spinOnce();

  lock_pose.lock();
  initial_pose=current_pose;
  lock_pose.unlock();

  /** Set lower angle threshold based on starting angle*/
  double r,p,y;
  getRPYFromQuaternionMSG(initial_pose.pose.orientation,r,p,y);
  lower_angle_thresh = r;

  /**  Dynamically calculate the rotation angle per step based on the angle limits and number of arm states per the markov model*/
  rotation_step = (UPPER_FEED_ANGLE_THRESH - angles::to_degrees(lower_angle_thresh))/NUMBER_OF_ARM_SUB_STATES;

  std::vector<int> active_policy = selectActivePolicy(argv[5], argv);

  ROS_INFO_STREAM("Lower angle thresh : " << (int)angles::to_degrees(lower_angle_thresh) << ". Upper angle thresh : " << UPPER_FEED_ANGLE_THRESH);
  ROS_INFO_STREAM("No. of arm substates :" <<  NUMBER_OF_ARM_SUB_STATES << ". Rotation per step: " << rotation_step << " Max roll : "
                  << angles::to_degrees(lower_angle_thresh) + (rotation_step * (NUMBER_OF_ARM_SUB_STATES-1))) ;
  ROS_INFO_STREAM("Policy to be used :");
  for (int action:active_policy) std::cout << action << " ";
  std::cout << std::endl;

  /** Internally track the step count as robot arm sub state*/
  step_count = 0;
  int prev_step_count = 0;
  ros::Rate loop_rate(10);

  /**  Main control loop*/
  while(ros::ok())
  {
    ros::spinOnce();

    lock_force.lock();
    local_force_f=force_f;
    lock_force.unlock();

    int state = calculateCurrentState(force_f, step_count);
    int action = active_policy[state];

    switch(action)
    {
      case ACTION_DOWN:
      if(!checkLowerAngleThreshold())
      {
        if(prev_step_count > step_count)
        {
          callFallbackTimer(3);
          break;
        }
        ROS_WARN_STREAM("LOWER THRESHOLD REACHED!!");
        break;
      }
      ROS_INFO("---------------------------------------------------------------------");
      ROS_INFO_STREAM("Current state : "<< state << " (" <<  force_f   <<"N) " << " Selected action : " << action);
      driveToRollGoalWithVelocity(LOWER_CUP);
//      ros::Duration(1.0).sleep(); // Allow inertial settlement
      safePauseFor(1.0);
      prev_step_count = step_count--;
      ros::spinOnce(); // Update subscribed values ie: current pose
      ROS_WARN_STREAM("Step : " << prev_step_count << " -> " << step_count  << " @ roll : " << angles::to_degrees(getCurrentRoll()));
      print_once_only=true;
      ROS_INFO("---------------------------------------------------------------------");
      ROS_INFO(" ");

      if ((prev_step_count > step_count &&
          std::fabs((int)angles::to_degrees(getCurrentRoll()) - (int)angles::to_degrees(lower_angle_thresh)) <=  0.75*rotation_step) ||
          (prev_step_count == 1 && step_count == 0 ))
      {
        callFallbackTimer(3);
        break;
      }



      case ACTION_STAY:
      if (print_once_only)
          {
            ROS_INFO("---------------------------------------------------------------------");
            ROS_INFO_STREAM("Current state : "<< state << " (" <<  force_f   <<"N) " << " Selected action : " << action);
            ROS_INFO_STREAM("Do nothing.");
            print_once_only=false;
            ros::spinOnce(); // Update subscribed values ie: current pose
            ROS_WARN_STREAM("Step : " << prev_step_count << " -> " << step_count  << " @ roll : " << angles::to_degrees(getCurrentRoll()));
            ROS_INFO("---------------------------------------------------------------------");
            ROS_INFO(" ");
          }
      break;



      case ACTION_UP:
      if(!checkUpperAngleThreshold())
      {
        ROS_INFO_STREAM("UPPER THRESHOLD REACHED!!");
        break;
      }
      ROS_INFO("---------------------------------------------------------------------");
      ROS_INFO_STREAM("Current state : "<< state << " (" <<  force_f   <<"N) " << " Selected action : " << action);
      driveToRollGoalWithVelocity(RAISE_CUP);
//      ros::Duration(1.0).sleep(); // Allow inertial settlement
      safePauseFor(1.0);
      prev_step_count =  step_count++;
      ros::spinOnce(); // Update subscribed values ie: current pose
      ROS_WARN_STREAM("Step : " << prev_step_count << " -> " << step_count  << " @ roll : " << angles::to_degrees(getCurrentRoll()));
      print_once_only=true;
      ROS_INFO("---------------------------------------------------------------------");
      ROS_INFO(" ");
      break;
    }

    loop_rate.sleep();
    ros::spinOnce();
    std_msgs::Int32 arm_pose_msg;
    arm_pose_msg.data=step_count;
    arm_pose_pub.publish(arm_pose_msg);
    if (!isAudioSafe())
    {
      ROS_WARN_STREAM("ABORTING SEQUENCE!!");
      fallback(true);
      ros::shutdown();
    }


  }
  ros::spinOnce();
  return 0;
}
