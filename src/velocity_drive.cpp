/// DRIVE TO POSE


#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "mutex"
#include "kinova_msgs/PoseVelocity.h"
#include "tf/transform_datatypes.h"
#include "angles/angles.h"

#define VEL_LIN_MAX 0.04
#define VEL_ANG_MAX 0.4

//float thresh_lin = 0.01;
//float thresh_ang = 0.05;

//double temp_rol, temp_pit, temp_yaw;
//double goal_rol, goal_pit, goal_yaw;
//double start_rol, start_pit, start_yaw;
//double del_rol, del_pit, del_yaw;


//std::string result;
//std::mutex lock_pose;
//std::mutex lock_status;
//std::mutex lock_proc;
//std::mutex lock_force;
//double force_f, force_b;
//bool processing=false;
//ros::Publisher cmd_pos;
//ros::Publisher cmd_vel;

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "mutex"
#include "angles/angles.h"
#include "std_msgs/Int32.h"


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
#define FORCE_F_1_2_THRESH 0.2
#define FORCE_F_2_3_THRESH 1.0
#define ROTATION_STEP 10
#define MAX_STEPS 200
#define VEL_LIN_MAX 0.04
#define VEL_ANG_MAX 0.4
#define VEL_CMD_DURATION 0.8
#define UPPER_FEED_ANGLE_THRESH 140 // was 140

#define ACTION_DOWN 0
#define ACTION_STAY 1
#define ACTION_UP 2


double lower_angle_thresh = 85;
float thresh_lin = 0.01;
float thresh_ang = 0.05;

int step_count;

std::string result;
std::mutex lock_pose;
std::mutex lock_status;
std::mutex lock_proc;
std::mutex lock_force;
double force_f, force_b;
bool processing=false;
ros::Publisher cmd_pos;
ros::Publisher cmd_vel;
ros::Publisher pub_kinova_vel;

geometry_msgs::PoseStamped current_pose, start_pose, goal_pose;
std::mutex pose_lock;

void getRPYFromQuaternionMSG(geometry_msgs::Quaternion orientation, double& roll,double& pitch, double& yaw)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(orientation,quat);
  quat.normalize();
  tf::Matrix3x3 mat(quat);
  mat.getRPY(roll, pitch,yaw);

  //ROS_INFO_STREAM("Quaternion is : " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w());
  //ROS_INFO_STREAM("Converted R: " << roll << " P: " << pitch << " Y: " << yaw);
}

void printRoll(std::string text)
{
  ros::spinOnce();
  lock_pose.lock();
  geometry_msgs::PoseStamped initial_pose=current_pose;
  lock_pose.unlock();

  double r,p,y;
  getRPYFromQuaternionMSG(initial_pose.pose.orientation,r,p,y);
  lower_angle_thresh = r;
  ROS_INFO_STREAM(text << angles::to_degrees(lower_angle_thresh));
}

void printPitch(std::string text)
{
  ros::spinOnce();
  lock_pose.lock();
  geometry_msgs::PoseStamped initial_pose=current_pose;
  lock_pose.unlock();

  double r,p,y;
  getRPYFromQuaternionMSG(initial_pose.pose.orientation,r,p,y);
  lower_angle_thresh = p;
  ROS_INFO_STREAM(text << angles::to_degrees(lower_angle_thresh));
}


void printRPYCurrent(std::string text)
{
  ros::spinOnce();
  lock_pose.lock();
  geometry_msgs::PoseStamped initial_pose=current_pose;
  lock_pose.unlock();

  double r,p,y;
  getRPYFromQuaternionMSG(initial_pose.pose.orientation,r,p,y);

  ROS_INFO_STREAM(text << " R: " << angles::to_degrees(r)
                  << " P: " << angles::to_degrees(p)
                  << " Y: " << angles::to_degrees(y));
}

void printRPY(std::string text, double roll, double pitch, double yaw)
{
  ROS_INFO_STREAM(text << " R: " << angles::to_degrees(roll)
                  << " P: " << angles::to_degrees(pitch)
                  << " Y: " << angles::to_degrees(yaw));
}


void poseGrabber(geometry_msgs::PoseStamped pose)
{
  pose_lock.lock();
  current_pose = pose;
  pose_lock.unlock();
  //ROS_INFO_STREAM("Got current pose as x=" << pose << " saved as " << current_pose);
}


void publishTwistForDuration(geometry_msgs::TwistStamped twist_msg, double duration)
{
  ros::Time time_start = ros::Time::now();
  while (ros::Time::now() - time_start < ros::Duration(duration))
  {
    ros::spinOnce();
    cmd_vel.publish(twist_msg);
  }
}

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

  tf::Quaternion q_rot = tf::createQuaternionFromRPY(angles::from_degrees( direction==RAISE_CUP?ROTATION_STEP:-ROTATION_STEP),angles::from_degrees(0),angles::from_degrees(0)); // Rotate about x by 20 degrees
  tf::Quaternion q_start; tf::quaternionMsgToTF(start_pose.pose.orientation, q_start);
  tf::Quaternion q_goal = q_start*q_rot;

  tf::quaternionTFToMsg(q_goal,goal_pose.pose.orientation);


  ros::Rate loop_rate(100);
  //ros::Time start = ros::Time::now();
  while(ros::ok())
  {
    ros::spinOnce();
    lock_pose.lock();
    temp_pose = current_pose;
    lock_pose.unlock();

    getRPYFromQuaternionMSG(goal_pose.pose.orientation, goal_rol, goal_pit, goal_yaw);
    getRPYFromQuaternionMSG(temp_pose.pose.orientation, temp_rol, temp_pit, temp_yaw);
    getRPYFromQuaternionMSG(start_pose.pose.orientation, start_rol, start_pit, start_yaw);

    if (goal_rol < 0 ) goal_rol = goal_rol + angles::from_degrees(360);
    if (temp_rol < 0 ) temp_rol = temp_rol + angles::from_degrees(360);

    del_rol = goal_rol - temp_rol;

    if (std::fabs(del_rol)>=thresh_ang)
    {
      geometry_msgs::TwistStamped twist_msg;
      twist_msg.twist.linear.x = 0;
      twist_msg.twist.linear.y = 0;
      twist_msg.twist.linear.z = 0;
      twist_msg.twist.angular.x= del_rol>0?VEL_ANG_MAX:-VEL_ANG_MAX;
      twist_msg.twist.angular.y = 0;
      twist_msg.twist.angular.z= 0;

      ROS_INFO_STREAM("goal_rol: " << angles::to_degrees(goal_rol) << " temp_rol : " << angles::to_degrees(temp_rol) << " del_rol: " << del_rol);
      cmd_vel.publish(twist_msg);
    }
    else
    {
      //ROS_INFO_STREAM("Roll: " << angles::to_degrees(temp_rol)  << " Delta: " << del_rol << " Goal reached, breaking loop");
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

}

void driveToPitchGoalWithVelocity(int direction)
{
  if(direction == RAISE_CUP)
    ROS_INFO_STREAM("Raise Cup");
  else if (direction == LOWER_CUP)
    ROS_INFO_STREAM("Lower Cup");
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
  double del_pitch;

  goal_pose = start_pose;

  tf::Quaternion q_rot = tf::createQuaternionFromRPY(angles::from_degrees(0),angles::from_degrees(0),angles::from_degrees( direction==RAISE_CUP?-ROTATION_STEP:ROTATION_STEP)); // Change Pitch angle
  tf::Quaternion q_start; tf::quaternionMsgToTF(start_pose.pose.orientation, q_start);
  tf::Quaternion q_goal = q_start*q_rot;

  tf::quaternionTFToMsg(q_goal,goal_pose.pose.orientation);

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

    printRPY("goal", goal_rol, goal_pit, goal_yaw);
    printRPY("temp", temp_rol, temp_pit, temp_yaw);
    printRPY("start", start_rol, start_pit, start_yaw);

    del_pitch = goal_pit - temp_pit;
    ROS_INFO_STREAM("Goal_P: " << angles::to_degrees(goal_pit) << " Temp_P: " << angles::to_degrees(temp_pit) << " Del_P: " << del_pitch << " Thresh_A: " << thresh_ang);

    if (std::fabs(del_pitch)>=thresh_ang)
    {
      ROS_INFO_STREAM("Publishing velocity command");
      geometry_msgs::TwistStamped twist_msg;
      twist_msg.twist.linear.x = 0;
      twist_msg.twist.linear.y = 0;
      // Comment out next line ONLY if using end-effector mode ie: Mode 1 on jostick is active and 4th blue LED on far right is active.
//      twist_msg.twist.linear.z = direction == LOWER_CUP? -thresh_lin : thresh_lin;
      twist_msg.twist.angular.x = 0;
      twist_msg.twist.angular.z= del_pitch>0?-VEL_ANG_MAX:VEL_ANG_MAX;
      twist_msg.twist.angular.y= 0;

      cmd_vel.publish(twist_msg);

    }
    else
    {
      ROS_INFO_STREAM("Goal reached, breaking loop");
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_drive");
  ros::NodeHandle nh;

  //ros::Publisher cmd_vel = nh.advertise<geometry_msgs::TwistStamped>("/RobotControl/VelocityControl", 1000);
  cmd_vel = nh.advertise<geometry_msgs::TwistStamped>("/RobotControl/VelocityControl", 1000);
  pub_kinova_vel = nh.advertise<kinova_msgs::PoseVelocity>("/j2s7s300_driver/in/cartesian_velocity", 1000);
  ros::Subscriber tool_pose = nh.subscribe("/j2s7s300_driver/out/tool_pose",1000, poseGrabber );

  ros::Rate loop_rate(100);
  geometry_msgs::PoseStamped temp_pose;

  geometry_msgs::TwistStamped vel_msg;


  // This loop to wait until subscriber starts returning valid poses.
  while(ros::ok())
  {
    pose_lock.lock();
    temp_pose = current_pose;
    pose_lock.unlock();
    ros::spinOnce();
    loop_rate.sleep();
    if (temp_pose.pose.position.x > 0) break;
  }

  ROS_INFO("Valid pose available");

  printPitch("Pitch before change ");
  driveToPitchGoalWithVelocity(LOWER_CUP);
  printPitch("Pitch after change, pre pause ");
//  sleep(1.0);
//  printPitch("Pitch after change, post pause ");




  ros::spinOnce();
  return 0;
}
