/**
  * Deprecated node. See README.md for more info. Functions that have descriptions in active nodes are not described here.
  */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "mutex"
#include "angles/angles.h"


#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"

#include <hri_package/Sens_Force.h>
#include <kinova_msgs/PoseVelocity.h>


#define ARC_DOWN 1
#define ARC_UP 2
#define ROTATE_DOWN 3
#define ROTATE_LEFT 4
#define ROTATE_RIGHT 5
#define TRANSLATE_UP 6
#define TRANSLATE_DOWN 7
#define TRANSLATE_FRONT 8
#define TRANSLATE_BACK 9
#define TRANSLATE_LEFT 10
#define TRANSLATE_RIGHT 11
#define ARC_LEFT 12
#define ARC_RIGHT 13
#define END_EFF_FRAME "j2s7s300_end_effector"
#define BASE_FRAME "j2s7s300_link_base"
#define SENSOR_FRAME "forcesensor"
#define FORCE_F_TRIGGER_THRESH 0.4
#define FORCE_B_TRIGGER_THRESH 0.4
#define FORCE_BOTH_FALLBACK 1.0
#define FORCE_F_PAIN_THRESH 2.5
#define FORCE_B_PAIN_THRESH 2.5
#define ROTATION_STEP 10
#define MAX_STEPS 200
#define VEL_LIN_MAX 0.04
#define VEL_ANG_MAX 0.4
#define VEL_CMD_DURATION 1
#define UPPER_FEED_ANGLE_THRESH 80
#define LOWER_FEED_ANGLE_THRESH 0

float thresh_lin = 0.01;
float thresh_ang = 0.15;

static int seq=0;

std::string result;
std::mutex lock_pose;
std::mutex lock_status;
std::mutex lock_proc;
std::mutex lock_force;
double force_f, force_b;
bool processing=false;
ros::Publisher cmd_pos;
ros::Publisher cmd_vel;
geometry_msgs::PoseStamped current_pose, initial_pose;



void getRPYFromQuaternionMSG(geometry_msgs::Quaternion orientation, double& roll,double& pitch, double& yaw)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(orientation,quat);
  quat.normalize();
  tf::Matrix3x3 mat(quat);
  mat.getRPY(roll, pitch,yaw);
}


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


void setPoseForDirection(int direction, geometry_msgs::PoseStamped& start_pose,double  distance)
{
  geometry_msgs::Quaternion quat;
  switch(direction)
  {
  /// END EFFECTOR FRAME  Z-forward-blue, Y-up-green, X-left-red
  case ARC_DOWN :
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(ROTATION_STEP),angles::from_degrees(0),angles::from_degrees(0)); //Turn down
    start_pose.header.frame_id=END_EFF_FRAME;
    start_pose.pose.position.x=0;
    start_pose.pose.position.y=0.01;  // Slightly up
    start_pose.pose.position.z=-0.01; // Slightly back
    start_pose.pose.orientation = quat;
    break;
  case ARC_UP :
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(-ROTATION_STEP),angles::from_degrees(0),angles::from_degrees(0));
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
  case TRANSLATE_RIGHT :
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(0),angles::from_degrees(0),angles::from_degrees(0));
    start_pose.header.frame_id=END_EFF_FRAME;
    start_pose.pose.position.x=-distance;
    start_pose.pose.position.y=0;
    start_pose.pose.position.z=0;
    start_pose.pose.orientation = quat;
    break;
  case TRANSLATE_LEFT :
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(0),angles::from_degrees(0),angles::from_degrees(0));
    start_pose.header.frame_id=END_EFF_FRAME;
    start_pose.pose.position.x=distance;
    start_pose.pose.position.y=0;
    start_pose.pose.position.z=0;
    start_pose.pose.orientation = quat;
    break;

  /// SENSORFRAME  X-forward-red, Y-left-green, Z-up-blue
  }
}

geometry_msgs::TwistStamped getTwistForDirection(int direction)
{
  geometry_msgs::TwistStamped twist;
  twist.header.stamp=ros::Time::now();
  twist.header.frame_id="j2s7s300_link_7";
  twist.header.seq=seq++;
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

  case TRANSLATE_LEFT:
 //   ROS_INFO("Translate left");
    twist.twist.linear.x=VEL_LIN_MAX;
    break;

  case TRANSLATE_RIGHT:
//    ROS_INFO("Translate right");
    twist.twist.linear.x=-VEL_LIN_MAX;
    break;

  case ROTATE_DOWN:
    ROS_INFO("Rotate down");
    twist.twist.angular.x=VEL_ANG_MAX;
    break;

  case TRANSLATE_UP:
 //   ROS_INFO("Translate up");
    twist.twist.linear.z=VEL_LIN_MAX;
    break;

  case TRANSLATE_DOWN:
//    ROS_INFO("Translate down");
    twist.twist.linear.z=-VEL_LIN_MAX;
    break;

  case ROTATE_LEFT:
    ROS_INFO("Rotate left");
    twist.twist.angular.z=-VEL_ANG_MAX;
    break;

  case ROTATE_RIGHT:
    ROS_INFO("Rotate right");
    twist.twist.angular.z=VEL_ANG_MAX;
    break;

  case ARC_LEFT: //  Translate up, translate right, rotate left
    ROS_INFO("Arc left: Feed water");
    twist.twist.linear.z=VEL_LIN_MAX;
   // twist.twist.linear.x=-VEL_LIN_MAX;
    twist.twist.angular.z=-VEL_ANG_MAX;
    break;

  case ARC_RIGHT: //  Translate down, translate left, rotate right
    ROS_INFO("Arc right: Retreat");
    twist.twist.linear.z=-VEL_LIN_MAX;
   // twist.twist.linear.x=VEL_LIN_MAX;
    twist.twist.angular.z=VEL_ANG_MAX;
    break;

  }
  return twist;
}

bool isForceSafe()
{
  double local_force_f,local_force_b;
  lock_force.lock();
  local_force_f=force_f;
  local_force_b=force_b;
  lock_force.unlock();

  if (local_force_f >= FORCE_F_PAIN_THRESH || local_force_b >= FORCE_F_PAIN_THRESH)
    return false;
  else
    return true;
}

void publishTwistForDuration(geometry_msgs::TwistStamped twist_msg, double duration)
{
  ros::Time time_start = ros::Time::now();
  while (ros::Time::now() - time_start < ros::Duration(duration))
  {
    ros::spinOnce();
    if(isForceSafe())
    {
      cmd_vel.publish(twist_msg);
    }
    else
    {
      ROS_WARN("PAIN THRESHOLD BREACHED, ABORTING SEQUENCE!!!");
      break;
    }
  }
}

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


      setPoseForDirection(direction, start_pose, distance);

      tf_listener.transformPose(BASE_FRAME,start_pose,pose_in_base);
      break;

    }
    catch (tf::ExtrapolationException e)
    {
//      ROS_INFO_STREAM("Waiting for transform "
//                      << e.what()
//                      );
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  cmd_pos.publish(pose_in_base);
}

void moveCup(int direction, double duration=VEL_CMD_DURATION, double distance=0.1)
{
  //ROS_INFO_STREAM("Calling moveCup. Direction : " << direction << " Duration : " << duration << " Distance: " << distance);
  if (direction==TRANSLATE_BACK
      || direction==TRANSLATE_FRONT
      //|| direction==TRANSLATE_LEFT
      //|| direction==TRANSLATE_RIGHT
      )
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

void fallBack(geometry_msgs::PoseStamped initial_pose)
{
  cmd_pos.publish(initial_pose);
  waitForActionCompleted();
  moveCup(TRANSLATE_RIGHT, VEL_CMD_DURATION*3);
}

void poseGrabber(geometry_msgs::PoseStamped pose)
{
  lock_pose.lock();
  current_pose=pose;
  lock_pose.unlock();
}

void forceGrabber(const hri_package::Sens_Force::ConstPtr msg)
{
  lock_force.lock();
  force_f=msg->forceF;
  force_b=msg->forceB;
  lock_force.unlock();
}

void statusGrabber(std_msgs::String::ConstPtr status)
{
  lock_status.lock();
  result=status->data;
  lock_status.unlock();
}

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

  if ( angles::to_degrees(temp_pitch) > UPPER_FEED_ANGLE_THRESH )
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

  if (angles::to_degrees(temp_pitch) < LOWER_FEED_ANGLE_THRESH )
  {
    ROS_WARN_STREAM("MAX LOWER FEED ANGLE REACHED");
    return false;
  }
  else
  {
   return true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feeder_sideways");
  ros::NodeHandle nh;

  tf::TransformListener tf_listener;

  ros::Subscriber sub = nh.subscribe("/force_values", 1000, forceGrabber);
  cmd_vel = nh.advertise<geometry_msgs::TwistStamped>("/RobotControl/VelocityControl", 1000);
  cmd_pos = nh.advertise<geometry_msgs::PoseStamped>("/RobotControl/PoseControl", 1000);
  ros::Subscriber tool_pose = nh.subscribe("/j2s7s300_driver/out/tool_pose",1000, poseGrabber );
  ros::Subscriber control_status = nh.subscribe("/RobotControl/Status",1000, statusGrabber );

  ros::Rate loop_rate(9);

  waitForPoseDataAvailable();

  double local_force_f,local_force_b;
  bool print_once_only=true;

  lock_pose.lock();
  initial_pose=current_pose;
  lock_pose.unlock();

  /** Initialy rotation for user comfort */
//  moveCup(TRANSLATE_UP, VEL_CMD_DURATION*0.5);
  moveCup(TRANSLATE_RIGHT, VEL_CMD_DURATION*0.2);
  moveCup(ARC_LEFT, VEL_CMD_DURATION);



  while(ros::ok())
  {
    ros::spinOnce();

    lock_force.lock();
    local_force_f=force_f;  //Interchange sensors so that it makes practical sense
    local_force_b=force_b;
    lock_force.unlock();

    ///MAIN CONTROL LOOP
    if (local_force_f >= FORCE_F_TRIGGER_THRESH && local_force_b <FORCE_B_TRIGGER_THRESH && checkUpperAngleThreshold())
    {
      moveCup(TRANSLATE_UP, VEL_CMD_DURATION*0.2);
      moveCup(TRANSLATE_RIGHT, VEL_CMD_DURATION*0.1);
      moveCup(ARC_LEFT, VEL_CMD_DURATION);
      print_once_only=true;
    }
    else if (local_force_f <FORCE_F_TRIGGER_THRESH && local_force_b>=FORCE_B_TRIGGER_THRESH && checkLowerAngleThreshold())
    {
      moveCup(TRANSLATE_DOWN, VEL_CMD_DURATION*0.2);
      moveCup(TRANSLATE_LEFT, VEL_CMD_DURATION*0.1);
      moveCup(ARC_RIGHT, VEL_CMD_DURATION);
      print_once_only=true;
    }
   // else if (local_force_f >= FORCE_F_TRIGGER_THRESH && local_force_b >=FORCE_B_TRIGGER_THRESH)
     else if (local_force_f >= FORCE_BOTH_FALLBACK && local_force_b >=FORCE_BOTH_FALLBACK)
    {
      ROS_WARN("FALLBACK INITIATED, CLOSING NODE");
      fallBack(initial_pose);
      break;
    }
    else if (print_once_only)
    {
      ROS_INFO_STREAM("Do nothing.");
      print_once_only=false;
    }

    loop_rate.sleep();
  }



  ros::spinOnce();

  return 0;
}
