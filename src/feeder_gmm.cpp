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


#include <ros/console.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "mutex"
#include "angles/angles.h"
#include "std_msgs/Int32.h"
#include "audio_emergency/AudioMessage.h"
#include <trac_ik/trac_ik.hpp>

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

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>


#define FORCE_F_1_2_THRESH 0.05
#define FORCE_F_2_3_THRESH 0.5
#define NUMBER_OF_ARM_SUB_STATES 10
#define UPPER_FEED_ANGLE_THRESH 180.00 // was 140
#define FORCE_SAFETY 5
#define SUB_SAMPLED_SIZE 10

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

void getRPYFromQuaternionMSG(geometry_msgs::Quaternion orientation, double& roll,double& pitch, double& yaw)
{
  tf::Quaternion quat;
  // Suppress a weird warning that tells me to normalise the quats even though I do that.
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error) ) ros::console::notifyLoggerLevelsChanged();
  tf::quaternionMsgToTF(orientation,quat);
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) ros::console::notifyLoggerLevelsChanged();
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

bool checkUpperAngleThreshold();
bool checkLowerAngleThreshold();

void publishTwistForDuration(geometry_msgs::TwistStamped twist_msg, double duration)
{
  ros::Time time_start = ros::Time::now();
  while (ros::Time::now() - time_start < ros::Duration(duration))
  {
    ros::spinOnce();
    cmd_vel.publish(twist_msg);
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
     /* ROS_INFO_STREAM("Waiting for transform "
                      //<< e.what()
                      );*/
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  cmd_pos.publish(pose_in_base);
}

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

bool isAudioSafe()
{
  lock_emerg.lock();
  bool emerg = emergency;
  lock_emerg.unlock();

  if (emerg) ROS_WARN_STREAM("AUDIO EMERGENCY DETECTED!!");
  return !emerg;
}

void fallback(bool emerg=false);

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

  tf::Quaternion q_rot = tf::createQuaternionFromRPY(angles::from_degrees( direction==RAISE_CUP?rotation_step:-rotation_step),angles::from_degrees(0),angles::from_degrees(0)); // Rotate about x by 20 degrees
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

double getCurrentRoll()
{
  lock_pose.lock();
  geometry_msgs::PoseStamped temp_pose = current_pose;
  lock_pose.unlock();

  double temp_rol, temp_pitch, temp_yaw;
  getRPYFromQuaternionMSG(temp_pose.pose.orientation, temp_rol, temp_pitch, temp_yaw);


  return temp_rol<0?temp_rol+angles::from_degrees(360):temp_rol;
}

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

std::vector<std::vector<float>> getGMRFromCsv(std::string file)
{

   CSVReader reader(file);

   std::vector<std::vector<std::string> > dataList = reader.getData();

   int RR = dataList.size();
   int CC = dataList[0].size();
    std::vector<std::vector<float>> gmr(RR);
    for ( int i = 0 ; i < RR ; i++ )
       gmr[i].resize(CC);

  ROS_INFO_STREAM("Fetching GMR data from " << file );
  for (int i=0; i<dataList.size(); i++)
   {
     std::vector<std::string> vec = dataList[i];
     for(int j=0; j< vec.size();j++ )
     {
       gmr[i][j]= std::stof(vec[j]);
     }
   }
    return gmr;
}


/**
 * @brief Calculate the delta pose vector i.e.: the difference of each pose from the first pose
 * @param gmr
 * @return pose_delta
 */
std::vector<std::vector<float>> getPoseDeltaFromGMR(std::vector<std::vector<float>> gmr)
{


   int RR = gmr.size();
   int CC = 6; // 6 columns for deltas in x,y,z,roll,pitch,yaw
   std::vector<std::vector<float>> pose_delta(RR);
   for ( int i = 0 ; i < RR ; i++ )
      pose_delta[i].resize(CC);


   geometry_msgs::PoseStamped first_pose;
   first_pose.pose.position.x = gmr[0][1]; //columen 0 is index number, dropping it
   first_pose.pose.position.y = gmr[0][2];
   first_pose.pose.position.z = gmr[0][3];
   first_pose.pose.orientation.x = gmr[0][4];
   first_pose.pose.orientation.y = gmr[0][5];
   first_pose.pose.orientation.z = gmr[0][6];
   first_pose.pose.orientation.w = gmr[0][7];
   double roll_first, pitch_first, yaw_first;
   getRPYFromQuaternionMSG(first_pose.pose.orientation, roll_first, pitch_first, yaw_first);



// First row/pose is always 0 since it is the reference point.
   for (int i=0; i<RR; i++)
   {
     geometry_msgs::PoseStamped current_pose;
     current_pose.pose.position.x = gmr[i][1];  //columen 0 is index number, dropping it
     current_pose.pose.position.y = gmr[i][2];
     current_pose.pose.position.z = gmr[i][3];
     current_pose.pose.orientation.x = gmr[i][4];
     current_pose.pose.orientation.y = gmr[i][5];
     current_pose.pose.orientation.z = gmr[i][6];
     current_pose.pose.orientation.w = gmr[i][7];

     double roll_curr, pitch_curr, yaw_curr;

     getRPYFromQuaternionMSG(current_pose.pose.orientation,roll_curr, pitch_curr, yaw_curr);


     double delta_roll, delta_pitch, delta_yaw;
     delta_roll =  roll_curr - roll_first ;
     delta_pitch = pitch_curr - pitch_first ;
     delta_yaw = yaw_curr - yaw_first;

     pose_delta[i][0] = current_pose.pose.position.x - first_pose.pose.position.x;
     pose_delta[i][1] = current_pose.pose.position.y - first_pose.pose.position.y;
     pose_delta[i][2] = current_pose.pose.position.z - first_pose.pose.position.z;
     pose_delta[i][3] = delta_roll; // Forward feeding approach changes only roll. Change in pitch and yaw <1° and hence ignored. Direct addition of single euler angle works between quaternions. NOT multiple angles.
//     pose_delta[i][4] = delta_pitch;    // Even if there are pitches and yaws due to human error, they need to be suppressed to avoid_discomfort to the user/spillage_.
//     pose_delta[i][5] = delta_yaw;
   }

   return pose_delta;
}

// Take the actual starting pose from current run, consider the index of the GMR trajectory required, add the delta to take the starting pose to new index
geometry_msgs::PoseStamped getNewPoseAtIndex(geometry_msgs::PoseStamped starting_pose, int new_index_in_trajectory, std::vector<std::vector<float>> pose_delta_subsampled )
{
  geometry_msgs::PoseStamped new_pose;
  new_pose.pose.position.x = starting_pose.pose.position.x + pose_delta_subsampled[new_index_in_trajectory][0];
  new_pose.pose.position.y = starting_pose.pose.position.y + pose_delta_subsampled[new_index_in_trajectory][1];
  new_pose.pose.position.z = starting_pose.pose.position.z + pose_delta_subsampled[new_index_in_trajectory][2];

  double starting_roll, starting_pitch, starting_yaw, new_roll, new_pitch, new_yaw;
  getRPYFromQuaternionMSG(starting_pose.pose.orientation, starting_roll, starting_pitch, starting_yaw);
  new_roll = starting_roll + pose_delta_subsampled[new_index_in_trajectory][3];
  new_pitch = starting_pitch + pose_delta_subsampled[new_index_in_trajectory][4];
  new_yaw = starting_yaw+ pose_delta_subsampled[new_index_in_trajectory][5];

  tf::Quaternion q_new = tf::createQuaternionFromRPY(new_roll, new_pitch, new_yaw);
  q_new.normalize();
  tf::quaternionTFToMsg(q_new,new_pose.pose.orientation);

  return new_pose;
}


void audioGrabber(audio_emergency::AudioMessage::ConstPtr msg)
{
  lock_emerg.lock();
  if (msg->result == "talk") emergency = true;
  lock_emerg.unlock();
}

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


void printPose(geometry_msgs::PoseStamped pose, std::string text)
{
  double roll, pitch, yaw;
  getRPYFromQuaternionMSG(pose.pose.orientation, roll, pitch, yaw);
  ROS_INFO_STREAM(text << pose.pose.position.x << " "
                  << pose.pose.position.y << " "
                  << pose.pose.position.z << " "
                  << angles::to_degrees(roll) << " "
                  << angles::to_degrees(pitch) << " "
                  << angles::to_degrees(yaw) << " " );
}

void printGMRRow(std::vector<std::vector<float>> gmr, int row, std::string text)
{

    double roll, pitch, yaw;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = gmr[row][1];
    pose.pose.position.y = gmr[row][2];
    pose.pose.position.z = gmr[row][3];
    pose.pose.orientation.x = gmr[row][4];
    pose.pose.orientation.y = gmr[row][5];
    pose.pose.orientation.z = gmr[row][6];
    pose.pose.orientation.w = gmr[row][7];

    getRPYFromQuaternionMSG(pose.pose.orientation, roll, pitch, yaw);

    ROS_INFO_STREAM(text << pose.pose.position.x << " "
                    << pose.pose.position.y << " "
                    << pose.pose.position.z << " "
                    << angles::to_degrees(roll) << " "
                    << angles::to_degrees(pitch) << " "
                    << angles::to_degrees(yaw) << " " );

}

std::vector<std::vector<float>> subSamplePoseDelta(std::vector<std::vector<float>> pose_delta, int resultant_length)
{
  int sampling_freq = pose_delta.size()/resultant_length;
  std::vector<std::vector<float>> pose_delta_subsampled(resultant_length);
  // Resize the subsampled set to have the number of rows as required by resultant length but number of columns same as pose_delta
  for(int i=0; i< resultant_length; i++)
  {
    pose_delta_subsampled[i].resize(pose_delta[0].size());
  }

  // Subsample pose_delta according to sampling frequency
  for (int i=0; i<resultant_length; i++)
  {
    pose_delta_subsampled[i] = pose_delta[i*sampling_freq];
  }



  return pose_delta_subsampled;
}

void driveToPoseStep(int step_count,std::vector<std::vector<float>> pose_delta_subsampled )
{
  ros::spinOnce();

  lock_pose.lock();
  geometry_msgs::PoseStamped temp_pose=current_pose;
  lock_pose.unlock();

  ROS_INFO_STREAM("Moving to WP" << step_count);
  geometry_msgs::PoseStamped pose_new = getNewPoseAtIndex(initial_pose, step_count, pose_delta_subsampled);
  printPose(temp_pose , "Cur pose: ");
  printPose(pose_new, "New pose: ");
  cmd_pos.publish(pose_new);
  waitForActionCompleted();
  ROS_INFO_STREAM("Done");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feeder_gmm");
  ros::NodeHandle nh;

  std::vector<std::vector<float>> gmr = getGMRFromCsv(argv[1]);
  std::vector<std::vector<float>> pose_delta = getPoseDeltaFromGMR(gmr);
  std::vector<std::vector<float>> pose_delta_subsampled = subSamplePoseDelta(pose_delta, SUB_SAMPLED_SIZE);

  ROS_INFO_STREAM("Size of GMR : " << gmr.size() << " pose_delta : " << pose_delta.size() << " pose_delta_subsampled : " << pose_delta_subsampled.size());

  tf::TransformListener tf_listener;

  ros::Subscriber sub = nh.subscribe("/force_values", 1000, forceGrabber);
  cmd_vel = nh.advertise<geometry_msgs::TwistStamped>("/RobotControl/VelocityControl", 1000);
  cmd_pos = nh.advertise<geometry_msgs::PoseStamped>("/RobotControl/PoseControl", 1000);
  ros::Subscriber tool_pose = nh.subscribe("/j2s7s300_driver/out/tool_pose",1000, poseGrabber );
  ros::Subscriber control_status = nh.subscribe("/RobotControl/Status",1000, statusGrabber );
  ros::Publisher arm_pose_pub = nh.advertise<std_msgs::Int32>("/arm_state", 1000);
  ros::Subscriber audio_emerg = nh.subscribe("/audio_emergency",1000, audioGrabber );

  waitForPoseDataAvailable();

  double local_force_f;
  bool print_once_only=true;

  lock_pose.lock();
  initial_pose=current_pose;
  lock_pose.unlock();


 //TODO : Implement the driver below

  step_count = 0;
  int prev_step_count = 0;

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    ros::spinOnce();

    lock_force.lock();
    local_force_f=force_f;
    lock_force.unlock();



    if (local_force_f >= 0  && local_force_f <= FORCE_F_1_2_THRESH && step_count >0)
    {
      ROS_INFO("---------------------------------------------------------------------");
      ROS_INFO_STREAM("STEP_DOWN for " << local_force_f << "N from step " << step_count);
      prev_step_count = step_count--;
      driveToPoseStep(step_count,pose_delta_subsampled);
      print_once_only=true;
      ROS_INFO("---------------------------------------------------------------------");
      ROS_INFO(" ");

      if (prev_step_count > step_count && step_count==0)
      {
        callFallbackTimer(3);
      }
    }

    else if (local_force_f >= FORCE_F_2_3_THRESH && local_force_f <= FORCE_SAFETY && step_count < SUB_SAMPLED_SIZE-1)
    {
      ROS_INFO("---------------------------------------------------------------------");
      ROS_INFO_STREAM("STEP_UP for " << local_force_f << "N from step " << step_count);
      prev_step_count =  step_count++;
      driveToPoseStep(step_count,pose_delta_subsampled);
      print_once_only=true;
      ROS_INFO("---------------------------------------------------------------------");
      ROS_INFO(" ");
    }

    else if (print_once_only)
    {
      ROS_INFO("---------------------------------------------------------------------");
      ROS_INFO_STREAM("Do nothing.");
      print_once_only=false;
      ROS_WARN_STREAM("Step : " << prev_step_count << " -> " << step_count  << " @ roll : " << angles::to_degrees(getCurrentRoll()));
      ROS_INFO("---------------------------------------------------------------------");
      ROS_INFO(" ");

    }


    loop_rate.sleep();
    std_msgs::Int32 arm_pose_msg;
    arm_pose_msg.data=step_count;
    arm_pose_pub.publish(arm_pose_msg);
    ros::spinOnce();

    if (!isAudioSafe())
    {
      ROS_WARN_STREAM("ABORTING SEQUENCE!!");
      fallback(true);
      ros::shutdown();
    }
  }
  return 0;
}
