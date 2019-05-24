/**
  * See README.md for more info.
  */
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "mutex"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"

#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/PoseVelocity.h>

#define END_EFF_FRAME "j2s7s300_end_effector"
#define BASE_FRAME "j2s7s300_link_base"
#define SENSOR_FRAME "forcesensor"

geometry_msgs::PoseStamped current_pose, start_pose, pose_in_end_effector, pose_in_sensor;
std::string result;
std::mutex lock_pose;
std::mutex lock_status;

void poseGrabber(geometry_msgs::PoseStamped::ConstPtr pose)
{
  lock_pose.lock();
  current_pose.pose.position.x = pose->pose.position.x;
  current_pose.pose.position.y = pose->pose.position.y;
  current_pose.pose.position.z = pose->pose.position.z;
  current_pose.pose.orientation.w = pose->pose.orientation.w;
  current_pose.pose.orientation.x = pose->pose.orientation.x;
  current_pose.pose.orientation.y = pose->pose.orientation.y;
  current_pose.pose.orientation.z = pose->pose.orientation.z;
  current_pose.header.frame_id=pose->header.frame_id;
  current_pose.header.seq=pose->header.seq;
  current_pose.header.stamp=pose->header.stamp;
  lock_pose.unlock();
  /*ROS_INFO_STREAM("--------------------------------");
  ROS_INFO_STREAM("--------------------------------");
  ROS_INFO_STREAM("Got current pose as x=" << pose->pose.position.x << " saved as " << current_pose.pose.position.x);
  ROS_INFO_STREAM("--------------------------------");
  ROS_INFO_STREAM("--------------------------------");*/
}

void statusGrabber(std_msgs::String::ConstPtr status)
{
  lock_status.lock();
  result=status->data;
  lock_status.unlock();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_test");
  ros::NodeHandle nh;
  tf::TransformListener tf_listener;

  ros::Publisher cmd_pos = nh.advertise<geometry_msgs::PoseStamped>("/RobotControl/PoseControl", 1000);
  ros::Subscriber tool_pose = nh.subscribe("/j2s7s300_driver/out/tool_pose",1000, poseGrabber );
  ros::Subscriber control_status = nh.subscribe("/RobotControl/Status",1000, statusGrabber );

  ros::Rate loop_rate(9);
  geometry_msgs::PoseStamped temp_pose;

  // This loop to wait until subscriber starts returning valid poses.
  while(ros::ok())
  {
    lock_pose.lock();
    temp_pose.pose.position.x = current_pose.pose.position.x;
    lock_pose.unlock();

    ros::spinOnce();
    loop_rate.sleep();

    if (temp_pose.pose.position.x != 0) break;
  }

  ROS_INFO("Pose data available");
  bool success = false;


  //WORKING: Go from base frame to end effector frame
  //while (!success)
  while(ros::ok())
  {
    try
    {
      lock_pose.lock();
      start_pose=current_pose;
      lock_pose.unlock();


      //tf_listener.transformPose(END_EFF_FRAME,start_pose, pose_in_end_effector);
      tf_listener.transformPose(SENSOR_FRAME,start_pose, pose_in_sensor);


      success = true;
      ROS_INFO_STREAM("Current pose in base frame is :" << start_pose);
      //ROS_INFO_STREAM("Current pose in end effector frame is :" << pose_in_end_effector);
      ROS_INFO_STREAM("Current pose in sensor frame is :" << pose_in_sensor);

    } catch (tf::ExtrapolationException e)
    {
      ROS_INFO("Waiting for transform");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }


  ros::spinOnce();
  return 0;
}

