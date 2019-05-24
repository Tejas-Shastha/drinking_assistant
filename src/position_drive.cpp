/**
  * See README.md for more info.
  */

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "angles/angles.h"
#include "mutex"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"

#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/PoseVelocity.h>

geometry_msgs::PoseStamped current_pose, start_pose, pose_in_end_effector,pose_in_base;
std::string result;
std::mutex lock_pose;
std::mutex lock_status;

void poseGrabber(geometry_msgs::PoseStamped pose)
{
  lock_pose.lock();
  current_pose=pose;
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

  std::string end_eff_frame = tf_listener.resolve("j2s7s300_end_effector");
  std::string base_frame = tf_listener.resolve("j2s7s300_link_base");

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


  //// WORKING: Make a translation WRT end effector frame
  success = false;
  while(!success)
  {
    try
    {
      //Necessary to make sure tf and pose sync properly
      lock_pose.lock();
      start_pose=current_pose;
      lock_pose.unlock();

      geometry_msgs::Quaternion quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(10),angles::from_degrees(0),angles::from_degrees(0));
      start_pose.header.frame_id=end_eff_frame;
      start_pose.pose.position.x=0;
      start_pose.pose.position.y=0;
      start_pose.pose.position.z=0;
      start_pose.pose.orientation = quat;

      tf_listener.transformPose(base_frame,start_pose,pose_in_base);
      success=true;
    }
    catch (tf::ExtrapolationException e)
    {
      ROS_INFO_STREAM("Waiting for transform "
                      //<< e.what()
                      );
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO_STREAM("Start pose :" << start_pose);
  ROS_INFO_STREAM("pose_in_base pose :" << pose_in_base);
  ROS_INFO_STREAM("Publishing goal pose ");
  cmd_pos.publish(pose_in_base);

  //Wait until action completed
  std::string temp_res="";
  while(ros::ok())
  {
    ros::spinOnce();
    lock_status.lock();
    temp_res = result;
    lock_status.unlock();
    ROS_INFO_STREAM("Status : " << temp_res);
    if (temp_res=="Stopped") break;
  }

  //Wait 1.5s extra for arm to settle due to inertia.
  ros::Time start = ros::Time::now();
  while(ros::Time::now()-start < ros::Duration(1.5)) ros::spinOnce();

  ros::spinOnce();
  lock_pose.lock();
  start_pose=current_pose;
  lock_pose.unlock();
  ROS_INFO_STREAM("End pose :" << start_pose);


  ros::spinOnce();
  return 0;
}

