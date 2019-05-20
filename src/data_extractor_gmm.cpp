/**
Check README.md for more info
  */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "mutex"
#include "angles/angles.h"
#include <stdio.h>
#include <fstream>
#include "sstream"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"

#include <hri_package/Sens_Force.h>
#include <kinova_msgs/PoseVelocity.h>


std::mutex lock_pose;
std::string FILE_NAME;
ros::Time start_time;
float sequence_nr=0.00;



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
 * @brief Callback to read pose values from Kinova drivers. Writes the pose in X,Y,Z(meters) and Quaternions to files.
 * @param pose
 */
void poseGrabber(geometry_msgs::PoseStamped pose)
{

  double roll, pitch, yaw;
  getRPYFromQuaternionMSG(pose.pose.orientation, roll, pitch, yaw);
  std::ofstream tool_writer;
  std::string tool_filename="/home/tejas/data/extracted_data/" + FILE_NAME + ".txt";
  std::string tool_filename_ods="/home/tejas/data/extracted_data/" + FILE_NAME + ".ods";
  ROS_INFO_STREAM("Writing @ " << (float)(ros::Time::now().toSec() - start_time.toSec())  << " data to : " << tool_filename);

  tool_writer.open(tool_filename, std::ios_base::app);
//  tool_writer << std::setprecision(4) << (float)(ros::Time::now().toSec() - start_time.toSec())  << "\t"
  tool_writer << std::fixed << std::setprecision(3) << sequence_nr  << "\t"
              << std::setprecision(3) << pose.pose.position.x << "\t"
              << std::setprecision(3) << pose.pose.position.y << "\t"
              << std::setprecision(3) << pose.pose.position.z << "\t"
              << std::setprecision(3) << pose.pose.orientation.x << "\t"
              << std::setprecision(3) << pose.pose.orientation.y << "\t"
              << std::setprecision(3) << pose.pose.orientation.z << "\t"
              << std::setprecision(3) << pose.pose.orientation.w << "\t"
//              << std::setprecision(4) << angles::to_degrees(roll) << "\t"
//              << std::setprecision(4) << angles::to_degrees(pitch) << "\t"
//              << std::setprecision(4) << angles::to_degrees(yaw) << "\t"
              << "\n";
  tool_writer.close();

  ROS_INFO_STREAM("Writing @ " << (float)(ros::Time::now().toSec() - start_time.toSec())  << " data to : " << tool_filename_ods);
  tool_writer.open(tool_filename_ods, std::ios_base::app);
//  tool_writer << std::setprecision(4) << (float)(ros::Time::now().toSec() - start_time.toSec())  << "\t"
  tool_writer << std::fixed << std::setprecision(3) << sequence_nr++  << "\t"
              << std::setprecision(3) << pose.pose.position.x << "\t"
              << std::setprecision(3) << pose.pose.position.y << "\t"
              << std::setprecision(3) << pose.pose.position.z << "\t"
              << std::setprecision(3) << pose.pose.orientation.x << "\t"
              << std::setprecision(3) << pose.pose.orientation.y << "\t"
              << std::setprecision(3) << pose.pose.orientation.z << "\t"
              << std::setprecision(3) << pose.pose.orientation.w << "\t"
//              << std::setprecision(4) << angles::to_degrees(roll) << "\t"
//              << std::setprecision(4) << angles::to_degrees(pitch) << "\t"
//              << std::setprecision(4) << angles::to_degrees(yaw) << "\t"
              << "\n";
  tool_writer.close();
}

/**
 * @brief Main function needs 1 argument.
 * @param argc = 2
 * @param argv. Arg1 = filename
 * @return
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_extractor");
  ros::NodeHandle nh;

  FILE_NAME = argv[1];

  std::ofstream tool_writer;
  std::string tool_filename="/home/tejas/data/extracted_data/" + FILE_NAME + ".txt";
  std::string tool_filename_ods="/home/tejas/data/extracted_data/" + FILE_NAME + ".ods";

  ROS_INFO_STREAM("Writing data to" << tool_filename);

  tool_writer.open(tool_filename_ods, std::ios_base::app);
  tool_writer << "TIME(sec)" << "\t"
              << "X" << "\t"
              << "Y" << "\t"
              << "Z" << "\t"
              << "QX" << "\t"
              << "QY" << "\t"
              << "QZ" << "\t"
              << "QW" << "\t"
//              << "ROLL" << "\t"
//              << "PITCH" << "\t"
//              << "YAW" << "\t"
              << "\n";
  tool_writer.close();

  sleep(0.25);
  ROS_INFO("Ready to extract");


  start_time = ros::Time::now();

  ros::Subscriber tool_pose = nh.subscribe("/j2s7s300_driver/out/tool_pose",1000, poseGrabber );

  ros::spin();

  return 0;
}
