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


#define MOVE_DOWN 1
#define MOVE_UP 2
#define MOVE_BACK 3
#define END_EFF_FRAME "j2s7s300_end_effector"
#define BASE_FRAME "j2s7s300_link_base"
#define SENSOR_FRAME "forcesensor"
#define FORCE_F_THRESH 0.5
#define FORCE_B_THRESH 0.5
#define ROTATION_STEP 10
#define MAX_STEPS 3
#define VEL_LIN_MAX 0.04
#define VEL_ANG_MAX 0.4

float thresh_lin = 0.01;
float thresh_ang = 0.15;


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
int current_step=0;


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
      //Wait 1.5s extra for arm to settle due to inertia.
      ros::Time start = ros::Time::now();
      while(ros::Time::now()-start < ros::Duration(1.5)) ros::spinOnce();
      break;
    }
  }
}


void setPoseForDirection(int direction, geometry_msgs::PoseStamped& start_pose)
{
  geometry_msgs::Quaternion quat;
  switch(direction)
  {
  /// END EFFECTOR FRAME  Z-forward-blue, Y-up-green, X-left-red
  case MOVE_DOWN :
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(ROTATION_STEP),angles::from_degrees(0),angles::from_degrees(0)); //Turn down
    start_pose.header.frame_id=END_EFF_FRAME;
    start_pose.pose.position.x=0;
    start_pose.pose.position.y=0.01;  // Slightly up
    start_pose.pose.position.z=-0.01; // Slightly back
    start_pose.pose.orientation = quat;
    break;
  case MOVE_UP :
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(-ROTATION_STEP),angles::from_degrees(0),angles::from_degrees(0));
    start_pose.header.frame_id=END_EFF_FRAME;
    start_pose.pose.position.x=0;
    start_pose.pose.position.y=-0.01;
    start_pose.pose.position.z=0.01;
    start_pose.pose.orientation = quat;
    break;
  case MOVE_BACK :
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(0),angles::from_degrees(0),angles::from_degrees(0));
    start_pose.header.frame_id=END_EFF_FRAME;
    start_pose.pose.position.x=0;
    start_pose.pose.position.y=0;
    start_pose.pose.position.z=-0.1;
    start_pose.pose.orientation = quat;
    break;

  /// SENSORFRAME  X-forward-red, Y-left-green, Z-up-blue
  /*case MOVE_DOWN :
    ROS_INFO("Move down");
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(0),angles::from_degrees(20),angles::from_degrees(0)); //Turn down
    start_pose.header.frame_id=SENSOR_FRAME;
    start_pose.pose.position.x=-0.01; // Slightly back
    start_pose.pose.position.y=0;
    start_pose.pose.position.z=0.01;  // Slightly up
    start_pose.pose.orientation = quat;
    break;
  case MOVE_UP :
    ROS_INFO("Move up");
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(0),angles::from_degrees(-20),angles::from_degrees(0)); //Turn up
    start_pose.header.frame_id=SENSOR_FRAME;
    start_pose.pose.position.x=0.01; // Slightly forward
    start_pose.pose.position.y=0;
    start_pose.pose.position.z=-0.01;  // Slightly down
    start_pose.pose.orientation = quat;
    break;
  case MOVE_BACK :
    ROS_INFO("Move back");
    quat =  tf::createQuaternionMsgFromRollPitchYaw(angles::from_degrees(0),angles::from_degrees(-20),angles::from_degrees(0));
    start_pose.header.frame_id=SENSOR_FRAME;
    start_pose.pose.position.x=-0.1;
    start_pose.pose.position.y=0;
    start_pose.pose.position.z=0;
    start_pose.pose.orientation = quat;
    break;*/


  }
}

/**
 * @brief moveCup. Wrapper for position control. Different from the moveCup functions in other nodes.
 * @param direction
 */

void moveCup(int direction)
{
  ROS_INFO("Calling moveCup");
  geometry_msgs::PoseStamped start_pose, pose_in_base;
  tf::TransformListener tf_listener;
  //Wait for TF buffer to fill up
  ros::Duration(0.25).sleep();

  ros::Rate loop_rate(9);
  if(direction==MOVE_BACK)
  {
    // Return to initial pose

    ROS_INFO("Move to initial pose");
    cmd_pos.publish(initial_pose);
    waitForActionCompleted();
    ros::Duration(2.0).sleep();

    ROS_INFO("Move back");
    ros::Rate vel_rate(100);
    geometry_msgs::PoseStamped start_pose;

    lock_pose.lock();
    start_pose = current_pose;
    lock_pose.unlock();

    ROS_INFO_STREAM("Start_pose pose is : " << start_pose );

    /** A loop is needed to ensure TF transformation has execution attempts to succeed*/
    while(ros::ok())
    {
      try
      {
        lock_pose.lock();
        start_pose = current_pose;
        lock_pose.unlock();

        /** Blocking call*/
        setPoseForDirection(direction, start_pose);

        tf_listener.transformPose(BASE_FRAME,start_pose,start_pose);
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

    ROS_INFO_STREAM("goal pose is : " << start_pose );


    /** Use velocity commands to come approcimately close to target pose*/
    while(ros::ok())
    {
      lock_pose.lock();
      geometry_msgs::PoseStamped temp_pose = current_pose;
      lock_pose.unlock();

      float delta_lin_x = temp_pose.pose.position.x - start_pose.pose.position.x;
      float delta_lin_y = temp_pose.pose.position.y - start_pose.pose.position.y;
      float delta_lin_z = temp_pose.pose.position.z - start_pose.pose.position.z;

      if( std::fabs(delta_lin_x)>=thresh_lin
                  || std::fabs(delta_lin_y)>=thresh_lin
                  || std::fabs(delta_lin_z)>=thresh_lin)
      {
        geometry_msgs::TwistStamped vel_msg;
        vel_msg.twist.linear.x = -delta_lin_x;//<0?VEL_LIN_MAX:-VEL_LIN_MAX ;
        vel_msg.twist.linear.y = -delta_lin_y;//<0?VEL_LIN_MAX:-VEL_LIN_MAX;
        //vel_msg.twist.linear.z = temp_pose.pose.position.z - start_pose.pose.position.z;

        //ROS_INFO_STREAM("Publishing vel : " << vel_msg.twist.linear);
        cmd_vel.publish(vel_msg);
        ros::spinOnce();
        vel_rate.sleep();
      }
      else
        break;
    }

    return;
  }

  /** use position controller to precisely reach target pose*/
  while(ros::ok())
  {
    try
    {
      lock_pose.lock();
      start_pose = current_pose;
      lock_pose.unlock();

      setPoseForDirection(direction, start_pose);

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



  //ROS_INFO_STREAM("Goal pose in self frame:" << start_pose);
  //ROS_INFO_STREAM("pose_in_base pose :" << pose_in_base);
  ROS_INFO_STREAM("Publishing goal pose ");
  cmd_pos.publish(pose_in_base);

  waitForActionCompleted();
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feeder_position_control");
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

  moveCup(MOVE_DOWN);

  while(ros::ok())
  {
    ros::spinOnce();

    lock_force.lock();
    local_force_b=force_f;  //Interchange sensors so that it makes practical sense
    local_force_f=force_b;
    lock_force.unlock();

    if (local_force_f >= FORCE_F_THRESH && local_force_b <FORCE_B_THRESH)
    {
      if(current_step < -MAX_STEPS)
      {
        ROS_WARN_STREAM("Max lower steps of " << MAX_STEPS << " reached.");
      }
      else
      {
        ROS_INFO("Move down");
        moveCup(MOVE_DOWN);
        current_step--;
      }
      print_once_only=true;
    }
    else if (local_force_f <FORCE_F_THRESH && local_force_b>=FORCE_B_THRESH)
    {
      if (current_step > MAX_STEPS/2)
      {
        ROS_WARN_STREAM("Max upper steps of " << MAX_STEPS/2 << " reached.");
      }
      else
      {
        ROS_INFO("Move up");
        moveCup(MOVE_UP);
        current_step++;
      }
      print_once_only=true;
    }
    else if (local_force_f >= FORCE_F_THRESH && local_force_b >=FORCE_B_THRESH)
    {
      ROS_INFO("Move back");
      moveCup(MOVE_BACK);
      ROS_WARN("Closing node");
      break;
    }
    else if (print_once_only)
    {
      ROS_INFO_STREAM("Do nothing. Current step : " << current_step);
      print_once_only=false;
    }

    loop_rate.sleep();
  }

   ros::spinOnce();

  return 0;
}
