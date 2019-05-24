/**
  * See README.md for more info.
  *
  * This iteration of this node was used to test the IK solvers. They do not work.
  * TODO: Figure out why.
  *
  */
/// DRIVE FOR DURATION

#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "mutex"
#include "kinova_msgs/PoseVelocity.h"
#include <kinova_msgs/ArmJointAnglesActionGoal.h>
#include <kinova_msgs/JointAngles.h>

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include <tf_conversions/tf_kdl.h>
#include "kdl/frames.hpp"
#include "kdl_conversions/kdl_msg.h"

geometry_msgs::PoseStamped current_pose;
std::mutex pose_lock;
kinova_msgs::JointAngles current_joints;

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

void poseGrabber(geometry_msgs::PoseStamped::ConstPtr pose)
{

  pose_lock.lock();
  current_pose.pose.position.x = pose->pose.position.x;
  current_pose.pose.position.y = pose->pose.position.y;
  current_pose.pose.position.z = pose->pose.position.z;
  current_pose.pose.orientation.w = pose->pose.orientation.w;
  current_pose.pose.orientation.x = pose->pose.orientation.x;
  current_pose.pose.orientation.y = pose->pose.orientation.y;
  current_pose.pose.orientation.z = pose->pose.orientation.z;
  pose_lock.unlock();
  //ROS_INFO_STREAM("Got current pose as x=" << pose->pose.position.x << " saved as " << current_pose.pose.position.x);
}

void jointGrabber(kinova_msgs::JointAngles joints)
{
  current_joints = joints;
}


KDL::Frame poseStampedToKDLFrame(geometry_msgs::PoseStamped from_pose)
{
  ros::spinOnce();
  KDL::Frame to_frame;
  tf::poseMsgToKDL(from_pose.pose, to_frame);
  return to_frame;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_test");
  ros::NodeHandle nh;

  //ros::Publisher cmd_vel = nh.advertise<geometry_msgs::TwistStamped>("/RobotControl/VelocityControl", 1000);
  ros::Publisher cmd_vel = nh.advertise<kinova_msgs::PoseVelocity>("/j2s7s300_driver/in/cartesian_velocity", 1000);
  ros::Subscriber tool_pose = nh.subscribe("/j2s7s300_driver/out/tool_pose",1000, poseGrabber );


  ros::Publisher joint_pub = nh.advertise<kinova_msgs::ArmJointAnglesActionGoal>("/j2s7s300_driver/joints_action/joint_angles/goal", 1000);
  ros::Subscriber joint_sub = nh.subscribe("/j2s7s300_driver/out/joint_angles",1000,jointGrabber);
  while(ros::ok)
  {

    if ((current_joints.joint1==0 && current_joints.joint2==0) || (current_pose.pose.position.x ==0 && current_pose.pose.position.y==0)  )
    {
      //      ROS_INFO("Spinning");
      ros::spinOnce();
    }
    else
      break;
  }

  /// Here starts Solver testing

  std::string chain_start = "j2s7s300_link_base", chain_end = "j2s7s300_end_effector", urdf_param = "/robot_description";
  double timeout = 0.005;
  double eps = 1e-5;

  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);
  tracik_solver.getKDLChain(chain);
  tracik_solver.getKDLLimits(ll,ul);

  // Set up KDL & TRAC IK
  KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
  KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solver
  KDL::ChainIkSolverPos_NR_JL kdl_solver(chain,ll,ul,fk_solver, vik_solver, 1, eps); // Joint Limit Solver

//  // Spin to grab latest pose and joint angles, updating "current_pose" and "current_joints" in callbacks not shown here
//  ros::spinOnce();
//  /// Forward Kinematics test

//    // Current pose updated from Kinova publisher, convert msg to KDL frame
//    KDL::Frame current_pose_frame;
//    tf::PoseMsgToKDL(current_pose.pose, current_pose_frame);

//    // Current joints updated from Kinova publisher, convert msg to KDL Joint Array
//    KDL::JntArray current_joints_KDL(7);
//    current_joints_KDL(0)=current_joints.joint1;
//    current_joints_KDL(1)=current_joints.joint2;
//    current_joints_KDL(2)=current_joints.joint3;
//    current_joints_KDL(3)=current_joints.joint4;
//    current_joints_KDL(4)=current_joints.joint5;
//    current_joints_KDL(5)=current_joints.joint6;
//    current_joints_KDL(6)=current_joints.joint7;

//    // Forward IK for current pose as per KDL solver
//    KDL::Frame recalculated_current_pose_frame;
//    fk_solver.JntToCart(current_joints_KDL,recalculated_current_pose_frame);
//    // Convert resulting KDL frame to Pose msg
//    geometry_msgs::Pose recalculated_current_pose;
//    tf::PoseKDLToMsg(recalculated_current_pose_frame,recalculated_current_pose);

//    ROS_INFO_STREAM("Actual current pose : \n"
//                    << current_pose.pose.position.x <<  " "
//                    << current_pose.pose.position.y <<  " "
//                    << current_pose.pose.position.z <<  " "
//                    << current_pose.pose.orientation.x <<  " "
//                    << current_pose.pose.orientation.y <<  " "
//                    << current_pose.pose.orientation.z <<  " "
//                    << current_pose.pose.orientation.w <<  " "
//                    );

//    ROS_INFO_STREAM("Recalculated current pose : \n"
//                    << recalculated_current_pose.position.x <<  " "
//                    << recalculated_current_pose.position.y <<  " "
//                    << recalculated_current_pose.position.z <<  " "
//                    << recalculated_current_pose.orientation.x <<  " "
//                    << recalculated_current_pose.orientation.y <<  " "
//                    << recalculated_current_pose.orientation.z <<  " "
//                    << recalculated_current_pose.orientation.w <<  " "
//                    );

  /// Inverse Kinematics test


  // Current joints updated from Kinova publisher, convert msg to KDL Joint Array
  KDL::JntArray current_joints_KDL(7);
  current_joints_KDL(0)=current_joints.joint1;
  current_joints_KDL(1)=current_joints.joint2;
  current_joints_KDL(2)=current_joints.joint3;
  current_joints_KDL(3)=current_joints.joint4;
  current_joints_KDL(4)=current_joints.joint5;
  current_joints_KDL(5)=current_joints.joint6;
  current_joints_KDL(6)=current_joints.joint7;

  // Current pose updated from Kinova publisher, convert msg to KDL frame
  KDL::Frame target_pose_frame;
  geometry_msgs::Pose target_pose = current_pose.pose;
  target_pose.position.x += 0.5;
  target_pose.position.y += 0.5;
  target_pose.position.z += 0.5;
  tf::PoseMsgToKDL(target_pose, target_pose_frame);

  ROS_INFO_STREAM("Current joints : \n" << current_joints_KDL.data);
  KDL::JntArray result(7);
  int rc;
  ROS_INFO_STREAM("Target pose frame : " <<
                  target_pose_frame.p.x() << " " <<
                  target_pose_frame.p.y() << " " <<
                  target_pose_frame.p.z() << " "
                  );
  rc = tracik_solver.CartToJnt(current_joints_KDL,target_pose_frame,result);
//  rc = kdl_solver.CartToJnt(current_joints_KDL,target_pose_frame,result);
  ROS_INFO_STREAM("rc : " << rc << ". New joints : \n " << result.data);


  kinova_msgs::JointAngles result_joints;
  result_joints.joint1 = result(0);
  result_joints.joint2 = result(1);
  result_joints.joint3 = result(2);
  result_joints.joint4 = result(3);
  result_joints.joint5 = result(4);
  result_joints.joint6 = result(5);
  result_joints.joint7 = result(6);
  kinova_msgs::ArmJointAnglesActionGoal goal;
  goal.goal.angles = result_joints;

//  joint_pub.publish(goal);

  ros::spinOnce();
  return 0;
}
