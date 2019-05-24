/**
  * See README.md for more info.
  */

#include <ros/ros.h>
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "angles/angles.h"

void getRPYFromQuaternionMSG(geometry_msgs::Quaternion orientation, double& roll,double& pitch, double& yaw)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(orientation,quat);
  quat.normalize();
  tf::Matrix3x3 mat(quat);
  mat.getRPY(roll, pitch,yaw);
}

void printQuat(geometry_msgs::Quaternion quat, std::string name)
{
  ROS_INFO_STREAM(name << " quat : " << quat.x << " " << quat.y << " " << quat.z  << " " << quat.w << " ");
}

void printRPY(double roll, double pitch, double yaw, std::string name)
{
    ROS_INFO_STREAM(name<< " RPY (deg) : " << angles::to_degrees(roll) << " " << angles::to_degrees(pitch) << " " << angles::to_degrees(yaw) << " ");
}

geometry_msgs::Quaternion multiplyQuaternionsMSG(geometry_msgs::Quaternion quat_1, geometry_msgs::Quaternion quat_2)
{
    tf::Quaternion orig, rot, result;
    geometry_msgs::Quaternion q_new;

    tf::quaternionMsgToTF(quat_1, orig);
    tf::quaternionMsgToTF(quat_2, rot);

    result=orig*rot;
    result.normalize();

    tf::quaternionTFToMsg(result, q_new);

    return q_new;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quaternion_tester");
    ros::NodeHandle nh;

    geometry_msgs::Quaternion q_orig, q_rot, q_new;
    tf::Quaternion q_temp;

    // ORIGINAL
    q_temp.setRPY(angles::from_degrees(60),angles::from_degrees(0),angles::from_degrees(0));
    tf::quaternionTFToMsg(q_temp,q_orig);

    // ROTATION
    q_temp.setRPY(angles::from_degrees(40),angles::from_degrees(0),angles::from_degrees(0));
    tf::quaternionTFToMsg(q_temp,q_rot);

    q_new=multiplyQuaternionsMSG(q_orig, q_rot);

    double orig_rol, orig_pit, orig_yaw;
    double rot_rol, rot_pit, rot_yaw;
    double new_rol, new_pit, new_yaw;

    getRPYFromQuaternionMSG(q_orig,orig_rol, orig_pit, orig_yaw);
    getRPYFromQuaternionMSG(q_rot, rot_rol, rot_pit, rot_yaw);
    getRPYFromQuaternionMSG(q_new, new_rol, new_pit, new_yaw);

    ROS_INFO("TEST QUATERNION MULTIPLICATION");

    printQuat(q_orig,"Q_orig");
    printRPY(orig_rol,orig_pit,orig_yaw,"Q_orig");

    printQuat(q_rot,"Q_rot");
    printRPY(rot_rol, rot_pit, rot_yaw,"Q_rot");

    printQuat(q_new,"Q_new");
    printRPY(new_rol, new_pit, new_yaw, "Q_new");


    ROS_INFO("-------------------------------------------------------------------------------");
    ROS_INFO("-------------------------------------------------------------------------------");
    ROS_INFO("REDO TEST WITH RPY ADDITION");

    // NOTE : This is NOT a rotation. This is simply adding RPY.
    new_rol=orig_rol+rot_rol;
    new_pit=orig_pit+rot_pit;
    new_yaw=orig_yaw+rot_yaw;


    q_temp.setRPY(new_rol, new_pit, new_yaw);
    tf::quaternionTFToMsg(q_temp, q_new);

    printQuat(q_new,"Q_new");
    printRPY(new_rol, new_pit, new_yaw, "Q_new");



    ros::spinOnce();

}
