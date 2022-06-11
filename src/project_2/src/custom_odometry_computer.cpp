#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "shared/OdometryComputer.h"

using namespace ros;
using namespace Eigen;

void velocityStateCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

Publisher pub, pub2;
tf2_ros::TransformBroadcaster *tf_pub;

int main(int argc, char **argv)
{
    // Node setup
    init(argc, argv, "custom_odometry_computer");
    NodeHandle handle;
    tf_pub = new tf2_ros::TransformBroadcaster();

    Subscriber sub = handle.subscribe("cmd_vel", 1000, velocityStateCallback);
    pub            = handle.advertise<nav_msgs::Odometry>("odom/local", 1000);

    spin();
}

void velocityStateCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    Vector3d result = OdometryComputer::getInstance().computeOdometry(msg);
    tf2::Quaternion quat;

    nav_msgs::Odometry odometryMsg;
    geometry_msgs::TransformStamped broadcastOdom;

    odometryMsg.header       = msg->header;
    broadcastOdom.header     = msg->header;
    broadcastOdom.header.frame_id     = "odom";
    broadcastOdom.child_frame_id      = "base_link";

    odometryMsg.pose.pose.position.x      = result[0];
    odometryMsg.pose.pose.position.y      = result[1];
    broadcastOdom.transform.translation.x = result[0];
    broadcastOdom.transform.translation.y = result[1];

    quat.setEuler(0, 0, result[2]);
    odometryMsg.pose.pose.orientation.x = quat.getX();
    odometryMsg.pose.pose.orientation.y = quat.getY();
    odometryMsg.pose.pose.orientation.z = quat.getZ();
    odometryMsg.pose.pose.orientation.w = quat.getW();
    broadcastOdom.transform.rotation.x  = quat.getX();
    broadcastOdom.transform.rotation.y  = quat.getY();
    broadcastOdom.transform.rotation.z  = quat.getZ();
    broadcastOdom.transform.rotation.w  = quat.getW();

    pub.publish(odometryMsg);
    tf_pub->sendTransform(broadcastOdom);
}