#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace ros;

void odomCallback(const nav_msgs::Odometry &odometry);

// Tf2 publisher
tf2_ros::TransformBroadcaster *tf_pub;

int main(int argc, char **argv)
{
    // Node setup
    init(argc, argv, "tf_tree_computer");
    NodeHandle handle;

    // Create the tf publisher
    tf_pub = new tf2_ros::TransformBroadcaster();

    // Listen to odom topic
    Subscriber sub = handle.subscribe("odom", 1000, odomCallback);

    spin();
}

void odomCallback(const nav_msgs::Odometry &odometry)
{
    geometry_msgs::TransformStamped broadcastOdom;

    broadcastOdom.header.frame_id = "odom";
    broadcastOdom.child_frame_id  = "base_link";

    // Odom translates and rotates
    broadcastOdom.transform.translation.x = odometry.pose.pose.position.x;
    broadcastOdom.transform.translation.y = odometry.pose.pose.position.y;
    broadcastOdom.transform.translation.z = odometry.pose.pose.position.z;
    broadcastOdom.transform.rotation      = odometry.pose.pose.orientation;

    tf_pub->sendTransform(broadcastOdom);
}