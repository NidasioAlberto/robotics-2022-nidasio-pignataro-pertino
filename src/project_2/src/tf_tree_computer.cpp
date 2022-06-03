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
    geometry_msgs::TransformStamped broadcastBaseLink;
    tf2::Quaternion quat;

    broadcastOdom.header.frame_id     = "map";
    broadcastOdom.child_frame_id      = "odom";
    broadcastBaseLink.header.frame_id = "odom";
    broadcastBaseLink.child_frame_id  = "base_link";

    // Odom translates and rotates
    broadcastOdom.transform.translation.x = odometry.pose.pose.position.x;
    broadcastOdom.transform.translation.y = odometry.pose.pose.position.y;
    broadcastOdom.transform.translation.z = odometry.pose.pose.position.z;
    broadcastOdom.transform.rotation      = odometry.pose.pose.orientation;

    // Base link doesn't transform in respect to odom
    quat.setEuler(0, 0, 0);
    broadcastBaseLink.transform.rotation.x    = quat.getX();
    broadcastBaseLink.transform.rotation.y    = quat.getY();
    broadcastBaseLink.transform.rotation.z    = quat.getZ();
    broadcastBaseLink.transform.rotation.w    = quat.getW();
    broadcastBaseLink.transform.translation.x = 0;
    broadcastBaseLink.transform.translation.y = 0;
    broadcastBaseLink.transform.translation.z = 0;

    tf_pub->sendTransform(broadcastOdom);
    tf_pub->sendTransform(broadcastBaseLink);
}