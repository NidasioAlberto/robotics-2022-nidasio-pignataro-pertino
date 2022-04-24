#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include <project_1/ResetStartingPose.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace ros;

ServiceClient client;
project_1::ResetStartingPose server;

void posChangeCallback(const geometry_msgs::PoseStamped &msg);

int main(int argc, char **argv)
{
    // Node setup
    init(argc, argv, "startingPositionDetector");
    NodeHandle handle;

    // Registro il nodo alla pose
    Subscriber sub = handle.subscribe("/robot/pose", 1000, posChangeCallback);
    client         = handle.serviceClient<project_1::ResetStartingPose>(
        "reset_starting_pose");

    spin();
}

void posChangeCallback(const geometry_msgs::PoseStamped &msg)
{
    static double lastTimestamp = 0;

    // When the difference exceeds one second then i can reset the actual
    // position
    // printf("reset %f %f %f\n", msg.header.stamp.toSec(), lastTimestamp,
    //        msg.header.stamp.toSec() - lastTimestamp);
    if (abs(msg.header.stamp.toSec() - lastTimestamp) > 1)
    {
        double roll, pitch, yaw;
        tf2::Quaternion quat;
        tf2::convert(msg.pose.orientation, quat);
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        server.request.new_x     = msg.pose.position.x;
        server.request.new_y     = msg.pose.position.y;
        server.request.new_theta = yaw;

        client.call(server);
        // At the end i assign the new last timestamp
    }

    lastTimestamp = msg.header.stamp.toSec();
}
