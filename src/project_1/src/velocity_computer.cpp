#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Core>

#include "shared/VelocityComputer.h"

using namespace ros;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace Eigen;

#define R 0.07   // Wheel radius [m]
#define L 0.2    // Wheel position along x [m]
#define W 0.169  // Wheel position along y [m]

Publisher pub;

void wheelStateCallback(const JointState::ConstPtr &msg);

int main(int argc, char **argv)
{
    init(argc, argv, "listener");
    NodeHandle handle;

    Subscriber sub = handle.subscribe("wheel_states", 1000, wheelStateCallback);
    pub            = handle.advertise<TwistStamped>("cmd_vel", 1000);

    spin();
}

void wheelStateCallback(const JointState::ConstPtr &msg)
{
    // Compute the robot velocity
    auto V = VelocityComputer::getInstance().computeRobotVelocity(msg);

    // Publish the message
    TwistStamped twistMsg;
    twistMsg.header.stamp    = msg->header.stamp;
    twistMsg.twist.linear.x  = V(1);
    twistMsg.twist.linear.y  = V(2);
    twistMsg.twist.angular.z = V(0);
    pub.publish(twistMsg);
}