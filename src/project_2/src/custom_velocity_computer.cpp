#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Core>
#include <string>

#include "shared/VelocityComputer.h"

using namespace ros;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace Eigen;
using namespace dynamic_reconfigure;

Publisher pub;

void wheelStateCallback(const JointState::ConstPtr &msg);

int main(int argc, char **argv)
{
    init(argc, argv, "custom_velocity_computer");
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