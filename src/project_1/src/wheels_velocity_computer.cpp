#include <geometry_msgs/TwistStamped.h>
#include <project_1/WheelsRpm.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Core>

#include "shared/WheelsVelocityComputer.h"

using namespace ros;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace Eigen;

Publisher pub;

void robotVelocityCallback(const TwistStamped::ConstPtr &msg);

int main(int argc, char **argv)
{
    init(argc, argv, "wheels_velocity_computer");
    NodeHandle handle;

    Subscriber sub = handle.subscribe("cmd_vel", 1000, robotVelocityCallback);
    pub            = handle.advertise<project_1::WheelsRpm>("wheels_rpm", 1000);

    spin();
}

void robotVelocityCallback(const TwistStamped::ConstPtr &msg)
{
    // Compute the wheels velocities
    auto V = WheelsVelocityComputer::getInstance().computeWheelVelocities(msg);

    // Publis the message
    project_1::WheelsRpm wheelsMsg;
    wheelsMsg.header.stamp = msg->header.stamp;
    wheelsMsg.rpm_fl       = V[0];
    wheelsMsg.rpm_fr       = V[1];
    wheelsMsg.rpm_rr       = V[3];
    wheelsMsg.rpm_rl       = V[2];
    pub.publish(wheelsMsg);
}