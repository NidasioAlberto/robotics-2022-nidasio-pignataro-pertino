#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <project_1/wheelsDataSourceConfig.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Core>

#include "shared/VelocityComputer.h"

using namespace ros;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace Eigen;

Publisher pub;

void wheelStateCallback(const JointState::ConstPtr &msg);
void wheelsDataSourceChangeCallback(project_1::wheelsDataSourceConfig &config,
                                    uint32_t level);

int main(int argc, char **argv)
{
    init(argc, argv, "velocity_computer");
    NodeHandle handle;

    dynamic_reconfigure::Server<project_1::wheelsDataSourceConfig> dynServer;
    dynamic_reconfigure::Server<project_1::wheelsDataSourceConfig>::CallbackType
        callbackFunction;
    callbackFunction = boost::bind(&wheelsDataSourceChangeCallback, _1, _2);
    dynServer.setCallback(callbackFunction);

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

void wheelsDataSourceChangeCallback(project_1::wheelsDataSourceConfig &config,
                                    uint32_t level)
{
    ROS_INFO("Reconfiguring wheels data source to %s",
             config.wheel_data_source == 0 ? "RPM source" : "Ticks source");

    /**
     * Setting up the wheels data sources:
     *      0 - RPM wheels data source (default)
     *      1 - Ticks wheels data source
     */
    VelocityComputer::getInstance().setComputeMethod(
        config.wheel_data_source == 0
            ? VelocityComputer::ComputeMethod::RMP
            : VelocityComputer::ComputeMethod::ENCODER);
}