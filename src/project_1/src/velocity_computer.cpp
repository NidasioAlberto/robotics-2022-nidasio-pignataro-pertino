#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <project_1/velocityComputerParamsConfig.h>
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
using namespace project_1;

Publisher pub;

void wheelStateCallback(const JointState::ConstPtr &msg);
void velocityParametersChangeCallback(velocityComputerParamsConfig &config,
                                      uint32_t level);

int main(int argc, char **argv)
{
    init(argc, argv, "velocity_computer");
    NodeHandle handle;

    // Robot paramters recofiguration
    Server<velocityComputerParamsConfig> velocityParamsServer;
    Server<velocityComputerParamsConfig>::CallbackType velocityParamsCallback;
    velocityParamsCallback =
        boost::bind(&velocityParametersChangeCallback, _1, _2);
    velocityParamsServer.setCallback(velocityParamsCallback);

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

void velocityParametersChangeCallback(velocityComputerParamsConfig &config,
                                      uint32_t level)
{
    double newValue;
    std::string parameterChanged;

    switch (level)
    {
        case 0:
            parameterChanged = "Wheel radius [m]";
            newValue         = config.R;
            VelocityComputer::getInstance().setR(newValue);
            break;
        case 1:
            parameterChanged = "Wheel position along x [m]";
            newValue         = config.L;
            VelocityComputer::getInstance().setL(newValue);
            break;
        case 2:
            parameterChanged = "Wheel position along y [m]";
            newValue         = config.W;
            VelocityComputer::getInstance().setW(newValue);
            break;
        case 3:
            parameterChanged = "Gear ratio from motor to wheel";
            newValue         = config.T;
            VelocityComputer::getInstance().setT(newValue);
            break;
        case 4:
            parameterChanged = "Counts per revolution of the motor";
            newValue         = config.N;
            VelocityComputer::getInstance().setN(newValue);
            break;
    }

    VelocityComputer::getInstance().setComputeMethod(
        config.wheel_data_source == 0
            ? VelocityComputer::ComputeMethod::RMP
            : VelocityComputer::ComputeMethod::ENCODER);

    ROS_INFO(
        "[VelocityComputer] Reconfiguring robot parameter: %s; new value: %lf",
        parameterChanged.c_str(), newValue);
}
