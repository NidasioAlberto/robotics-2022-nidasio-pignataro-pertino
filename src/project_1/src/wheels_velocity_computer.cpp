#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <project_1/WheelsRpm.h>
#include <project_1/robotPhysicalParametersConfig.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Core>
#include <string>

#include "shared/WheelsVelocityComputer.h"

using namespace ros;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace Eigen;
using namespace dynamic_reconfigure;
using namespace project_1;

Publisher pub;

void robotVelocityCallback(const TwistStamped::ConstPtr &msg);
void robotParametersChangeCallback(robotPhysicalParametersConfig &config,
                                   uint32_t level);

int main(int argc, char **argv)
{
    init(argc, argv, "wheels_velocity_computer");
    NodeHandle handle;

    Subscriber sub = handle.subscribe("cmd_vel", 1000, robotVelocityCallback);
    pub            = handle.advertise<WheelsRpm>("wheels_rpm", 1000);

    // Robot paramters recofiguration
    Server<robotPhysicalParametersConfig> robotParamsServer;
    Server<robotPhysicalParametersConfig>::CallbackType robotParamsCallback;
    robotParamsCallback = boost::bind(&robotParametersChangeCallback, _1, _2);
    robotParamsServer.setCallback(robotParamsCallback);

    spin();
}

void robotVelocityCallback(const TwistStamped::ConstPtr &msg)
{
    // Compute the wheels velocities
    auto V = WheelsVelocityComputer::getInstance().computeWheelVelocities(msg);

    // Publis the message
    WheelsRpm wheelsMsg;
    wheelsMsg.header.stamp = msg->header.stamp;
    wheelsMsg.rpm_fl       = V[0];
    wheelsMsg.rpm_fr       = V[1];
    wheelsMsg.rpm_rr       = V[3];
    wheelsMsg.rpm_rl       = V[2];
    pub.publish(wheelsMsg);
}

void robotParametersChangeCallback(robotPhysicalParametersConfig &config,
                                   uint32_t level)
{
    double newValue;
    std::string parameterChanged;

    switch (level)
    {
        case -1:
            // Set all the parameters at the beginning
            VelocityComputer::getInstance().setR(config.R);
            VelocityComputer::getInstance().setL(config.L);
            VelocityComputer::getInstance().setW(config.W);
            VelocityComputer::getInstance().setT(config.T);
            VelocityComputer::getInstance().setN(config.N);
            VelocityComputer::getInstance().setComputeMethod(
                config.wheel_data_source == 0
                    ? VelocityComputer::ComputeMethod::RMP
                    : VelocityComputer::ComputeMethod::ENCODER);
        case 0:
            parameterChanged = "Wheel radius [m]";
            newValue         = config.R;
            WheelsVelocityComputer::getInstance().setR(newValue);
            break;
        case 1:
            parameterChanged = "Wheel position along x [m]";
            newValue         = config.L;
            WheelsVelocityComputer::getInstance().setL(newValue);
            break;
        case 2:
            parameterChanged = "Wheel position along y [m]";
            newValue         = config.W;
            WheelsVelocityComputer::getInstance().setW(newValue);
            break;
        case 3:
            parameterChanged = "Gear ratio from motor to wheel";
            newValue         = config.T;
            WheelsVelocityComputer::getInstance().setT(newValue);
            break;
        case 4:
            parameterChanged = "Counts per revolution of the motor";
            newValue         = config.N;
            WheelsVelocityComputer::getInstance().setT(newValue);
            break;
    }

    ROS_INFO(
        "[WheelsVelocityComputer] Reconfiguring robot parameter: %s; new "
        "value: %lf",
        parameterChanged.c_str(), newValue);
}
