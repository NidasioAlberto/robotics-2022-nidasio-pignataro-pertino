#include <dynamic_reconfigure/server.h>
#include <project_1/parametersConfig.h>
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
int wheelsDataSource = 0;

void wheelStateCallback(const JointState::ConstPtr& msg);
void wheelsDataSourceChangeCallback(int *dataSource, project_1::parametersConfig &config, uint32_t level);

int main(int argc, char** argv)
{
    init(argc, argv, "listener");
    NodeHandle handle;

    // TODO: verificare possibilità di raggruppare i callback per switch metodo di integrazione e switch data sources delle ruote
    //       in un unico nodo
    dynamic_reconfigure::Server<project_1::parametersConfig> dynServer;
    dynamic_reconfigure::Server<project_1::parametersConfig>::CallbackType callbackFunction;
    callbackFunction = boost::bind(&wheelsDataSourceChangeCallback, &wheelsDataSource, _1, _2);
    dynServer.setCallback(callbackFunction);

    Subscriber sub = handle.subscribe("wheel_states", 1000, wheelStateCallback);
    pub            = handle.advertise<TwistStamped>("cmd_vel", 1000);

    spin();
}

void wheelStateCallback(const JointState::ConstPtr& msg)
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

void wheelsDataSourceChangeCallback(int *dataSource, project_1::parametersConfig &config, uint32_t level)
{
    ROS_INFO(
        "Reconfiguring wheels data sources.\nPrevious wheels data sources: "
        "%s\nCurrent data source: %s",
        *dataSource == 0 ? "RPM source"
                         : "Ticks source",
        config.wheel_data_source == 0 ? "RPM source"
                                       : "Ticks source");

    /**
     * Switching to the new integration method selected by the user.
     * Setting up the wheels data sources:
     *      0 - RPM wheels data source (default)
     *      1 - Ticks wheels data source
     */
    VelocityComputer::getInstance().setComputeMethod(config.wheel_data_source == 0
                                                            ? VelocityComputer::ComputeMethod::RMP
                                                            : VelocityComputer::ComputeMethod::ENCODER);
    *dataSource = config.wheel_data_source;
}