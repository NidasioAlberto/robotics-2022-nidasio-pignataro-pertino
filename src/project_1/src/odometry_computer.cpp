#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <project_1/integrationMethodConfig.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include "shared/OdometryComputer.h"

using namespace ros;
using namespace Eigen;

void velocityStateCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
void integrationMethodChangeCallback(int *integrationMode,
                                     project_1::integrationMethodConfig &config,
                                     uint32_t level);

Publisher pub, pub2;
int integrationMode = 0;

int main(int argc, char **argv)
{
    // Node setup
    init(argc, argv, "odometry_computer");
    NodeHandle handle;

    // TODO: verificare possibilità di raggruppare i callback per switch metodo
    // di integrazione e switch data sources delle ruote
    //       in un unico nodo
    dynamic_reconfigure::Server<project_1::integrationMethodConfig> dynServer;
    dynamic_reconfigure::Server<
        project_1::integrationMethodConfig>::CallbackType callbackFunction;
    callbackFunction =
        boost::bind(&integrationMethodChangeCallback, &integrationMode, _1, _2);
    dynServer.setCallback(callbackFunction);

    OdometryComputer::getInstance().setIntegrationMethod(
        OdometryComputer::IntegrationMethod::EULER);
    OdometryComputer::getInstance().setPosition({0, 0, 0});

    Subscriber sub = handle.subscribe("cmd_vel", 1000, velocityStateCallback);
    pub            = handle.advertise<nav_msgs::Odometry>("odom", 1000);
    pub2           = handle.advertise<geometry_msgs::PoseStamped>("pose", 1000);

    spin();
    return 0;
}

void velocityStateCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    Vector3d result = OdometryComputer::getInstance().computeOdometry(msg);

    geometry_msgs::PoseStamped message;

    message.header          = msg->header;
    message.header.frame_id = "world";
    message.pose.position.x = result[0];
    message.pose.position.y = result[1];

    tf2::Quaternion quat;
    quat.setEuler(0, 0, result[2]);
    message.pose.orientation.x = quat.getX();
    message.pose.orientation.y = quat.getY();
    message.pose.orientation.z = quat.getZ();
    message.pose.orientation.w = quat.getW();

    pub2.publish(message);
}

/**
 * @brief Triggered when the `integrationMethodConfig` config parameter gets
 * dynamically reconfigured.
 *
 * @param integrationMode current integration mode (0 - Euler integration | 1 -
 * Runge-Kutta integration)
 * @param config configuration file to fetch parameters
 * @param level _
 */
void integrationMethodChangeCallback(int *integrationMode,
                                     project_1::integrationMethodConfig &config,
                                     uint32_t level)
{
    ROS_INFO(
        "Reconfiguring integration mode.\nPrevious integration mode: "
        "%s\nCurrent integration mode: %s",
        *integrationMode == 0 ? "Euler integration mode"
                              : "Runge-Kutta integration mode",
        config.integration_method == 0 ? "Euler integration mode"
                                       : "Runge-Kutta integration mode");

    /**
     * Switching to the new integration method selected by the user.
     * Setting up the integration mode:
     *      0 - Euler integration mode (default)
     *      1 - Runge-Kutta integration mode
     */
    OdometryComputer::getInstance().setIntegrationMethod(
        config.integration_method == 0
            ? OdometryComputer::IntegrationMethod::EULER
            : OdometryComputer::IntegrationMethod::RUNGE_KUTTA);
    *integrationMode = config.integration_method;
}
