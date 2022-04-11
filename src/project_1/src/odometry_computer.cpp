#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <project_1/parametersConfig.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include "shared/OdometryComputer.h"

using namespace ros;
using namespace Eigen;

void velocityStateCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
void integrationMethodChangeCallback(int *integrationMode, project_1::parametersConfig &config, uint32_t level);

Publisher pub, pub2;
int integrationMode = 0;

int main(int argc, char **argv)
{
    // Node setup
    init(argc, argv, "odometry_computer");
    NodeHandle handle;

    // TODO: verificare possibilità di raggruppare i callback per switch metodo di integrazione e switch data sources delle ruote
    //       in un unico nodo
    dynamic_reconfigure::Server<project_1::parametersConfig> dynServer;
    dynamic_reconfigure::Server<project_1::parametersConfig>::CallbackType callbackFunction;
    callbackFunction = boost::bind(&integrationMethodChangeCallback, &integrationMode, _1, _2);
    dynServer.setCallback(callbackFunction);

    OdometryComputer::getInstance().setIntegrationMethod(OdometryComputer::OdometryIntegration::EULER);
    OdometryComputer::getInstance().setInitialPosition({0, 0, 0});

    Subscriber sub = handle.subscribe("cmd_vel", 1000, velocityStateCallback);
    pub            = handle.advertise<nav_msgs::Odometry>("odom", 1000);
    pub2           = handle.advertise<geometry_msgs::PoseStamped>("pose", 1000);

    spin();
    return 0;
}

void velocityStateCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    /*
    transformStamped.transform.translation.x = msg->x;
    transformStamped.transform.translation.y = msg->y;
    transformStamped.transform.translation.z = 0.0;
    // set theta
    tf2::Quaternion q;
    //convert from angles to quaternion
    q.setRPY(0, 0, msg->theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    */
    Vector3d result = OdometryComputer::getInstance().computeOdometry(msg);

    geometry_msgs::PoseStamped message;

    message.header          = msg->header;
    message.header.frame_id = "world";
    message.pose.position.x = result[0];
    message.pose.position.y = result[1];

    tf2::Quaternion quat;
    quat.setEulerZYX(result[2], 0, 0);
    message.pose.orientation.x = quat.getX();
    message.pose.orientation.y = quat.getY();
    message.pose.orientation.z = quat.getZ();
    message.pose.orientation.w = quat.getW();

    pub2.publish(message);
}

/**
 * @brief Triggered when the `integration_method` config parameter gets
 * dynamically reconfigured.
 *
 * @param integrationMode current integration mode (0 - Euler integration | 1 -
 * Runge-Kutta integration)
 * @param config configuration file to fetch parameters
 * @param level _
 */
void integrationMethodChangeCallback(int *integrationMode, project_1::parametersConfig &config, uint32_t level)
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
    OdometryComputer::getInstance().setIntegrationMethod(config.integration_method == 0
                                                            ? OdometryComputer::OdometryIntegration::EULER 
                                                            : OdometryComputer::OdometryIntegration::RUNGE_KUTTA);
    *integrationMode = config.integration_method;
}
