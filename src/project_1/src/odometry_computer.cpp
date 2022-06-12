#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <project_1/ResetStartingPose.h>
#include <project_1/integrationMethodConfig.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "shared/OdometryComputer.h"

using namespace ros;
using namespace Eigen;

void velocityStateCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
void integrationMethodChangeCallback(project_1::integrationMethodConfig &config,
                                     uint32_t level);
bool resetStartingPose(project_1::ResetStartingPose::Request &req,
                       project_1::ResetStartingPose::Response &res);

Publisher pub, pub2;
tf2_ros::TransformBroadcaster *tf_pub;

int main(int argc, char **argv)
{
    // Node setup
    init(argc, argv, "odometry_computer");
    NodeHandle handle;
    tf_pub = new tf2_ros::TransformBroadcaster();

    // Setting up the dynamic server for dynamically reconfigure the integration
    // mode between EULER & RUNGHE_KUTTA
    dynamic_reconfigure::Server<project_1::integrationMethodConfig> dynServer;
    dynamic_reconfigure::Server<
        project_1::integrationMethodConfig>::CallbackType callbackFunction;
    callbackFunction = boost::bind(&integrationMethodChangeCallback, _1, _2);
    dynServer.setCallback(callbackFunction);

    // Setting up the services server
    ServiceServer startingPoseResetterService =
        handle.advertiseService<project_1::ResetStartingPose::Request,
                                project_1::ResetStartingPose::Response>(
            "reset_starting_pose", boost::bind(&resetStartingPose, _1, _2));
    OdometryComputer::getInstance().setIntegrationMethod(
        OdometryComputer::IntegrationMethod::EULER);

    Subscriber sub = handle.subscribe("cmd_vel", 1000, velocityStateCallback);
    pub            = handle.advertise<nav_msgs::Odometry>("odom", 1000);
    pub2           = handle.advertise<geometry_msgs::PoseStamped>("pose", 1000);

    spin();
}

void velocityStateCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    Vector3d result = OdometryComputer::getInstance().computeOdometry(msg);
    tf2::Quaternion quat;
    quat.setEuler(0, 0, result[2]);

    // Odometry message
    {
        nav_msgs::Odometry odometryMsg;

        odometryMsg.header          = msg->header;
        odometryMsg.header.frame_id = "world";

        odometryMsg.pose.pose.position.x = result[0];
        odometryMsg.pose.pose.position.y = result[1];

        odometryMsg.pose.pose.orientation.x = quat.getX();
        odometryMsg.pose.pose.orientation.y = quat.getY();
        odometryMsg.pose.pose.orientation.z = quat.getZ();
        odometryMsg.pose.pose.orientation.w = quat.getW();

        pub.publish(odometryMsg);
    }

    // Pose message
    {
        geometry_msgs::PoseStamped message;

        message.header          = msg->header;
        message.header.frame_id = "world";
        message.pose.position.x = result[0];
        message.pose.position.y = result[1];

        quat.setEuler(0, 0, result[2]);
        message.pose.orientation.x = quat.getX();
        message.pose.orientation.y = quat.getY();
        message.pose.orientation.z = quat.getZ();
        message.pose.orientation.w = quat.getW();

        pub2.publish(message);
    }

    // TF from world to odom
    {
        geometry_msgs::TransformStamped worldToOdom;

        // This way TF doesn't die on bag reset. We didn't find a better way to
        // solve TF_OLD_DATA.
        worldToOdom.header.stamp = ros::Time::now();

        worldToOdom.header.frame_id = "world";  // Parent
        worldToOdom.child_frame_id  = "odom";   // Child

        // Word and odom coincide
        worldToOdom.transform.rotation.x    = 0;
        worldToOdom.transform.rotation.y    = 0;
        worldToOdom.transform.rotation.z    = 0;
        worldToOdom.transform.rotation.w    = 1;
        worldToOdom.transform.translation.x = 0;
        worldToOdom.transform.translation.y = 0;
        worldToOdom.transform.translation.z = 0;

        tf_pub->sendTransform(worldToOdom);
    }

    // TF from odom to base_link
    {
        geometry_msgs::TransformStamped odomToBaseLink;

        // This way TF doesn't die on bag reset. We didn't find a better way to
        // solve TF_OLD_DATA.
        odomToBaseLink.header.stamp = ros::Time::now();

        odomToBaseLink.header.frame_id = "odom";       // Parent
        odomToBaseLink.child_frame_id  = "base_link";  // Child

        odomToBaseLink.transform.rotation.x    = quat.getX();
        odomToBaseLink.transform.rotation.y    = quat.getY();
        odomToBaseLink.transform.rotation.z    = quat.getZ();
        odomToBaseLink.transform.rotation.w    = quat.getW();
        odomToBaseLink.transform.translation.x = result[0];
        odomToBaseLink.transform.translation.y = result[1];
        tf_pub->sendTransform(odomToBaseLink);
    }
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
void integrationMethodChangeCallback(project_1::integrationMethodConfig &config,
                                     uint32_t level)
{
    ROS_INFO("Reconfiguring integration mode to: %s",
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
}

bool resetStartingPose(project_1::ResetStartingPose::Request &req,
                       project_1::ResetStartingPose::Response &res)
{
    Eigen::Vector3d V;
    V[0] = req.new_x;
    V[1] = req.new_y;
    V[2] = req.new_theta;

    OdometryComputer::getInstance().setPosition(V);
    return true;
}