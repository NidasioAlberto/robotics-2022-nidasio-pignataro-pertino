#include <geometry_msgs/TwistStamped.h>

#include <Eigen/Core>

#include "Singleton.h"

#pragma once

/**
 * Euler integration:
 *   X(k + 1) = X(k) + V(k) * T * cos(Theta(k))
 *   Y(k + 1) = Y(k) + V(k) * T * sin(Theta(k))
 *   Theta(k + 1) = Theta(k) + w(k) * T
 *   T = t(k + 1) - t(k)
 *
 * Runge-Kutta integration:
 *   X(k + 1) = X(k) + V(k) * T * cos(Theta(k) + w(k) * T / 2)
 *   Y(k + 1) = Y(k) + V(k) * T * sin(Theta(k) + w(k) * T / 2)
 *   Theta(k + 1) = Theta(k) + w(k) * T
 *   T = t(k + 1) - t(k)
 */

class OdometryComputer : public Singleton<OdometryComputer>
{
    friend class Singleton<OdometryComputer>;

public:
    enum class IntegrationMethod
    {
        EULER,
        RUNGE_KUTTA
    };

    /**
     * @brief Computes the odometry given the linear and angular velocities.
     *
     * @param msg The twisted msg received from the topic
     * @return The robot linear position and rotation [x y theta]
     */
    Eigen::Vector3d computeOdometry(
        const geometry_msgs::TwistStamped::ConstPtr &msg);

    /**
     * @brief Set the integration method between EULER and RUNGE_KUTTA.
     */
    void setIntegrationMethod(const IntegrationMethod integrationMethod);

    /**
     * @brief Override the robot's position.
     *
     * @param position Linear and angular position [x y theta]
     */
    void setPosition(const Eigen::Vector3d position);

private:
    OdometryComputer();

    Eigen::Vector3d computeOdometryEuler(
        const geometry_msgs::TwistStamped::ConstPtr &msg);

    Eigen::Vector3d computeOdometryRunge(
        const geometry_msgs::TwistStamped::ConstPtr &msg);

    IntegrationMethod integrationMethod;

    Eigen::Vector3d position =
        Eigen::Vector3d::Zero();  // Position in world frame [x y theta]

    // Previous timestamp and velocities
    double previousTimestamp = 0;
};

void OdometryComputer::setIntegrationMethod(
    const IntegrationMethod integrationMethod)
{
    this->integrationMethod = integrationMethod;
}

void OdometryComputer::setPosition(const Eigen::Vector3d position)
{
    this->position          = position;
    this->previousTimestamp = 0;
}

Eigen::Vector3d OdometryComputer::computeOdometry(
    const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    // Dispatch the request depending on the integration method
    switch (integrationMethod)
    {
        case IntegrationMethod::EULER:
            return computeOdometryEuler(msg);
        case IntegrationMethod::RUNGE_KUTTA:
            return computeOdometryRunge(msg);
        default:
            return Eigen::Vector3d::Zero();
    }
}

OdometryComputer::OdometryComputer() {}

Eigen::Vector3d OdometryComputer::computeOdometryEuler(
    const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    // Compute the time delta
    double deltaT = msg->header.stamp.toSec() - previousTimestamp;

    // Compute the position only after the first message
    if (previousTimestamp != 0 && deltaT > 0)
    {
        // Extract the robot's velocities
        Eigen::Vector3d V;
        V[0] = msg->twist.linear.x;   // x
        V[1] = msg->twist.linear.y;   // y
        V[2] = msg->twist.angular.z;  // theta

        // Prepare the rotation matrix for the velocities
        // clang-format off
        Eigen::Matrix3d rotationMatrix;
        rotationMatrix << cos(position[2]), -sin(position[2]), 0,
                          sin(position[2]),  cos(position[2]), 0,
                          0,                 0,                1;
        // clang-format on

        // Rotate the reported robot velocities in the current world frame
        V = rotationMatrix * V;

        // Integrate
        position = position + V * deltaT;
    }

    // Save current timestamp for next iteration
    previousTimestamp = msg->header.stamp.toSec();

    return position;
}

Eigen::Vector3d OdometryComputer::computeOdometryRunge(
    const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    // Compute the time delta
    double deltaT = msg->header.stamp.toSec() - previousTimestamp;

    // Compute the position only after the first message
    if (previousTimestamp != 0 && deltaT > 0)
    {
        // Extract the robot's velocities
        Eigen::Vector3d V;
        V[0] = msg->twist.linear.x;   // x
        V[1] = msg->twist.linear.y;   // y
        V[2] = msg->twist.angular.z;  // theta

        // Prepare the rotation matrix for the velocities
        // clang-format off
        Eigen::Matrix3d rotationMatrix;
        rotationMatrix << cos(position[2] + V[2] * deltaT / 2), -sin(position[2] + V[2] * deltaT / 2), 0,
                          sin(position[2] + V[2] * deltaT / 2),  cos(position[2] + V[2] * deltaT / 2), 0,
                          0,                                     0,                                    1;
        // clang-format on

        // Rotate the reported robot velocities in the current world
        // frame
        V = rotationMatrix * V;

        // Integrate
        position = position + V * deltaT;
    }

    // Save current timestamp for next iteration
    previousTimestamp = msg->header.stamp.toSec();

    return position;
}