#include <geometry_msgs/TwistStamped.h>

#include <Eigen/Core>

#include "Singleton.h"

#pragma once

/**
 * Euler integration:
 * X(k + 1) = X(k) + V(k) * T * cos(Theta(k))
 * Y(k + 1) = Y(k) + V(k) * T * sin(Theta(k))
 * Theta(k + 1) = Theta(k) + w(k) * T
 * T = t(k + 1) - t(k)
 *
 * Runge-Kutta integration:
 * X(k + 1) = X(k) + V(k) * T * cos(Theta(k) + w(k) * T / 2)
 * Y(k + 1) = Y(k) + V(k) * T * sin(Theta(k) + w(k) * T / 2)
 * Theta(k + 1) = Theta(k) + w(k) * T
 * T = t(k + 1) - t(k)
 */

class OdometryComputer : public Singleton<OdometryComputer>
{
    friend class Singleton<OdometryComputer>;

public:
    enum class OdometryIntegration
    {
        EULER,
        RUNGE_KUTTA
    };

    /**
     * @brief Computes the odometry given the linear velocity and angular
     * velocity
     *
     * @param msg The twisted msg received from the topic
     * @return Eigen::Vector3d The vector that contains the 2d position and
     * theta
     */
    Eigen::Vector3d computeOdometry(
        const geometry_msgs::TwistStamped::ConstPtr &msg);

    /**
     * @brief Set the Integration Method object
     *
     * @param i The method integration
     */
    void setIntegrationMethod(const OdometryIntegration i);

    /**
     * @brief Set the Initial Position of the robot
     *
     * @param position The 3d eigen vector with posX, posY and theta format
     */
    void setInitialPosition(const Eigen::Vector3d position);

private:
    /**
     * @brief Private constructor for singleton purposes
     */
    OdometryComputer() {}

    /**
     * @brief Method to compute the odometry following the Euler integration
     * method
     *
     * @param msg The message to compute
     * @return Eigen::Vector3d The vector of results
     */
    Eigen::Vector3d computeOdometryEuler(
        const geometry_msgs::TwistStamped::ConstPtr &msg);

    /**
     * @brief Method to compute the odometry following the Runge-Kutta
     * integration method
     *
     * @param msg The message to compute
     * @return Eigen::Vector3d The vector of results
     */
    Eigen::Vector3d computeOdometryRunge(
        const geometry_msgs::TwistStamped::ConstPtr &msg);

    /**
     * @brief Internal integration method
     */
    OdometryIntegration integration;

    /**
     * @brief Initial position
     * FORMAT: PosX, PosY, Theta
     */
    Eigen::Vector3d initialPosition;
};

void OdometryComputer::setIntegrationMethod(const OdometryIntegration i)
{
    this->integration = i;
}

void OdometryComputer::setInitialPosition(const Eigen::Vector3d position)
{
    this->initialPosition[0] = position[0];
    this->initialPosition[1] = position[1];
    this->initialPosition[2] = position[2];
}

Eigen::Vector3d OdometryComputer::computeOdometry(
    const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    // Dispatch the request depending on the integration method
    switch (integration)
    {
        case OdometryIntegration::EULER:
            return computeOdometryEuler(msg);
        case OdometryIntegration::RUNGE_KUTTA:
            return computeOdometryRunge(msg);
        default:
            return Eigen::Vector3d::Zero();
    }
}

Eigen::Vector3d OdometryComputer::computeOdometryEuler(
    const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    // Informations about the previous step
    static double previousTime              = 0;
    static Eigen::Vector3d previousVelocity = Eigen::Vector3d::Zero();

    // Calculate the delta of time
    double T = msg->header.stamp.toSec() - previousTime;

    // Calculate and rotate the velocities
    Eigen::Vector2d V;
    V[0] = previousVelocity[0];
    V[1] = previousVelocity[1];

    Eigen::Matrix2d rotationMatrix;
    rotationMatrix << cos(initialPosition[2]), -sin(initialPosition[2]),
        sin(initialPosition[2]), cos(initialPosition[2]);

    std::cout << "rotationMatrix" << std::endl;
    std::cout << rotationMatrix << std::endl;

    V = rotationMatrix * V;

    std::cout << "V" << std::endl;
    std::cout << V << std::endl;

    // Integrate
    initialPosition[0] = initialPosition[0] + V[0] * T;
    initialPosition[1] = initialPosition[1] + V[1] * T;
    initialPosition[2] = initialPosition[2] + previousVelocity[2] * T;

    // Save current timestamp for next iteration
    previousTime = msg->header.stamp.toSec();

    // Set the new velocities
    previousVelocity[0] = msg->twist.linear.x;
    previousVelocity[1] = msg->twist.linear.y;
    previousVelocity[2] = msg->twist.angular.z;

    // Return the result
    Eigen::Vector3d result;
    result[0] = initialPosition[0];
    result[1] = initialPosition[1];
    result[2] = initialPosition[2];
    return result;
}

//TODO To be tried
Eigen::Vector3d OdometryComputer::computeOdometryRunge(
    const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    // Informations about the previous step
    static double previousTime              = 0;
    static Eigen::Vector3d previousVelocity = Eigen::Vector3d::Zero();

    // Calculate the delta of time
    double T = msg->header.stamp.toSec() - previousTime;

    // Calculate and rotate the velocities
    Eigen::Vector2d V;
    V[0] = previousVelocity[0];
    V[1] = previousVelocity[1];

    Eigen::Matrix2d rotationMatrix;
    rotationMatrix << cos(initialPosition[2] + previousVelocity[2] * T / 2), -sin(initialPosition[2] + previousVelocity[2] * T / 2),
        sin(initialPosition[2] + previousVelocity[2] * T / 2), cos(initialPosition[2] + previousVelocity[2] * T / 2);

    //Rotate
    V = rotationMatrix * V;

    // clang-format off
    // Integrate
    initialPosition[0] = initialPosition[0] + V[0] * T;
    initialPosition[1] = initialPosition[1] + V[1] * T;
    initialPosition[2] = initialPosition[2] + previousVelocity[2] * T;
    // clang-format on

    // Set the new previous time
    previousTime = msg->header.stamp.toSec();

    // Set the new velocities
    previousVelocity[0] = msg->twist.linear.x;
    previousVelocity[1] = msg->twist.linear.y;
    previousVelocity[2] = msg->twist.angular.z;

    // Return the result
    Eigen::Vector3d result;
    result[0] = initialPosition[0];
    result[1] = initialPosition[1];
    result[2] = initialPosition[2];
    return result;
}