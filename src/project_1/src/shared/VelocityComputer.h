#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Core>

#include "Singleton.h"

#pragma once

class VelocityComputer : public Singleton<VelocityComputer>
{
    friend class Singleton<VelocityComputer>;

public:
    /**
     * @brief Changes the robot parameters.
     *
     * @param R Wheel radius [m]
     * @param L Wheel position along x [m]
     * @param W Wheel position along y [m]
     * @param T Gear ration from motor to wheel
     */
    void setRobotParameters(double R, double L, double W, double T);

    /**
     * @brief Computes the robot velocity and publish data on the topic.
     */
    Eigen::Vector3d computeRobotVelocity(
        const sensor_msgs::JointState::ConstPtr &msg);

    /**
     * @brief Computes the robot velocity from wheels velocities.
     *
     * @param U Wheels velocities [rad/s]
     * @param deltaT Time delta between this and the previous measurement [s]
     * @return Robot velocity [theta, x, t][m/s]
     */
    Eigen::Vector3d computeRobotVelocity(Eigen::Vector4d U);

private:
    VelocityComputer();

    /**
     * @brief Initialize the transformation matrix used to compute the robot
     * velocity from wheels velocity.
     */
    void initTransformationMatrix();

    // Robot parameters
    double R = 0.07;       // Wheel radius [m]
    double L = 0.2;        // Wheel position along x [m]
    double W = 0.169;      // Wheel position along y [m]
    double T = 1.0 / 5.0;  // Gear ration from motor to wheel

    Eigen::Matrix<double, 3, 4> M;
};

void VelocityComputer::setRobotParameters(double R, double L, double W,
                                          double T)
{
    this->R = R;
    this->L = L;
    this->W = W;
    this->T = T;

    initTransformationMatrix();
}

Eigen::Vector3d VelocityComputer::computeRobotVelocity(
    const sensor_msgs::JointState::ConstPtr &msg)
{
    // Extract the wheels velocities
    Eigen::Vector4d U;
    U(0) = msg->velocity[0];  // Front left
    U(1) = msg->velocity[1];  // Front right
    U(2) = msg->velocity[3];  // Rear right
    U(3) = msg->velocity[2];  // Rear left

    // Transform velocities from RMP to rad/s
    U = U * 2 * M_PI / 60;

    // Apply the gear ratio 5:1
    U = U / T;

    // Compute the robot velicity
    return computeRobotVelocity(U);
}

Eigen::Vector3d VelocityComputer::computeRobotVelocity(Eigen::Vector4d U)
{
    return (R / 4) * M * U;
}

VelocityComputer::VelocityComputer() { initTransformationMatrix(); }

void VelocityComputer::initTransformationMatrix()
{
    // Conversion matrix
    // clang-format off
    M << -1.0 / (L + W), 1.0 / (L + W), 1.0 / (L + W), -1.0 / (L + W),
          1.0,           1.0,           1.0,            1.0,
         -1.0,           1.0,          -1.0,            1.0;
    // clang-format on
}