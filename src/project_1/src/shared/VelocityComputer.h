#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Core>

#include "Singleton.h"

#pragma once

class VelocityComputer : public Singleton<VelocityComputer>
{
    friend class Singleton<VelocityComputer>;

public:
    enum class ComputeMethod
    {
        RMP,
        ENCODER
    };

    /**
     * @brief Set the compute method between RMP and ENCODER.
     */
    void setComputeMethod(ComputeMethod computeMethod);

    /**
     * @brief Changes the robot parameters.
     *
     * @param R Wheel radius [m]
     * @param L Wheel position along x [m]
     * @param W Wheel position along y [m]
     * @param T Gear ration from motor to wheel
     */
    void setRobotParameters(double R, double L, double W, double T, double N);

    /**
     * @brief Computes the robot velocity and publish data on the topic.
     */
    Eigen::Vector3d computeRobotVelocity(
        const sensor_msgs::JointState::ConstPtr& msg);

private:
    VelocityComputer();

    /**
     * @brief Initialize the transformation matrix used to compute the robot
     * velocity from wheels velocity.
     */
    void initTransformationMatrix();

    Eigen::Vector3d computeRobotVelocityWithRPM(
        const sensor_msgs::JointState::ConstPtr& msg);

    Eigen::Vector3d computeRobotVelocityWithENCODER(
        const sensor_msgs::JointState::ConstPtr& msg);

    /**
     * @brief Computes the robot velocity from wheels velocities.
     *
     * @param U Wheels velocities [rad/s]
     * @param deltaT Time delta between this and the previous measurement
     * [s]
     * @return Robot velocity [theta, x, t][m/s]
     */
    Eigen::Vector3d computeRobotVelocityFromWheelVelocity(Eigen::Vector4d U);

    ComputeMethod computeMethod = ComputeMethod::RMP;

    // Robot parameters
    double R = 0.07;       // Wheel radius [m]
    double L = 0.2;        // Wheel position along x [m]
    double W = 0.169;      // Wheel position along y [m]
    double T = 1.0 / 5.0;  // Gear ration from motor to wheel
    double N = 42;         // Counts per revolution of the motor

    // Transformation matrix
    Eigen::Matrix<double, 3, 4> M;

    // Previous timestamp and encoders position
    double previousTimestamp;
    Eigen::Vector4d previousE;
};

void VelocityComputer::setComputeMethod(ComputeMethod computeMethod)
{
    this->computeMethod = computeMethod;
}

void VelocityComputer::setRobotParameters(double R, double L, double W,
                                          double T, double N)
{
    this->R = R;
    this->L = L;
    this->W = W;
    this->T = T;
    this->N = N;

    initTransformationMatrix();
}

Eigen::Vector3d VelocityComputer::computeRobotVelocity(
    const sensor_msgs::JointState::ConstPtr& msg)
{
    switch (computeMethod)
    {
        case ComputeMethod::RMP:
            return computeRobotVelocityWithRPM(msg);
        case ComputeMethod::ENCODER:
            return computeRobotVelocityWithENCODER(msg);
        default:
            return Eigen::Vector3d::Zero();
    }
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

Eigen::Vector3d VelocityComputer::computeRobotVelocityWithRPM(
    const sensor_msgs::JointState::ConstPtr& msg)
{
    // Extract the wheels velocities from the encoders
    Eigen::Vector4d U;
    U(0) = msg->velocity[0];  // Front left
    U(1) = msg->velocity[1];  // Front right
    U(2) = msg->velocity[3];  // Rear right
    U(3) = msg->velocity[2];  // Rear left

    // Transform velocities from RMP to rad/s
    U = U / 60;

    // Apply the gear ratio 5:1
    U = U * T;

    // TODO: Why a second reduction is needed??????
    U = U * T;

    // Compute the robot velicity
    return computeRobotVelocityFromWheelVelocity(U);
}

Eigen::Vector3d VelocityComputer::computeRobotVelocityWithENCODER(
    const sensor_msgs::JointState::ConstPtr& msg)
{
    // Extract the encoders positions
    Eigen::Vector4d E;
    E(0) = msg->position[0];  // Front left
    E(1) = msg->position[1];  // Front right
    E(2) = msg->position[3];  // Rear right
    E(3) = msg->position[2];  // Rear left

    // Compute the delta values
    double deltaT          = msg->header.stamp.toSec() - previousTimestamp;
    Eigen::Vector4d deltaE = E - previousE;

    // Save current for the next iteration
    previousTimestamp = msg->header.stamp.toSec();
    previousE         = E;

    // Compute the wheels velocities
    Eigen::Vector4d U = deltaE / deltaT;

    // Transform the velocity from counts/s to rad/s
    U = U / N * 2 * M_PI;

    // Apply the gear ratio 5:1
    U = U * T;

    // Compute the robot velicity
    return computeRobotVelocityFromWheelVelocity(U);
}

Eigen::Vector3d VelocityComputer::computeRobotVelocityFromWheelVelocity(
    Eigen::Vector4d U)
{
    return (R / 4) * M * U;
}
