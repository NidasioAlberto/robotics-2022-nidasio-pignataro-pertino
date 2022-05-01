#pragma once

#include <sensor_msgs/JointState.h>

#include <Eigen/Core>

#include "Singleton.h"

/**
 * @brief The VelocityComputer calculate the robot's speed from all the 4
 * meccanum wheels velocities.
 *
 * The computeRobotVelocity function is used to get the robot's velocity given a
 * JointState message which the follogwing information:
 * - velocity: All 4 wheels velocities in radiants per seconds
 * - position: All 4 wheels encoders incremental count
 *
 * The order of the 4 joints in the JointState message must be:
 * - Front left
 * - Front right
 * - Rear left
 * - Read right
 *
 * Two compute method are available to compute the robot's velocity:
 * - RMP: The wheels' rad/min are used
 * - ENCODER: The encoders' count are used
 */
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
     * @brief Changes the robot wheel radius.
     *
     * @param R Wheel radius [m]
     */
    void setR(double R);

    /**
     * @brief Changes the robot wheel position along the x axis.
     *
     * @param L Wheel position along x [m]
     */
    void setL(double L);

    /**
     * @brief Changes the robot wheel position along the y axis.
     *
     * @param W Wheel position along y [m]
     */
    void setW(double W);

    /**
     * @brief Changes the robot gear ratio.
     *
     * @param T Gear ratio from motor to wheel
     */
    void setT(double T);

    /**
     * @brief Changes the robot encoders resolution.
     *
     * @param N Encoders resolution [counts/revolution]
     */
    void setN(double N);

    /**
     * @brief Computes the robot velocity from the wheels motion.
     *
     * @param msg The JointState message describing the wheels condition.
     * @return The robot's linear and angular velocities [vx vy vtheta]
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
     * @param deltaT Time delta between this and the previous measurement [s]
     * @return Robot velocity [theta, x, t][m/s]
     */
    Eigen::Vector3d computeRobotVelocityFromWheelVelocity(Eigen::Vector4d U);

    ComputeMethod computeMethod = ComputeMethod::RMP;

    // Robot parameters
    double R = 0.07;       // Wheel radius [m]
    double L = 0.2;        // Wheel position along x [m]
    double W = 0.169;      // Wheel position along y [m]
    double T = 1.0 / 5.0;  // Gear ratio from motor to wheel
    double N = 42;         // Counts per revolution of the motor

    // Transformation matrix
    Eigen::Matrix<double, 3, 4> M;

    // Previous timestamp and encoders position
    double previousTimestamp  = 0;
    Eigen::Vector4d previousE = Eigen::Vector4d::Zero();
};

void VelocityComputer::setComputeMethod(ComputeMethod computeMethod)
{
    this->computeMethod = computeMethod;
}

void VelocityComputer::setR(double R) { this->R = R; }

void VelocityComputer::setL(double L)
{
    this->L = L;
    initTransformationMatrix();
}

void VelocityComputer::setW(double W)
{
    this->W = W;
    initTransformationMatrix();
}

void VelocityComputer::setT(double T) { this->T = T; }

void VelocityComputer::setN(double N) { this->N = N; }

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
    //std::cout << "Wheel radius " << R << std::endl;
    // Extract the wheels velocities from the encoders
    Eigen::Vector4d U;
    U[0] = msg->velocity[0];  // Front left
    U[1] = msg->velocity[1];  // Front right
    U[2] = msg->velocity[3];  // Rear right
    U[3] = msg->velocity[2];  // Rear left

    // Transform velocities from rad/min to rad/s
    U = U / 60;

    // Apply the gear ratio 5:1
    U = U * T;

    // Compute the robot velicity
    return computeRobotVelocityFromWheelVelocity(U);
}

Eigen::Vector3d VelocityComputer::computeRobotVelocityWithENCODER(
    const sensor_msgs::JointState::ConstPtr& msg)
{
    Eigen::Vector4d U = Eigen::Vector4d::Zero();  // Wheels velocities [rad/s]

    // Extract the encoders positions
    Eigen::Vector4d E;
    E[0] = msg->position[0];  // Front left
    E[1] = msg->position[1];  // Front right
    E[2] = msg->position[3];  // Rear right
    E[3] = msg->position[2];  // Rear left

    // Compute the time delta
    double deltaT = msg->header.stamp.toSec() - previousTimestamp;

    // Compute the velocities only after the first message
    if (previousTimestamp != 0 && deltaT > 0)
    {
        // Compute the encoders tick delta
        Eigen::Vector4d deltaE = E - previousE;

        // Compute the wheels velocities
        U = deltaE / deltaT;

        // Transform the velocity from counts/s to rad/s
        U = U / N * 2 * M_PI;

        // Apply the gear ratio 5:1
        U = U * T;
    }

    // Save current values for the next iteration
    previousTimestamp = msg->header.stamp.toSec();
    previousE         = E;

    // Compute the robot velicity
    return computeRobotVelocityFromWheelVelocity(U);
}

Eigen::Vector3d VelocityComputer::computeRobotVelocityFromWheelVelocity(
    Eigen::Vector4d U)
{
    return (R / 4) * M * U;
}
