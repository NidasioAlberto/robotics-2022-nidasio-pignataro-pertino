#pragma once

#include <geometry_msgs/TwistStamped.h>

#include <Eigen/Core>

#include "Singleton.h"

class WheelsVelocityComputer : public Singleton<WheelsVelocityComputer>
{
    friend class Singleton<WheelsVelocityComputer>;

public:
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
     * @brief Computes the wheels velocity from the robot motion.
     *
     * @param msg The TwistStamped message describing the robot velocities.
     * @return The wheels velocities [fl fr rr rl].
     */
    Eigen::Vector4d computeWheelVelocities(
        const geometry_msgs::TwistStamped::ConstPtr& msg);

private:
    WheelsVelocityComputer();

    /**
     * @brief Initialize the transformation matrix used to compute the robot
     * velocity from wheels velocity.
     */
    void initTransformationMatrix();

    // Robot parameters
    double R = 0.075797;   // Wheel radius [m]
    double L = 0.2;        // Wheel position along x [m]
    double W = 0.158886;   // Wheel position along y [m]
    double T = 1.0 / 5.0;  // Gear ratio from motor to wheel
    double N = 42;         // Counts per revolution of the motor

    // Transformation matrix
    Eigen::Matrix<double, 4, 3> M;
};

void WheelsVelocityComputer::setR(double R) { this->R = R; }

void WheelsVelocityComputer::setL(double L)
{
    this->L = L;
    initTransformationMatrix();
}

void WheelsVelocityComputer::setW(double W)
{
    this->W = W;
    initTransformationMatrix();
}

void WheelsVelocityComputer::setT(double T) { this->T = T; }

void WheelsVelocityComputer::setN(double N) { this->N = N; }

Eigen::Vector4d WheelsVelocityComputer::computeWheelVelocities(
    const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    Eigen::Vector3d V;
    V[0] = msg->twist.angular.z;
    V[1] = msg->twist.linear.x;
    V[2] = msg->twist.linear.y;

    return (1 / R) * M * V * 60 * 5;
}

WheelsVelocityComputer::WheelsVelocityComputer() { initTransformationMatrix(); }

void WheelsVelocityComputer::initTransformationMatrix()
{
    // Conversion matrix
    // clang-format off
    M << - L - W, 1.0, -1.0,
         + L + W, 1.0,  1.0,
         + L + W, 1.0, -1.0,
         - L - W, 1.0,  1.0;
    // clang-format on
}