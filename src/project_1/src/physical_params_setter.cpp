#include <dynamic_reconfigure/server.h>
#include <project_1/robotPhysicalParametersConfig.h>

#include "shared/VelocityComputer.h"
#include "shared/WheelsVelocityComputer.h"

void robotParametersChangeCallback(project_1::robotPhysicalParametersConfig &config,
                                     uint32_t level);

using namespace ros;

int main(int argc, char **argv)
{
    // Node setup
    init(argc, argv, "physical_parameter_setter");
    // Setting up the dynamic server for dynamically reconfigure the robot params
    dynamic_reconfigure::Server<project_1::robotPhysicalParametersConfig> dynServer;
    dynamic_reconfigure::Server<
        project_1::robotPhysicalParametersConfig>::CallbackType callbackFunction;
    callbackFunction = boost::bind(&robotParametersChangeCallback, _1, _2);
    dynServer.setCallback(callbackFunction);

    spin();
}

void robotParametersChangeCallback(project_1::robotPhysicalParametersConfig &config,
                                     uint32_t level)
{
    ROS_INFO("Reconfiguring robot parameter");

    VelocityComputer::getInstance().setR(config.R);
    VelocityComputer::getInstance().setL(config.L);
    VelocityComputer::getInstance().setW(config.W);
    VelocityComputer::getInstance().setT(config.T);
    VelocityComputer::getInstance().setN(config.N);
    WheelsVelocityComputer::getInstance().setR(config.R);
    WheelsVelocityComputer::getInstance().setL(config.L);
    WheelsVelocityComputer::getInstance().setW(config.W);
    WheelsVelocityComputer::getInstance().setT(config.T);
    WheelsVelocityComputer::getInstance().setN(config.N);


}
