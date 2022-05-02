#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace ros;
using namespace sensor_msgs;

void wheelStateCallback(const JointState::ConstPtr &msg);

int main(int argc, char **argv)
{
    init(argc, argv, "listener");
    NodeHandle handle;

    Subscriber sub =
        handle.subscribe("/wheel_states", 1000, wheelStateCallback);

    spin();
}

void wheelStateCallback(const JointState::ConstPtr &msg)
{
    ROS_INFO("Joints count: %d", msg->name.size());

    for (int i = 0; i < msg->name.size(); i++)
        ROS_INFO("Joint %s: %f %f", msg->name[i].c_str(), msg->position[i],
                 msg->velocity[i]);
}