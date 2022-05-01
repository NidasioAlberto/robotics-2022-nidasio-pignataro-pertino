#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <Eigen/Core>

using namespace ros;
using namespace Eigen;

Vector3d referencePosition;
Vector3d odometryPosition;
Publisher pub;

void referenceCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
void odometryCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

int main(int argc, char **argv)
{
    // Node setup
    init(argc, argv, "position_comparison");
    NodeHandle handle;

    // Setting up the services server
    Subscriber sub1 = handle.subscribe("/robot/pose", 1000, referenceCallback);
    Subscriber sub2 = handle.subscribe("/pose", 1000, odometryCallback);
    pub             = handle.advertise<std_msgs::Float64>("distance", 1000);

    spin();
}

void referenceCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    referencePosition[0] = msg->pose.position.x;
    referencePosition[1] = msg->pose.position.y;
    referencePosition[2] = msg->pose.position.z;

    // Compute the distance between reference and odometry
    Vector3d diff = referencePosition - odometryPosition;
    std_msgs::Float64 distance;
    distance.data = diff.norm();
    pub.publish(distance);
}

void odometryCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    odometryPosition[0] = msg->pose.position.x;
    odometryPosition[1] = msg->pose.position.y;
    odometryPosition[2] = msg->pose.position.z;
}

/*

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"


void callback(const geometry_msgs::Vector3StampedConstPtr& msg1, const
geometry_msgs::Vector3StampedConstPtr& msg2)
{
  ROS_INFO ("Received two messages: (%f,%f,%f) and (%f,%f,%f)",
msg1->vector.x,msg1->vector.y,msg1->vector.z, msg2->vector.x, msg2->vector.y,
msg2->vector.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "subscriber_sync");

  ros::NodeHandle n;

  message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub1(n, "topic1",
1); message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub2(n, "topic2",
1);

  //typedef
message_filters::sync_policies::ExactTime<geometry_msgs::Vector3Stamped,
geometry_msgs::Vector3Stamped> MySyncPolicy;
  // Smarter way to deal with timestamps which differs for some nanosecs.
They'll be considered symoultanously sent, so the callback will be called
  // even if the timestamps slightly differ.
  typedef
message_filters::sync_policies::ApproximateTime<geometry_msgs::Vector3Stamped,
geometry_msgs::Vector3Stamped> MySyncPolicy;

  // Synchronizer: it lists all the messages types (can be different types)
message_filters::Synchronizer<MySyncPolicy>
sync(MySyncPolicy(10), sub1, sub2);
sync.registerCallback(boost::bind(callback,_1, _2));

  ros::spin();

  return 0;
}
*/