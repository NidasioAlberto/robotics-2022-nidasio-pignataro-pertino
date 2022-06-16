#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <project_2/SaveRobotTrajectory.h>

using namespace ros;

void currentPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &currentPose);
void mapUpdateCallback(const nav_msgs::OccupancyGrid &updatedMap);
bool saveMapWithTrajectory(project_2::SaveRobotTrajectory::Request &req, project_2::SaveRobotTrajectory::Response &res);
cv::Mat occupancyGridToCvMat(const nav_msgs::OccupancyGrid map);

Publisher pub;
nav_msgs::Path robotTrajectory;
nav_msgs::OccupancyGrid currentMap;

int main(int argc, char **argv)
{
    // Node setup
    init(argc, argv, "trajectory_printer");
    NodeHandle handle;

    Subscriber AMCLPoseSubscriber = handle.subscribe("amcl_pose", 1000, currentPoseCallback); // Listen to amcl_pose topic
    Subscriber MAPTopicSubscriber = handle.subscribe("map", 1000, mapUpdateCallback);

    // Publisher for the path in order to visualize it on rviz.
    pub = handle.advertise<nav_msgs::Path>("trajectory", 1000);

    // Setting up the services server
    ServiceServer startingPoseResetterService =
        handle.advertiseService<project_2::SaveRobotTrajectory::Request,
                                project_2::SaveRobotTrajectory::Response>(
            "save_map_with_trajectory", boost::bind(&saveMapWithTrajectory, _1, _2));

    // Setting up the trajectory with absolutely random vars XD
    robotTrajectory.header.frame_id = "map";
    robotTrajectory.header.stamp = Time::now();
    robotTrajectory.header.seq = 45;

    spin();
}

void currentPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &currentPose) {
    geometry_msgs::PoseStamped newPose;

    newPose.header = currentPose.header;
    newPose.pose = currentPose.pose.pose;

    robotTrajectory.poses.push_back(newPose);
    pub.publish(robotTrajectory);
}

void mapUpdateCallback(const nav_msgs::OccupancyGrid &updatedMap) {
    currentMap = updatedMap;
}

bool saveMapWithTrajectory(project_2::SaveRobotTrajectory::Request &req, project_2::SaveRobotTrajectory::Response &res) {
    ROS_INFO("Received request to save the map with the trajectory.");

    cv::Mat imageFromOccupancyGrid = occupancyGridToCvMat(currentMap);

    cv::imwrite("/home/paolo/Scrivania/lol.png", imageFromOccupancyGrid);

    return true;
}

cv::Mat occupancyGridToCvMat(const nav_msgs::OccupancyGrid map)
{
  uint8_t *data = (uint8_t*) map.data.data(),
           testpoint = data[0];
  bool mapHasPoints = false;

  cv::Mat im(map.info.height, map.info.width, CV_8UC1);
  
  ROS_INFO("Map size %zd", map.data.size());
  

  // transform the map in the same way the map_saver component does
  for (size_t i=0; i<map.data.size(); i++)
  {
    if (data[i] == 0)        im.data[i] = 254;
    else if (data[i] == 100) im.data[i] = 0;
    else im.data[i] = 205;

    // just check if there is actually something in the map
    if (i!=0) mapHasPoints = mapHasPoints || (data[i] != testpoint);
    testpoint = data[i];
  }

  // sanity check
  if (!mapHasPoints) { ROS_WARN("map is empty, ignoring update."); }

  return im;
}