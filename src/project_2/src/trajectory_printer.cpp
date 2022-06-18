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
    int pixelX, pixelY;
    int lastPixelXColored = -1;
    int lastPixelYColored = -1;

    cv::Mat imageFromOccupancyGrid = occupancyGridToCvMat(currentMap);

    // Setting up the color for the trajectory
    cv::Vec3b trajectoryColor;
    trajectoryColor[0] = 0;
    trajectoryColor[1] = 153;
    trajectoryColor[2] = 0;


    for(size_t i=0; i < robotTrajectory.poses.size(); i++) {
        pixelX = (robotTrajectory.poses[i].pose.position.x + abs(currentMap.info.origin.position.x)) / currentMap.info.resolution;
        pixelY = (robotTrajectory.poses[i].pose.position.y + abs(currentMap.info.origin.position.y)) / currentMap.info.resolution;

        imageFromOccupancyGrid.at<cv::Vec3b>(cv::Point(pixelX,pixelY)) = trajectoryColor;

        if(lastPixelXColored != -1 && lastPixelYColored != -1) {
            cv::line(imageFromOccupancyGrid, cv::Point(lastPixelXColored, lastPixelYColored), cv::Point(pixelX, pixelY), trajectoryColor);
        }

        lastPixelXColored = pixelX;
        lastPixelYColored = pixelY;
    }

    cv::flip(imageFromOccupancyGrid,imageFromOccupancyGrid, 0);
    cv::rotate(imageFromOccupancyGrid, imageFromOccupancyGrid, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::imwrite("/home/paolo/Scrivania/map_with_trajectory.png", imageFromOccupancyGrid);

    return true;
}

cv::Mat occupancyGridToCvMat(const nav_msgs::OccupancyGrid map)
{
  uint8_t *data = (uint8_t*) map.data.data(),
           testpoint = data[0];
  bool mapHasPoints = false;
  
  // Creating the image and the color to use.
  cv::Mat im(map.info.height, map.info.width, CV_8UC3);
  cv::Vec3b colorBlack;
  cv::Vec3b colorWhite;
  cv::Vec3b colorGrey;
  
  colorBlack[0] = 0;
  colorBlack[1] = 0;
  colorBlack[2] = 0;
  colorWhite[0] = 254;
  colorWhite[1] = 254;
  colorWhite[2] = 254;
  colorGrey[0] = 205;
  colorGrey[1] = 205;
  colorGrey[2] = 205;
  
  // transform the map in the same way the map_saver component does
  for (size_t i=0; i<map.data.size(); i++)
  {
    if (data[i] == 0)        im.at<cv::Vec3b>(cv::Point(i%currentMap.info.width,i/currentMap.info.height)) = colorWhite;
    else if (data[i] == 100) im.at<cv::Vec3b>(cv::Point(i%currentMap.info.width,i/currentMap.info.height)) = colorBlack;
    else im.at<cv::Vec3b>(cv::Point(i%currentMap.info.width,i/currentMap.info.height)) = colorGrey;

    // just check if there is actually something in the map
    if (i!=0) mapHasPoints = mapHasPoints || (data[i] != testpoint);
    testpoint = data[i];
  }

  // sanity check
  if (!mapHasPoints) { ROS_WARN("map is empty, ignoring update."); }

  return im;
}