#include <ros/ros.h>
#include <ros/package.h>
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

    /**
     * We use 2 subscribers:
     *  (*) /amcl_pose topic to save the current estimated robot position.
     *  (*) /map to save locally the OccupancyGrid we are using.
     */
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

/**
 * @brief Updates the path followed by the robot.
 * 
 * Once AMCL is running, it publishes the robot current pose with covariance into the /amcl_pose
 * topic. This method stores that position into a nav_msgs::Path object.
 * 
 * @param currentPose the pose with covariance published by AMCL under the /amcl_pose topic. 
 */
void currentPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &currentPose) {
    geometry_msgs::PoseStamped newPose;

    newPose.header = currentPose.header;
    newPose.pose = currentPose.pose.pose;

    robotTrajectory.poses.push_back(newPose);
    pub.publish(robotTrajectory);
}

/**
 * @brief Update the local version of the OccupancyGrid whenever it gets updated by the map server.
 * 
 * @param updatedMap the OccupancyGrid update coming from the map server.
 */
void mapUpdateCallback(const nav_msgs::OccupancyGrid &updatedMap) {
    currentMap = updatedMap;
}

/**
 * @brief callback for the SaveMapWithTrajectory service. It takes the local Path object with all the poses the robot has been until now
 * and plots them onto the OccupancyGrid, finally saving the computed image locally into the robot_trajectory folder.
 */
bool saveMapWithTrajectory(project_2::SaveRobotTrajectory::Request &req, project_2::SaveRobotTrajectory::Response &res) {
    ROS_INFO("Received request to save the map with the trajectory.");
    int pixelX, pixelY;
    int lastPixelXColored = -1;
    int lastPixelYColored = -1;

    cv::Mat imageFromOccupancyGrid = occupancyGridToCvMat(currentMap);

    // Setting up the color for the trajectory (BGR format) (in this case (0,153,0) == GREEN)
    cv::Vec3b trajectoryColor;
    trajectoryColor[0] = 0;
    trajectoryColor[1] = 153;
    trajectoryColor[2] = 0;

    /** Foreach pose saved into the local Path object:
     *      (*) If that pose is the first one it draws a point;
     *      (*) if it's not the first pose examined, it draws a line between the current pixel examined and the last one.
    */
    for(size_t i=0; i < robotTrajectory.poses.size(); i++) {
        /**
         * pixelX = (robotPoseX + mapOriginX) / mapResolution  (see more infos in map_bag1.yaml)
         * pixelY = (robotPoseY + mapOriginY) / mapResolution  (see more infos in map_bag1.yaml)
         */
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
    cv::imwrite(package::getPath("project_2").append("/robot_trajectory/map_with_trajectory.png"), imageFromOccupancyGrid);

    res.service_status = true;
    return true;
}

/**
 * @brief Converts an OccupancyGrid object into an OpenCv Mat object (easier to edit).
 * 
 * @param map the OccupancyGrid to convert.
 * @return cv::Mat the OccupancyGrid transformed into an OpenCv::Mat object.
 */
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