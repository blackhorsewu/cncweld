#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>

#include "boost/bind.hpp"
#include "boost/ref.hpp"

// Visualisation markers
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>

// Visual Tools
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <string>
#include <iostream>
#include <fstream>

#include <ctime>

using namespace std;

// values LJ Navigator uses for out-of-range points (in meters)
const static double KEYENCE_INFINITE_DISTANCE_VALUE_SI = -999.9990 / 1e3;
const static double KEYENCE_INFINITE_DISTANCE_VALUE_SI2 = -999.9970 / 1e3;

ros::Publisher pub; // publisher for the cumulated cloud; it will be initialised in main

pcl::PointCloud<pcl::PointXYZ> pcl_Y_cloud; // cumulated cloud

// needed for transformation from scanner frame to world frame
std::string sensor_host;
std::string scanner_frame;
std::string world_frame;

bool write_Y_file = false; // write the whole scan brief data to a file
bool write_X_file = false; // write data of each scan line to a file

ofstream Yfile;
ofstream Xfile;

int line_no = 0;
int file_no = 0;
  
// Point index of first and last valid points in a scan line
int valid_begin, valid_end;

/*
 * Find the deepest point of the input scan line (a cross section of the groove).
 */
void deepest_pt(pcl::PointCloud<pcl::PointXYZ> pointcloud)
{
  int cloudSize = pointcloud.size();
  //  double Z[cloudSize], avgz[cloudSize], avgdavgz[cloudSize], avgddavgz[cloudSize];
  //  double davgz[cloudSize], davgdavgz[cloudSize];

  double minZ = 0.0;
  double zz = 0.0;

  for (int i = 0; i < (cloudSize - 50); ++i) // Neglect the first 50 points
  {
    zz = pointcloud[i].z;
    // Check to make sure the point is a valid point, some points may be invalid
    if ((zz != KEYENCE_INFINITE_DISTANCE_VALUE_SI) && (zz != KEYENCE_INFINITE_DISTANCE_VALUE_SI2)
          && (zz != std::numeric_limits<double>::infinity()))
    {   // then this is a piece of normal point
      if (zz < minZ) minZ = zz;
    }
  }

  ROS_INFO("Minimum depth: %.2f mm", zz*1000);

/* Do not write to any file yet
  if (write_X_file)
  {
    Xfile.close();
    file_no++;
  }
*/
}

/*
 * This handles one scan line, published by the scanner driver.
 */
void callback(const sensor_msgs::PointCloud2ConstPtr& ros_cloud)
{
  // The publisher is initialised in main
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

  // Only ros cloud can be transformed!
  tf::TransformListener listener;
  tf::StampedTransform  stransform;
  try
  {
    listener.waitForTransform(world_frame,
                              ros_cloud->header.frame_id,
                              ros::Time::now(),
                              ros::Duration(0.3));
    listener.lookupTransform (world_frame,
                              ros_cloud->header.frame_id,
                              ros::Time(0),
                              stransform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  sensor_msgs::PointCloud2 transformed_ros_cloud;
  pcl_ros::transformPointCloud(world_frame,
                               stransform,
                               *ros_cloud,
                               transformed_ros_cloud);
  /*
   * pcl cloud is used because the operator += cannot work with ros cloud!
   */
  pcl::fromROSMsg(transformed_ros_cloud, pcl_cloud);
  pcl_Y_cloud.header.frame_id = pcl_cloud.header.frame_id; // cannot be done away with; must keep
  pcl_Y_cloud += pcl_cloud;
  deepest_pt(pcl_cloud); // try to find the deepest point of this cross section
  pub.publish(pcl_Y_cloud);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "concave_scanner");
  ros::NodeHandle nh, pnh("~");

  // Point Cloud topic
  std::string cloud_topic;
  
  cloud_topic = "profiles"; // The cloud published by the Keyence Driver
  // world_frame = "world";
  world_frame = "base_link";
  scanner_frame = "lj_v7200_optical_frame";

  // set up profile cloud publisher for PCL point clouds
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("Y_profiles", 1);

  /*
   * Listen for Point Cloud - profile from Laser Scanner
   */
  std::string topic = nh.resolveName(cloud_topic);
  ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic " << topic);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topic, 1, callback);

  ros::spin();
  return 0;
}
