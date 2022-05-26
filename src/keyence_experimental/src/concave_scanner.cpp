/*********1*********2*********3*********4*********5*********6*********7**********
 *                                                                              *
 *     Chinese National Engineering Research Centre for Steel Construction      *
 *                                (Hong Kong Branch)                            *
 *                                                                              *
 * This file, concave_scanner.cpp tries to work out the welding groove from a   *
 * laser scanned point cloud. The point cloud is published by the Keyence Driver*
 * node, which was downloaded from ROS Industrial.                              *
 * 
 * This program subscribes to the topic: /profiles published by Keyence Driver  *
 * node. It then concantenate these individual point clouds of a scan line into *
 * a cumulated surface point cloud. This point cloud is then published via the  *
 * topic: /Y_profiles. At the same time, this program also works out the deepest*
 * point of every individual scan line and publish these points as Visualization*
 * markers. They are then visualized in RViz. These points will then be the way *
 * points of the welding path the CNC Welding will follow.                      *
 *                                                                              *
 * Author: Victor Wai Hung WU                                                   *
 * Date: 26 May 2022.                                                           *
 *                                                                              *
 ********************************************************************************/
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

ros::Publisher pub;     // publisher for the cumulated cloud; it will be initialised in main
ros::Publisher mkr_pub; // publisher for the deepest points; it will be initialised in main
int marker_id = 0;

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

void publish_deepest_pt(pcl::PointXYZ deepest_point)
{
  // setup deepest point Marker message
  visualization_msgs::Marker dpst_pt; 
  dpst_pt.header.frame_id = world_frame;
  dpst_pt.header.stamp = ros::Time();
  dpst_pt.ns = "dpst_pt";
  dpst_pt.action = visualization_msgs::Marker::ADD;
  dpst_pt.pose.orientation.w = 1.0; // Quarternion
  dpst_pt.id = marker_id++; // This should be incremented by an number counting number of points published
  dpst_pt.type = visualization_msgs::Marker::CUBE;
  dpst_pt.scale.x = 0.0001; // so the line is shown as of 0.1mm wide
  dpst_pt.scale.y = 0.0001;
  dpst_pt.scale.z = 0.0001;
  dpst_pt.color.r = 1;   // in red
  dpst_pt.color.a = 1;   //

  // set the location of the deepest point
  dpst_pt.pose.position.x = deepest_point.x;
  dpst_pt.pose.position.y = deepest_point.y;
  dpst_pt.pose.position.z = deepest_point.z;

  // publish the point as a marker in RViz
  mkr_pub.publish(dpst_pt);
}

geometry_msgs::Point points;

/*
 * Find the deepest point of the input scan line (a cross section of the groove).
 */
void deepest_pt(pcl::PointCloud<pcl::PointXYZ> pointcloud)
{
  int cloudSize = pointcloud.size();

  double minZ = 100.0;
  double zz = 0.0;

  int dpst = 0;

  for (int i = 0; i < (cloudSize); ++i) // Neglect the first 50 points
  {
    zz = pointcloud[i].z;
    // Check to make sure the point is a valid point, some points may be invalid
    if ((zz != KEYENCE_INFINITE_DISTANCE_VALUE_SI) && (zz != KEYENCE_INFINITE_DISTANCE_VALUE_SI2)
          && (zz != std::numeric_limits<double>::infinity()))
    {   // then this is a piece of normal point
      if (zz < minZ)
      {
        minZ = zz;
        dpst = i;
      }
    }
  }

  ROS_INFO("Minimum depth: %.2f mm; cloud size: %d ", minZ*1000, cloudSize);

  publish_deepest_pt(pointcloud[dpst]);

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
  world_frame = "world";
  // world_frame = "base_link";
  scanner_frame = "lj_v7200_optical_frame";

  // set up profile cloud publisher for PCL point clouds
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("Y_profiles", 1);

  // set up deepest point marker publisher
  mkr_pub = nh.advertise<visualization_msgs::Marker>("deepest_point", 0);

  /*
   * Listen for Point Cloud - profile from Laser Scanner
   */
  std::string topic = nh.resolveName(cloud_topic);
  ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic " << topic);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topic, 1, callback);

  ros::spin();
  return 0;
}
