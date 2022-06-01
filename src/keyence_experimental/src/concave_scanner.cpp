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
#include <visualization_msgs/MarkerArray.h>
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

// the publishers and subscriber declared here to be global and used in functions,
// will be initialised in main
ros::Publisher pub;     // publisher for the cumulated cloud
ros::Publisher mkr_pub; // publisher for the deepest points
ros::Publisher pos_pub; // publisher for the position to move to

ros::Subscriber sub; // subscriber for point cloud

int marker_id = 0;

pcl::PointCloud<pcl::PointXYZ> pcl_Y_cloud; // cumulated cloud

// needed for transformation from scanner frame to world frame
std::string sensor_host;
std::string scanner_frame;
std::string world_frame;

// CNC Machine home offsets in mm
// Meaning, when the CNC Machine is homed, ROS reports this position
double home_off_x = 320.0;
double home_off_y = -100.0;
double home_off_z = 93.0; // for torch; for optical frame 97.73 mm

double maxX = 550.0 + home_off_x;

double target_x = 0.0; // mm

double start_x = -0.85; // mm
double x_step = 25.0; // mm

bool finish_scanning = false;

/* Declarations for writing to files
bool write_Y_file = false; // write the whole scan brief data to a file
bool write_X_file = false; // write data of each scan line to a file

ofstream Yfile;
ofstream Xfile;

int line_no = 0;
int file_no = 0;
*/

// Point index of first and last valid points in a scan line
int valid_begin, valid_end;

visualization_msgs::MarkerArray dpst_pt_array;

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
  dpst_pt.type = visualization_msgs::Marker::SPHERE;
  dpst_pt.scale.x = 0.003; // so the line is shown as of 0.1mm wide
  dpst_pt.scale.y = 0.003;
  dpst_pt.scale.z = 0.003;
  dpst_pt.color.r = 1;   // in red
  dpst_pt.color.a = 1;   //

  // set the location of the deepest point
  dpst_pt.pose.position.x = deepest_point.x;
  dpst_pt.pose.position.y = deepest_point.y;
  dpst_pt.pose.position.z = deepest_point.z;

  // publish the point as a marker in RViz
  mkr_pub.publish(dpst_pt);
  dpst_pt_array.markers.push_back(dpst_pt);
}

geometry_msgs::Point points;

void edit_markers()
{
  /* edit the markers stored in the marker array dpst_pt_array */
  ROS_INFO("Start to change colour of markers");
  
  for (int id = 0; id <= marker_id; id++)
  {
    /* go through each marker and change its colour indicate to user */
    dpst_pt_array.markers[id].color.r = 0;
    dpst_pt_array.markers[id].color.g = 1;
    mkr_pub.publish(dpst_pt_array.markers[id]);
  }
}

/*
 * Publish a Twist topic to cnc_interface to move to the specified position.
 *
 * It should not be done in this way, but just do it for the time being to test
 * out the idea. When it works, redo the cnc_interface and integrate it in here,
 * concave_scanner.
 * 
 * Actually, the whole thing must be carefully re-think and arrange it in a
 * more logical structure.
 * 
 * When moving the CNC Device, there should also be a specified speed.
 */
void move_scanner_to(double x, double y, double z)
{
  geometry_msgs::Twist position;

  position.linear.x = x - home_off_x - start_x; // scanner head offset - start_x 

  if (y <= home_off_y )
  {
    position.linear.y = 0;
  }
  else
  {
    position.linear.y = y - home_off_y;
  }

  position.linear.z = 50 - home_off_z; // because the z moves in reverse direction

  ROS_INFO("Position: x: %.3f", position.linear.x);
  ROS_INFO("Old Target_x: x: %.3f", target_x);

  if ((position.linear.x <= 0.0) || (position.linear.x >= (target_x - 0.25))) // reached target, work for next target
  {
    if (target_x <= 0) target_x = x_step; else target_x = position.linear.x + x_step;
    ROS_INFO("New Target_x: x: %.3f", target_x);
    position.linear.x = target_x;
    pos_pub.publish(position);
    ROS_INFO("Target x: %.3f, y: %.3f, z: %.3f", position.linear.x, position.linear.y, position.linear.z );
    ROS_INFO("Command published. ******************************");
  }
}

void move_torch_to(double x, double y, double z)
{
  geometry_msgs::Twist position;

  position.linear.x = x + home_off_x;
  position.linear.y = y + home_off_y;
  position.linear.z = z - home_off_z;

  ROS_INFO("Position: x: %.2f, y: %.2f, z: %.2f", position.linear.x, position.linear.y, position.linear.z);
  // pos_pub.publish(position);
}

/*
 * Find the deepest point of the input scan line (a cross section of the groove).
 * move to that point.
 */
void deepest_pt(pcl::PointCloud<pcl::PointXYZ> pointcloud)
{
  int cloudSize = pointcloud.size();

  double minZ = 100.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  int dpst = 0;

  for (int i = 0; i < (cloudSize); ++i) // Neglect the first 50 points
  {
    z = pointcloud[i].z;
    // Check to make sure the point is a valid point, some points may be invalid
    if ((z != KEYENCE_INFINITE_DISTANCE_VALUE_SI) && (z != KEYENCE_INFINITE_DISTANCE_VALUE_SI2)
          && (z != std::numeric_limits<double>::infinity()))
    {   // then this is a piece of normal point
      if (z < minZ)
      {
        minZ = z;
        dpst = i;
      }
    }
  }

  x = pointcloud[dpst].x * 1e3;
  y = pointcloud[dpst].y * 1e3;
  z = pointcloud[dpst].z * 1e3;

  ROS_INFO("Deepest Point: x: %.2f y: %.2f z: %.2f ", x, y, z );

  /*
   * Check if x exceeds the Max X, if yes then stop scanning and go into markers editing
   */
  // if (x >= maxX)
  if (x >= 800)
  {
    /* stop scanning and start editing markers */
    ROS_INFO("Finish scanning and going to edit the markers.");
    finish_scanning = true;
    sub.shutdown();
    edit_markers();
  }
  else
  {
    /* carry on as usual */
    move_scanner_to(x, y, z);
    // There should be another one to move the torch to.
    // The difference is the position in the z direction.

    publish_deepest_pt(pointcloud[dpst]);
  }

}

/*
 * This handles one scan line, published by the scanner driver.
 * Transform it to world coordinate, publish the concatenated point cloud so far.
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
  pcl_Y_cloud += pcl_cloud; // concatenate the front line to the cumulated point cloud
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

  // set up position publisher
  pos_pub = nh.advertise<geometry_msgs::Twist>("/cnc_interface/cmd", 1);

  /*
   * Listen for Point Cloud - /profiles from Laser Scanner
   */
  std::string topic = nh.resolveName(cloud_topic);
  ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic " << topic);

  sub = nh.subscribe<sensor_msgs::PointCloud2>(topic, 1, callback);

  ros::spin();
  return 0;
}
