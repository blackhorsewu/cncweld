/**********************************************************************************
 *                The Chinese National Engineering Research Centre                *
 *                                Steel Construction                              *
 *                                (Hong Kong Branch)                              *
 *                                                                                *
 *                Department of Civil and Environmental Engineering               *
 *                        Hong Kong Polytechnic University                        *
 *                                                                                *
 *                                  Robotic Welding                               *
 *                                                                                *
 * Victor W H Wu                                                                  *
 * 11 August 2021.                                                                *
 *                                                                                *
 * RS Percept node for the Realsense camera to locate and identify welding        *
 * groove in ROS.                                                                 *
 *                                                                                *
 **********************************************************************************/

/**********************************************************************************
 *                                                                                *
 * This node assumes that the Realsense2 camera package has already been          *
 * running so that it publishes the images and point clouds for use.              *
 * It is imperative that the Point Clouds published are ORGANIZED. This can be    *
 * achieved by the command:                                                       *
 * $ roslaunch realsense2_camera rs_rgbd.launch camera:=d435i                     *
 * where the rs_rgbd.launch is a launch file for the rs_rgbd nodelet and needs to *
 * be installed separately.                                                       *
 * It also assumes the robot, in this case is the CNC mechanism, and the RViz are *
 * running with the tf for the camera frames are readily available.               *
 *                                                                                *
 * It subscribes to point clouds, topic /d435i/depth_registered/points,from the   *
 * Realsense D435i camera. This is the ORGANIZED point cloud.                     *
 *                                                                                *
 **********************************************************************************/



#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

// PCL (Point Cloud Library) specific includes
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"

#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/bilateral.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "geometry_msgs/Point.h"
#include <vector>

// These include files are for Markers to be displayed in rviz
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>

// Visual Tools
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <string>

#include <time.h>

using namespace std;

std::string cloud_topic, world_frame, edges_indices;
ros::Publisher organised_pub;

float x_filter_min, x_filter_max,
      y_filter_min, y_filter_max, 
      z_filter_min, z_filter_max;

/*
 * The Callback function to process the edges indices
*/
void indices_callback(const std::vector<pcl::PointIndices>& straight_edge_indices)
{   // straight_edge_indices is a vector of indices of a point cloud.

  // pc2_edges is a vector of edges in ROS format
  std::vector<sensor_msgs::PointCloud2::Ptr> pc2_edges;
  // clusters is a vector of edges in PCL format
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > edges;

  // straight edge indices is a vector of indices of a point cloud.
  // this point cloud in this case is the /d435i/depth_registered/points.
}

/*
 * The Callback function to process the listened point cloud
 */
void callback(const sensor_msgs::PointCloud2ConstPtr& recent_cloud)
{  //recent_cloud is the raw ROS point cloud received from the camera

  /*
   * Convert Point Cloud from ROS format to PCL format
   */
  pcl::PointCloud<pcl::PointXYZRGB> cloud; // Be careful this is SMALL cloud

  // local types
  typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud; // Be careful this is CAPITAL Cloud

  // Now, cloud is the original cloud in pcl format.

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
                       cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB> (cloud));

  pcl::fromROSMsg(*recent_cloud, *cloud_ptr);

  Cloud::Ptr transformed_pcl_cloud (new Cloud);
  transformed_pcl_cloud->header.frame_id = world_frame;
  pcl_conversions::toPCL(ros::Time::now(), transformed_pcl_cloud->header.stamp);
  sensor_msgs::PointCloud2::Ptr transformed_ros_cloud (new sensor_msgs::PointCloud2);
  transformed_ros_cloud->header.frame_id =  world_frame;

  // from here on we are dealing with PCL point cloud

  /*********************************************************************************
   *                                                                               *
   *  PASSTHROUGH FILTER(S)                                                        *
   *                                                                               *
   *  Concentrate on the working area. Crop the image to focus on our intereset,   *
   *  that is the welding groove.                                                  *
   *  We are trying to crop the image from the perspective of the camera.          *
   *  The x direction is pointing to the right of the camera                       *
   *  The y direction is pointing downward of the camera                           *
   *  The z direction is pointing forward of the camera                            *
   *                                                                               *
   *********************************************************************************/

  //filter in x
  pcl::PointCloud<pcl::PointXYZRGB> xf_cloud, yf_cloud, zf_cloud;
  pcl::PassThrough<pcl::PointXYZRGB> pass_x;

  pass_x.setInputCloud(cloud_ptr);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(x_filter_min, x_filter_max);
  pass_x.filter(xf_cloud);

  //filter in y
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xf_cloud_ptr
                              (new pcl::PointCloud<pcl::PointXYZRGB>(xf_cloud));
  pcl::PassThrough<pcl::PointXYZRGB> pass_y;

  pass_y.setInputCloud(xf_cloud_ptr);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(y_filter_min, y_filter_max);
  pass_y.filter(yf_cloud);

  //filter in z 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr yf_cloud_ptr
                              (new pcl::PointCloud<pcl::PointXYZRGB>(yf_cloud));
  pcl::PassThrough<pcl::PointXYZRGB> pass_z;

  pass_z.setInputCloud(yf_cloud_ptr);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(z_filter_min, z_filter_max);
  pass_z.filter(zf_cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr croped_cloud_ptr 
                            (new pcl::PointCloud<pcl::PointXYZRGB> (zf_cloud));


  // croped_cloud_ptr points to a pcl cloud
  // Needs to convert to ROS format before publishing

  /*
   * croped_cloud_ptr is now pointing to the cropped cloud.
   */

  sensor_msgs::PointCloud2::Ptr transformed_ros_cloud_local (new sensor_msgs::PointCloud2);
  Cloud::Ptr transformed_pcl_cloud_local (new Cloud);

  *transformed_pcl_cloud = *croped_cloud_ptr;

  pcl::toROSMsg(*croped_cloud_ptr, *transformed_ros_cloud);

  organised_pub.publish(*transformed_ros_cloud);
}

/**********************************************************************************
 *                                                                                *
 * The Main of this package starts here.                                          *
 *                                                                                *
 **********************************************************************************/
int main(int argc, char *argv[])
{ // Begin of Main
  /*
   * INITIALIZE ROS NODE
   */
  ros::init(argc, argv, "rs_percept_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");

  /*
   * Set up parameters (Could be input from launch file/terminal)
   */

  /*
   * Parameters for the cloud topic and the reference frames
   */
  cloud_topic = priv_nh_.param<std::string>("cloud_topic", "organized_edge_detector/output_rgb_edge");
  edges_indices = priv_nh_.param<std::string>("edges_topic", "organized_edge_detector/output_straight_edges_indices");
  world_frame = priv_nh_.param<std::string>("world_frame", "world");

  // We should not use the camera frame but what is reported by the point cloud in the
  // header.frame_id instead. camera_frame;
  // camera_frame = priv_nh_.param<std::string>("camera_frame", "d435i_link");

  /*
   * we need to specify how much we want to see, ie how to crop the image in the camera 
   * frame. That is before transforming it to the World frame.
   */
  x_filter_min = priv_nh_.param<float>("x_filter_min", -1.000);
  x_filter_max = priv_nh_.param<float>("x_filter_max",  1.000);

  y_filter_min = priv_nh_.param<float>("y_filter_min", -1.000);
  y_filter_max = priv_nh_.param<float>("y_filter_max",  1.000);

  z_filter_min = priv_nh_.param<float>("z_filter_min", -1.000);
  z_filter_max = priv_nh_.param<float>("z_filter_max",  1.000);

  /*
   * Setup publisher to publish ROS point clouds to RViz
   */
  organised_pub = nh.advertise<sensor_msgs::PointCloud2>("organised", 1);


  /*
   * Listen for Point Cloud - Detected Edges point cloud
   */

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic, 1, callback);

  // Listen for Edges Indices - Edges indices
  ros::Subscriber indices_sub = nh.subscribe<std::vector<pcl::PointIndices>>(edges_indices, 1, indices_callback);

  ros::spin();

  return 0;
} // End of Main
