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
#include <pcl/filters/crop_box.h>
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

#include "jsk_pcl_ros/organized_edge_detector.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"

#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>
#include "jsk_recognition_utils/pcl_util.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/ColorRGBA.h>

#include <jsk_topic_tools/color_utils.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>

using namespace std;

std::string raw_organized_cloud, world_frame, edges_normal;

float x_filter_min, x_filter_max,
      y_filter_min, y_filter_max, 
      z_filter_min, z_filter_max;

//
// My convention here:
// variables ending pc2 are ROS PointCloud2
// variables ending cloud are PCL PointCloud
//


void my_crop(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in_cloud, pcl::PointCloud<pcl::PointXYZRGB> out_cloud)
{

  /*********************************************************************************
   *                                                                               *
   *  Crop the Point Cloud to pre-specified X, Y, Z dimensions using the           *
   *                                 CROPBOX filter                                *
   *  Concentrate on the working area. Crop the image to focus on our interest,    *
   *  that is the welding groove.                                                  *
   *  We are trying to crop the image from the perspective of WORLD.               *
   *                                                                               *
   *********************************************************************************/

  // pcl::PointCloud<pcl::PointXYZRGB> cropped_cloud;
  pcl::CropBox<pcl::PointXYZRGB> crop;

  Eigen::Vector4f min_pt = Eigen::Vector4f(x_filter_min, y_filter_min, z_filter_min, 0.0);
  Eigen::Vector4f max_pt = Eigen::Vector4f(x_filter_max, y_filter_max, z_filter_max, 0.0);

  crop.setInputCloud(in_cloud);
  crop.setMin(min_pt);
  crop.setMax(max_pt);
  crop.filter(out_cloud);
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
  ros::Publisher resultant_pub;

  /*
   * Parameters for the cloud topic and the reference frames
   */

  /* 
   * According to Jsk PCL ROS document, the topic organized_edge_detector/output 
   * contains all the edge point clouds.
   */
  raw_organized_cloud = priv_nh_.param<std::string>("cloud_topic", "/d435i/depth_registered/points");
  // organized_edge_detector/output_straight_edges_indices are clusters of straight edges indices
  edges_normal = priv_nh_.param<std::string>("edges_topic", "/normal_estimation/output_with_xyz");

  world_frame = priv_nh_.param<std::string>("world_frame", "world");

  // We should NOT use the camera frame but what is reported by the point cloud in the
  // header.frame_id instead. camera_frame;
  // camera_frame = priv_nh_.param<std::string>("camera_frame", "d435i_link");

  /*
   * Because it is necessary to stitch clouds together, each individual cloud needs to
   * be transformed before they can be stitched. So, after they have been transformed
   * and stitched, it can only be cropped in the WORLD coordinate.
   * Therefore, the X, Y, and Z min and max must be specified in the WORLD coordinate.
   */
  x_filter_min = priv_nh_.param<float>("x_filter_min", 0.340);
  x_filter_max = priv_nh_.param<float>("x_filter_max", 0.940);

  y_filter_min = priv_nh_.param<float>("y_filter_min", -0.1375);
  y_filter_max = priv_nh_.param<float>("y_filter_max", 0.1375);

  z_filter_min = priv_nh_.param<float>("z_filter_min", -0.285);
  z_filter_max = priv_nh_.param<float>("z_filter_max",  0.000);

  /*
   * Setup publisher to publish ROS point clouds to RViz
   */
  resultant_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("resultant", 1);

  sensor_msgs::PointCloud2 resultant_pc2;
  pcl::PointCloud<pcl::PointXYZRGB> resultant_cloud;
  
  // Transforms to transform the point cloud from camera frame to world frame
  tf::TransformListener listener;
  tf::StampedTransform stransform;

  sensor_msgs::PointCloud2 transformed_pc2;


//  while (ros::ok())
//  {
    pcl::PointCloud<pcl::PointXYZRGB> local_cloud, cropped_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(local_cloud));
    string reply1;

   /*
    * Listen for Raw Organized Point Cloud 
    *
    * Instead of Listening to the topic, WAIT for the Raw here.
    * 
    * The working area of the CNC is about 600mm in the X direction.
    * The D435i camera can cover about 200mm (slightly more) in the X direction.
    * 
    * It is going to read the FIRST 200 mm.
    */

    sensor_msgs::PointCloud2::ConstPtr organized_pc2 =     // organized_cloud is a ROS format cloud
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(raw_organized_cloud, nh);

    ROS_INFO_STREAM("Organized cloud topic: " << raw_organized_cloud << " received.");

    // must transform it to WORLD coordinate first before anything can be done to it.
    try
    {
      listener.waitForTransform(world_frame, organized_pc2->header.frame_id, ros::Time::now(), ros::Duration(0.5));
      listener.lookupTransform (world_frame, organized_pc2->header.frame_id, ros::Time(0),     stransform);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    
    pcl_ros::transformPointCloud(world_frame, stransform, *organized_pc2, transformed_pc2);

    pcl::fromROSMsg(transformed_pc2, local_cloud);

    // my_crop(local_cloud, cropped_cloud);
    resultant_cloud.header.frame_id = local_cloud.header.frame_id;
    resultant_cloud += local_cloud; // concatenate it to form a complete cloud

    pcl::toROSMsg(resultant_cloud, resultant_pc2);
    resultant_pc2.header.frame_id = world_frame;
    // publish the resultant cloud
    resultant_pub.publish(resultant_cloud);

    cout << "Hit enter after the camera has moved 200mm for the 2nd scan";
    getline(cin, reply1); // Hit return after the camera has moved 200mm for the 2nd scan

   /* 
    * It is going to read the SECOND 200 mm.
    */

    organized_pc2 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(raw_organized_cloud, nh);

    ROS_INFO_STREAM("Organized cloud topic: " << raw_organized_cloud << " received.");

    try
    {
      listener.waitForTransform(world_frame, organized_pc2->header.frame_id, ros::Time::now(), ros::Duration(0.5));
      listener.lookupTransform (world_frame, organized_pc2->header.frame_id, ros::Time(0),     stransform);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    
    pcl_ros::transformPointCloud(world_frame, stransform, *organized_pc2, transformed_pc2);

    pcl::fromROSMsg(transformed_pc2, local_cloud);

    local_cloud.header.frame_id = world_frame;

    resultant_cloud.header.frame_id = local_cloud.header.frame_id;
    resultant_cloud += local_cloud; // concatenate it to form a complete cloud

    pcl::toROSMsg(resultant_cloud, resultant_pc2);
    resultant_pc2.header.frame_id = world_frame;
     // publish the resultant cloud
    resultant_pub.publish(resultant_cloud);

    cout << "Hit enter after the camera has moved 200mm for the 3rd scan";
    getline(cin, reply1); // Hit return after the camera has moved 200mm for the 3rd scan

   /* 
    * It is going to read the THIRD 200 mm.
    */

    organized_pc2 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(raw_organized_cloud, nh);

    ROS_INFO_STREAM("Organized cloud topic: " << raw_organized_cloud << " received.");

    try
    {
      listener.waitForTransform(world_frame, organized_pc2->header.frame_id, ros::Time::now(), ros::Duration(0.5));
      listener.lookupTransform (world_frame, organized_pc2->header.frame_id, ros::Time(0),     stransform);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    
    pcl_ros::transformPointCloud(world_frame, stransform, *organized_pc2, transformed_pc2);

    pcl::fromROSMsg(transformed_pc2, local_cloud);

    local_cloud.header.frame_id = world_frame;

    resultant_cloud.header.frame_id = local_cloud.header.frame_id;
    resultant_cloud += local_cloud; // concatenate it to form a complete cloud

    pcl::toROSMsg(resultant_cloud, resultant_pc2);
    resultant_pc2.header.frame_id = world_frame;
    // publish the resultant cloud
    resultant_pub.publish(resultant_cloud);

    cout << "Hit enter when ready to proceed.";
    getline(cin, reply1);

    // 
    // Do not read the Edges Normals yet but read the 2nd organized cloud.
    //


     /*
      * Wait for Edges Normals - edge normals
      *

      pcl::PointCloud<pcl::PointXYZRGBNormal> normals_cloud;

      sensor_msgs::PointCloud2::ConstPtr edge_normals =
          ros::topic::waitForMessage<sensor_msgs::PointCloud2>(edges_normal, nh);

      ROS_INFO_STREAM("Edges Normal topic: " << edges_normal << " received.");

      pcl::fromROSMsg(*edge_normals, normals_cloud);

      int n = normals_cloud.points.size();

      ROS_INFO_STREAM("There are " << n << "points in the edges.");

      for (int i = 0; i < n; i++)
      {
        if (normals_cloud.points.at(i).curvature != 0.0) {
        ROS_INFO_STREAM("Point " << i+1 << ":\n X: " << normals_cloud.points.at(i).x <<
                                            " Y: " << normals_cloud.points.at(i).y <<
                                            " Z: " << normals_cloud.points.at(i).z <<
//                                            " R: " << normals_cloud.points.at(i).r <<
//                                            " G: " << normals_cloud.points.at(i).g <<
//                                            " B: " << normals_cloud.points.at(i).b <<
                                            " nX: " << normals_cloud.points.at(i).normal_x <<
                                            " nY: " << normals_cloud.points.at(i).normal_y <<
                                            " nZ: " << normals_cloud.points.at(i).normal_z <<
                                            " Curvature: " << normals_cloud.points.at(i).curvature <<
        ".");}
      }
    */
//  }
  return 0;
} // End of Main
// ROS_INFO("Here 1.");
