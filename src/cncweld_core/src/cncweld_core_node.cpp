
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>

// Visualisation markers
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>

// Visual Tools
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <termios.h>

#include "SimpleSerial.h"


using namespace std;


// ros parameters for GRBL
string devname;
int baudrate;

int xaccel, yaccel, zaccel;
int xmin, ymin, zmin;
int xmax, ymax, zmax;
int xspeed, yspeed, zspeed;
int xsteps, ysteps, zsteps;

string input_line;

// the publishers declared here to be global and used in functions,
// will be initialised in main

ros::Publisher jsp_pub; // Joint State Publisher

ros::Publisher pub;     // publisher for the cumulated cloud
ros::Publisher mkr_pub; // publisher for the deepest points
ros::Publisher pos_pub; // publisher for the position to move to

int marker_id = 0;

// Create an object of type "jointState"
sensor_msgs::JointState jointState;

pcl::PointCloud<pcl::PointXYZ> pcl_Y_cloud; // cumulated cloud

// needed for transformation from scanner frame to world frame
std::string sensor_host;
std::string scanner_frame;
std::string world_frame;

void initJointStates()
{
  jointState.name.push_back("base_link_X_link_joint");
  jointState.name.push_back("X_link_Y_link_joint");
  jointState.name.push_back("Y_link_Z_link_joint");
  jointState.name.push_back("Z_link_tool0_");

  for (int i=0; i < 3; i++) // allocate memory and initialize joint values
    jointState.position.push_back(0.0);
}

void setupPublishers(ros::NodeHandle nh)
{
  // profile cloud publisher for PCL point clouds
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("Y_profiles", 1);

  // deepest point marker publisher
  mkr_pub = nh.advertise<visualization_msgs::Marker>("deepest_point", 0);

  // position publisher
  pos_pub = nh.advertise<geometry_msgs::Twist>("/cnc_interface/cmd", 1);

  // joint state publisher
  jsp_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);


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

void setupSubscribers(ros::NodeHandle nh)
{
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topic, 1, callback);

}

int getrosparams(ros::NodeHandle pnh)
{
  pnh.getParam("port", devname);
  pnh.getParam("baudrate", baudrate);

  pnh.getParam("x_accel", xaccel);
  pnh.getParam("y_accel", xaccel);
  pnh.getParam("z_accel", xaccel);

  pnh.getParam("x_min", xmin);
  pnh.getParam("y_min", ymin);
  pnh.getParam("z_min", zmin);

  pnh.getParam("x_max", xmax);
  pnh.getParam("y_max", zmax);
  pnh.getParam("z_max", zmax);

  pnh.getParam("x_max_speed", xspeed);
  pnh.getParam("y_max_speed", yspeed);
  pnh.getParam("z_max_speed", zspeed);

  pnh.getParam("x_steps", xsteps);
  pnh.getParam("y_steps", ysteps);
  pnh.getParam("z_steps", zsteps);

  return(1);
}

// the serial port must be setup HERE first
// Open GRBL serial port. This is a GRBL specific program.
SimpleSerial serial("/dev/ttyACM0", 115200);


int homeGrbl()
{
  try
  {
    serial.writeString("$H\n"); // Send the code
     // Wait at least for 10 seconds before expect to receive any response
    sleep(10);
    input_line = serial.readLine(); // wait for
    while (input_line != "ok")
      input_line = serial.readLine();
    cout << "Received (4): " << input_line << " : end" << endl;
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  return(1);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "cncweld_core_node");
  ros::NodeHandle nh, pnh("~");

  // check required parameters
  if (!pnh.hasParam("port"))
  {
    ROS_FATAL("Parameter 'port' missing. Cannot continue.");
    return -1;
  }

  setupPublishers(ros::NodeHandle nh);

  setupSubscribers(ros::NodeHandle nh);

  // Get parameters from launch file
  getrosparams(pnh);

  initJointStates();

  // Before doing anything, wake up Grbl
  serial.writeString("\n\n");
  sleep(2); // Wait for Grbl to initialize
  ROS_INFO("Grbl woke up");
  cout << "Received (1) : " << serial.readLine() << " : end" << endl;
  cout << "Received (2) : " << serial.readLine() << " : end" << endl;
  cout << "Received (3) : " << serial.readLine() << " : end" << endl;

  homeGrbl();

}
