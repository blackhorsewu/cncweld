
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

#include "boost/bind.hpp"
#include "boost/ref.hpp"

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

// values LJ Navigator uses for out-of-range points (in meters)
const static double KEYENCE_INFINITE_DISTANCE_VALUE_SI = -999.9990 / 1e3;
const static double KEYENCE_INFINITE_DISTANCE_VALUE_SI2 = -999.9970 / 1e3;

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

// CNC Machine home offsets in mm
// Meaning, when the CNC Machine is homed, ROS reports this position
double home_off_x = 320.0;
double home_off_y = -100.0;
double home_off_z = 93.0;

double target_x = 0.0; // mm

double start_x = -0.85; // mm
double x_step = 30.0; // mm

// the serial port must be setup HERE first
// Open GRBL serial port. This is a GRBL specific program.
SimpleSerial serial("/dev/ttyACM0", 115200);

void initJointStates()
{
  jointState.name.push_back("base_link_X_link_joint");
  jointState.name.push_back("X_link_Y_link_joint");
  jointState.name.push_back("Y_link_Z_link_joint");
  jointState.name.push_back("Z_link_tool0_");

  for (int i=0; i < 3; i++) // allocate memory and initialize joint values
    jointState.position.push_back(0.0);
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
  double linearX, linearY, linearZ;
  string gcode;

  linearX = x - home_off_x - start_x; // scanner head offset - start_x 

  if (y <= home_off_y )
  {
    linearY = 0;
  }
  else
  {
    linearY = y - home_off_y;
  }

  linearZ = 50 - home_off_z; // because the z moves in reverse direction
/*
  ROS_INFO("Position: x: %.3f", linearX);
  ROS_INFO("Old Target_x: x: %.3f", target_x);
*/
/*
  if ((position.linear.x <= 0.0) || (position.linear.x >= (target_x - 0.25))) // reached target, work for next target
  {
    if (target_x <= 0) target_x = x_step; else target_x = position.linear.x + x_step;
    ROS_INFO("New Target_x: x: %.3f", target_x);
    position.linear.x = target_x;
    pos_pub.publish(position);
    ROS_INFO("Target x: %.3f, y: %.3f, z: %.3f", position.linear.x, position.linear.y, position.linear.z );
    ROS_INFO("Command published. ******************************");
  }
*/
  // Replace the above with something else.
  // No need to check for anything yet.
  gcode = "G0 F300 X"+to_string(linearX)+" Y"+to_string(linearY)+" Z"+to_string(linearZ) + "\n";

  cout << gcode;

  serial.writeString(gcode);

  sleep(2);
  // serial.writeString("?");
  sleep(2);
/*
  input_line = serial.readLine(); // block until done
  while (input_line != "ok") {
    // serial.writeString("?");
    sleep(2);
    input_line = serial.readLine();
  }
  */
  cout << "Received (5): " << serial.readLine() << " : end" << endl;

  cout << "G-Code sent." << endl;
}

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

//  move_scanner_to(x, y, z);
  // There should be another one to move the torch to.
  // The difference is the position in the z direction.

  publish_deepest_pt(pointcloud[dpst]);

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


int homeGrbl()
{
  try
  {
    serial.writeString("$H\n"); // Send the code
     // Wait at least for 10 seconds before expect to receive any response
    sleep(10);
    cout << "Received (4): " << serial.readLine() << " : end" << endl;
/*
    input_line = serial.readLine(); // wait for
    while (input_line != "ok")
      input_line = serial.readLine();
    cout << "Received (4): " << input_line << " : end" << endl;
*/
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

  // Point Cloud topic
  std::string cloud_topic;
  
  cloud_topic = "profiles"; // The cloud published by the Keyence Driver
  world_frame = "world";
  scanner_frame = "lj_v7200_optical_frame";

  // profile cloud publisher for PCL point clouds
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("Y_profiles", 1);

  // deepest point marker publisher
  mkr_pub = nh.advertise<visualization_msgs::Marker>("deepest_point", 0);

  // position publisher
  pos_pub = nh.advertise<geometry_msgs::Twist>("/cnc_interface/cmd", 1);

  // joint state publisher
  jsp_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  std::string topic = nh.resolveName(cloud_topic);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topic, 1, callback);

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

  // Try to move to somewhere
  move_scanner_to(520, -50, -20);

}
