
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

#include <SerialStream.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <regex>

using namespace std;
using namespace LibSerial;

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

SerialStream serial;

enum grblCmd { Homing, Wakeup, ViewSettings };

void initSerial()
{
  serial.Open( "/dev/ttyACM0" );
  if ( !serial.good() )
  {
    std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
              << "Error: Could not open serial port."
              << std::endl ;
    exit(1) ;
  }

  serial.SetBaudRate(SerialStreamBuf::BAUD_115200) ;
  if ( !serial.good() )
  {
    std::cerr << "Error: Could not set the baud rate." << std::endl ;
    exit(1) ;
  }

  cout << "Serial port to GRBL setup completed." << endl;

}

void waitGrblResponse()
{
  string inString;

  while( serial.rdbuf()->in_avail() == 0 ) // Wait for character
  {
      usleep(100000) ; // 100 milli second or 0.1 second
  }

  inString = "";
  while( serial.rdbuf()->in_avail() > 0  )
  {
    char next_byte;
    serial.get(next_byte);
    inString += next_byte;
    if ( next_byte == '\n' )
      {
        cout << inString;
        inString = "";
      }
  }

}

/*
 * Global Variables for Status and Position
 */
enum status { Startup, Alarm, Running, Idle,  OK, Error };
status Status;
struct Position
{
    double X;
    double Y;
    double Z;
  
    double A;
    double B;
    double C;
};

void cmdGrbl(grblCmd cmd)
{
  switch (cmd) {
    case Wakeup:
      cout << "GRBL waking up ..." << endl;
      serial << endl;
      break;
    case Homing:
      cout << "GRBL Homing ..." << endl;
      serial << "$H" << endl;
      break;
    case ViewSettings:
      cout << "GRBL Settings ..." << endl;
      serial << "$$" << endl;
      break;
    default:
      break;
  }
  waitGrblResponse();
}

struct Position position;

void inqGrbl()
{
  string inString;

  serial << "?" << endl; // send an inquiry request to GRBL

  while( serial.rdbuf()->in_avail() == 0 ) // Wait for response
  {
      usleep(10000) ; // 100 milli second or 0.01 second
  }

  inString = "";
  while( serial.rdbuf()->in_avail() > 0  ) // Read the response
  {
    char next_byte;
    serial.get(next_byte);
    inString += next_byte;
    if ( next_byte == '\n' ) break;
  }

  cout << inString;

  regex str_expr("ok|<([A-Z][a-z]+)\\|WPos:(-?[0-9]+\\.[0-9]+),(-?[0-9]+\\.[0-9]+),(-?[0-9]+\\.[0-9]+)");
  smatch sm;
  if (regex_search(inString, sm, str_expr ))
  {
      //cout << sm[0] << endl;
      if (sm[0] != "ok"){
      if (sm[1] == "Idle") Status = Idle;
      if (sm[1] == "Run") Status = Running;
      
      position.X = stod(sm[2]);
      position.Y = stod(sm[3]);
      position.Z = stod(sm[4]);
      position.A = 0.0;
      }
  }
  else cout << "Sorry, no match found!" << endl;

}

void jumpTo(double x, double y, double z) // Jump to uses G00
{
  while (Status != Idle) inqGrbl();
  cout << "Jumping to: " << x << ", " << y << ", " << z << endl;
  serial <<"G00 X"+to_string(x)+" Y"+to_string(y)+" Z"+to_string(z)<< endl;
  waitGrblResponse();
}

void moveTo() // Move to uses G01
{

}

void initJointStates()
{
  jointState.name.push_back("base_link_X_link_joint");
  jointState.name.push_back("X_link_Y_link_joint");
  jointState.name.push_back("Y_link_Z_link_joint");
  jointState.name.push_back("Z_link_tool0");

  for (int i=0; i < 4; i++) // allocate memory and initialize joint values
    jointState.position.push_back(0.0);
}

void publishJointState()
{
  inqGrbl();
  jointState.header.stamp = ros::Time::now();
  jointState.position[0] = position.X / 1e3;
  jointState.position[1] = position.Y / 1e3;
  jointState.position[2] = position.Z / 1e3;
  jointState.position[3] = position.A / 1e3;
/*
  cout << jointState.name[0] << " X: " << jointState.position[0] << endl;
  cout << jointState.name[1] << " Y: " << jointState.position[1] << endl;
  cout << jointState.name[2] << " Z: " << jointState.position[2] << endl;
  cout << jointState.name[3] << " A: " << jointState.position[3] << endl;
*/
  jsp_pub.publish(jointState);
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

//  serial.writeString(gcode);

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
//  cout << "Received (5): " << serial.readLine() << " : end" << endl;

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

  // publish_deepest_pt(pointcloud[dpst]);

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
                              ros::Duration(0.5));
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
  // deepest_pt(pcl_cloud); // try to find the deepest point of this cross section
  // pub.publish(pcl_Y_cloud);
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

  initSerial( );

  // Point Cloud topic
  std::string cloud_topic;
  
  cloud_topic = "profiles"; // The cloud published by the Keyence Driver
  world_frame = "world";
  scanner_frame = "lj_v7200_optical_frame";

  // profile cloud publisher for PCL point clouds
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("Y_profiles", 1);

  // deepest point marker publisher
  mkr_pub = nh.advertise<visualization_msgs::Marker>("deepest_point", 0);

  // joint state publisher
  jsp_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  std::string topic = nh.resolveName(cloud_topic);

  // Get parameters from launch file
  getrosparams(pnh);

  cmdGrbl(Wakeup);
  cmdGrbl(ViewSettings);
  cmdGrbl(Homing);

  jumpTo(0, 0, -35);
  // Only scribe to the scanner point cloud after homing
  // otherwise, no tf between world and lj_v7200_optical_frame
  while (Status != Idle) inqGrbl();
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topic, 1, callback);

  initJointStates();

  Status = Startup;

  ros::Rate loop_rate(10); // get GRBL status 10 times a second.
  while (ros::ok())
  {
    // inqGrbl();
    // should publish the jointState
    publishJointState();
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  // Try to move to somewhere
//  move_scanner_to(520, -50, -20);
  serial.Close();
}
