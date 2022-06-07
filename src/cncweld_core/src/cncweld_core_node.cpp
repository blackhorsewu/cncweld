
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

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

// Joint State Publisher
ros::Publisher jsp_pub;

// Create an object of type "jointState"
sensor_msgs::JointState jointState;

void initJointStates()
{
  jointState.name.push_back("base_link_X_link_joint");
  jointState.name.push_back("X_link_Y_link_joint");
  jointState.name.push_back("Y_link_Z_link_joint");
  jointState.name.push_back("Z_link_tool0_");

  for (int i=0; i < 3; i++) // allocate memory and initialize joint values
    jointState.position.push_back(0.0);
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

  jsp_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  // check required parameters
  if (!pnh.hasParam("port"))
  {
    ROS_FATAL("Parameter 'port' missing. Cannot continue.");
    return -1;
  }

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

  // After 'home', do something useful
  // The laser scanner driver should have been started by the launch file
  // Move to the starting point for scanning, and save its position
  // Subscribe the laser scanner point cloud
  // Switch on the laser of the scanner
  // An infinite loop to 
  // 1. Query Grbl for CNC positions and publish them as jointStates
  // 2. Exit the loop when destination reached and switch off the scanner
  // 
  // Instruct Grbl to go back to the starting point
  // Construct a G-Code program for the welding track
  // Send the G-Code program to Grbl and switch on the welding torch
  // Go into another loop to query Grbl for positions and publish them as
  // jointStates.
  // Exit the loop when destination reached

  // Most of the useful works should also be done in callbacks
  // that is to handle events. One of the major events is the receipt of a
  // point cloud from the laser scanner.
  // 1. When /profiles point clouds callback, transform it and publish it.
  //    At the same time, work out the Grbl state and decide if send new target
  //    of movement.
  // 2. Save the deepest point of every received point cloud.
  // 3. Finish the loop when the point cloud reached the destination (end point).
}
