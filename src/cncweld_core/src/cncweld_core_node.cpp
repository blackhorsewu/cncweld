
#include <ros/ros.h>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <termios.h>

#include "SimpleSerial.h"
using namespace std;

// ros parameters
string devname;
int baudrate;

int xaccel, yaccel, zaccel;
int xmin, ymin, zmin;
int xmax, ymax, zmax;
int xspeed, yspeed, zspeed;
int xsteps, ysteps, zsteps;

string input_line;

// static int serial;
struct termios SerialPortSettings;

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

  // Get parameters from launch file
  getrosparams(pnh);

  // Before doing anything, wake up Grbl
  serial.writeString("\n\n");
  sleep(2); // Wait for Grbl to initialize
  ROS_INFO("Grbl woke up");
  cout << "Received (1) : " << serial.readLine() << " : end" << endl;
  cout << "Received (2) : " << serial.readLine() << " : end" << endl;
  cout << "Received (3) : " << serial.readLine() << " : end" << endl;

  homeGrbl();

}
