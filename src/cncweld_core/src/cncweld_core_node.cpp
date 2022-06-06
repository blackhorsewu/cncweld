
#include <ros/ros.h>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#include "AsyncSerial.h"
using namespace std;

// ros parameters
string devname;
int baudrate;

int xaccel, yaccel, zaccel;
int xmin, ymin, zmin;
int xmax, ymax, zmax;
int xspeed, yspeed, zspeed;
int xsteps, ysteps, zsteps;

// static int serial;
struct termios SerialPortSettings;

CallbackAsyncSerial serial("/dev/ttyACM0", 115200);

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

int setGrbl()
{

  try {
/*
    // setup the serial port first
    serial = open(devname.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    tcgetattr(serial, &SerialPortSettings);
    cfsetspeed(&SerialPortSettings, B115200); // Set baudrate

    string out_msg;
    out_msg = "$100=" + to_string(xsteps) + "\n";
    // set the steps per mm
    write(serial, out_msg.c_str(), strlen(out_msg.c_str()));
*/    
    serial.writeString("$100=" + to_string(xsteps) + "\n");
    serial.writeString("$101=" + to_string(ysteps) + "\n");
    serial.writeString("$102=" + to_string(zsteps) + "\n");

    // set the maximum speed mm/min
    serial.writeString("$110=" + to_string(xspeed) + "\n");
    serial.writeString("$111=" + to_string(yspeed) + "\n");
    serial.writeString("$112=" + to_string(zspeed) + "\n");

    // set the maximum acceleration mm/sec^2
    serial.writeString("$120=" + to_string(xaccel) + "\n");
    serial.writeString("$121=" + to_string(yaccel) + "\n");
    serial.writeString("$122=" + to_string(zaccel) + "\n");

    // Set the maximum travel distance mm
    serial.writeString("$130=" + to_string(xmax - xmin) + "\n");
    serial.writeString("$131=" + to_string(ymax - ymin) + "\n");
    serial.writeString("$132=" + to_string(zmax - zmin) + "\n");

/*
    for(;;)
    {
      if(serial.errorStatus() || serial.isOpen()==false)
      {
        cerr<<"Error: serial port unexpectedly closed"<<endl;
        break;
      }
      char c;
      cin.get(c); //blocking wait for standard input
      if(c==3) //if Ctrl-C
      {
        cin.get(c);
        switch(c)
        {
          case 3:
            serial.write(&c,1);//Ctrl-C + Ctrl-C, send Ctrl-C
            break;
          case 'x': //fall-through
          case 'X':
            goto quit;//Ctrl-C + x, quit
          default:
            serial.write(&c,1);//Ctrl-C + any other char, ignore
        }
      } else serial.write(&c,1);
    }
    // quit:
    // serial.close();
*/  
  } catch (std::exception& e) {
    cerr<<"Exception: "<<e.what()<<endl;
  }

  // tcsetattr(0, TCSANOW, &stored_settings);

  return(1);
}

int homeGrbl()
{

  serial.writeString("$H\n");

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

  // setup the serial port first 
  // SimpleSerial serial(devname, baudrate);
  // serial.setCallback(received);

  // Setup GRBL with these parameters
  setGrbl();

  homeGrbl();

}
