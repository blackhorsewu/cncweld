
#include <ros/ros.h>

#include <iostream>
#include <stdio.h>
#include <string.h>
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
CallbackAsyncSerial serial("/dev/ttyACM0", 115200);

void received(const char *data, unsigned int len)
{
    // cout << "Received " << len << " characters: ";
    vector<char> v(data,data+len);
    for(unsigned int i=0;i<v.size();i++)
    {
        if(v[i]=='\n')
        {
            cout<<endl;
        } else {
            if(v[i]<32 || v[i]>=0x7f) cout.put(' ');//Remove non-ascii char
            else cout.put(v[i]);
        }
    }
    cout.flush();//Flush screen buffer
}

int setGrbl()
{
  ROS_INFO("In setGrbl");
  try {

    // set the steps per mm
    serial.writeString("$100=" + to_string(xsteps) + "\n");
    serial.writeString("$101=" + to_string(ysteps) + "\n");
    serial.writeString("$102=" + to_string(zsteps) + "\n");

    ROS_INFO("Steps set.");

    // set the maximum speed mm/min
    serial.writeString("$110=" + to_string(xspeed) + "\n");
    serial.writeString("$111=" + to_string(yspeed) + "\n");
    serial.writeString("$112=" + to_string(zspeed) + "\n");

    ROS_INFO("Speeds set.");

    // set the maximum acceleration mm/sec^2
    serial.writeString("$120=" + to_string(xaccel) + "\n");
    serial.writeString("$121=" + to_string(yaccel) + "\n");
    serial.writeString("$122=" + to_string(zaccel) + "\n");

    ROS_INFO("Accelerations set.");

    // Set the maximum travel distance mm
    serial.writeString("$130=" + to_string(xmax - xmin) + "\n");
    serial.writeString("$131=" + to_string(ymax - ymin) + "\n");
    serial.writeString("$132=" + to_string(zmax - zmin) + "\n");

    ROS_INFO("Travels set.");

  } catch (std::exception& e) {
    cerr<<"Exception: "<<e.what()<<endl;
  }

  return(1);
}

int homeGrbl()
{

  ROS_INFO("In home.");

  try
  {
    serial.writeString("$H\n");
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
  ROS_INFO("Sent home.");

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
  ROS_INFO("Going to set callback.");
  serial.setCallback(received);
  ROS_INFO("Finish set callback.");

  termios stored_settings;
  tcgetattr(0, &stored_settings);
  termios new_settings = stored_settings;
  new_settings.c_lflag &= (~ICANON);
  new_settings.c_lflag &= (~ISIG); // don't automatically handle control-C
  new_settings.c_lflag &= ~(ECHO); // no echo
  tcsetattr(0, TCSANOW, &new_settings);

  cout<<"\e[2J\e[1;1H"; //Clear screen and put cursor to 1;1

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
  quit:
//    serial.close();
  // Setup GRBL with these parameters
  setGrbl();

  ROS_INFO("Finish setGrbl, going to home.");
  homeGrbl();

  try
  {
    tcsetattr(0, TCSANOW, &stored_settings);
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
}
