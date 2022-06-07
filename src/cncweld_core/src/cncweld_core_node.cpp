
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

/* Do not need callback for Synchronous serial
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
*/

int setGrbl()
{
  ROS_INFO("In setGrbl");
  try {

    // set the steps per mm
    serial.writeString("$100=" + to_string(xsteps) + "\n");
    cout << "Received (2): " << serial.readLine() << " : end" << endl;
    serial.writeString("$101=" + to_string(ysteps) + "\n");
    cout << "Received (3): " << serial.readLine() << " : end" << endl;
    serial.writeString("$102=" + to_string(zsteps) + "\n");
    cout << "Received (4): " << serial.readLine() << " : end" << endl;

    ROS_INFO("Steps set.");

    // set the maximum speed mm/min
    serial.writeString("$110=" + to_string(xspeed) + "\n");
    cout << "Received (5): " << serial.readLine() << " : end" << endl;
    serial.writeString("$111=" + to_string(yspeed) + "\n");
    cout << "Received (6): " << serial.readLine() << " : end" << endl;
    serial.writeString("$112=" + to_string(zspeed) + "\n");
    cout << "Received (7): " << serial.readLine() << " : end" << endl;

    ROS_INFO("Speeds set.");

    // set the maximum acceleration mm/sec^2
    serial.writeString("$120=" + to_string(xaccel) + "\n");
    cout << "Received (8): " << serial.readLine() << " : end" << endl;
    serial.writeString("$121=" + to_string(yaccel) + "\n");
    cout << "Received (9): " << serial.readLine() << " : end" << endl;
    serial.writeString("$122=" + to_string(zaccel) + "\n");
    cout << "Received (10): " << serial.readLine() << " : end" << endl;

    ROS_INFO("Accelerations set.");

    // Set the maximum travel distance mm
    serial.writeString("$130=" + to_string(xmax - xmin) + "\n");
    cout << "Received (11): " << serial.readLine() << " : end" << endl;
    serial.writeString("$131=" + to_string(ymax - ymin) + "\n");
    cout << "Received (12): " << serial.readLine() << " : end" << endl;
    serial.writeString("$132=" + to_string(zmax - zmin) + "\n");
    cout << "Received (13): " << serial.readLine() << " : end" << endl;

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
    // serial.writeString("$$\n");
    // cout << "Received (4): " << serial.readLine() << " : end" << endl;

    // serial.writeString("?\n");
    // cout << "Received: " << serial.readLine() << " : end" << endl;
    serial.writeString("$H\n");
    sleep(10);
    input_line = serial.readLine();
    while (input_line != "ok")
      input_line = serial.readLine();
    cout << "Received (4): " << input_line << " : end" << endl;
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

  // Before doing anything, wake up Grbl
  ROS_INFO("waking up Grbl");
  serial.writeString("\n");

  sleep(2); // Wait for Grbl to initialize

  ROS_INFO("Grbl woke up");
  cout << "Received (1) : " << serial.readLine() << " : end" << endl;
  cout << "Received (2) : " << serial.readLine() << " : end" << endl;
  cout << "Received (3) : " << serial.readLine() << " : end" << endl;

  // setGrbl(); // May be it should not be setup every time.

  // ROS_INFO("Finish setGrbl, going to home.");
  homeGrbl();

}
