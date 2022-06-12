/*
 *
 * Interface with GRBL
 * A class grbl is defined, and a weld_grbl object will be created
 * 
 * Victor W H Wu
 * 5 June, 2022. (Sunday)
 * 
 * Working
 * 12 June, 2022. (Sunday)
 * 
 */

#include "AsyncSerial.h"

#include <iostream>
#include <termios.h>
#include <regex>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

using namespace std;

ros::Publisher jsp_pub; // Joint State Publisher

ros::Subscriber cmd_sub; // Grbl command Subscriber

// Create an object of type "jointState"
sensor_msgs::JointState jointState;

/*
 * Global Variables for Status and Position
 */
enum status { Startup, Alarm, Running, Idle,  OK, Error };
enum grblCmd { Homing, Wakeup, ViewSettings };

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

struct Position position;

bool responded = false;
bool startJspPub = false;

regex str_expr("ok|<([A-Z][a-z]+)\\|WPos:(-?[0-9]+\\.[0-9]+),(-?[0-9]+\\.[0-9]+),(-?[0-9]+\\.[0-9]+)");

void initJointStates()
{
  jointState.name.push_back("base_link_X_link_joint");
  jointState.name.push_back("X_link_Y_link_joint");
  jointState.name.push_back("Y_link_Z_link_joint");
  jointState.name.push_back("Z_link_tool0");

  for (int i=0; i < 4; i++) // allocate memory and initialize joint values
    jointState.position.push_back(0.0);
}

void received(const char *data, unsigned int len)
{
  responded = true;
  vector<char> v(data,data+len);
  string in_line(v.begin(), v.end());
  // cout << in_line;

  smatch sm;
  if (regex_search(in_line, sm, str_expr ))
  {
    //cout << sm[0] << endl;
    if (sm[0] != "ok"){ // it should then be either Idle or Run
      if (sm[1] == "Idle") Status = Idle;
      if (sm[1] == "Run") Status = Running;

      // if it has no position data do not publish it
      if ((startJspPub) && (sizeof(sm)==5))
      {
        jointState.header.stamp = ros::Time::now();
        jointState.position[0] = stod(sm[2]) / 1e3;
        jointState.position[1] = stod(sm[3]) / 1e3;
        jointState.position[2] = stod(sm[4]) / 1e3;
        jointState.position[3] = 0.0;

        jsp_pub.publish(jointState);
      }
    }
  }
}

CallbackAsyncSerial serial("/dev/ttyACM0", 115200);

void cmdGrbl(grblCmd cmd)
{
  switch (cmd) {
    case Wakeup:
      cout << "Waking GRBL up ..." << endl;
      serial.writeString("\n");
      break;
    case Homing:
      cout << "GRBL Homing ..." << endl;
      serial.writeString("$H\n");
      break;
    case ViewSettings:
      cout << "GRBL Settings ..." << endl;
      serial.writeString("$$\n");
      break;
    default:
      break;
  }
  // Wait for Grbl Response to the command just sent
  while (responded == false) usleep(10000); // wait for 0.001 second

}

void cmdCb(const std_msgs::String::ConstPtr& msg)
{
  // The msg is a string of G-Code and can be sent to Grbl directly
  serial.writeString(msg->data);
  // Wait for Grbl Response
  while (responded == false) usleep(10000); // wait for steps of 0.001 second
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "grbl_driver_node");
  ros::NodeHandle nh, pnh("~");

  string in_line="";
  string cmd_topic = "grbl_cmd";

  string cmd = nh.resolveName(cmd_topic);

  // joint state publisher
  jsp_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  serial.setCallback(received);

  cmdGrbl(Wakeup);
  cmdGrbl(ViewSettings);
  cmdGrbl(Homing);

  initJointStates(); // Initialise the joint States before publishing
  startJspPub = true;

  cmd_sub = nh.subscribe(cmd_topic, 1, cmdCb);
  // inqGrbl();
  ros::Rate loop_rate(20); // get GRBL status 10 times a second
  while (ros::ok)
  {
    responded = false;
    serial.writeString("?\n"); // send an inquiry request to GRBL
    while (responded == false) usleep(10000); // wait for 0.001 second
    ros::spinOnce();
    loop_rate.sleep();
  }
}
