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
// #include <SerialStream.h>

#include <iostream>
#include <termios.h>
#include <regex>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <sstream>

using namespace std;
// using namespace LibSerial;

// ROS parameters for GRBL
string devname;
int baudrate;

/* The Joint State Publisher here is not used (at least for the time being)
ros::Publisher jsp_pub; // Joint State Publisher
*/

ros::Publisher pos_pub; // Position Publisher

ros::Publisher status_pub; // Publish the Grbl status

ros::Subscriber cmd_sub; // Grbl command Subscriber

// Create an object of type "jointState"
sensor_msgs::JointState jointState;

geometry_msgs::Twist position;


/*
 * Global Variables for Status and Position
 */
enum status { Startup, Alarm, Running, Idle,  OK, Error, Home };
enum grblCmd { Homing, Wakeup, ViewSettings, Inquire, OffLaser };

status Status = Startup;
/*
struct Position
{
    double X;
    double Y;
    double Z;
  
    double A;
    double B;
    double C;
};
*/
// struct Position position;

bool responded = false;
bool startJspPub = false;

regex str_expr("ok|<([A-Z][a-z]+)\\|WPos:(-?[0-9]+\\.[0-9]+),(-?[0-9]+\\.[0-9]+),(-?[0-9]+\\.[0-9]+)");

/* This work for Async Serial */

string in_line="";
string buffer1="";
bool completed = false;
bool completed1 = false;
bool touched = false;

void process(string inString)
{
  std_msgs::String msg;
  status PrevStatus;

  if (completed)
  {
    responded = true;

    if (inString[0] == '$') cout << inString;

//cout << inString << endl;
    smatch sm;
    if (regex_search(inString, sm, str_expr ))
    {
      // cout << sm[0] << endl;
      PrevStatus = Status;
      if (sm[0] == "ok")
        {
          Status = OK;
          msg.data = "OK";
          if (PrevStatus != Status) status_pub.publish(msg);
        }
      else 
      { // it should then be either Idle or Run
        if (sm[1] == "Idle")
        {
          Status = Idle;
          msg.data = "Idle";
        }
        if (sm[1] == "Run")
        {
          Status = Running;
          msg.data = "Run";
        }
        if (sm[1] == "Home")
        {
          Status = Home;
          msg.data = "Home";
        }
        if (sm[1] == "Alarm")
        {
          Status = Home;
          msg.data = "Alarm";
        }
        if (PrevStatus != Status) status_pub.publish(msg);

        position.linear.x = stod(sm[2]);
        position.linear.y = stod(sm[3]);
        position.linear.z = stod(sm[4]);
        position.angular.x = 0.0;
        pos_pub.publish(position);
      }
    }
  }
}

void received(const char *data, unsigned int len)
{
  vector<char> v(data,data+len);
  if (touched)
  {
    in_line = buffer1;
    buffer1 = "";
    touched = false;
  }

  for(unsigned int i=0;i<v.size();i++)
  {
    if (!completed)
    {
      if(v[i]=='\n')
      {
        in_line += "\n"; // the received line is completed
        completed = true;
      } else 
      {
        if (v[i] >= 32 && v[i] < 0x7f) in_line += v[i];
        // if(v[i]<32 || v[i]>=0x7f) cout.put('X');//Remove non-ascii char
        else in_line += v[i];
      }
    }
    else
    {
      touched = true;
      if(v[i]=='\n')
      {
        buffer1 += "\n"; // the received line is completed
        completed1 = true;
      } else 
      {
        if (v[i] >= 32 && v[i] < 0x7f) buffer1 += v[i];
        // if(v[i]<32 || v[i]>=0x7f) cout.put('Y');//Remove non-ascii char
        else buffer1 += v[i];
      }
    }
  }
  if (completed)
  {
    process(in_line);
    in_line = "";
    completed = false;
  }
}

CallbackAsyncSerial serial("/dev/ttyACM0", 115200);

/*
void waitGrblResponse()
{
  string inString = "";
  std_msgs::String msg;
  status Prev_Status = Status;
  smatch sm;
  bool completed = false;
  
  while( serial.rdbuf()->in_avail() == 0 ) // Wait for response
  {
      usleep(10000) ; // wait for 10 milli second or 0.01 second
  }

  while (!completed)
  {
    while( serial.rdbuf()->in_avail() > 0  ) // Read the response
    {
      char next_byte;
      serial.get(next_byte);
      inString += next_byte;
      if ( next_byte == '\n' ) {completed = true; break;}
    }
    if (completed) break; else usleep(10000); // wait for 10 milli second
  }

  // cout << "I am here (1): " << inString << endl;

  if (inString[0] == 'o') // it is most likely ok
  {
    if (inString == "ok\n")
    {
      Status = OK;
      msg.data = "OK";
      status_pub.publish(msg);
    }
  } else
  if (inString[0] == '$') cout << inString; // do not need an endl here
  else
  if (regex_search(inString, sm, str_expr ))
  {
    { // it can be Home, Idle, or Run
      // cout << "I am here (2) " << endl;
      if (sm[1] == "Idle")
      {
        Status = Idle;
        msg.data = "Idle";
      }
      if (sm[1] == "Run")
      {
        Status = Running;
        msg.data = "Run";
      }
      if (sm[1] == "Home")
      {
        Status = Home;
        msg.data = "Home";
      }
      if (Prev_Status != Status)
      {
        cout << "Status changed: " << inString; // do not need an endl here, inString has it.
        status_pub.publish(msg); // only publish when changed
      }
      //cout << "I am here (3); " << sm[1] << endl;
      position.linear.x = stod(sm[2]);
      position.linear.y = stod(sm[3]);
      position.linear.z = stod(sm[4]);
      position.angular.x = 0.0;
      pos_pub.publish(position);
    }
  }
  else cout << "Sorry, no match found!" << endl << inString << endl;

}
*/

void cmdGrbl(grblCmd cmd)
{
  // responded = false;
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
    case Inquire:
      // do not output to screen messages otherwise it will fill the screen.
      serial.writeString("?\n");
      break;
/*    case OffLaser:
      serial.writeString("M9\n");
      break; */
    default:
      break;
  }
  // Wait for Grbl Response to the command just sent
  while (responded == false) usleep(10000); // wait for 10 milli second

}

void cmdCb(const std_msgs::String::ConstPtr& msg)
{
  cout << "Driver received cmd: " << msg->data << endl;
  // responded = false;
  // The msg is a string of G-Code and can be sent to Grbl directly
  serial.writeString(msg->data);
  // serial<<msg->data<<endl;
  cout << "Just sent the msg to Grbl: " << msg->data << endl;
  // Wait for Grbl Response
  while (responded == false) usleep(10000); // wait for steps of 0.001 second
  // cout << "response received from Grbl." << endl;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "grbl_driver_node");
  ros::NodeHandle nh, pnh("~");

  string in_line="";
  string cmd_topic = "grbl_cmd";

  string cmd = nh.resolveName(cmd_topic);

  // joint state publisher
  // jsp_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  // position publisher
  pos_pub = nh.advertise<geometry_msgs::Twist>("grbl_pos", 1);

  // status publisher
  status_pub = nh.advertise<std_msgs::String>("grbl_status", 1);

  // Clear the screen first
  std::system("clear");

  serial.setCallback(received);
  // initSerial();

  cmdGrbl(Wakeup);
  cmdGrbl(ViewSettings);
  cmdGrbl(Homing);
  cmdGrbl(OffLaser); // Make sure the laser is off.

  ros::Subscriber cmd_sub = nh.subscribe<std_msgs::String>("grbl_cmd", 10, cmdCb);
  // inqGrbl();
  ros::Rate loop_rate(100); // get GRBL status 10 times a second
  while (ros::ok)
  {
    // responded = false;
    // serial << "?\n"; // send an inquiry request to GRBL
    cmdGrbl(Inquire); // No need to wait for response, inqGrbl handles all.
    // waitGrblResponse(); 
    // while (responded == false) usleep(10000); // wait for 0.001 second
    ros::spinOnce(); //this is only valid for subscribed topics
    loop_rate.sleep();
  }

  //cmdGrbl(OffLaser); // Make sure the laser is off.

  serial.close();
}
