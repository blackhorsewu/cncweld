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

// #include "AsyncSerial.h"
#include <SerialStream.h>

#include <iostream>
#include <termios.h>
#include <regex>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <sstream>

using namespace std;
using namespace LibSerial;

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

SerialStream serial;

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
/*
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
        if (inString != "ok\n")
        cout << inString;
        inString = "";
      }
  }

}
*/
/*
void initJointStates()
{
  jointState.name.push_back("X_link_Y_link_joint");
  jointState.name.push_back("Y_link_Z_link_joint");
  jointState.name.push_back("base_link_X_link_joint");
  jointState.name.push_back("Z_link_tool0");

//  for (int i=0; i < 4; i++) // allocate memory and initialize joint values
//    jointState.position.push_back(0.0);
  jointState.position.push_back(0.0);
  jointState.position.push_back(0.0);
  jointState.position.push_back(0.09);
  jointState.position.push_back(0.0);
}

string in_line="";
*/
/*
void received(const char *data, unsigned int len)
{
  std_msgs::String msg;
  bool completed = false;

  vector<char> v(data,data+len);
  for(unsigned int i=0;i<v.size();i++)
  {
    if(v[i]=='\n')
    {
      in_line += "\n"; // the received line is completed
      completed = true; break;
    } else 
    {
      if(v[i]<32 || v[i]>=0x7f) cout.put(' ');//Remove non-ascii char
      else in_line += v[i];
    }
  }

  if (completed)
  {
    responded = true;

cout << in_line << endl;
    smatch sm;
    if (regex_search(in_line, sm, str_expr ))
    {
      // cout << sm[0] << endl;
      if (sm[0] == "ok")
        {
          Status = OK;
          msg.data = "OK";
          status_pub.publish(msg);
        }
      else 
      { // it should then be either Idle or Run
        if (sm[1] == "Idle")
        {
          Status = Idle;
          msg.data = "Idle";
          status_pub.publish(msg);
        }
        if (sm[1] == "Run")
        {
          Status = Running;
          msg.data = "Run";
          status_pub.publish(msg);
        }
        if (sm[1] == "Home")
        {
          Status = Home;
          msg.data = "Home";
          status_pub.publish(msg);
        }

        // if it has no position data do not publish it
  //      if ((startJspPub) && (sizeof(sm)==5))
  //cout << sizeof(sm)/sizeof(sm[0]) << endl;
        if ((startJspPub))
  -----
        {
          jointState.header.stamp = ros::Time::now();
  //        jointState.header.stamp = ros::Time(0);
          jointState.position[0] = stod(sm[3]) / 1e3;
          jointState.position[1] = stod(sm[4]) / 1e3;
          jointState.position[2] = (stod(sm[2]) + 90) / 1e3;
          jointState.position[3] = 0.0;

          jsp_pub.publish(jointState);
        }
  -----
        {
          position.linear.x = stod(sm[2]);
          position.linear.y = stod(sm[3]);
          position.linear.z = stod(sm[4]);
          position.angular.x = 0.0;
          pos_pub.publish(position);
        }
      }
    }
    in_line = "";
  }
}
*/
// CallbackAsyncSerial serial("/dev/ttyACM0", 115200);

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

void cmdGrbl(grblCmd cmd)
{
  // responded = false;
  switch (cmd) {
    case Wakeup:
      cout << "Waking GRBL up ..." << endl;
      serial<< endl;
      break;
    case Homing:
      cout << "GRBL Homing ..." << endl;
      serial<< "$H" << endl;
      break;
    case ViewSettings:
      cout << "GRBL Settings ..." << endl;
      serial << "$$\n" << endl;
      break;
    case Inquire:
      // do not output to screen messages otherwise it will fill the screen.
      serial << "?" << endl;
      break;
    case OffLaser:
      serial << "M9\n" << endl;
      break;
    default:
      break;
  }
  // Wait for Grbl Response to the command just sent,
  // if it is an inquiry, it will handle it.
  waitGrblResponse();
  //while (responded == false) usleep(10000); // wait for 0.001 second

}

void cmdCb(const std_msgs::String::ConstPtr& msg)
{
  cout << "Driver received command: " << msg->data << endl;
  // responded = false;
  // The msg is a string of G-Code and can be sent to Grbl directly
  serial << msg->data << endl;
  // Wait for Grbl Response
  waitGrblResponse();
  // while (responded == false) usleep(10000); // wait for steps of 0.001 second
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

  // serial.setCallback(received);
  initSerial();

  cmdGrbl(Wakeup);
  cmdGrbl(ViewSettings);
  cmdGrbl(Homing);
  cmdGrbl(OffLaser); // Make sure the laser is off.

  // Do not publish joint state here now!
  // initJointStates(); // Initialise the joint States before publishing
  startJspPub = true;

  Status = Startup;

  cmd_sub = nh.subscribe(cmd_topic, 1, cmdCb);
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

  cmdGrbl(OffLaser); // Make sure the laser is off.

  serial.Close();
}
