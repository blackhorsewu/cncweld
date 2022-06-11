/*
 *
 * Interface with GRBL
 * A class grbl is defined, and a weld_grbl object will be created
 * 
 * Victor W H Wu
 * 5 June, 2022. (Sunday)
 * 
 */

#include "AsyncSerial.h"

#include <iostream>
#include <termios.h>
#include <regex>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace std;

ros::Publisher jsp_pub; // Joint State Publisher

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

void publishJointState()
{
  // inqGrbl();
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

void received(const char *data, unsigned int len)
{
  responded = true;
  vector<char> v(data,data+len);
  string in_line(v.begin(), v.end());
  cout << in_line;

  regex str_expr("ok|<([A-Z][a-z]+)\\|WPos:(-?[0-9]+\\.[0-9]+),(-?[0-9]+\\.[0-9]+),(-?[0-9]+\\.[0-9]+)");
  smatch sm;
  if (regex_search(in_line, sm, str_expr ))
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
  // else cout << "Sorry, no match found!" << endl;

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
  // serial.writeString("?\n"); // send an inquiry request to GRBL
  // Wait for Grbl Response to the command just sent
  while (responded == false) usleep(100000); // wait for 0.01 second

}

/*
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
*/

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "grbl_driver_node");
  ros::NodeHandle nh, pnh("~");

  termios stored_settings;
  tcgetattr(0, &stored_settings);
  termios new_settings = stored_settings;
  new_settings.c_lflag &= (~ICANON);
  new_settings.c_lflag &= (~ISIG); // don't automatically handle control-C
  new_settings.c_lflag &= ~(ECHO); // no echo
  tcsetattr(0, TCSANOW, &new_settings);

  cout<<"\e[2J\e[1;1H"; //Clear screen and put cursor to 1;1

  // joint state publisher
  jsp_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);


  try {
    // CallbackAsyncSerial serial(argv[1],stoi(argv[2]));
      serial.setCallback(received);
/*
      cout << "Waking GRBL up ..." << endl;
      responded = false;
      serial.writeString("\n");
      cout << "End of Line sent out" << endl;
*/
    cmdGrbl(Wakeup);
    cmdGrbl(ViewSettings);
    cmdGrbl(Homing);
    // inqGrbl();
    ros::Rate loop_rate(10); // get GRBL status 10 times a second
/*    while (ros::ok)
    {*/
      responded = false;
      serial.writeString("?\n"); // send an inquiry request to GRBL
      while (responded == false) usleep(10000); // wait for 0.01 second
      /* ros::spinOnce();
      loop_rate.sleep();
    }*/

    
    for(;;)
    {
      if(serial.errorStatus() || serial.isOpen()==false)
      {
          cerr<<"Error: serial port unexpectedly closed"<<endl;
          break;
      }
      char c;
      // cout << "Enter character to send" << endl;
      cin.get(c); //blocking wait for standard input
      // cout << "Transmitting character " << c << endl;
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
      serial.close();
    
  } catch (std::exception& e) {
    cerr<<"Exception: "<<e.what()<<endl;
  }

  tcsetattr(0, TCSANOW, &stored_settings);
}
