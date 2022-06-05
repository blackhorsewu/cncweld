/*
 *
 * Interface with GRBL
 * A class grbl is defined, and a weld_grbl object will be created
 * 
 * Victor W H Wu
 * 5 June, 2022. (Sunday)
 * 
 */

#include <ros/ros.h>
#include "AsyncSerial.h"

// This is very important otherwise the compiler will not have the class
// declaration and will not know this is the definition
#include "grbl_interface.h"

using namespace std;

namespace grbl_interface
{

CallbackAsyncSerial serial_;

Grbl::Grbl(const std::string& devname,
           unsigned int baud_rate
           )
{
    try
    {
        CallbackAsyncSerial serial_(devname, baud_rate);
        // serial_.setCallback(callback);
        ROS_INFO("I am here.");
    }
    catch (std::exception& e) {
            cerr<<"Exception: "<<e.what()<<endl;
    }
}

void Grbl::test(const std::function<void (const char*, unsigned int)>& callback)
{
    termios stored_settings;
    tcgetattr(0, &stored_settings);
    termios new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_lflag &= (~ISIG); // don't automatically handle control-C
    new_settings.c_lflag &= ~(ECHO); // no echo
    tcsetattr(0, TCSANOW, &new_settings);

    cout<<"\e[2J\e[1;1H"; //Clear screen and put cursor to 1;1

    try {
        // CallbackAsyncSerial serial(argv[1],stoi(argv[2]));
        serial_.setCallback(callback);
        for(;;)
        {
            if(serial_.errorStatus() || serial_.isOpen()==false)
            {
                cerr<<"Error: serial port unexpectedly closed"<<endl;
                break;
            }
            char c;
            cout << "Enter character to send" << endl;
            cin.get(c); //blocking wait for standard input
            cout << "Transmitting character " << c << endl;
            if(c==3) //if Ctrl-C
            {
                cin.get(c);
                switch(c)
                {
                    case 3:
                        serial_.write(&c,1);//Ctrl-C + Ctrl-C, send Ctrl-C
                    break;
                    case 'x': //fall-through
                    case 'X':
                        goto quit;//Ctrl-C + x, quit
                    default:
                        serial_.write(&c,1);//Ctrl-C + any other char, ignore
                }
            } else serial_.write(&c,1);
        }
        quit:
        serial_.close();
    } catch (std::exception& e) {
        cerr<<"Exception: "<<e.what()<<endl;
    }

    tcsetattr(0, TCSANOW, &stored_settings);
}

}