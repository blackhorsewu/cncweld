
#include "AsyncSerial.h"

#include <iostream>
#include <termios.h>

#include <ros/ros.h>

#include "grbl_interface.h"

using namespace std;

using namespace grbl_interface;

void received(const char *data, unsigned int len)
{
    cout << "Received " << len << " characters: ";
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

int main(int argc, char* argv[])
{
    if(argc!=3)
    {
        cerr<<"Usage: serial port baudrate"<<endl<<
                "To quit type Ctrl-C x"<<endl<<
                "To send Ctrl-C type Ctrl-C Ctrl-C"<<endl;
        return 1;
    }

    Grbl grbl(argv[1], atoi(argv[2]), received);

    grbl.test();

    /*
    ROS_INFO("Going to talk with GRBL");
    talk_with_grbl(argv);
    */
}
