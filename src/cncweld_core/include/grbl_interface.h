/*
 *
 * Header file for GRBL interface
 * 
 * Victor W H Wu
 * 5 June, 2022. (Sunday)
 * 
 */

#ifndef GRBL_INTERFACE_GRBL_H
#define GRBL_INTERFACE_GRBL_H

#include <ros/ros.h>
#include "AsyncSerial.h"

using namespace std;

namespace grbl_interface
{

class Grbl
{
    public:

        Grbl(const std::string& devname,
            unsigned int baud_rate
            );

        ~Grbl(){}

        void test(const std::function<void (const char*, unsigned int)>& callback);

    private:
        /* data */

}; // end of class Grbl

} // End of namespace grbl_interface 

#endif // GRBL_INTERFACE_GRBL_H
