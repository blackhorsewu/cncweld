/* 
 * File:   SimpleSerial.cpp
 * Author: Terraneo Federico wrote SimpleSerial.h, but ericfont copied to this .cpp file
 * Distributed under the Boost Software License, Version 1.0.
 *
 * Created on September 10, 2009, 12:12 PM
 */

#include <boost/asio.hpp>
#include "SimpleSerial.h"

SimpleSerial::SimpleSerial(std::string port, unsigned int baud_rate)
    : io(), serial(io,port)
{
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
}

void SimpleSerial::writeString(std::string s)
{
    boost::asio::write(serial,boost::asio::buffer(s.c_str(),s.size()));
}

std::string SimpleSerial::readLine()
{
    //Reading data char by char, code is optimized for simplicity, not speed
    using namespace boost;
    char c;
    std::string result;
    for(;;)
    {
        asio::read(serial,asio::buffer(&c,1));
        switch(c)
        {
        case '\r':
            break;
        case '\n':
            return result;
        default:
            result+=c;
        }
    }
}
