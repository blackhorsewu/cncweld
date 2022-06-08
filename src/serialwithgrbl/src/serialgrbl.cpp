#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>

using namespace std;
using namespace LibSerial ;

int main(int argc, char** argv)
{
  SerialStream serial;
  char c;

  serial.Open( "/dev/ttyACM0" ) ;
  if ( !serial.good() )
  {
    std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
              << "Error: Could not open serial port."
              << std::endl ;
    exit(1) ;
  }

  serial.SetBaudRate( SerialStreamBuf::BAUD_115200) ;
  if ( !serial.good() )
  {
    std::cerr << "Error: Could not set the baud rate." << std::endl ;
    exit(1) ;
  }

  cout << "Everything setup." << endl;

  string inString;

  serial << "\n\n" << endl;

  while( serial.rdbuf()->in_avail() == 0 )
  {
      usleep(100000) ; // 100 milli second or 0.1 second
  }

  while (serial.rdbuf()->in_avail() > 0)
  {
    serial >> inString;
    cout << inString << endl;
  }
  

  string outString = "$$\n";
  //char out_buf[] = "$$\n";
  //string str(out_buf);
  cout << "Going to write: " << outString;
  //serial.write(out_buf, 3);
  serial << "$$" << endl;

  while( serial.rdbuf()->in_avail() == 0 )
  {
      usleep(50000) ; // 100 micro second.
  }
/*
  while( serial.rdbuf()->in_avail() > 0  )
  {
    char next_byte;
    serial.get(next_byte);
    cout << next_byte;
  }
*/

  serial >> inString;
  cout << inString << endl;
/*
  char out_buf1[] = "?\n";
  string str1(out_buf1);
  cout << "Going to write: " << str1;
  serial.write(out_buf1, 2);
*/

  serial << "?\n";

  while( serial.rdbuf()->in_avail() == 0 )
  {
      usleep(100000) ; // 100 milli second or 0.1 second
  }

  serial >> inString;
  cout << inString << endl;

/*
  while( serial.rdbuf()->in_avail() > 0  )
  {
    char next_byte;
    serial.get(next_byte);
    cout << next_byte;
  }
*/

  return 0 ;   
}