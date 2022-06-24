#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <regex>

using namespace std;
using namespace LibSerial ;

  SerialStream serial;
  char c;

int main(int argc, char** argv)
{
//  SerialStream serial;
//  char c;

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

  serial << endl ; // Try to wake up Grbl

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
        cout << inString;
        inString = "";
      }
  }

  string outString = "$$\n";
  //char out_buf[] = "$$\n";
  //string str(out_buf);
  cout << "Going to write: " << outString;
  //serial.write(out_buf, 3);
  serial << "$$" << endl;

  while( serial.rdbuf()->in_avail() == 0 )
  {
      usleep(100000) ; // 100 micro second.
  }

  inString = "";
  while( serial.rdbuf()->in_avail() > 0  )
  {
    char next_byte;
    serial.get(next_byte);
    inString += next_byte;
    if ( next_byte == '\n' )
      {
        cout << inString;
        inString = "";
      }
  }

/*
  serial >> inString;
  cout << inString << endl;
*/
/*
  char out_buf1[] = "?\n";
  string str1(out_buf1);
  cout << "Going to write: " << str1;
  serial.write(out_buf1, 2);
*/

  cout << "Going to write: " << "?\n";
  serial << "?\n";

  while( serial.rdbuf()->in_avail() == 0 )
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
        cout << inString;
        inString = "";
      }
  }

  usleep(100000); // Wait for another 0.1 second
/*
  cout << "Going to write: " << "$H\n";
  serial << "$H\n";

  while( serial.rdbuf()->in_avail() == 0 )
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
        cout << inString;
        inString = "";
      }
  }
*/
  outString = "G00 F300 X200 Y50 Z-40\n";
  cout << "Going to write: " << outString;
  serial << outString;

  while( serial.rdbuf()->in_avail() == 0 )
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
        cout << inString;
        inString = "";
      }
  }

  cout << "Going to write: " << "?\n";
  serial << "?\n";

  while( serial.rdbuf()->in_avail() == 0 )
  {
      usleep(100000) ; // 100 milli second or 0.1 second
  }

  inString = "";
  while( serial.rdbuf()->in_avail() > 0  )
  {
    char next_byte;
    serial.get(next_byte);
    inString += next_byte;
    if ( next_byte == '\n' ) break;
  }

  cout << inString;

  regex str_expr("<([A-Z][a-z]+)\\|WPos:(-?[0-9]+\\.[0-9]+),(-?[0-9]+\\.[0-9]+),(-?[0-9]+\\.[0-9]+)");
  smatch sm;
  if (regex_search(inString, sm, str_expr ))
  {
    for (int i=1; i<sm.size(); i++)
    {
      cout << sm[i] << endl;
    }
  }
  else cout << "Sorry, no match found!" << endl;

  serial.Close();

  return 0 ;   
}