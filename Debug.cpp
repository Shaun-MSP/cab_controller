/***************************************************************************************************
 * Debug_MID
 * 
 * This module is written as an interface to accessing the Serial port using non-member functions 
 * to use for outputting debug messages.
 *
 * Circuit:
 *  - Portenta H7
 *  - Machine Control
 *
 * Date:
 * 21/02/2023
 *
 * Author:
 * Shaun Mcsherry
 *
 **************************************************************************************************/
#include <Arduino_MachineControl.h>

#define DEBUG_BAUD_RATE    9600U

using namespace machinecontrol;

/***************************************************************************************************
 * Debug_Setup
 * 
 * This function initialises the serial port used for debug output.
 *
 * Parameters:
 * None
 *
 * Return:
 * None
 *
 * Date:
 * 21/02/2023
 *
 * Author:
 * Shaun Mcsherry
 *
 **************************************************************************************************/
void Debug_Setup(void) 
{
  Serial.begin(DEBUG_BAUD_RATE);
  while (!Serial) 
  {
    ; // wait for serial port to connect.
  }
  Serial.println("Debug initialised");
}

extern "C" void C_SerialPrint(char * string)
{
  Serial.print(string);
}

extern "C" void C_SerialPrintln(char * string)
{
  Serial.println(string);
}

extern "C" void C_SerialPrintnum(int number)
{
  Serial.println(number);
}

extern "C" void C_SerialPrintfloat(double number)
{
  Serial.println(number);
}