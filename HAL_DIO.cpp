/***************************************************************************************************
 * HAL_Timer
 * 
 * This module is written as an interface to accessing the Digital IO using non-member functions.
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
#include "Wire.h"
#include "HAL/HAL_DIO.h"

using namespace machinecontrol;

/* Private functions */
/***************************************************************************************************
 * DIO_InitOutputs
 * 
 * This function initialises the digital output pins.  
 * Please remember that pin "24V IN" of the connector DIGITAL_OUTPUTS must be connected to 24V.
 * The DIGITAL OUT channels are high side switches capable to handle up to 0.5A. There is an over 
 * current protection that open the channel when the current is above 0.7A with a +-20% tolerance.
 * The over current protection can be set to have two different behaviors, and it is the same for 
 * all channels:
 *   1) Latch mode: when the over current is detected the channel is opened, and will remain 
 *      opened until it is toggled via software.
 *
 *   2) Auto retry: when the over current is detected the channel is opened, but after some tens of
 *      milliseconds the channel will automatically try to close itself again. In case of a 
 *      persistent overcurrent the channel will continuously toggle.
 *
 * Parameters:
 * None
 *
 * Return:
 * None
 *
 * Date:
 * 28/02/2023
 *
 **************************************************************************************************/
void DIO_InitOutputs(void)
{
  //Set over current behavior of all channels to latch mode:
  digital_outputs.setLatch();

  // Uncomment this line to set over current behavior of all
  // channels to auto retry mode instead of latch mode:
  //digital_outputs.setRetry();
  
  //At startup set all channels to OPEN
  digital_outputs.setAll(0);  
}

/***************************************************************************************************
 * DIO_InitOutputs
 * 
 * This function initialises the digital input pins.  
 *
 * Parameters:
 * None
 *
 * Return:
 * None
 *
 * Date:
 * 28/02/2023
 *
 **************************************************************************************************/
void DIO_InitInputs(void)
{
  Wire.begin();

  if (!digital_inputs.init()) 
  {
    Serial.println("Digital input GPIO expander initialization fail!!");
  }
}

/* public functions */
/***************************************************************************************************
 * DIO_Init
 * 
 * This function initialises the digital input and output pins.
 *
 * Parameters:
 * None
 *
 * Return:
 * None
 *
 * Date:
 * 28/02/2023
 *
 **************************************************************************************************/
void DIO_Init() 
{
  Serial.println("Initialising DIO");
  DIO_InitOutputs();
  DIO_InitInputs();
  Serial.println("DIO initialised");
}

/***************************************************************************************************
 * DIO_OutPin
 * 
 * This function sets the specified output pin to the specified state.
 *
 * Parameters:
 * pin - The pin to configure the state of.
 * state - PIN_HIGH or PIN_LOW
 *
 * Return:
 * None
 *
 * Date:
 * 28/02/2023
 *
 **************************************************************************************************/
 void DIO_OutPin(outPinId_enum_t pin, pinStateEnum_t state)
 {
   digital_outputs.set(pin, state);
   digital_outputs.set(0, HIGH);   
 }
