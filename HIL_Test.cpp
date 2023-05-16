/***************************************************************************************************
 * HIL_Test
 * 
 * This module uses the Typhoon HIL for testing.
 *
 *
 * Date:
 * 17/04/2023
 *
 * Author:
 * Shaun Mcsherry
 *
 **************************************************************************************************/
#include <Arduino_MachineControl.h>
#include "APP/HIL_Test.h"

using namespace machinecontrol;

double res_divider = 0.28057;
double reference = 3.3;

#define MAX_ADC_16_BIT  65535

#define MIN_FREQ_DEV    -0.7F
#define MAX_FREQ_DEV    0.7F

#define FREQ_SLOPE      ((MAX_FREQ_DEV - MIN_FREQ_DEV)/(double)MAX_ADC_16_BIT)
#define FREQ_OFFSET     MIN_FREQ_DEV

#define NOMINAL_FREQ    50    // in hertz

double powerSlope = 0.0;
double powerOffset = 0.0;

/* private functions */

/* Public functions */
/***************************************************************************************************
 * HIL_TestInit
 * 
 * This function initialises conditions for running on Typhoon HIL.
 *
 * Parameters:
 * None
 *
 * Return:
 * None
 *
 **************************************************************************************************/
void HIL_TEST::Init(int16_t ratedPower)
{
  analogReadResolution(16);   // Configure ADCs as 16-bit
  analog_in.set0_10V();       // Configure input resistors 

  powerSlope = (((double)ratedPower - (-1.0 * (double)ratedPower)) / 65535.0);

  powerOffset = (double)(-ratedPower);

}

/***************************************************************************************************
 * GetFreq
 * 
 * This function reads the analogue voltage from the HIL that represents frequency and converts
 * it into frequency (Hz).
 *
 * Parameters:
 * None
 *
 * Return:
 * None
 *
 **************************************************************************************************/
double HIL_TEST::GetFreq(void)
{
  uint16_t rawAdc;
  double frequency;
  
  rawAdc = analog_in.read(0);   // Gets 16-bit raw ADC value. This is a representation of
                                // frequency deviation from 50Hz: -0.5Hz = 0, +0.5Hz = 65535

  frequency = NOMINAL_FREQ + (FREQ_OFFSET + (FREQ_SLOPE * (float)rawAdc));
  
  return frequency;
}

/***************************************************************************************************
 * GetPower
 * 
 * This function reads the analogue voltage from the HIL that represents power and converts
 * it into watts.
 *
 * Parameters:
 * None
 *
 * Return:
 * None
 *
 **************************************************************************************************/
double HIL_TEST::GetPower(void)
{
  uint16_t rawAdc;
  double filteredAdc;
  double power;

  rawAdc = analog_in.read(1);   // Gets 16-bit raw ADC value. This is a representation of
                                // output power: -ratedPower = 0, +ratedPower = 65535  

  //power = powerOffset + (powerSlope * filteredAdc);
  power = ((double)rawAdc * 0.5113) - 15252;

  // adjust power for non-linearities in arduino
  //shaun power = 970.0F + (power * 1.14F);

  return power;
}
/* end public functions */
