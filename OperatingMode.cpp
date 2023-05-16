/***************************************************************************************************
 * Acuvim2
 * 
 * This module reads the Acuvim meter.
 *
 *
 * Date:
 * 28/03/2023
 *
 * Author:
 * Shaun Mcsherry
 *
 **************************************************************************************************/
#include <Arduino_MachineControl.h>
#include "APP/OperatingMode.h"

using namespace machinecontrol;

#define DC_FREQ_NOMINAL            50     // in Hz
#define DC_DEADBAND_FREQ_DEV_LIM   0.015F // deadband if frequency deviation between 
                                          // -0.015 and +0.015 */
#define DC_SMALL_DEL_FREQ_DEV_LIM  0.2F   // small power delivery if frequency deviation between 
                                          // -0.2 to -0.015 and between +0.015 to 0.2.
                                          // large power delivery if frequency deviation between 
                                          // -0.5 to -0.2 and between +0.2 to 0.5 */
#define DC_MAX_FREQ_DEV             0.5F
#define MAX_FRACTION_SMALL_DELIVERY 0.05F
#define MAX_FRACTION_LARGE_DELIVERY 1.0F

#define LARGE_DELIVERY_SLOPE  ((MAX_FRACTION_LARGE_DELIVERY - MAX_FRACTION_SMALL_DELIVERY) / \
                               (DC_MAX_FREQ_DEV - DC_SMALL_DEL_FREQ_DEV_LIM))

#define SMALL_DELIVERY_SLOPE  ((MAX_FRACTION_SMALL_DELIVERY) / \
                               (DC_SMALL_DEL_FREQ_DEV_LIM - DC_DEADBAND_FREQ_DEV_LIM))

#define FREQ_BUFFER_SIZE           18U

#define DC_THREE_HUNDRED_MS        300U  // Used as a 300ms counter in 1ms intervals */

#define PID_TEST_INTERVAL          2000U // in ms
#define PID_TEST_LOW               -10000     // in 0.1 kW units
#define PID_TEST_HIGH              10000  // in 0.1 kW units

#define PID_TEST2_VALUE            5000  // in 0.1 kW units

//#define DC_TEST_1_1
//#define DC_TEST_1_2
//#define DC_TEST_1_3
//#define DC_TEST_1_4
//#define DC_TEST_1_5
//#define DC_TEST_1_6
//#define DC_TEST_1_7
//#define DC_TEST_1_8
//#define DC_TEST_1_9
//#define DC_TEST_1_10
//#define DC_TEST_1_11
//#define DC_TEST_1_12
#define DC_TEST_1_13
//#define DC_TEST_1_14

double maxDeliveryPower = 0.0F;

/* private functions */
/***************************************************************************************************
 * UpdateHeadTail
 * 
 * This function increments the value of a ring buffer head pointer and resets to zero if overflow.
 * It also returns the tail pointer, reset to zero if overflowed.
 *
 * Parameters:
 * 
 *
 * Return:
 * Power demand
 *
 **************************************************************************************************/
uint16_t OP_MODE::UpdateHeadTail(uint16_t *headIndex, uint16_t maxIndex)
{
  uint16_t tailIndex;

  *headIndex = *headIndex + 1U;

  if(*headIndex >= maxIndex)
  {
    *headIndex = 0U;
  }
  
  tailIndex = *headIndex + 1; // Point to oldest value
  
  if(tailIndex >= FREQ_BUFFER_SIZE)
  {
    tailIndex = 0U;
  }  

  return tailIndex;
}

/***************************************************************************************************
 * DC_SmallDelivery
 * 
 * This function calculates the power demand for frequency changes that fall into the small
 * power delivery band.
 *
 * Parameters:
 * freqDev - frequency deviation from nominal grid frequency
 * maxPowKw - maximum rated power demand of inverter.
 *
 * Return:
 * Power demand in kW
 *
 **************************************************************************************************/
uint16_t OP_MODE::DC_SmallDelivery (double freqDev)
{
  uint16_t smallPowerDemand;
  double powerFraction; 
  double freqDevOffset;

  /* Limit frequency devation */
  if(freqDev > DC_SMALL_DEL_FREQ_DEV_LIM)
  {
    freqDev = DC_SMALL_DEL_FREQ_DEV_LIM;    
  }

  freqDevOffset = freqDev - DC_DEADBAND_FREQ_DEV_LIM;

  powerFraction = (SMALL_DELIVERY_SLOPE * freqDevOffset);  //y=a+bx 

  /* Calculated value should be within limits, but limit it just in case */
  if(powerFraction < 0)
  {
    powerFraction = 0;
  }
  else if (powerFraction > MAX_FRACTION_SMALL_DELIVERY)
  {
    powerFraction = MAX_FRACTION_SMALL_DELIVERY;
  }
  else
  {
    /* leave unchanged as it is within limits */
  }

  /* calculate power demand and round up/down */
  smallPowerDemand =  int16_t((powerFraction * maxDeliveryPower) + 0.5F);

  return smallPowerDemand;
}

/***************************************************************************************************
 * DC_LargeDelivery
 * 
 * This function calculates the power demand for frequency changes that fall into the large
 * power delivery band.
 *
 * Parameters:
 * freqDev - frequency deviation from nominal grid frequency
 *
 * Return:
 * Power demand in kW
 *
 **************************************************************************************************/
uint16_t OP_MODE::DC_LargeDelivery (double freqDev)
{
  uint16_t largePowerDemand;
  double powerFraction; 
  double freqDevOffset;

  /* Limit frequency devation */
  if(freqDev > DC_MAX_FREQ_DEV)
  {
    freqDev = DC_MAX_FREQ_DEV;    
  }

  freqDevOffset = freqDev - DC_SMALL_DEL_FREQ_DEV_LIM; 

  powerFraction = MAX_FRACTION_SMALL_DELIVERY + (LARGE_DELIVERY_SLOPE * freqDevOffset);  //y=a+bx 

  /* Calculated value should be within limits, but limit it just in case */
  if(powerFraction < MAX_FRACTION_SMALL_DELIVERY)
  {
    powerFraction = MAX_FRACTION_SMALL_DELIVERY;
  }
  else if (powerFraction > MAX_FRACTION_LARGE_DELIVERY)
  {
    powerFraction = MAX_FRACTION_LARGE_DELIVERY;
  }
  else
  {
    /* leave unchanged as it is within limits */
  }

  /* calculate power demand and round up/down */
  largePowerDemand =  int16_t((powerFraction * maxDeliveryPower) + 0.5F);

  return largePowerDemand;
}

/***************************************************************************************************
 * DC_RampPowerDemand
 * 
 * This function adjusts the current power demand towards the target at a rate not exceeding the
 * target power.
 *
 * Parameters:
 * target - the taget power demand
 * lastDemand - the current demanded power
 *
 * Return:
 * The updated power demand with ramp rate applied
 *
 **************************************************************************************************/
bool OP_MODE::DC_RampPowerDemand(int16_t target, 
                                 int16_t oldDemand, 
                                 int16_t *newDemand, 
                                 uint16_t rampTime, 
                                 double rampRatePer_ms)
{
  int16_t error;
  int16_t absError;
  int16_t change;
  int16_t absChange;
  bool rampInProgress = false;

  error = target - oldDemand;
  absError = abs(error);

  change = (int16_t)(((double)rampTime * rampRatePer_ms) + 0.5F);
  absChange = abs(change);

  if(absError > absChange)
  {
    *newDemand = oldDemand + change;
    rampInProgress = true;
  }
  else
  {
    *newDemand = target;
  }

  return rampInProgress;
}

/***************************************************************************************************
 * UpdatePowerTarget
 * 
 * This function adjusts the current power demand based on the frequency deviation from the
 * nominal.
 *
 * Parameters:
 * freqDiff - the deviation from the nominal frequency.
 *
 * Return:
 * The target power demand.
 *
 **************************************************************************************************/
int16_t OP_MODE::DC_UpdatePowerTarget(double freqDiff)
{
  double absFreqDeviation;
  int16_t targetPower;

  absFreqDeviation = fabs(freqDiff);

  if (absFreqDeviation <= DC_DEADBAND_FREQ_DEV_LIM)
  {
    targetPower = 0;
  }
  else if(absFreqDeviation <= DC_SMALL_DEL_FREQ_DEV_LIM)
  {
    targetPower = (int16_t)DC_SmallDelivery(absFreqDeviation);
  }
  else
  {
    /* frequency deviation requires large power delivery */
    targetPower = (int16_t)DC_LargeDelivery(absFreqDeviation);    
  }

  /* Take power from grid to charge batteries if frequency above nominal, i.e. negative demand */
  if(freqDiff > 0.0F)
  {
    targetPower *= -1;  
  }

  return targetPower;
}

/***************************************************************************************************
 * DC_Test_1_1
 * 
 * This function simulates Test 1.1 of National Grid DC spec. It should be called every 20ms.
 *
 * Parameters:
 * None
 *
 * Return:
 * Frequency in Hz
 *
 **************************************************************************************************/
double OP_MODE::DC_Test_1_1(void)
{
  static uint16_t dcTestCount = 0U;
  static double testFreq = 50.00;

  dcTestCount++;

  if (50U == dcTestCount)
  {
    testFreq = 50.01;
    digital_outputs.set(1, HIGH); 
  }

  if(dcTestCount >= 100U)
  {
    dcTestCount = 0U;
    testFreq = 50.00;
    digital_outputs.set(1, LOW);
  }

  return testFreq;
}

/***************************************************************************************************
 * DC_Test_1_2
 * 
 * This function simulates Test 1.2 of National Grid DC spec. It should be called every 20ms.
 *
 * Parameters:
 * None
 *
 * Return:
 * Frequency in Hz
 *
 **************************************************************************************************/
double OP_MODE::DC_Test_1_2(void)
{
  static uint16_t dcTestCount = 0U;
  static double testFreq = 50.00;

  dcTestCount++;

  if (50U == dcTestCount)
  {
    testFreq = 49.99;
    digital_outputs.set(1, LOW); 
  }

  if(dcTestCount >= 100U)
  {
    dcTestCount = 0U;
    testFreq = 50.00;
    digital_outputs.set(1, HIGH);
  }

  return testFreq;
}

/***************************************************************************************************
 * DC_Test_1_5
 * 
 * This function simulates Test 1.5 of National Grid DC spec. It should be called every 20ms.
 *
 * Parameters:
 * None
 *
 * Return:
 * Frequency in Hz
 *
 **************************************************************************************************/
double OP_MODE::DC_Test_1_5(void)
{
  static uint16_t dcTestCount = 0U;
  static double testFreq = 50.00;

  dcTestCount++;

  if (50U == dcTestCount)
  {
    testFreq = 50.10;
    digital_outputs.set(1, HIGH); 
  }

  if(dcTestCount >= 100U)
  {
    dcTestCount = 0U;
    testFreq = 50.0;
    digital_outputs.set(1, LOW);
  }

  return testFreq;
}

/***************************************************************************************************
 * DC_Test_1_7
 * 
 * This function simulates Test 1.7 of National Grid DC spec. It should be called every 20ms.
 *
 * Parameters:
 * None
 *
 * Return:
 * Frequency in Hz
 *
 **************************************************************************************************/
double OP_MODE::DC_Test_1_7(void)
{
  static uint16_t dcTestCount = 0U;
  static double testFreq = 50.00;

  dcTestCount++;

  if (50U == dcTestCount)
  {
    testFreq = 50.20;
    digital_outputs.set(1, HIGH); 
  }

  if(dcTestCount >= 100U)
  {
    dcTestCount = 0U;
    testFreq = 50.0;
    digital_outputs.set(1, LOW);
  }

  return testFreq;
}

/***************************************************************************************************
 * DC_Test_1_9
 * 
 * This function simulates Test 1.9 of National Grid DC spec. It should be called every 20ms.
 *
 * Parameters:
 * None
 *
 * Return:
 * Frequency in Hz
 *
 **************************************************************************************************/
double OP_MODE::DC_Test_1_9(void)
{
  static uint16_t dcTestCount = 0U;
  static double testFreq = 50.00;

  dcTestCount++;

  if (50U == dcTestCount)
  {
    testFreq = 50.30;
    digital_outputs.set(1, HIGH); 
  }

  if(dcTestCount >= 100U)
  {
    dcTestCount = 0U;
    testFreq = 50.0;
    digital_outputs.set(1, LOW);
  }

  return testFreq;
}

/***************************************************************************************************
 * DC_Test_1_11
 * 
 * This function simulates Test 1.7 of National Grid DC spec. It should be called every 20ms.
 *
 * Parameters:
 * None
 *
 * Return:
 * Frequency in Hz
 *
 **************************************************************************************************/
double OP_MODE::DC_Test_1_11(void)
{
  static uint16_t dcTestCount = 0U;
  static double testFreq = 50.00;

  dcTestCount++;

  if (50U == dcTestCount)
  {
    testFreq = 50.40;
    digital_outputs.set(1, HIGH); 
  }

  if(dcTestCount >= 100U)
  {
    dcTestCount = 0U;
    testFreq = 50.0;
    digital_outputs.set(1, LOW);
  }

  return testFreq;
}

/***************************************************************************************************
 * DC_Test_1_13
 * 
 * This function simulates Test 1.13 of National Grid DC spec. It should be called every 20ms.
 *
 * Parameters:
 * None
 *
 * Return:
 * Frequency in Hz
 *
 **************************************************************************************************/
double OP_MODE::DC_Test_1_13(void)
{
  static uint16_t dcTestCount = 0U;
  static double testFreq = 50.00;

  dcTestCount++;

  if (50U == dcTestCount)
  {
    testFreq = 50.50;
    digital_outputs.set(1, HIGH); 
  }

  if(dcTestCount >= 100U)
  {
    dcTestCount = 0U;
    testFreq = 50.0;
    digital_outputs.set(1, LOW);
  }

  return testFreq;
}

/* Public functions */
/***************************************************************************************************
 * DC_Init
 * 
 * This function initialises power limits for dynamic containment.
 *
 * Parameters:
 * maxPower - the maximum rated power transfer of the inverter
 * systemCounter
 *
 * Return:
 * None
 *
 **************************************************************************************************/
void OP_MODE::DC_Init(uint16_t maxPower, uint16_t systemCounter)
{
  maxDeliveryPower = (double)maxPower;
}

/***************************************************************************************************
 * DC_Control
 * 
 * This function implements Dynamic Containment. It should be called once every 20ms.
 * The specification for DC states that power response to frequency changes must be delayed between
 * 250 to 500ms. In order to achieve this, measured frequency is held in a ring buffer containing
 * the last 18 samples (320ms delay), using the tail value (oldest value) of the buffer to 
 * provide the required power response.
 *
 * Parameters:
 * frequency - the most current measured frequency
 *
 * Return:
 * Power demand
 *
 **************************************************************************************************/
int16_t OP_MODE::DC_Control(double frequency, uint16_t sysCount)
{
  static double freqBuffer[FREQ_BUFFER_SIZE] =
  {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
  };
  static uint16_t headPtr = 0;
  uint16_t tailPtr;
  double freqDelayed; 
  double freqDeviation;
  static double oldFreqDeviation = 0.0;
  static int16_t targetPowerDemand = 0;
  static uint16_t oldSysCount;
  uint16_t rampTime = 0U;
  int16_t newPowerDemand;
  static int16_t oldPowerDemand = 0;
  static double rampRatePer_ms = 0.0;
  static bool isRamping = false;

  #ifdef DC_TEST_1_1
    freqBuffer[headPtr] = DC_Test_1_1();
  #elif defined DC_TEST_1_2
    freqBuffer[headPtr] = DC_Test_1_2();
  #elif defined DC_TEST_1_5
    freqBuffer[headPtr] = DC_Test_1_5();
  #elif defined DC_TEST_1_7
    freqBuffer[headPtr] = DC_Test_1_7();
  #elif defined DC_TEST_1_9
    freqBuffer[headPtr] = DC_Test_1_9();
  #elif defined DC_TEST_1_11
    freqBuffer[headPtr] = DC_Test_1_11();
  #elif defined DC_TEST_1_13
    freqBuffer[headPtr] = DC_Test_1_13();
  #else
   freqBuffer[headPtr] = frequency;
  #endif

  tailPtr = UpdateHeadTail(&headPtr, FREQ_BUFFER_SIZE);

  freqDelayed = freqBuffer[tailPtr];

  freqDeviation = freqDelayed - DC_FREQ_NOMINAL;

  /* if frequency deviation is less than 0.01Hz treat this as no change from previous
     time when it was >= 0.01Hz */
  if(fabs(freqDeviation - oldFreqDeviation) >= 0.01)
  {
    /* >= 0.01Hz, so update oldFreqDeviation */
    oldFreqDeviation = freqDeviation;
    targetPowerDemand = DC_UpdatePowerTarget(freqDeviation);
    rampRatePer_ms = (double)(targetPowerDemand - oldPowerDemand) / (double)DC_THREE_HUNDRED_MS; 
  }
  else
  {
    /* targetPowerDemand is static, so use last calculated value */
  }

  if(true == isRamping)
  {
    /* if output is ramping to new demand, get the elapsed time since the last ramp demand */
    rampTime = sysCount - oldSysCount;
  }
  else
  {
    rampTime = 0U;
  }

  isRamping = DC_RampPowerDemand(targetPowerDemand, 
                                 oldPowerDemand, 
                                 &newPowerDemand, 
                                 rampTime, 
                                 rampRatePer_ms);

  oldSysCount = sysCount;
  oldPowerDemand = newPowerDemand;

  //return targetPowerDemand;
  return newPowerDemand;
}

uint16_t OP_MODE::FFR_Control(double frequency)
{

}

uint16_t OP_MODE::DS3_Control(double frequency)
{

}

int16_t OP_MODE::PID_TestControl1(uint16_t sysCount)
{
  static uint16_t oldSysCount = 0;
  static bool toggleDemand = false;
  static int16_t powerDemand = 0;
  uint16_t duration;

  duration = sysCount - oldSysCount;

  if(duration >= PID_TEST_INTERVAL)
  {
    if(true == toggleDemand)
    {
      digital_outputs.set(1, HIGH);
      powerDemand = PID_TEST_HIGH;
      toggleDemand = false;
    }
    else
    {
      digital_outputs.set(1, LOW);
      powerDemand = PID_TEST_LOW;
      toggleDemand = true;
    }
    oldSysCount = sysCount;
  }
  else
  {
    ; //do nothing
  }

  return powerDemand;
}

int16_t OP_MODE::PID_TestControl2(void)
{
  static int16_t powerDemand;

  powerDemand = PID_TEST2_VALUE;

  return powerDemand;
}

/* end public functions */
