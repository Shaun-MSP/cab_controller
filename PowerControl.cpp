/***************************************************************************************************
 * Power Control
 * 
 * This module contains the main application ac power control functions.
 *
 * Date:
 * 22/02/2023
 *
 * Author:
 * Shaun McSherry
 *
 * Va.0
 *
 **************************************************************************************************/
#include <Arduino_MachineControl.h>
#include <PID_v1.h>
#include <stdint.h>
#include <stdbool.h>
#include "APP/PowerControl.h"
#include "APP/Acuvim2.h"
#include "APP/APP_CAN.h"
#include "APP/Controller.h"
#include "HAL/HAL_Timer.h"
#include "APP/Flex.h"
#include "APP/Controller.h"
#include "APP/OperatingMode.h"
extern "C" 
{
  #include "UTILS/lp_filter.h"
}
#ifdef HIL_TST
 #include "APP/HIL_Test.h"

 #define METER_DELAY_20MS  2U // in 10ms units
#endif 

using namespace machinecontrol;

#define INVERTER_ON_OFF_SCHEDULE 100U  // in ms units
#define CAN_TX_DELAY_TIME        3U    // in ms units

typedef enum CONTROLLER_STATE_ENUM
{
  CONTROLLER_STATE_STOP_ENTRY     = 0,
  CONTROLLER_STATE_STOP_DURING    = 1,
  CONTROLLER_STATE_INIT_ENTRY     = 2,
  CONTROLLER_STATE_INIT_DURING    = 3,
  CONTROLLER_STATE_RUN_ENTRY      = 4,
  CONTROLLER_STATE_RUN_DURING     = 5
}controllerStateEnum_t;

double CAB1000_LUT[LUT_MAX_INDEX][2] =
{
   /* Requested power, actual power - all in 0.1kW units */
  {   -15000.0,           -15000.0}, 
  {   -14000.0,           -14005.0}, 
  {   -13000.0,           -13000.0}, 
  {   -12000.0,           -11906.0}, 
  {   -11000.0,           -10905.0}, 
  {   -10000.0,           -9920.0}, 
  {   -9000.0,            -8890.0}, 
  {   -8000.0,            -7840.0}, 
  {   -7000.0,            -6810.0}, 
  {   -6000.0,            -5800.0}, 
  {   -5000.0,            -4770.0}, 
  {   -4000.0,            -3750.0}, 
  {   -3000.0,            -2780.0}, 
  {   -2000.0,            -1780.0}, 
  {   -1000.0,            -890.0}, 
  {   0.0,                10.0}, 
  {   1000.0,             890.0},
  {   2000.0,             1780.0},
  {   3000.0,             2800.0},
  {   4000.0,             3800.0},
  {   5000.0,             4750.0},
  {   6000.0,             5760.0},
  {   7000.0,             6780.0},
  {   8000.0,             7810.0},
  {   9000.0,             8820.0},
  {   10000.0,            9850.0},
  {   11000.0,            10870.0},
  {   12000.0,            11900.0},
  {   13000.0,            12910.0},
  {   14000.0,            13920.0},
  {   15000.0,            15000.0}
};

ACUVIM_II acuvimObj;
FLEX flexObj;
APP_CAN canObj;
OP_MODE opModeObj;

#ifdef HIL_TST
 HIL_TEST hilTestObj;
#endif

acuvimBasicMeasurement20ms_t meterData;
flexOperatingStateStruct_t requestedState;
static bool newMeterData = false;
 
#ifdef GRID_VOLTAGE_480_RMS
 uint16_t maxRated = 10430U; // in 0.1kW units
#elif defined GRID_VOLTAGE_600_RMS
 uint16_t maxRated = 13040U; // in 0.1kW units
#elif defined GRID_VOLTAGE_630_RMS
 uint16_t maxRated = 13690U; // in 0.1kW units
#elif defined GRID_VOLTAGE_660_RMS
 uint16_t maxRated = 14350U; // in 0.1kW units
#elif defined GRID_VOLTAGE_690_RMS
 uint16_t maxRated = 15000U; // in 0.1kW units
#endif

/* array of objects to control */
pcAcObjStruct_t pcAcObj[(uint8_t)NOOF_PC_AC_OBJECTS];

/* set up PID objects for current and power */
PID powerPid(&pcAcObj[AC_POWER_CONTROL].measuredScaled, 
             &pcAcObj[AC_POWER_CONTROL].pidOutput, 
             &pcAcObj[AC_POWER_CONTROL].setPointScaled,
             pcAcObj[AC_POWER_CONTROL].pGain, 
             pcAcObj[AC_POWER_CONTROL].iGain,
             pcAcObj[AC_POWER_CONTROL].dGain, DIRECT);

PID currentPid(&pcAcObj[AC_CURRENT_CONTROL].measuredScaled, 
               &pcAcObj[AC_CURRENT_CONTROL].pidOutput, 
               &pcAcObj[AC_CURRENT_CONTROL].setPointScaled,
               pcAcObj[AC_CURRENT_CONTROL].pGain, 
               pcAcObj[AC_CURRENT_CONTROL].iGain,
               pcAcObj[AC_CURRENT_CONTROL].dGain, DIRECT);

lp_filter_ModelData hil_filter;

/***************************************************************************************************
 * GetStoredParams
 * 
 * This function is called at initialisation, and reads the PID process parameters from NVR.
 *
 * Parameters:
 * None
 *
 * Return:
 * None
 *
 **************************************************************************************************/
void POWER_CTRL::GetStoredParams(void)
{
    /* todo read parameters from NVR */
}

/***************************************************************************************************
 *
 * ScaleEngUnit
 * Thus function scales a number representing engineering units, e.g. kW to a value between 
 * +1.0 and -1.0 (if value is within min-max limits). 
 * If limit is set and the result is > 1.0 it is limited to 1.0, and if < -1.0 it is limited 
 * to -1.0.
 * If limit is not set, the scaled value can exceed -1.0 and 1.0 if it is outside the min-
 * max limits.
 *
 * Parameter(s): 
 * value - the demanded or measured value (as an engineering unit).
 * max - the maximum expected demanded value.
 * min - the minimum expected demanded value.
 * limit - if true, scaled value cannot exceed +1.0 or -1.0.
 *
 * Return:
 * A value between -1.0 and 1.0
 *
 **************************************************************************************************/
inline double POWER_CTRL::ScaleEngUnit(int16_t value, int16_t min, int16_t max, bool limit)
{
    double scaledValue;

    scaledValue = (double)((double)value / (((double)max - (double)min) / 2.0));

    if (true == limit)
    {
        if (scaledValue > 1.0)
        {
            scaledValue = 1.0;
        }
        else if (scaledValue < -1.0)
        {
            scaledValue = -1.0;
        }
        else
        {
            /* within limits */
        }
    }   

    return scaledValue;
}

/***************************************************************************************************
 *
 * Unscale
 * Thus function restores a scaled number back into engineering units.
 * If the descaled number is outside the min/max limits, the number is limited to these limits.
 *
 * Parameter(s): 
 * value - the scaled number.
 * max - the maximum expected converted value.
 * min - the minimum expected converted value.
 *
 * Return:
 * A value between -1.0 and 1.0
 *
 **************************************************************************************************/
inline int16_t POWER_CTRL::Unscale(double value, int16_t min, int16_t max)
{
    int16_t engUnit;

    engUnit = (int16_t)(value * (double)(((double)max - (double)min)/2.0));

    if (engUnit > max)
    {
        engUnit = max;
    }
    else if (engUnit < min)
    {
        engUnit = min;
    }
    else
    {
        /* within limits */
    }

    return engUnit;
}

/***************************************************************************************************
 *
 * ReadMeter
 * This function attempts to read the meter.
 * If the meter cannot be read for 2 seconds, it is considered to be unavailable and signals this
 * to calling function.
 *
 * Parameter(s): 
 * None
 *
 * Return:
 * True if meter is available, otherwise false.
 *
 **************************************************************************************************/
bool POWER_CTRL::ReadMeter(void)
{
  bool isMeterDataAvail;
  static uint16_t meterLatency_ms = 0U;
  bool meterAvailable = true;
  bool isReady;

  isMeterDataAvail = acuvimObj.Control(&meterData);   //poll meter for new measurements.

  isReady = acuvimObj.GetReadyState();

  if(true == isReady) // Only proceed if no fault detected with meter
  {
    if (true == isMeterDataAvail)   // proceed if fresh meter data
    {
      meterLatency_ms = 0U;         // Reset meter latency timer
      newMeterData = true;
    }
    else
    {
      if (meterLatency_ms <= UINT16_MAX)
      {
        meterLatency_ms++;
      }
    }
  }

  if ((meterLatency_ms >= TWO_SECONDS_MS) || (true == isReady))
  {
    meterAvailable = false;
  }

  return meterAvailable;
}

/***************************************************************************************************
 * ScaleAnalogue
 *
 * This function is called to scale the -1.0 to +1.0 pid values to 0 to 10.5V values suitable for
 * analogue output channels.
 * 
 * Parameter(s): 
 * value - value to convert
 *
 * Return:
 * the converted value
 *
 **************************************************************************************************/
double POWER_CTRL::ScaleAnalogue(double value)
{
  double scaledValue;

  scaledValue = ((value / 2) + 0.5) * 10.5 ;

  if(scaledValue > 10.5F)
  {
    scaledValue > 10.5F;
  }
  if(scaledValue < 0.0F)
  {
    scaledValue = 0.0F;
  }

  return scaledValue;
}

/***************************************************************************************************
 *
 * PidParamsAnaOut
 * This function is called to take the -1.0 to 1.0 pid scaling and output them on analogue 
 * outputs in a 0 to 10.5V form.
 * 
 * Parameter(s): 
 * pidParams - PID parameter structure
 *
 * Return:
 * None
 *
 **************************************************************************************************/
void POWER_CTRL::PidParamsAnaOut(double setpoint, double measuredValue, double pidOut)
{
  double setpointAna;
  double measuredAna;
  double pidOutAna;

  setpointAna = setpoint / (double)maxRated;
  measuredAna = measuredValue / (double)maxRated;
  pidOutAna = pidOut / (double)maxRated;

  setpointAna = ScaleAnalogue(setpointAna);
  measuredAna = ScaleAnalogue(measuredAna);
  pidOutAna   = ScaleAnalogue(pidOutAna);

  analog_out.write(0, setpointAna);
  analog_out.write(1, measuredAna);
  analog_out.write(2, pidOutAna);
}

/***************************************************************************************************
 *
 * ManagePower
 * This function is called to get the power demand and run the PID.
 *
 * Parameter(s): 
 * None
 *
 * Return:
 * true if CAN message has been transmitted.
 *
 **************************************************************************************************/
bool POWER_CTRL::ManagePower(uint16_t sysCount)
{
  double unadjustedDemand;
  double adjustedDemand;
  bool txInProgress = false;
  static bool pinToggle = false;
  double error;

  if(true == pinToggle)
  {
    digital_outputs.set(0, HIGH);
    pinToggle = false;
  }
  else
  {
    digital_outputs.set(0, LOW);
    pinToggle = true;
  }

  switch (requestedState.operatingMode)
  {
    case TRADING:
      unadjustedDemand = flexObj.GetDemand();
      break;

    case DC:
      unadjustedDemand = opModeObj.DC_Control(meterData.frequency, sysCount);
      break;        
      
    case FFR:
      unadjustedDemand = opModeObj.FFR_Control(meterData.frequency);
      break;

    case DS3:
      unadjustedDemand = opModeObj.DS3_Control(meterData.frequency);
      break;

    case PID_TEST1:
      unadjustedDemand = opModeObj.PID_TestControl1(sysCount);
      break;

    case PID_TEST2:
      unadjustedDemand = opModeObj.PID_TestControl2();
      break;

    default:
      /* invalid state */
      break;
  }

  if (AC_POWER_CONTROL_MODE == mode)
  {
    pcAcObj[AC_POWER_CONTROL].setPointScaled = unadjustedDemand;
    pcAcObj[AC_POWER_CONTROL].measuredScaled = meterData.totalPowerReal;

    
    /* Run the PID controller */
    powerPid.Compute();
    /* Filter PID output */
    adjustedDemand = pcAcObj[AC_POWER_CONTROL].pidOutput;

    txInProgress = canObj.SetPower(adjustedDemand, 0);
    
    /* write representation of PID parameters to Analogue out for test/tuning */
    PidParamsAnaOut(pcAcObj[AC_POWER_CONTROL].setPointScaled,   /* AO 0 */
                    pcAcObj[AC_POWER_CONTROL].measuredScaled,   /* AO 1 */
                    pcAcObj[AC_POWER_CONTROL].pidOutput);       /* AO 2 */
  }
  else if (AC_CURRENT_CONTROL_MODE == mode)
  {
    /* Scale the setpoint between -1.0 and +1.0 */
    SetCurrentSetpoint(adjustedDemand);

    pcAcObj[AC_CURRENT_CONTROL].measuredScaled = ScaleEngUnit(meterData.averagePhaseCurrent, 
                                                                  -(maxRated), 
                                                                  maxRated, 
                                                                  false);
    
    /* real current PID control. Output is a scaled number from -1.0 to +1.0 */
    //pid_output = pidObj.Update(&pcAcObj[AC_CURRENT_CONTROL_MODE].real);
    currentPid.Compute();
    /* Convert scaled output into real units */        
    adjustedDemand = SetCurrentControl(pcAcObj[AC_CURRENT_CONTROL].pidOutput);
    
    //txInProgress = canObj.SetCurrent(adjustedDemand, 0);
    txInProgress = canObj.SetCurrent(adjustedDemand, 0);

    /* write representation of PID parameters to Analogue out for test/tuning */
    PidParamsAnaOut(pcAcObj[AC_CURRENT_CONTROL].setPointScaled,
                    pcAcObj[AC_CURRENT_CONTROL].measuredScaled, 
                    pcAcObj[AC_CURRENT_CONTROL].pidOutput);
  }
  else
  {
    /* invalid control mode */
  } 

  return txInProgress;
}

/***************************************************************************************************
 *
 * TxSchedule
 * This function is called to schedule transmission of the periodic CAN messages. It ensures
 * a gap between transmission to prevent message overrun.
 *
 * Parameter(s): 
 * sysCounter
 *
 * Return:
 * true if CAN message sent (will always be this), otherwise false
 *
 **************************************************************************************************/
bool POWER_CTRL::TxInverterOnOff(bool inverterEnable)
{
  bool txInProgress = false;
  // Send inverter disable/enable signal every 100ms
  if (true == inverterEnable)
  {
    txInProgress = canObj.InverterEnable();
  }
  else
  {
    txInProgress = canObj.InverterDisable();
  }
  return txInProgress;
}

/***************************************************************************************************
 *
 * DisplayControllerState
 * This function is called to transmit the inverter state out of the debug port.
 *
 * Parameter(s): 
 * state
 *
 * Return:
 * None
 *
 **************************************************************************************************/
void POWER_CTRL::DisplayControllerState(statusBitsEnum_t state)
{
  Serial.print("Inverter State: ");

  switch (state)
  {
    case POWER_ON_RESET: 
      Serial.println("POWER_ON_RESET");
      break;
    case READY: 
      Serial.println("READY");
      break;        
    case FOLLOWING: 
      Serial.println("FOLLOWING");
      break;     
    case FAULT:          
      Serial.println("FAULT");
      break;
    case FORMING:        
      Serial.println("FORMING");
      break;
    case RECONNECT_DELAY:
      Serial.println("RECONNECT_DELAY");
      break;
    case NA_1:           
      Serial.println("NA_1");
      break;
    case GRID_LOST:      
      Serial.println("GRID_LOST");
      break;
    case CHARGING:       
      Serial.println("CHARGING");
      break;
    case RIDE_THROUGH:   
      Serial.println("RIDE_THROUGH");
      break;
    case CESSATION:      
      Serial.println("CESSATION");
      break;
    case TRANSITIONING:  
      Serial.println("TRANSITIONING");
      break;
    case INHIBITED:      
      Serial.println("INHIBITED");
      break;
    case CONNECT_DELAY:  
      Serial.println("CONNECT_DELAY");
      break;
    case NA_2:           
      Serial.println("NA_2");
      break;
    case NA_3:           
      Serial.println("NA_3");
      break;
    default:
      Serial.println("UNKNOWN_STATE");
      break;  
  }
}

#ifdef PID_TUNE
void POWER_CTRL::PID_TuneParams(void)
{
  String pidCommand;
  String pidType;
  String valueString;
  double value;
  int strLen;

  pidCommand = Serial.readString();

  strLen = pidCommand.length();
  
  if(5U == strLen)
  {
    valueString = pidCommand.substring(1,4);
    value = valueString.toDouble();    
    pidType = (pidCommand.substring(0,1)); // Get p, i or d

    if ("p" == pidType)
    {
      pcAcObj[AC_POWER_CONTROL].pGain = value;
      Serial.print("P=");
      Serial.println(value);
    }
    else if ("i" == pidType)
    {
      pcAcObj[AC_POWER_CONTROL].iGain = value;
      Serial.print("I=");
      Serial.println(value);
    }
    else if ("d" == pidType)
    {
      pcAcObj[AC_POWER_CONTROL].dGain = value;
      Serial.print("D=");
      Serial.println(value);
    }
    else
    {
      /* any 5 characters not starting with p, i or d will display current settings */
      Serial.print("P=");
      Serial.println(pcAcObj[AC_POWER_CONTROL].pGain);
      Serial.print("I=");
      Serial.println(pcAcObj[AC_POWER_CONTROL].iGain);
      Serial.print("D=");
      Serial.println(pcAcObj[AC_POWER_CONTROL].dGain);
    }

    powerPid.SetTunings(pcAcObj[AC_POWER_CONTROL].pGain,
                        pcAcObj[AC_POWER_CONTROL].iGain, 
                        pcAcObj[AC_POWER_CONTROL].dGain);
  }

  Serial.flush();
}
#endif

/* Public functions */
/***************************************************************************************************
 * Init
 * 
 * This function initialises PID control process parameters.
 *
 * Parameters:
 * None
 *
 * Return:
 * None
 *
 **************************************************************************************************/
void POWER_CTRL::Init(void)
{
    pcAcObj[AC_POWER_CONTROL].setPointScaled = 0.0;
    pcAcObj[AC_POWER_CONTROL].measuredScaled = 0.0;

    pcAcObj[AC_CURRENT_CONTROL].setPointScaled = 0.0;
    pcAcObj[AC_CURRENT_CONTROL].measuredScaled = 0.0;

    GetStoredParams();
    
    /* GetStoredParams must be called before initialising these params */
    pcAcObj[AC_POWER_CONTROL].pGain = p_realPowerGain;
    pcAcObj[AC_POWER_CONTROL].iGain = i_realPowerGain;
    pcAcObj[AC_POWER_CONTROL].dGain = d_realPowerGain;

    pcAcObj[AC_CURRENT_CONTROL].pGain = p_currentGain;
    pcAcObj[AC_CURRENT_CONTROL].iGain = i_currentGain;
    pcAcObj[AC_CURRENT_CONTROL].dGain = d_currentGain;
    /* End GetStoredParams must be called before initialising these params */

    acuvimObj.Init();   /* Initialise the meter */
    canObj.Init();      /* Initialise the CAN bus */
    #ifdef HIL_TST
     hilTestObj.Init(maxRated);
    #endif

    powerPid.SetOutputLimits(-(double)maxRated, (double)maxRated);
    powerPid.SetTunings(pcAcObj[AC_POWER_CONTROL].pGain,
                        pcAcObj[AC_POWER_CONTROL].iGain, 
                        pcAcObj[AC_POWER_CONTROL].dGain);
    powerPid.SetSampleTime(20);
    powerPid.SetMode(AUTOMATIC);

    currentPid.SetOutputLimits(-10.0, 10.0);
    currentPid.SetTunings(pcAcObj[AC_CURRENT_CONTROL].pGain,
                          pcAcObj[AC_CURRENT_CONTROL].iGain, 
                          pcAcObj[AC_CURRENT_CONTROL].dGain);
    currentPid.SetSampleTime(20);
    currentPid.SetMode(AUTOMATIC);

    #ifdef PID_TUNE
    Serial.setTimeout(1);  /*2ms timeout for reading serial port */
    #endif

    //lp_filter_init(&hil_filter);
}

/***************************************************************************************************
 * Control
 * 
 * This function should be called periodically (every 1 ms). It maintains the operation state 
 * machine for the controller and schedules the following tasks:
 * - Flex communications
 * - meter readings
 * - PID control of inverter
 * - fault monitoring
 * - mode selection
 *
 * Parameters:
 * None.
 *
 * Return:
 * None.
 *
 **************************************************************************************************/
void POWER_CTRL::Control(uint16_t sysCounter) 
{
  static controllerStateEnum_t controllerState = CONTROLLER_STATE_STOP_ENTRY;
  bool isMeterOk;
  static uint16_t startTime = 0U;
  statusBitsEnum_t inverterState;
  static statusBitsEnum_t oldInverterState = NA_1;
  bool canRxTimeout;
  bool flexFault;
  static bool inverterEnable = false;
  bool txInProgress = false;
  static bool sendOnOff = false;
  static uint16_t txDelay = 0U;
  static bool toggle0 = false;
  static bool toggle1 = false;

  #ifdef HIL_TST
   static uint16_t meterDelay = 0U;
   //isMeterOk = ReadMeter();              //shaun not needed - just for test
   isMeterOk = true;
   flexFault = false;  
   meterData.frequency = hilTestObj.GetFreq();
   meterData.totalPowerReal = hilTestObj.GetPower();
   newMeterData = true;
  #else
   isMeterOk = ReadMeter();              // Read the meter
   flexFault = flexObj.Control();        // comms with Flex controller
  #endif

  canRxTimeout = canObj.RxPoll();       // poll for received CAN messages 

  /* if controller state changes, output new state to debug port */
  inverterState = canObj.GetInverterState();
  if(oldInverterState != inverterState)
  {
    DisplayControllerState(inverterState);
    oldInverterState = inverterState;
  }
  
  /* Main state machine for operation of controller */
  switch (controllerState)
  {
    case CONTROLLER_STATE_STOP_ENTRY:
      inverterEnable = false;
      txInProgress = canObj.SetCanMode();      // Put the inverter in CAN control mode
      controllerState = CONTROLLER_STATE_STOP_DURING;
      break;

    case CONTROLLER_STATE_STOP_DURING:
      #ifdef HIL_TST
       requestedState.enable = true;
       requestedState.operatingMode = DC;
      #else
      if (false == flexFault) 
       {
         flexObj.GetOperatingState(&requestedState);
       }
      #endif

      if (true == requestedState.enable)
      {
        switch (requestedState.operatingMode)
        {
          case TRADING:
          case DC:
            #ifdef HIL_TST
             /* leave maxRated as default value */
            #else
             maxRated = flexObj.GetMaxPowerRating();
            #endif
            opModeObj.DC_Init(maxRated, sysCounter);            
          case FFR:
          case DS3:
          case PID_TEST1:
          case PID_TEST2:
            /* valid operating state received, so move to next state */
            txInProgress = canObj.InverterClrFaults();
            startTime = sysCounter;
            controllerState = CONTROLLER_STATE_INIT_ENTRY;
            Serial.println("Controller State: STOP TO INIT");
            break;

          default:
            /* Invalid operating mode - stay in stop state */
            break;
        } // switch (requestedState.operatingMode)
      } // if (true == requestedState.enable)
      else
      {
        /* no request to enable controller - stay in stop state */
      }
      // case CONTROLLER_STATE_STOP:
      break;

    case CONTROLLER_STATE_INIT_ENTRY:
      controllerState = CONTROLLER_STATE_INIT_DURING;
      break;

    case CONTROLLER_STATE_INIT_DURING:
      if((true == canRxTimeout) ||
         (false == isMeterOk))       
      {
        controllerState = CONTROLLER_STATE_STOP_ENTRY;
        Serial.println("Controller State: INIT TO STOP (1)");        
      }
      else if ((sysCounter - startTime) >= INVERTER_STARTUP_DELAY_MS)
      {
        /* inverter startup delay time has elapsed so check if it is ready */
        if(READY == inverterState) 
        {
          controllerState = CONTROLLER_STATE_RUN_ENTRY;
          Serial.println("Controller State: INIT TO RUN");          
        }
        else
        {
          controllerState = CONTROLLER_STATE_STOP_ENTRY;
          Serial.println("Controller State: INIT TO STOP (2)");
        }
      }
      break; 

    case CONTROLLER_STATE_RUN_ENTRY:
      inverterEnable = true;
      controllerState = CONTROLLER_STATE_RUN_DURING;
      break;

    case CONTROLLER_STATE_RUN_DURING:
      inverterState = canObj.GetInverterState();
      
      if((true == canRxTimeout) ||
         (false == isMeterOk)   ||
         (true == flexFault)    ||
         (FAULT == inverterState))       
      {
        controllerState = CONTROLLER_STATE_STOP_ENTRY;
        Serial.println("Controller State: RUN TO STOP");
      }
      else
      {
        #ifdef HIL_TST
         if((0 == (sysCounter % 20U)) &&
        #else
         if((true == newMeterData) && 
        #endif 
            (FOLLOWING == inverterState))
        {
          newMeterData = false;
          #ifdef PID_TUNE
           /* allow update of PID gains via serial port */
           PID_TuneParams();
          #endif

          txInProgress = ManagePower(sysCounter);                    
        } //if(true == newMeterData) 
      }     
      break;
      
    default:
      /* invalid state */
      break;
  } 

  /* control of enable/disable signal - send every 100ms, but hold off if message has just been
     transmitted by state machine */
  if(0U == (sysCounter % INVERTER_ON_OFF_SCHEDULE))
  {
    sendOnOff = true;
  }
  
  if(true == txInProgress)
  {
    txDelay = CAN_TX_DELAY_TIME;
  }

  if((false == canRxTimeout) && (true == sendOnOff) && (0U == txDelay))
  {
    (void)TxInverterOnOff(inverterEnable);
    sendOnOff = false;
  }

  if(txDelay > 0U)
  {
    txDelay--;
  }
}

/***************************************************************************************************
 * SetPowerRealSetpoint
 * 
 * This function is called to convert the newly arrived value it to a scaled value, limited
 * between -1.0 and 1.0.
 *
 * Parameters:
 * value - the value to scale.
 *
 * Return:
 * None.
 *
 **************************************************************************************************/
void POWER_CTRL::SetPowerRealSetpoint(int16_t value)
{
    pcAcObj[AC_POWER_CONTROL].setPointScaled = ScaleEngUnit(value, 
                                                                -(maxRated), 
                                                                maxRated, 
                                                                true);
}

/***************************************************************************************************
 * SetCurrentSetpoint
 * 
 * This function is called to convert the newly arrived value it to a scaled value, limited
 * between -1.0 and 1.0.
 *
 * Parameters:
 * value - the value to scale.
 *
 * Return:
 * None.
 *
 **************************************************************************************************/
void POWER_CTRL::SetCurrentSetpoint(int16_t value)
{
    pcAcObj[AC_CURRENT_CONTROL].setPointScaled = ScaleEngUnit(value, 
                                                                  -(maxRated), 
                                                                  maxRated,
                                                                  true);
}

/***************************************************************************************************
 * SetPowerRealControl
 * 
 * This function is called to convert the newly arrived value to engineering units for
 * transmission to the inverter. If input value exceeds the -1.0, 1.0 range, the resulting
 * engineering unit value is limited to its min, max value.
 *
 * Parameters:
 * controlScaled - the scaled value.
 *
 * Return:
 * None.
 *
 **************************************************************************************************/
int16_t POWER_CTRL::SetPowerRealControl(double controlScaled)
{
  int16_t powerRealControl;  
    
  powerRealControl = Unscale(controlScaled, 
                             -(maxRated), 
                             maxRated);

  return powerRealControl;
}

/***************************************************************************************************
 * SetCurrentControl
 * 
 * This function is called to convert the newly arrived value it to engineering units for
 * transmission to the inverter. If input value exceeds the -1.0, 1.0 range, the resulting
 * engineering unit value is limited to its min, max value.
 *
 * Parameters:
 * controlScaled - the scaled value.
 *
 * Return:
 * None.
 *
 **************************************************************************************************/
int16_t POWER_CTRL::SetCurrentControl(double controlScaled)
{
    int16_t currentControl;

    currentControl = Unscale(controlScaled, 
                             -(maxRated), 
                             maxRated);

    return currentControl;
}

/***************************************************************************************************
 * DemandAdjust
 * 
 * This function adjusts the demanded power into one that more accurately produces the demanded
 * power based on empiracal measurements. The adjustments are taken from a look-up table.
 *
 * Parameters:
 * Power demand.
 *
 * Return:
 * The adjusted power demand which gives an output closer to the demanded power.
 *
 **************************************************************************************************/
double POWER_CTRL::DemandAdjust(double powerDemand)
{
  uint16_t lutRowIndex = 0;  
  double rangeFraction;
  double adjustedPowerDemand;
  
  /* limit the power demand */
  if (powerDemand > (double)maxRated)
  {
    powerDemand = (double)maxRated;
  }
  else if (powerDemand < -((double)maxRated))
  {
    powerDemand = -((double)maxRated);
  }
  else
  {
    /* demanded power is within range */
  }

  while (powerDemand > CAB1000_LUT[lutRowIndex][1U])
  {
    /* Find the row in the LUT where the range starts */
    lutRowIndex++;
  }

  rangeFraction = ((powerDemand - CAB1000_LUT[lutRowIndex][1U]) / 
                   (CAB1000_LUT[lutRowIndex + 1U][1U] - CAB1000_LUT[lutRowIndex][1U]));

  /* apply the fraction from the right hand column of LUT to the left hand column and interpolate
     it, round it up/down and cast back to integer. */
  adjustedPowerDemand = (int16_t)(((CAB1000_LUT[lutRowIndex + 1U][0U] - 
                                     CAB1000_LUT[lutRowIndex][0U]) * rangeFraction) + 
                                     CAB1000_LUT[lutRowIndex][0U]);
  
  return adjustedPowerDemand;
}
