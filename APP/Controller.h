/***************************************************************************************************
 * 
 * Header containing general definitions and config for controller
 * 
 * Date: 04/04/2023
 * 
 * Author: Shaun Mcsherry
 * 
 * ************************************************************************************************/
#ifndef CONTROLLER_H
#define CONTROLLER_H

#define CAB1000_MAX_DEMAND    1542
#define CAB1000_MIN_DEMAND    -1462
#define LUT_MAX_INDEX         32

/* general definitions */
#define INT16_MAX     32767
#define INT16_MIN     -32768 
#define UINT16_MAX    65535U

/* The CAN timeout period in ms */
#define CAN_TIMEOUT_MS    300U

/* The time allowed for the inverter to be READY after enable signal is sent */
#define INVERTER_STARTUP_DELAY_MS    10000U

/* define the firmware loaded on the CAB1000 controller */
//#define CAB1000_FW_3C625C9
#define CAB1000_FW_6DE948B

#define HIL_TST

#define PID_TUNE

/* Define only one of the grid voltages below */
//#define GRID_VOLTAGE_480_RMS
//#define GRID_VOLTAGE_600_RMS
//#define GRID_VOLTAGE_630_RMS
//#define GRID_VOLTAGE_660_RMS
#define GRID_VOLTAGE_690_RMS

/* inverter status */
typedef enum STATE_BITS_ENUM
{
  POWER_ON_RESET    = 0,
  READY             = 1,
  FOLLOWING         = 2,
  FAULT             = 3,
  FORMING           = 4,
  RECONNECT_DELAY   = 5,
  NA_1              = 6,
  GRID_LOST         = 7,
  CHARGING          = 8,
  RIDE_THROUGH      = 9,
  CESSATION         = 10,
  TRANSITIONING     = 11,
  INHIBITED         = 12,
  CONNECT_DELAY     = 13,
  NA_2              = 14,
  NA_3              = 15
}statusBitsEnum_t;

#endif /* CONTROLLER_H */
  
