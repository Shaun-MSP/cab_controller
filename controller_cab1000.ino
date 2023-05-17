/***************************************************************************************************
 * controller_cab1000_ino
 *
 * Description:
 * This module contains the calls to the applications setup functions and the main loop.
 *
 * Date:
 * 21/02/2023
 * 
 * Author:
 * Shaun McSherry
 *
 **************************************************************************************************/
#include <Arduino_MachineControl.h>
#include "HAL/HAL_Timer.h"
#include "HAL/HAL_DIO.h"
#include "APP/Debug.h"
#include "APP/PowerControl.h"

using namespace machinecontrol;

uint16_t sysCounter = 0U;
bool flexConnectedFlag = false;

POWER_CTRL powerControlObj;

void setup() 
{
  // put your setup code here, to run once:
  Debug_Setup();
  powerControlObj.Init();
  TIM_Init();
  //DIO_Init();
  //TCP_ModbusServerStart();
  digital_outputs.setLatch();
  digital_outputs.setAll(0);
  /* These are configured for analogue outputs */
  analog_out.period_ms(0, 1);
  analog_out.period_ms(1, 1);
  analog_out.period_ms(2, 1);
  analog_out.period_ms(3, 1);
}

void loop() 
{

  if(true == TIM_GetSysInterruptFlag())
  {
    sysCounter++;

    /* This is the main controller routine */
    powerControlObj.Control(sysCounter);    
  }
}
