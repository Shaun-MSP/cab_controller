/***************************************************************************************************
 * HAL_Timer
 * 
 * This module is written as an interface to accessing the Timer module using non-member functions.
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
// These define's must be placed at the beginning before #include "Portenta_H7_TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#include <Arduino_MachineControl.h>
#include "Portenta_H7_TimerInterrupt.h"
#include "HAL/HAL_DIO.h"

#define LED_OFF             HIGH
#define LED_ON              LOW

#ifndef LED_BUILTIN
  #define LED_BUILTIN       24    //LEDG               // Pin 24 control on-board LED_GREEN on Portenta_H7
#endif

#ifndef LED_BLUE
  #define LED_BLUE          25    //LEDB               // Pin 25 control on-board LED_BLUE on Portenta_H7
#endif

#ifndef LED_RED
  #define LED_RED           23   // LEDR              // Pin 23 control on-board LED_RED on Portenta_H7
#endif

unsigned int SWPin = D5;

/* System interrupt time = 1ms. This is 20 times faster than PID input of 20ms. */
#define TIMER0_SYSTEM_INT_1MS     1000   /* micrsecond units */
#define TIMER0_SYSTEM_INT_1S      1000000
#define TIMER0_DURATION_MS        10000

#define TIMER1_INTERVAL_MS        0    //0 = run indefinitely
#define TIMER1_DURATION_MS        65535

// Depending on the board, you can select STM32H7 Hardware Timer from TIM1-TIM22
// If you select a Timer incorrectly, you'll get a message from compiler
// 'TIMxx' was not declared in this scope; did you mean 'TIMyy'? 

// Portenta_H7 OK       : TIM1, TIM4, TIM7, TIM8, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17
// Portenta_H7 Not OK   : TIM2, TIM3, TIM5, TIM6, TIM18, TIM19, TIM20, TIM21, TIM22
// Portenta_H7 No timer : TIM9, TIM10, TIM11. Only for STM32F2, STM32F4 and STM32L1 
// Portenta_H7 No timer : TIM18, TIM19, TIM20, TIM21, TIM22

// Init timer TIM15
Portenta_H7_Timer ITimer0(TIM15);

volatile bool systemInterruptFlag = false;

/***************************************************************************************************
 * TIM_SystemInterrupt_1ms
 * 
 * This interrupt service routine is the main system interrupt, driven by timer0. It is called
 * periodically (every 1ms) to perform the system foreground tasks.
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
void TIM_SystemInterrupt_1ms()
{
  systemInterruptFlag = true;
}

/***************************************************************************************************
 * TIM_Init
 * 
 * This function initialises the timers.
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
void TIM_Init(void)
{
  // Interval in microsecs
  if (ITimer0.attachInterruptInterval(TIMER0_SYSTEM_INT_1MS, TIM_SystemInterrupt_1ms))
  {
    Serial.println("Starting ITimer0 OK"); 
  }
  else
  {
    Serial.println("Can't set ITimer0");
  }
}

bool TIM_GetSysInterruptFlag(void)
{
  bool retVal;

  retVal = systemInterruptFlag;
  systemInterruptFlag = false;

  return retVal;
}