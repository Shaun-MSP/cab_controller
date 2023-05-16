/***************************************************************************************************
 * 
 * Header for for Timer.cpp
 * 
 * Date: 27/02/2023
 * 
 * Author: Shaun Mcsherry
 * 
 * ************************************************************************************************/
#ifndef HAL_TIMER_H
#define HAL_TIMER_H

#define HALF_SECOND_MS      500UL
#define TWO_SECONDS_MS      2000UL
#define TEN_SECONDS_MS      10000UL
#define FIFTEEN_SECONDS_MS  15000UL
#define THIRTY_SECONDS_MS   30000UL

extern void TIM_Init(void);
bool TIM_GetSysInterruptFlag(void);

#endif /* HAL_TIMER_H */
  
