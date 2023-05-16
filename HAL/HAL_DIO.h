/***************************************************************************************************
 * 
 * Header for for HAL_DIO.cpp
 * 
 * Date: 27/02/2023
 * 
 * Author: Shaun Mcsherry
 * 
 * ************************************************************************************************/
#ifndef HAL_DIO_H
#define HAL_DIO_H

typedef enum OUT_PIN_ID
{
  OUT_PIN_0 = 0,
  OUT_PIN_1 = 1,
  OUT_PIN_2 = 2,
  OUT_PIN_3 = 3,
  OUT_PIN_4 = 4,
  OUT_PIN_5 = 5,
  OUT_PIN_6 = 6,
  OUT_PIN_7 = 7
}outPinId_enum_t;

typedef enum IN_PIN_ID
{
  IN_PIN_0 = 0,
  IN_PIN_1 = 1,
  IN_PIN_2 = 2,
  IN_PIN_3 = 3,
  IN_PIN_4 = 4,
  IN_PIN_5 = 5,
  IN_PIN_6 = 6,
  IN_PIN_7 = 7
}inPinId_enum_t;

typedef enum PIN_STATE_ENUM
{
  PIN_LOW  = 0,
  PIN_HIGH = 1
}pinStateEnum_t;

extern void DIO_Init(void);

#endif /* HAL_DIO_H */
  
