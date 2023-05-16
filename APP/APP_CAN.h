/***************************************************************************************************
 * 
 * Header for for CAN.cpp
 * 
 * Date: 03/04/2023
 * 
 * Author: Shaun Mcsherry
 * 
 * ************************************************************************************************/
#ifndef APP_CAN_H
#define APP_CAN_H

#include "Controller.h"

class APP_CAN
{
  private:
    

  public:
    APP_CAN(void)
    {        
      ;//constructor
    }
    void Init(void);
    bool RxPoll(void);
    bool SetPower(int16_t realPower_kW, int16_t reactivePower_kVA);    
    bool SetCurrent(int16_t realAmps, int16_t reactiveAmps);
    bool InverterClrFaults(void);
    bool InverterEnable(void);
    bool InverterDisable(void);
    statusBitsEnum_t GetInverterState(void); 
    bool SetCanMode(void);
    bool SetManageDio(void);
};

#endif /* APP_CAN_H */
