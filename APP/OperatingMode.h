/***************************************************************************************************
 * 
 * Header for for OperatingMode.cpp
 * 
 * Date: 06/04/2023
 * 
 * Author: Shaun Mcsherry
 * 
 * ************************************************************************************************/
#ifndef OPERATING_MODE_H
#define OPERATING_MODE_H

class OP_MODE 
{
  private:
    uint16_t UpdateHeadTail(uint16_t *headIndex, uint16_t maxIndex);
    uint16_t DC_SmallDelivery (double freqDev);
    uint16_t DC_LargeDelivery (double freqDev);
    bool DC_RampPowerDemand(int16_t target, 
                            int16_t measuredPower, 
                            int16_t *newDemand, 
                            uint16_t rampTime, 
                            double rampRatePer_ms);
    int16_t DC_UpdatePowerTarget(double freqDiff);
    double DC_Test_1_1(void);
    double DC_Test_1_2(void);
    double DC_Test_1_5(void);
    double DC_Test_1_7(void);
    double DC_Test_1_9(void);
    double DC_Test_1_11(void);
    double DC_Test_1_13(void);
  public:
    OP_MODE()  //constructor
    {
      ;      
    } 
    void DC_Init(uint16_t maxPower, uint16_t systemCounter);
    int16_t DC_Control(double frequency, uint16_t sysCount);
    uint16_t FFR_Control(double frequency);
    uint16_t DS3_Control(double frequency);
    int16_t PID_TestControl1(uint16_t sysCount);
    int16_t PID_TestControl2(void);
};

#endif */ OPERATING_MODE_H */
  
