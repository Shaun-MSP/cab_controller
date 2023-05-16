/***************************************************************************************************
 * 
 * Header for for Flex.cpp
 * 
 * Date: 04/04/2023
 * 
 * Author: Shaun Mcsherry
 * 
 * ************************************************************************************************/
#ifndef FLEX_H
#define FLEX_H

typedef enum FLEX_CONTROL_MODE_ENUM
{
  TRADING = 2,
  DC      = 4,
  FFR     = 8,
  DS3     = 16,
  PID_TEST1 = 32,
  PID_TEST2 = 33
}flexControlModeEnum_t;

typedef struct FLEX_OPERATING_STATE_STRUCT
{
  bool enable;
  flexControlModeEnum_t operatingMode;
}flexOperatingStateStruct_t;

class FLEX
{
  private:
    void SendHeartbeat(uint16_t counter);
    uint16_t GetHeartbeat(void);

  public:
    FLEX(void)
    {
      ; //constructor 
    }
    
    void Init(void);
    bool Control(void);
    void GetOperatingState(flexOperatingStateStruct_t *operatingState);
    uint16_t GetDemand(void);
    uint16_t GetMaxPowerRating(void);
    uint16_t GetMaxOnlineCapacity(void);
    void PowerMeasured(uint16_t powerMeasured);
};

#endif /* FLEX_H */
  
