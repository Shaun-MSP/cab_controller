/***************************************************************************************************
 * 
 * Header for for Debug_MID.cpp
 * 
 * Date: 21/02/2023
 * 
 * Author: Shaun Mcsherry
 * 
 * ************************************************************************************************/
#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H


#include <PID_v1.h>
#include "Controller.h"
//#include "../UTILS/PID.h"

/* Power demand limits for CAB1000 */
#define CAB1000_MAX_REAL_P_KW          1000
#define CAB1000_MIN_REAL_P_KW          -1000
#define CAB1000_MAX_REACTIVE_P_KVA     1000
#define CAB1000_MIN_REACTIVE_P_KVA     -1000

/* Current demand limits for CAB1000 */
#define CAB1000_MAX_REAL_I_AMPS        1000
#define CAB1000_MIN_REAL_I_AMPS        -1000
#define CAB1000_MAX_REACTIVE_I_AMPS    1000
#define CAB1000_MIN_REACTIVE_I_AMPS    -1000

typedef enum PC_AC_OBJ_NAME_ENUM
{
    AC_POWER_CONTROL      = 0,
    AC_CURRENT_CONTROL    = 1,
    NOOF_PC_AC_OBJECTS    = 2
}pcAcObjNameEnum_t;

typedef enum PC_AC_MODE_ENUM
{
    AC_POWER_CONTROL_MODE = 0,
    AC_CURRENT_CONTROL_MODE = 1
}pcAcModeEnum_t;

/* Each object to control, i.e. current and power, has a real and reactive element */
//typedef struct PC_AC_OBJ_STRUCT_T
//{
//  pidParamStruct_t real;
//}pcAcObjStruct_t;
typedef struct PC_AC_OBJ_STRUCT_T
{
  double pGain;
  double iGain;
  double dGain;  
  double setPointScaled;    
  double measuredScaled;    
  double pidOutput;  
}pcAcObjStruct_t;

class POWER_CTRL
{
  private:
    pcAcModeEnum_t mode;

    double p_realPowerGain;
    double i_realPowerGain;
    double d_realPowerGain;

    double p_currentGain;
    double i_currentGain;
    double d_currentGain;

    void GetStoredParams(void);
    inline double ScaleEngUnit(int16_t value, int16_t min, int16_t max, bool limit);
    inline int16_t Unscale(double value, int16_t min, int16_t max);
    bool ManagePower(uint16_t sysCount);
    bool ReadMeter(void);
    bool TxInverterOnOff(bool inverterEnable);
    void PidParamsAnaOut(double setpoint, double measuredValue, double pidOut);
    double ScaleAnalogue(double value);
    void PID_TuneParams(void);
  public:
    POWER_CTRL() //constructor
    {
      /* Set default values of power control object parameters (these will be overwritten
      by any values stored in NVR)*/
      p_realPowerGain = 0.7;
      i_realPowerGain = 10.0;
      d_realPowerGain = 0.0;

      p_currentGain = 0.5;
      i_currentGain = 0.2;
      d_currentGain = 0.0;

      /* Comment out as necessary */
      mode = AC_POWER_CONTROL_MODE;
      //static pcAcModeEnum_t mode = AC_CURRENT_CONTROL_MODE;
    }
  
    void Init(void);
    void Control(uint16_t sysCounter);
    void SetPowerRealSetpoint(int16_t value);
    void SetCurrentSetpoint(int16_t value);
    int16_t SetPowerRealControl(double controlScaled);
    int16_t SetCurrentControl(double controlScaled);
    int16_t GetPowerRealControl(void);
    int16_t GetCurrentControl(void);
    double DemandAdjust(double powerDemand);
    void DisplayControllerState(statusBitsEnum_t state);
};

#endif /* POWER_CONTROL_H */
  
