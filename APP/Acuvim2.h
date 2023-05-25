/***************************************************************************************************
 * 
 * Header for for Acuvim2.cpp
 * 
 * Date: 30/03/2023
 * 
 * Author: Shaun Mcsherry
 * 
 * ************************************************************************************************/
#ifndef ACUVIM2_H
#define ACUVIM2_H

#define DEBUG_TX Serial.println 

typedef struct ACUVIM_BASIC_MEASUREMENT_20MS
{
  double phaseVoltageA;        /* volts */
  double phaseVoltageB;        /* volts */
  double phaseVoltageC;        /* volts */
  double averagePhaseVoltage;  /* volts */
  double lineVoltageA;         /* volts */
  double lineVoltageB;         /* volts */
  double lineVoltageC;         /* volts */
  double averageLineVoltage;   /* volts */
  double phaseCurrentA;        /* amps */
  double phaseCurrentB;        /* amps */
  double phaseCurrentC;        /* amps */
  double averagePhaseCurrent;  /* amps */
  double totalPowerReal;       /* kilowatts */
  double totalPowerReactive;   /* kilowatts */
  double frequency;            /* Hertz */
}acuvimBasicMeasurement20ms_t;

class ACUVIM_II 
{
  private:
    bool AcuvimFault;
    acuvimBasicMeasurement20ms_t acuvim;
    bool ConnectEthernet(void);
    bool InitEthernet(void);
    bool InitTcpSocket(void);
    bool ConnectServer(void);
    bool ModbusClientInit(void);   
    bool BasicRequest20ms(void);
    bool BasicRead20ms(void);
    bool CheckModbus(void);

  public:
    ACUVIM_II()  //constructor
    {
      ;      
    } 
    void Init(void);
    bool Control(acuvimBasicMeasurement20ms_t *measurements);
    bool GetReadyState(void);
};

#endif */ ACUVIM2_H */
  
