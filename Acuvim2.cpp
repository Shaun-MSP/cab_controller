/***************************************************************************************************
 * Acuvim2
 * 
 * This module reads the Acuvim meter.
 *
 *
 * Date:
 * 28/03/2023
 *
 * Author:
 * Shaun Mcsherry
 *
 **************************************************************************************************/
#include <Arduino_MachineControl.h>
//#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include <SPI.h>
#include <Ethernet.h>
#include "APP/Acuvim2.h"
#include "HAL/HAL_Timer.h"

#define ACUVIM_TIMEOUT            FIFTEEN_SECONDS_MS
#define ACUVIM_RESPONSE_TIMEOUT   TEN_SECONDS_MS

/* uncomment for additional debug output */
//#define DEBUG_ACUVIEW

#define NOOF_BASIC_REGS_20MS              30
#define ACCUVIM_MB_ID                     1

#define ACUVIM_MB_FREQ_ADDR               0x3400 /* TBD */
#define ACUVIM_MB_REAL_POWER_TOTAL_ADDR   0x3400 /* TBD */
#define ACUVIM_MB_BASIC_20MS_ADDR         0x3400

typedef enum ACUVIM_FAULT
{
  ACU_FAULT_OK                = 0,
  ACU_FAULT_NO_ETH_CONNECTION = 1
}acuvimFault_t;

typedef enum READ_STATE_ENUM
{
  ACUVIM_INIT               = 0,
  ACUVIM_READ_IDLE          = 1,
  ACUVIM_READ_IN_PROGRESS   = 2
}acuvimReadState_t;

using namespace machinecontrol;

const char SERVER_IP_ADDRESS[] = "192.168.30.10";
const char CLIENT_IP_ADDRESS[] = "192.168.30.5";
const char SUBNET_MASK[] = "255.255.255.0";
const char DEFAULT_GW[] = "192.168.30.5";

EthernetInterface net;
EthernetServer server;
TCPSocket localSocket;
TCPSocket *serverSocketPtr;
ModbusTCPClient modbusTCPClient;

bool AcuvimFault = false;
acuvimBasicMeasurement20ms_t acuvim;

/* private functions */
/***************************************************************************************************
 * ConnectEthernet
 * 
 * This function is called during initialisation to attempt AcuVim connection over ethernet.
 *
 * Parameters:
 * None
 *
 * Return:
 * true if modbus connected, otherwise false
 *
 **************************************************************************************************/
bool ACUVIM_II::ConnectEthernet(void)
{
  bool isConnected = true;
  nsapi_error_t networkError;
  nsapi_connection_status_t networkStat;

  networkStat = net.get_connection_status();  
  
  if(NSAPI_STATUS_DISCONNECTED == networkStat)
  {
    //setup static ip address
    net.set_network(SERVER_IP_ADDRESS, SUBNET_MASK, DEFAULT_GW);
    
    /* Bring up the ethernet interface */
    networkError = net.connect(); 
  }
  
  networkStat = net.get_connection_status();
    
  //while((networkStat != NSAPI_STATUS_GLOBAL_UP) && (elapsedTime < 10000));   
  if ((networkStat != NSAPI_STATUS_GLOBAL_UP) || (networkError != NSAPI_ERROR_OK))
  {
    isConnected = false;
  }

  return isConnected;

}      

/***************************************************************************************************
 * CheckModbus
 * 
 * This function is called before each transaction to verify whether Modbus is connected.
 *
 * If not, it attempts to connect.
 *
 * Parameters:
 * None
 *
 * Return:
 * true if modbus connected, otherwise false
 *
 **************************************************************************************************/
bool ACUVIM_II::CheckModbus(void)
{
  bool isConnected = true;

  if (!modbusTCPClient.connected()) 
  {
    // client not connected, start the Modbus TCP client
    Serial.println("Attempting to connect to Modbus TCP server");

      if (!modbusTCPClient.begin(acuvimServerIp, 502)) 
      {
        Serial.println("Modbus TCP Client failed to connect!");
        isConnected = false;
      } 
      else 
      {
        Serial.println("Modbus TCP Client connected");
        isConnected = true;
      }
  }      
}

/***************************************************************************************************
 * BasicRead20ms
 * 
 * This function is called to read the requested basic measurements.
 *
 * It first checks all the data has been received and then copies it into the data structure.
 *
 * Parameters:
 * None
 *
 * Return:
 * None
 *
 **************************************************************************************************/
bool ACUVIM_II::BasicRead20ms(void)
{
  int noofValues;
  int16_t valueArray[NOOF_BASIC_REGS_20MS];
  uint8_t loopCount;
  bool isNewData = false;

  if(true == CheckModbus())
  { 
    /* Check to see how many values received */
    noofValues = modbusTCPClient.available();

    if (NOOF_BASIC_REGS_20MS == noofValues)   // only proceed if all expected values
    {                                         // have been received.
      for(loopCount = 0; loopCount < NOOF_BASIC_REGS_20MS; loopCount++)
      {
        valueArray[loopCount] = modbusTCPClient.read();
      }

      /* update measurement structure */
      acuvim.frequency            = (double)((uint32_t)valueArray[0] + 
                                              (uint32_t)(valueArray[1] << 16U));

      acuvim.phaseVoltageA        = (double)((uint32_t)valueArray[2] + 
                                              (uint32_t)(valueArray[3] << 16U));

      acuvim.phaseVoltageB        = (double)((uint32_t)valueArray[4] + 
                                              (uint32_t)(valueArray[5] << 16U));

      acuvim.phaseVoltageC        = (double)((uint32_t)valueArray[6] + 
                                              (uint32_t)(valueArray[7] << 16U));

      acuvim.averagePhaseVoltage  = (double)((uint32_t)valueArray[8] + 
                                              (uint32_t)(valueArray[9] << 16U));

      acuvim.lineVoltageA         = (double)((uint32_t)valueArray[10] + 
                                              (uint32_t)(valueArray[11] << 16U));

      acuvim.lineVoltageB         = (double)((uint32_t)valueArray[12] + 
                                              (uint32_t)(valueArray[13] << 16U));

      acuvim.lineVoltageC         = (double)((uint32_t)valueArray[14] + 
                                              (uint32_t)(valueArray[15] << 16U));

      acuvim.averageLineVoltage   = (double)((uint32_t)valueArray[16] + 
                                              (uint32_t)(valueArray[17] << 16U));

      acuvim.phaseCurrentA        = (double)((uint32_t)valueArray[18] + 
                                              (uint32_t)(valueArray[19] << 16U));

      acuvim.phaseCurrentB        = (double)((uint32_t)valueArray[20] + 
                                              (uint32_t)(valueArray[21] << 16U));

      acuvim.phaseCurrentC        = (double)((uint32_t)valueArray[22] + 
                                              (uint32_t)(valueArray[23] << 16U));

      acuvim.averagePhaseCurrent  = (double)((uint32_t)valueArray[24] + 
                                              (uint32_t)(valueArray[25] << 16U));

      acuvim.totalPowerReal       = (double)((uint32_t)valueArray[26] + 
                                              (uint32_t)(valueArray[27] << 16U));
                                              
      acuvim.totalPowerReactive   = (double)((uint32_t)valueArray[28] + 
                                              (uint32_t)(valueArray[29] << 16U));

      isNewData = true;                                                
    }/* if (NOOF_BASIC_REGS_20MS == noofValues) */
  }/* if(true == APP_AcuvimCheck()) */
  
  return isNewData;
}

/****************************************************************************************************
 * BasicRequest20ms
 * 
 * This function is called to request the basic measurements that can be returned within 20ms:
 * - phase voltage (A, B, C)
 * - Average phase voltage
 * - line voltage (A-B, B-C, C-A)
 * - Average line voltage
 * - phase current (A, B, C)
 * - average phase current
 * - total active/reactive power 
 * - frequency
 *
 * Parameters:
 * None
 *
 * Return:
 * None
 *
 **************************************************************************************************/
void ACUVIM_II::BasicRequest20ms(void)
{
  if(true == CheckModbus())
  {      
    if (!modbusTCPClient.requestFrom(HOLDING_REGISTERS, 
                                    ACUVIM_MB_BASIC_20MS_ADDR, 
                                    NOOF_BASIC_REGS_20MS)) 
    {
      Serial.print("Failed to send Acuview read request: ");
      Serial.println(modbusTCPClient.lastError());
    }
  }
}

/* End private functions */

/* Public functions */
/***************************************************************************************************
 * Init
 * 
 * This function configures the controller as a Modbus TCP client to read the meter
 *
 * Parameters:
 * None
 *
 * Return:
 * None
 *
 **************************************************************************************************/
void ACUVIM_II::Init(void)
{
  /* Enter a MAC address for your controller below.
      Newer Ethernet shields have a MAC address printed on a sticker on the shield */
  uint8_t mac[] = 
  {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  }; 

  Serial.println("Initialising AcuVim II");

  /* disable blocking whilst waiting for network */
  net.set_blocking(false);

  /* disable DHCP - use static IP address */
  net.set_dhcp(false);

  /* Get MAC address of ethernet shield */
  Ethernet.MACAddress(mac);

  /* start the Ethernet connection and the server: */
  Ethernet.begin(mac, acuvimClientIp, 
                      acuvimClientDns, 
                      acuvimClientGateway, 
                      acuvimClientSubnet, ACUVIM_TIMEOUT, ACUVIM_RESPONSE_TIMEOUT);
  //Ethernet.begin(mac, acuvimClient);
  /* Check for Ethernet hardware present */
  if (EthernetNoHardware == Ethernet.hardwareStatus()) 
  {
    Serial.println("Ethernet shield not found");
    AcuvimFault = ACU_FAULT_NO_ETH_SHIELD;
  }
  else if (Ethernet.linkStatus() == LinkOFF) 
  {
    Serial.println("Accuvim ethernet cable is not connected.");
    AcuvimFault = ACU_FAULT_NO_ETH_CONNECTION;
  }
  else
  {
    AcuvimFault = ACU_FAULT_SHIELD_OK;
  }
}

/***********************************************************************************************
 * Control
 * 
 * This is the main control state machine for reading and retrieving measurement data from the
 * meter.
 *
 * Parameters:
 * measurements - pointer to measurement structure.
 *
 * Return:
 * true if new set of measurements available, otherwise false.
 *
 **********************************************************************************************/
bool ACUVIM_II::Control(acuvimBasicMeasurement20ms_t *measurements)
{
  bool isNewData = false;
  static acuvimReadState_t readState = ACUVIM_READ_IDLE;    

  if (ACU_FAULT_SHIELD_OK == AcuvimFault)
  {
    switch (readState)
    {
      case ACUVIM_READ_IDLE:
        BasicRequest20ms();                     // initiate new read
        readState = ACUVIM_READ_IN_PROGRESS;
        break;

      case ACUVIM_READ_IN_PROGRESS:           // poll for data
        isNewData = BasicRead20ms();
        if (true == isNewData)
        {
          *measurements = acuvim;              // transfer read data to pointer
          readState = ACUVIM_READ_IDLE;
        }
        break;

      default:
        /* invalid state, so reset */
        readState = ACUVIM_READ_IDLE;
        break;
    } /* switch (readState) */
  } /* if (ACU_FAULT_SHIELD_OK == AcuvimFault) */

  return isNewData;
}

/***********************************************************************************************
 * GetFaultState
 * 
 * This function is calles to determine whether there is a fault on the meter preventing it
 * from providing measurements.
 *
 * Parameters:
 * None.
 *
 * Return:
 * true if fault present, otherwise false.
 *
 **********************************************************************************************/
bool ACUVIM_II::GetFaultState(void)
{
  bool fault = false;

  if (AcuvimFault != ACU_FAULT_SHIELD_OK)
  {
    fault = true;
  }

  return fault;  
}


/* end public functions */
