/***************************************************************************************************
 * HAL_CAN
 * 
 * This module is written as an interface to accessing the CAN module using non-member functions.
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
#include <Arduino_MachineControl.h>
#include "APP/APP_CAN.h"
#include "APP/Controller.h"

using namespace machinecontrol;
#include <CAN.h>

#define DATARATE_1MB     1000000
#define DATARATE_500K    500000

#define STATUS_MSG_HANDLE  0x100U

#if defined CAB1000_FW_3C625C9
  /********** CAN Message IDs ******************/
  #define MID_PROCESS_TO_INVERTER   0x0CEFF741U  
  #define MID_STATUS                0x0CFFC3F7U
  #define MID_PARAMETER_QUERY       0x1DEFF741U
/***********************************************/  
#elif defined CAB1000_FW_6DE948B
 /********** CAN Message IDs ******************/
  #define MID_PROCESS_TO_INVERTER   0x0CEF0141U  
  #define MID_STATUS                0x0CFFC301U
  #define MID_PARAMETER_QUERY       0x1DEF0141U
/***********************************************/  
#endif 
  typedef struct PARAMETER_QUERY_MODE13_STRUCT
  {
    uint16_t mode             : 11U;   // bits 10:0
    uint16_t meta             : 3U;    // bits 13:11
    uint16_t readParamCommand : 2U;    // bits 15:14
    uint16_t stopBits         : 4U;    // bits 19:16
    uint16_t controlSource    : 2U;    // bits 21:20
    uint16_t parity           : 2U;    // bits 23:22
    uint16_t dropNum          : 8U;    // bits 31:24
    uint32_t monitorTimeout;           // bits 63:32
  }parameterQueryMode13_struct;

  typedef struct PARAMETER_QUERY_MODE20_STRUCT
  {
    uint16_t mode             : 11U;   // bits 10:0
    uint16_t meta             : 3U;    // bits 13:11
    uint16_t readParamCommand : 2U;    // bits 15:14
    uint16_t invertDI1        : 2U;    // bits 17:16
    uint16_t invertDI2        : 2U;    // bits 19:18
    uint16_t invertDI3        : 2U;    // bits 21:20
    uint16_t invertDI4        : 2U;    // bits 23:22
    uint16_t invertDO1        : 2U;    // bits 25:24
    uint16_t invertDO2        : 2U;    // bits 27:26
    uint16_t invertDO3        : 2U;    // bits 29:28
    uint16_t invertDO4        : 2U;    // bits 31:30
    uint16_t forceRelayK2DcRun: 2U;    // bits 33:32
    uint16_t forceRelayK1PreCh: 2U;    // bits 35:34
    uint16_t forceRelayMx2    : 2U;    // bits 37:36
    uint16_t forceRelayMx1    : 2U;    // bits 39:38
    uint16_t do4Controller    : 2U;    // bits 41:40
    uint16_t do3Controller    : 2U;    // bits 43:42
    uint16_t do2Controller    : 2U;    // bits 45:44
    uint16_t do1Controller    : 2U;    // bits 47:46
    uint16_t do4Command       : 2U;    // bits 49:48
    uint16_t do3Command       : 2U;    // bits 51:50
    uint16_t do2Command       : 2U;    // bits 53:52
    uint16_t do1Command       : 2U;    // bits 55:54
    uint16_t DI1Function      : 2U;    // bits 57:56
    uint16_t unused1          : 4U;    // bits 61:58
    uint16_t inverHwEnable    : 2U;    // bits 63:62
  }parameterQueryMode20_struct;

  typedef union PARAMETER_QUERY_MODE13_UNION
  {
    parameterQueryMode13_struct dataMode13;
    parameterQueryMode20_struct dataMode20;
    unsigned char byte[8];
  }parameterQuery_union_t;

  typedef struct PROCESS_TO_INVERTER_MODE1_STRUCT
  {
    /* byte 0 */
    uint16_t mode            : 8U;   // bits 7:0
    /* byte 1 */
    uint16_t enable          : 2U;   // bits 9:8
    uint16_t clearFault      : 2U;   // bits 11:10
    uint16_t clearWarning    : 2U;   // bits 13:12
    uint16_t islandReconnect : 2U;   // bits 15:14
    /* byte 2 */
    uint16_t protBusSequence : 4U;   // bits 19:16
    uint16_t byte2_unused    : 4U;   // bits 23:20
    /* byte 3 */
    uint16_t byte3_unused;           // bits 31:24
    /* byte 4 */
    uint16_t byte4_unused;           // bits 39:32
    /* byte 5 */
    uint16_t byte5_unused;           // bits 47:40
    /* byte 6 */
    uint16_t byte6_unused;           // bits 55:48
    /* byte 7 */
    uint16_t byte7_unused;           // bits 63:56
  }processToInverterMode1Struct_t;

  typedef struct PROCESS_TO_INVERTER_MODE2_STRUCT
  {
    /* byte 0 */
    uint8_t mode;                   // bits 7:0
    /* byte 1 */
    uint8_t byte1_unused;           // bits 15:8
    /* byte 2 & byte 3 */
    uint16_t realPowerDemand;       // bits 31:16
    /* byte 4 & 5 */
    uint16_t reactivePowerDemand;   // bits 47:32
    /* byte 6 */
    uint8_t byte6_unused;           // bits 55:48
    /* byte 7 */
    uint8_t byte7_unused;           // bits 63:56
  }processToInverterMode2Struct_t;

  typedef struct PROCESS_TO_INVERTER_MODE3_STRUCT
  {
    /* byte 0 */
    uint8_t mode;                   // bits 7:0
    /* byte 1 */
    uint8_t byte1_unused;           // bits 15:8
    /* byte 2 & byte 3 */
    uint16_t realCurrentDemand;     // bits 31:16
    /* byte 4 & 5 */
    uint16_t reactiveCurrentDemand; // bits 47:32
    /* byte 6 */
    uint8_t byte6_unused;           // bits 55:48
    /* byte 7 */
    uint8_t byte7_unused;           // bits 63:56
  }processToInverterMode3Struct_t;

/* this union contains holders for each of the used message modes of the
  process to inverter messages */
typedef union PROCESS_TO_INVERTER_UNION
{
  processToInverterMode1Struct_t processToInverterMode1;
  processToInverterMode2Struct_t processToInverterMode2;
  processToInverterMode3Struct_t processToInverterMode3; 
  unsigned char data[8];
}processToInverterUnion_t;

  typedef struct STATUS_STRUCT
  {
    /* byte 0 */
    uint8_t state                   : 4U;
    uint8_t protBusSequence         : 4U;
    /* byte 1 */
    uint8_t islandReconnect_echo    : 2U;
    uint8_t warningClr_echo         : 2U;
    uint8_t faultClr_echo           : 2U;
    uint8_t enable_echo             : 2U;
    /* byte 2 */
    uint8_t warning                 : 2U;
    uint8_t hardwareEnable          : 2U;
    uint8_t powerAvailDC            : 2U;
    uint8_t powerCircuitEnabled     : 2U;
    /* byte 3 */
    uint8_t K2DCRunPermissive       : 2U;
    uint8_t K1PrechargePermissive   : 2U;
    uint8_t MX2Permissive           : 2U;
    uint8_t MX1Permissive           : 2U;
    /* byte 4 */
    uint8_t DI4                     : 2U;
    uint8_t DI3                     : 2U;
    uint8_t DI2                     : 2U;
    uint8_t DI1                     : 2U;
    /* byte 5 */
    uint8_t PumpFault               : 2U;
    uint8_t PumpRun                 : 2U;
    uint8_t MessageValidModeControl : 2U;
    uint8_t MessageValidPowerCMD    : 2U;
    /* byte 6 */
    uint8_t MessageValidCurrentCMD  : 2U;
    uint8_t MessageValidDcControl   : 2U;
    uint8_t unused_byte6            : 4U;
    /* byte 7 */
    uint8_t unused_byte7;
  }statusMsgDataStruct_t;

  typedef union STATUS_UNION
  {
    statusMsgDataStruct_t data;
    unsigned char byte[8];
  }statusMsgDataUnion_t;

static statusMsgDataUnion_t statusMsgRx;

unsigned char tempData[] = {0U,0U,0U,0U,0U,0U,0U,0U};

mbed::CANMessage canProcessToInverter(MID_PROCESS_TO_INVERTER, tempData, 8, CANData, CANExtended);
mbed::CANMessage canParameterQuery(MID_PARAMETER_QUERY, tempData, 8, CANData, CANExtended);


uint16_t statRxHandle = 0;

/* Private functions */

/* Public functions */
/***************************************************************************************************
 * RxPoll
 * 
 * This is the polled monitor routine for received CAN messages. 
 *
 * It should be called every 1ms as it provides indication of a CAN rx timeout.
 * 
 * Upon receipt of a message, if the message is valid, it copies it into the data relevant 
 * structure.
 * 
 * Parameters:
 * None.
 *
 * Return:
 * true no new message received within timeout period, otherwise false
 *
 **************************************************************************************************/
bool APP_CAN::RxPoll(void)
{ 
  mbed::CANMessage rxMsg;
  uint16_t newMsg;
  static uint16_t timeoutCounter = 0U;
  bool canTimedOut = false;
  uint16_t index;
  static bool statusMsgRxed = false;

  newMsg = comm_protocols.can.read(rxMsg, statRxHandle);
  
  if (1 == newMsg)
  {
    switch(rxMsg.id)
    {
      case MID_STATUS:
        statusMsgRxed = true;
        /* status message received - copy all data across */
        for(index = 0U; index < 8U; index++)
        {
          statusMsgRx.byte[index] = rxMsg.data[index];
        }
        
        break;
  
      default:
        //Serial.println(rxMsg.id);
      break;
    }
    if (comm_protocols.can.rderror() > 0)
    {
      Serial.print ("RxErr: ");
      Serial.println(comm_protocols.can.rderror());
    }
    if (comm_protocols.can.tderror() > 0)
    {
      Serial.print ("TxErr: ");
      Serial.println(comm_protocols.can.tderror());
    }
  }
  
  if(true == statusMsgRxed)
  {
    statusMsgRxed = false;
    timeoutCounter = 0U;  // new message, so reset timeout count
  }
  else
  {
    /* no new message yet */
    if(timeoutCounter < CAN_TIMEOUT_MS)
    {
      timeoutCounter++;
    }
    else
    {
      /* no new message within timeout period - so set timeout flag */
      canTimedOut = true;
    }
  }
  
  return canTimedOut;
}

/***************************************************************************************************
 * Init
 * 
 * Initialises the CANBus datarate.
 * Initialises the inverter data structures.
 * Initialises the CAN interrupt for received CAN messages
 *
 **************************************************************************************************/
void APP_CAN::Init(void)
{
  Serial.println("Starting CAN initialisation");
  comm_protocols.enableCAN();
  comm_protocols.can.frequency(DATARATE_500K);

  statRxHandle = comm_protocols.can.filter(MID_STATUS, 0x1FFFFFFFU, CANExtended, STATUS_MSG_HANDLE);
  Serial.print("Status msg handle: ");
  Serial.println(statRxHandle);
  Serial.println("CAN Initialisation done");
}


/***************************************************************************************************
 * InverterClrFaults
 * 
 * Transmits the CAN message to clear inverter faults.
 *
 * Params:
 * None
 *
 * Return:
 * true to indicate to calling function message has been sent
 *
 **************************************************************************************************/
bool APP_CAN::InverterClrFaults(void)
{
  processToInverterUnion_t canData;
  uint16_t index;

  canData.processToInverterMode1.clearFault = 1;
  canData.processToInverterMode1.enable = 0;
  canData.processToInverterMode1.clearWarning = 0;
  canData.processToInverterMode1.islandReconnect = 0; // normal
  canData.processToInverterMode1.protBusSequence = 0; // n/a
  canData.processToInverterMode1.mode = 1;            // command mode control
  canData.processToInverterMode1.byte2_unused = 0;
  canData.processToInverterMode1.byte3_unused = 0;
  canData.processToInverterMode1.byte4_unused = 0;
  canData.processToInverterMode1.byte5_unused = 0;
  canData.processToInverterMode1.byte6_unused = 0;
  canData.processToInverterMode1.byte7_unused = 0;  

  /* populate the data into the CAN message */
  for(index = 0; index < 8; index++)
  {
    canProcessToInverter.data[index] = canData.data[index];
  }

  /* transmit the CAN message */
  comm_protocols.can.write(canProcessToInverter);

  return true;
}
    
/***************************************************************************************************
 * InverterEnable
 * 
 * Transmits the CAN message to enable the inverter.
 *
 * Params:
 * None
 *
 * Return:
 * true to indicate to calling function message has been sent
 **************************************************************************************************/
bool APP_CAN::InverterEnable(void)
{
  processToInverterUnion_t canData;
  uint16_t index;

  canData.processToInverterMode1.clearFault = 0;
  canData.processToInverterMode1.enable = 1;
  canData.processToInverterMode1.clearWarning = 0;
  canData.processToInverterMode1.islandReconnect = 0; // normal
  canData.processToInverterMode1.protBusSequence = 0; // n/a
  canData.processToInverterMode1.mode = 1;            // command mode control
  canData.processToInverterMode1.byte2_unused = 0;
  canData.processToInverterMode1.byte3_unused = 0;
  canData.processToInverterMode1.byte4_unused = 0;
  canData.processToInverterMode1.byte5_unused = 0;
  canData.processToInverterMode1.byte6_unused = 0;
  canData.processToInverterMode1.byte7_unused = 0;

  /* populate the data into the CAN message */
  for(index = 0; index < 8; index++)
  {
    canProcessToInverter.data[index] = canData.data[index];
  }

  /* transmit the CAN message */
  comm_protocols.can.write(canProcessToInverter);

  return true;
}

/***************************************************************************************************
 * InverterDisable
 * 
 * Transmits the CAN message to enable the inverter.
 *
 * Params:
 * None
 *
 * Return:
 * true to indicate to calling function message has been sent
 *
 **************************************************************************************************/
bool APP_CAN::InverterDisable(void)
{
  processToInverterUnion_t canData;
  uint16_t index;

  canData.processToInverterMode1.clearFault = 0;
  canData.processToInverterMode1.enable = 0;
  canData.processToInverterMode1.clearWarning = 0;
  canData.processToInverterMode1.islandReconnect = 0; // normal
  canData.processToInverterMode1.protBusSequence = 0; // n/a
  canData.processToInverterMode1.mode = 1;            // command mode control
  canData.processToInverterMode1.byte2_unused = 0;
  canData.processToInverterMode1.byte3_unused = 0;
  canData.processToInverterMode1.byte4_unused = 0;
  canData.processToInverterMode1.byte5_unused = 0;
  canData.processToInverterMode1.byte6_unused = 0;
  canData.processToInverterMode1.byte7_unused = 0;

  /* populate the data into the CAN message */
  for(index = 0; index < 8; index++)
  {
    canProcessToInverter.data[index] = canData.data[index];
  }

  /* transmit the CAN message */
  comm_protocols.can.write(canProcessToInverter);

  return true;
}

/***************************************************************************************************
 * SetPower
 * 
 * Transmits the CAN message to set the power.
 *
 * Params:
 * realPower_kW - the commanded power
 * reactivePower_kVA - the reactive power
 *
 * Return:
 * true to indicate to calling function message has been sent
 *
 **************************************************************************************************/
bool APP_CAN::SetPower(int16_t realPower_kW, int16_t reactivePower_kVA)
{
  processToInverterUnion_t canData;
  uint16_t index;

  canData.processToInverterMode2.reactivePowerDemand = reactivePower_kVA;
  canData.processToInverterMode2.realPowerDemand = realPower_kW;
  canData.processToInverterMode2.mode = 2;
  canData.processToInverterMode2.byte1_unused = 0;
  canData.processToInverterMode2.byte6_unused = 0;
  canData.processToInverterMode2.byte7_unused = 0;

  /* populate the data into the CAN message */
  for(index = 0; index < 8; index++)
  {
    canProcessToInverter.data[index] = canData.data[index];
  }

  /* transmit the CAN message */
  comm_protocols.can.write(canProcessToInverter);
}

/***************************************************************************************************
 * SetCurrent
 * 
 * Transmits the CAN message to set the current.
 * 
 * Params:
 * None
 *
 * Return:
 * true to indicate to calling function message has been sent
 *
 **************************************************************************************************/
bool APP_CAN::SetCurrent(int16_t realAmps, int16_t reactiveAmps)
{
  processToInverterUnion_t canData;
  uint16_t index;

  canData.processToInverterMode3.reactiveCurrentDemand = reactiveAmps;
  canData.processToInverterMode3.realCurrentDemand = realAmps;
  canData.processToInverterMode3.mode = 3;
  canData.processToInverterMode3.byte1_unused = 0;
  canData.processToInverterMode3.byte6_unused = 0;
  canData.processToInverterMode3.byte7_unused = 0;

  /* populate the data into the CAN message */
  for(index = 0; index < 8; index++)
  {
    canProcessToInverter.data[index] = canData.data[index];
  }

  /* transmit the CAN message */
  comm_protocols.can.write(canProcessToInverter);

  return true;
}

/***************************************************************************************************
 * GetReadyStatus
 * 
 * Extract the inverter state from the received message.
 *
 * Parameters:
 * None
 *
 * Return:
 * The reported inverter state
 *
 **************************************************************************************************/
statusBitsEnum_t APP_CAN::GetInverterState(void)
{
  return (statusBitsEnum_t)statusMsgRx.data.state;
}

/***************************************************************************************************
 * GetSetCanMode
 * 
 * Set the inverter control mode as CAN Bus.
 *
 * Params:
 * None
 *
 * Return:
 * true to indicate to calling function message has been sent
 *
 **************************************************************************************************/
bool APP_CAN::SetCanMode(void)
{
  parameterQuery_union_t canData;
  uint16_t index;

  canData.dataMode13.mode = 13U;
  canData.dataMode13.meta = 0U;
  canData.dataMode13.readParamCommand = 0U;
  canData.dataMode13.stopBits = 1U;
  canData.dataMode13.controlSource = 0U;     // 0 = CAN, 1 = Modbus
  canData.dataMode13.parity = 0U;
  canData.dataMode13.dropNum = 1U;           // Modbus device address
  canData.dataMode13.monitorTimeout = 1000U; // Timeout in ms for monitored messages

  /* populate the data into the CAN message */
  for(index = 0; index < 8; index++)
  {
    canParameterQuery.data[index] = canData.byte[index];
  }

  /* transmit the CAN message */
  comm_protocols.can.write(canParameterQuery);

  return true;
}

/***************************************************************************************************
 * SetManageDio
 * 
 * Set the active/inactive state of DIOs.
 *
 * Params:
 * None
 *
 * Return:
 * true to indicate to calling function message has been sent
 *
 **************************************************************************************************/
bool APP_CAN::SetManageDio(void)
{
  parameterQuery_union_t canData;
  uint16_t index;

  canData.dataMode20.mode              = 20U;   
  canData.dataMode20.meta              = 0U;
  canData.dataMode20.readParamCommand  = 0U;
  canData.dataMode20.invertDI1         = 0U;        
  canData.dataMode20.invertDI2         = 0U;        
  canData.dataMode20.invertDI3         = 0U;        
  canData.dataMode20.invertDI4         = 0U;        
  canData.dataMode20.invertDO1         = 0U;        
  canData.dataMode20.invertDO2         = 0U;        
  canData.dataMode20.invertDO3         = 0U;        
  canData.dataMode20.invertDO4         = 0U;        
  canData.dataMode20.forceRelayK2DcRun = 0U;
  canData.dataMode20.forceRelayK1PreCh = 0U;
  canData.dataMode20.forceRelayMx2     = 0U;    
  canData.dataMode20.forceRelayMx1     = 0U;    
  canData.dataMode20.do4Controller     = 0U;    
  canData.dataMode20.do3Controller     = 0U;    
  canData.dataMode20.do2Controller     = 0U;    
  canData.dataMode20.do1Controller     = 0U;    
  canData.dataMode20.do4Command        = 0U;       
  canData.dataMode20.do3Command        = 0U;      
  canData.dataMode20.do2Command        = 0U;       
  canData.dataMode20.do1Command        = 0U;       
  canData.dataMode20.DI1Function       = 0U;      
  canData.dataMode20.unused1           = 0U;        
  canData.dataMode20.inverHwEnable     = 0U;

  /* populate the data into the CAN message */
  for(index = 0; index < 8; index++)
  {
    canParameterQuery.data[index] = canData.byte[index];
  }

  /* transmit the CAN message */
  comm_protocols.can.write(canParameterQuery);

  return true;
}
