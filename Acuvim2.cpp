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
#include <ArduinoModbus.h>
#include <EthernetInterface.h>
#include <Ethernet.h>
#include "APP/Controller.h"
#include "APP/Acuvim2.h"
#include "HAL/HAL_Timer.h"

#define ACUVIM_TIMEOUT            FIFTEEN_SECONDS_MS
#define ACUVIM_RESPONSE_TIMEOUT   TEN_SECONDS_MS
#define ETHERNET_CONNECT_TIMEOUT  10000U    // in ms

/* uncomment for additional debug output */
//#define DEBUG_ACUVIEW

#define NOOF_BASIC_REGS_20MS              30
#define ACCUVIM_MB_ID                     1

#define ACUVIM_MB_FREQ_ADDR               0x3400 /* TBD */
#define ACUVIM_MB_REAL_POWER_TOTAL_ADDR   0x3400 /* TBD */
#define ACUVIM_MB_BASIC_20MS_ADDR         0x4000

typedef enum ACUVIM_FAULT
{
  ACU_FAULT_OK         = 0,
  ACU_FAULT_NO_COMMS   = 1
}acuvimFault_t;

typedef enum READ_STATE_ENUM
{
  ACUVIM_ETH_CONNECT_ENTRY       = 0,
  ACUVIM_ETH_CONNECT_DURING      = 1,
  ACUVIM_TCP_SOCKET_INIT_ENTRY   = 2,
  ACUVIM_TCP_SOCKET_INIT_DURING  = 3,
  ACUVIM_CONNECT_SERVER_ENTRY    = 4,
  ACUVIM_CONNECT_SERVER_DURING   = 5,
  ACUVIM_INIT_MODBUS_ENTRY       = 6,
  ACUVIM_INIT_MODBUS_DURING      = 7,
  ACUVIM_READ_IDLE_ENTRY         = 8,
  ACUVIM_READ_IDLE_DURING        = 9,
  ACUVIM_READ_IN_PROGRESS_ENTRY  = 10,
  ACUVIM_READ_IN_PROGRESS_DURING = 11
}acuvimReadState_t;

using namespace machinecontrol;

const char SERVER_IP_ADDRESS[] = "192.168.1.254";  // AcuVim IP Addr
const IPAddress MODBUS_SERVER_IP_ADDRESS{192, 168, 1, 254};
const char CLIENT_IP_ADDRESS[] = "192.168.1.10";   // Controller IP Address
const char SUBNET_MASK[] = "255.255.255.0";
const char DEFAULT_GW[] = "192.168.1.1";

EthernetInterface net;
EthernetClient ethClient;

//shaun still works with laptop if this is removed TCPSocket localSocket;
TCPSocket *serverSocketPtr;
ModbusTCPClient modbusTCPClient(ethClient);

acuvimBasicMeasurement20ms_t acuvim;
bool isMeterReady = false;

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
  bool isConnected = false;
  nsapi_error_t networkError;
  nsapi_connection_status_t networkStat;
  SocketAddress addr;

  networkStat = net.get_connection_status();  
  
  if(NSAPI_STATUS_DISCONNECTED == networkStat)
  {
    //setup static ip address
    net.set_network(CLIENT_IP_ADDRESS, SUBNET_MASK, DEFAULT_GW);

    /* Bring up the ethernet interface */
    networkError = net.connect(); 

    networkStat = net.get_connection_status();
  }

  if (networkStat == NSAPI_STATUS_GLOBAL_UP)
  {
    isConnected = true;
  }

  return isConnected;
}

/***************************************************************************************************
 * InitTcpSocket
 * 
 * This function is called after ethernet connection to initialise the TCP socket for the ACUVIM
 * Modbus connection.
 *
 * Parameters:
 * None
 *
 * Return:
 * true if TCP socket configured, otherwise false
 *
 **************************************************************************************************/
bool ACUVIM_II::InitTcpSocket(void)
{
  bool isSocketInit = false;
  nsapi_error_t networkError;
  SocketAddress addr;
  TCPSocket localSocket;

  /* Open a socket on the network interface */
  //shaun still works with laptop if this is removed networkError = localSocket.open(&net); 

  //shaun still works with laptop if this is removed if (NSAPI_ERROR_OK == networkError)
  //shaun still works with laptop if this is removed {
    /* addr is intermediate variable used for providing information to bind socket */
    //shaun still works with laptop if this is removed addr.set_ip_address(CLIENT_IP_ADDRESS);
    
    //shaun still works with laptop if this is removed addr.set_port(502);

    /* configure socket */
    //shaun still works with laptop if this is removed networkError = localSocket.bind(addr); 
    
    //shaun still works with laptop if this is removed 
    ethClient.setSocket(&localSocket);
  //shaun still works with laptop if this is removed }

  //shaun still works with laptop if this is removed if (NSAPI_ERROR_OK == networkError)
  //shaun still works with laptop if this is removed {
    isSocketInit = true;
  //shaun still works with laptop if this is removed }

  return isSocketInit;
}

/***************************************************************************************************
 * ConnectServer
 * 
 * After network and socket is initialised, attempts to connect to Acuvim meter. It is recommended
 * if this fails to connect, the calling function should close down the socket and start again.
 *
 * Parameters:
 * None
 *
 * Return:
 * true if TCP socket configured, otherwise false
 *
 **************************************************************************************************/
bool ACUVIM_II::ConnectServer(void)
{
  bool isServerConnected = false;
  nsapi_error_t networkError;
  SocketAddress addr;
  uint8_t status;
 
  addr.set_ip_address(SERVER_IP_ADDRESS);
  addr.set_port(MODBUS_DEFAULT_PORT);

  // shaun works with laptop if this is removed networkError = localSocket.connect(addr); 
  // Stops working with laptop if this is removed
  //if(NSAPI_ERROR_OK == networkError)
  //{ 
  Serial.println("DB1");
  isServerConnected = ethClient.connect(addr);
  Serial.println("DB2");
  
  //}
  return isServerConnected;
}

/***************************************************************************************************
 * ModbusClientInit
 * 
 * After network connection has been established with acuvim meter, configure modbus client.
 *
 * Parameters:
 * None
 *
 * Return:
 * true if TCP socket configured, otherwise false
 *
 **************************************************************************************************/
bool ACUVIM_II::ModbusClientInit(void)
{
  bool isConnected;
  IPAddress ip;
  
  isConnected = (bool)modbusTCPClient.connected();

  if (true == isConnected)
  {
    isConnected = modbusTCPClient.begin(MODBUS_SERVER_IP_ADDRESS, MODBUS_DEFAULT_PORT);    
  }

  Serial.print("Modbus connected: ");
  Serial.println(isConnected);

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

  // if (!modbusTCPClient.connected()) 
  // {
  //   // client not connected, start the Modbus TCP client
  //   Serial.println("Attempting to connect to Modbus TCP server");

  //     if (!modbusTCPClient.begin(acuvimClientIp, 502)) 
  //     {
  //       Serial.println("Modbus TCP Client failed to connect!");
  //       isConnected = false;
  //     } 
  //     else 
  //     {
  //       Serial.println("Modbus TCP Client connected");
  //       isConnected = true;
  //     }
  // }   
  return isConnected;
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

  if (true == (bool)modbusTCPClient.connected())
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
bool ACUVIM_II::BasicRequest20ms(void)
{
  SocketAddress addr;
  uint16_t data[] = {0x55, 0xAA};
  bool rqstPass = false;

  addr.set_ip_address(SERVER_IP_ADDRESS);
  addr.set_port(MODBUS_DEFAULT_PORT);

  if (true == (bool)modbusTCPClient.connected())
  {  
    
    //if (!modbusTCPClient.requestFrom(HOLDING_REGISTERS, 
    //                                ACUVIM_MB_BASIC_20MS_ADDR, 
    //                                NOOF_BASIC_REGS_20MS)) 
    if (!modbusTCPClient.requestFrom(COILS, 
                                    1, 
                                    1)) 
    {
      //Serial.print("Failed to send Acuview read request: ");
      //Serial.println(modbusTCPClient.lastError());
    }
    else
    {
      rqstPass = true;
      Serial.print("READ OK");
    }
  }

  return rqstPass;
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

  /* prevent execution blocking when operating on socket */
  //shaun still works with laptop if this is removed localSocket.set_blocking(false);

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
  static uint16_t timeoutTimer = 0U;
  bool result;
  bool isNewData = false;
  static acuvimReadState_t readState = ACUVIM_ETH_CONNECT_ENTRY;    

  switch (readState)
  {
    case ACUVIM_ETH_CONNECT_ENTRY:     
      /* Close down any open network connections */
      //shaun still works with laptop if this is removed localSocket.close();
      net.disconnect();
      isMeterReady = false;
      Serial.println("ACUVIM: Connecting ethernet...");
      readState = ACUVIM_ETH_CONNECT_DURING;
      /* Fall straight through to.... */
    case ACUVIM_ETH_CONNECT_DURING:
      result = ConnectEthernet();
      if(true == result)
      {
        timeoutTimer = 0U;
        Serial.println("ACUVIM: Ethernet connected...");
        readState = ACUVIM_TCP_SOCKET_INIT_ENTRY;
      }
      else
      {
        if(timeoutTimer < ETHERNET_CONNECT_TIMEOUT)
        {
          timeoutTimer++;
        }
        else
        {
          net.disconnect();
          timeoutTimer = 0U;
        }
      }
      break;

    case ACUVIM_TCP_SOCKET_INIT_ENTRY:
      Serial.println("ACUVIM: Initialising TCP socket...");
      readState = ACUVIM_TCP_SOCKET_INIT_DURING;
      /* Fall straight through to.... */
    case ACUVIM_TCP_SOCKET_INIT_DURING:
      result = InitTcpSocket();

      if(true == result)
      {
        Serial.println("ACUVIM: TCP socket initialised...");
        readState = ACUVIM_CONNECT_SERVER_ENTRY;
      }        
      break;

    case ACUVIM_CONNECT_SERVER_ENTRY:
      Serial.println("ACUVIM: Waiting to connect...");
      readState = ACUVIM_CONNECT_SERVER_DURING;
      /* Fall straight through to.... */
    case ACUVIM_CONNECT_SERVER_DURING:
      result = ConnectServer();

      if(true == result)
      {
        readState = ACUVIM_INIT_MODBUS_ENTRY;
        Serial.println("ACUVIM: Connected");
      }
      else
      {
        /* if connection to server fails try again */
        readState = ACUVIM_ETH_CONNECT_ENTRY;
        Serial.println("ACUVIM: Connection failed, retrying...");
      }
      break;

    case ACUVIM_INIT_MODBUS_ENTRY:
      Serial.println("ACUVIM: Initialising Modbus client...");
      readState = ACUVIM_INIT_MODBUS_DURING;
      /* Fall straight through to.... */
    case ACUVIM_INIT_MODBUS_DURING:
      result = ModbusClientInit();
      if(true == result)
      {
        readState = ACUVIM_READ_IDLE_ENTRY;
        isMeterReady = true;
        Serial.println("ACUVIM: Complete");
      }
      else
      {
        readState = ACUVIM_ETH_CONNECT_ENTRY;
      }
      break;

    case ACUVIM_READ_IDLE_ENTRY:
      readState = ACUVIM_ETH_CONNECT_DURING;
      /* Fall straight through to.... */
    case ACUVIM_READ_IDLE_DURING:
      digital_outputs.set(2, HIGH);
      result = BasicRequest20ms();                     // initiate new read
      digital_outputs.set(2, LOW);
      if(true == result)
      {
        //shaun readState = ACUVIM_READ_IN_PROGRESS_ENTRY;
      }
      else
      {
        readState = ACUVIM_ETH_CONNECT_ENTRY;
      }
      break;

    case ACUVIM_READ_IN_PROGRESS_ENTRY:
      readState = ACUVIM_READ_IN_PROGRESS_DURING;
      /* Fall straight through to.... */
    case ACUVIM_READ_IN_PROGRESS_DURING:           // poll for data
      digital_outputs.set(3, HIGH);
      isNewData = BasicRead20ms();
      digital_outputs.set(3, LOW);
      if (true == isNewData)
      {
        *measurements = acuvim;              // transfer read data to pointer
        readState = ACUVIM_READ_IDLE_ENTRY;
      }
      break;

    default:
      /* invalid state, so reset */
      readState = ACUVIM_ETH_CONNECT_ENTRY;
      break;
  } /* switch (readState) */

  return isNewData;
}

/***********************************************************************************************
 * GetReadyState
 * 
 * This function is called to determine whether the meter is able to provide measurements.
 *
 * Parameters:
 * None.
 *
 * Return:
 * true if fault present, otherwise false.
 *
 **********************************************************************************************/
bool ACUVIM_II::GetReadyState(void)
{
  return isMeterReady;  
}


/* end public functions */
