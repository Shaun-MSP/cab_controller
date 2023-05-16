/***************************************************************************************************
 * HAL_ModbusTcp
 * 
 * This module is written as an interface Modbus TCP using non-member functions.
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
#include "EthernetInterface.h"
#include "HAL/HAL_Timer.h"
#include "Modbus/mb_slave.h"
#include "Modbus/mb_tcp.h"
#include "Modbus/mb_slave_init.h"
//#include "IPAddress.h"

#define ETHERNET_PORT_MODBUS    502
#define SIZE_OF_TCP_RX_BUFF     100

const char SERVER_IP_ADDRESS[] = "192.168.30.10";
const char CLIENT_IP_ADDRESS[] = "192.168.30.1";
const char SUBNET_MASK[] = "255.255.255.0";
const char DEFAULT_GW[] = "192.168.30.1";

/* Private function declarations */
bool TCP_EthernetSetup(void);    

EthernetInterface net;
TCPSocket localSocket;
TCPSocket *clientSocketPtr;
mb_slave_t * server;

  /* Private functions */
  /***************************************************************************************************
  * TCP_EthernetSetup
  * 
  * This function initialises the ethernet port.
  *
  * Parameters:
  * None
  *
  * Return:
  * True if ethernet initialised OK, otherwise false.
  *
  * Date:
  * 02/03/2023
  *
  **************************************************************************************************/
  bool TCP_EthernetSetup(void) 
  {
    SocketAddress addr;
    nsapi_error_t networkError;
    nsapi_connection_status_t networkStat;
    bool isGood = true;
    static nsapi_connection_status_t oldNetworkStat = NSAPI_STATUS_ERROR_UNSUPPORTED;
    uint32_t startTime;
    uint32_t elapsedTime;  

    nsapi_error_t setNetResult;

    Serial.println("Initialise Ethernet");

    /* disable blocking whilst waiting for network */
    //shaun net.set_blocking(false);

    /* disable DHCP - use static IP address */
    net.set_dhcp(false);

    networkStat = net.get_connection_status();  
    
    if(NSAPI_STATUS_DISCONNECTED == networkStat)
    {
      //setup static ip address
      net.set_network(SERVER_IP_ADDRESS, SUBNET_MASK, DEFAULT_GW);
      
      /* Bring up the ethernet interface */
      networkError = net.connect(); 
    }
    
    /* Get start time for measuring network timeout */
    startTime = millis();    

    Serial.print("Trying to connect... ");

    do
    {
      /* If no network connected and blocking is true, it will get stuck here forever. Instead
         blocking is FALSE and there is a timeout in the loop while it waits to connect */
      networkStat = net.get_connection_status();
      elapsedTime = millis() - startTime;    
    }
    /* Wait until network connected, or timeout reached */
    while((networkStat != NSAPI_STATUS_GLOBAL_UP) && (elapsedTime < TEN_SECONDS_MS));   

    if(elapsedTime >= TEN_SECONDS_MS)   
    {
      /* Network cable unplugged? */
      networkError = NSAPI_ERROR_CONNECTION_TIMEOUT;
      Serial.println("Network connection timed out");
    }

    if(NSAPI_ERROR_OK == networkError)
    {
      net.get_ip_address(&addr);
      Serial.print("IP address: ");
      Serial.println(addr.get_ip_address() ? addr.get_ip_address() : "None");
      
      // Open a socket on the network interface
      networkError = localSocket.open(&net); 
    }

    if(NSAPI_ERROR_OK == networkError)
    {  
      Serial.println("Network status: Socket open");  
      addr.set_ip_address(SERVER_IP_ADDRESS);
      addr.set_port(ETHERNET_PORT_MODBUS);
      networkError = localSocket.bind(addr);      
    }

    if(NSAPI_ERROR_OK == networkError)
    {   
      Serial.println("Network status: Socket bound to 192.168.10:502");     
      networkError = localSocket.listen();
    }

    if (NSAPI_ERROR_OK != networkError)
    {
      // Close the socket to return its memory and bring down the network interface
      localSocket.close();
      Serial.println ("Skt closed");
      // Bring down the ethernet interface
      net.disconnect();

      Serial.println("Network connection fail");
      isGood = false;
    }
    else
    {
      Serial.println("Network status: Socket passive");
      Serial.println("Network connection OK");

      /* Blocks here until client connects */
      Serial.println("Listening for client...");
      clientSocketPtr = localSocket.accept(&networkError);

      if(NSAPI_ERROR_OK == networkError)
      {
        Serial.println("Client connected");
      }
    }

    return isGood;
  }

  /* Public functions */
/***************************************************************************************************
  * TCP_ModbusServerStart
  * 
  * This function initialises the Modbus TCP server .
  *
  * Parameters:
  * None
  *
  * Return:
  * True if ethernet initialised OK, otherwise false.
  *
  * Date:
  * 02/03/2023
  *
  **************************************************************************************************/
  void TCP_ModbusServerStart(void)
  {
    mb_transport_t * tcp;
    static const mb_tcp_cfg_t mb_tcp_cfg = 
    {
        .port = ETHERNET_PORT_MODBUS,
    };

    tcp = mb_tcp_init (&mb_tcp_cfg);
    server = mb_slave_init (&mb_slave_cfg, tcp);
    //return server;
  }

  /***************************************************************************************************
  * TCP_CloseSocket
  * 
  * This function closes the open network socket.
  *
  * Parameters:
  * None
  *
  * Return:
  * None
  *
  **************************************************************************************************/
  extern "C" void TCP_CloseSocket() 
  {
    Serial.println("TCP_CloseSocket");
    // Close the socket to return its memory and bring down the network interface
    localSocket.close();

    // Bring down the ethernet interface
    net.disconnect();   
  }

  /***************************************************************************************************
  * TCP_Init
  * 
  * This function initialises the CAN port to 500kbs.
  *
  * Parameters:
  * None
  *
  * Return:
  * True if ethernet initialised OK, otherwise false.
  *
  * Date:
  * 02/03/2023
  *
  **************************************************************************************************/
  bool TCP_Init(void) 
  {
    bool isGood;

    isGood = TCP_EthernetSetup();

    return isGood;
  } 

  /***************************************************************************************************
  * TCP_IsConnected
  * 
  * This function is called to check the status of the network.
  *
  * Parameters:
  * None
  *
  * Return:
  * true if the network is connected, otherwise false.
  *
  **************************************************************************************************/
  bool TCP_IsConnected(void) 
  {
    nsapi_connection_status_t networkStat;
    bool connectFlag = false;

    networkStat = net.get_connection_status();  

    if (networkStat != NSAPI_ERROR_OK)
    {
      connectFlag = true;
    }
  }

  /***************************************************************************************************
  * TCP_Recv
  * 
  * This function is called to check for received TCP data. The Ethernet socket is set to 
  * non-blocking, meaning if no data has been received the call to recv() returns 
  * NSAPI_ERROR_WOULD_BLOCK.
  *
  * Parameters:
  * None
  *
  * Return:
  * true if the network is connected, otherwise false.
  *
  **************************************************************************************************/
  extern "C" uint16_t TCP_Recv(void * buffer, size_t size)
  {
    int16_t noofBytes = 0;
    nsapi_error_t errorMsg = NSAPI_ERROR_OK;

    noofBytes = localSocket.recv(buffer, size);

    if (noofBytes < 0)
    {
      Serial.print("noofBytes error: ");
      Serial.print(noofBytes);
      Serial.println("");
      errorMsg = noofBytes;
      noofBytes = 0;
    }
    else
    {
      Serial.print("noofBytes rxed: ");
      Serial.print(noofBytes);
    }

    return noofBytes;
  }

  /***************************************************************************************************
  * TCP_Send
  * 
  * This function is called to send TCP data. 
  *
  * Parameters:
  * None
  *
  * Return:
  * The numnber of bytes sent.
  *
  **************************************************************************************************/
  extern "C" uint16_t TCP_Send(void * buffer, size_t size)
  {
    uint16_t noofBytes;

    noofBytes = localSocket.send(buffer, size);

    if((int16_t)NSAPI_ERROR_WOULD_BLOCK == noofBytes)
    {
      noofBytes = 0;
    }

    return noofBytes;
  }

  void TCP_ModbusPoll(void)
  {
    mb_slave(server);
  }

  extern "C" int16_t TCP_Bringup(void)
  {
    bool result;
    int16_t returnVal;

    result = TCP_Init();

    if(true == result)
    {
      returnVal = 1;
    }
    else
    {
      returnVal = 0;

    }
    return returnVal;
  }

  int16_t TCP_NetGetStat(void)
  {
    int16_t status;

    status = (int16_t)net.get_connection_status();

    return status;
  }
