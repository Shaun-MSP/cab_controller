/***************************************************************************************************
 * Flex
 * 
 * This module is the communications interface (modbus TCP) to the Flex3. It is used to receive
 * messages from Flex, extract information and formulate responses
 *
 * Date:
 * 04/04/2023
 *
 * Author:
 * Shaun McSherry
 *
 **************************************************************************************************/
#include <Arduino_MachineControl.h>
#include "APP/Flex.h"

/* Private functions */
void SendHeartbeat(uint16_t counter)
{

}
    
uint16_t GetHeartbeat(void)
{
  uint16_t counter;

  return counter;
}


/* Public functions */
/***************************************************************************************************
 * Control
 * 
 * This function should be called periodically to poll for received Modbus messages from Flex.
 *
 * It also schedules responses where necessary.
 *
 * Parameters:
 * None
 *
 * Return:
 * None
 *
 **************************************************************************************************/
bool FLEX::Control(void)
{
  bool flexFaultFlag = false;
    
  return flexFaultFlag;
}

/***************************************************************************************************
 * Init
 * 
 * This function is called to initialise the Modbus TCP interface to Flex.
 * Parameters:
 * None.
 *
 * Return:
 * None.
 *
 **************************************************************************************************/
void FLEX::Init(void) 
{
  //   /* Enter a MAC address for your controller below.
  //     Newer Ethernet shields have a MAC address printed on a sticker on the shield */
  // uint8_t mac[] = 
  // {
  //   0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  // }; 

  // Serial.println("Initialising AcuVim II");

  // /* Get MAC address of ethernet shield */
  // Ethernet.MACAddress(mac);

  // /* start the Ethernet connection and the server: */
  // Ethernet.begin(mac, acuvimClientIp, 
  //                     acuvimClientDns, 
  //                     acuvimClientGateway, 
  //                     acuvimClientSubnet, ACUVIM_TIMEOUT, ACUVIM_RESPONSE_TIMEOUT);
  // //Ethernet.begin(mac, acuvimClient);
  // /* Check for Ethernet hardware present */
  // if (EthernetNoHardware == Ethernet.hardwareStatus()) 
  // {
  //   Serial.println("Ethernet shield not found");
  //   AcuvimFault = ACU_FAULT_NO_ETH_SHIELD;
  // }
  // else if (Ethernet.linkStatus() == LinkOFF) 
  // {
  //   Serial.println("Accuvim ethernet cable is not connected.");
  //   AcuvimFault = ACU_FAULT_NO_ETH_CONNECTION;
  // }
  // else
  // {
  //   AcuvimFault = ACU_FAULT_SHIELD_OK;
  // }
}

/***************************************************************************************************
 * getOperatingState
 * 
 * This function is called to extract operating state, i.e. enable/disable and operating mode
 * from Modbus message.
 * Parameters:
 * None.
 *
 * Return:
 * None.
 *
 **************************************************************************************************/
void FLEX::GetOperatingState(flexOperatingStateStruct_t *operatingState)
{

}
    
uint16_t FLEX::GetDemand(void)
{
    uint16_t demand = 0U;

    return demand;
}
    
uint16_t FLEX::GetMaxPowerRating(void)
{
    uint16_t maxPowerRating = 0U;

    return maxPowerRating;
}

uint16_t FLEX::GetMaxOnlineCapacity(void)
{
    uint16_t getMaxOnlineCapacity;

    return getMaxOnlineCapacity;
}

void FLEX::PowerMeasured(uint16_t powerMeasured)
{

}