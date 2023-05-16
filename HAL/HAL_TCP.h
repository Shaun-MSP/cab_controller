/***************************************************************************************************
 * 
 * Header for for CAN_MID.cpp
 * 
 * Date: 21/02/2023
 * 
 * Author: Shaun Mcsherry
 * 
 * ************************************************************************************************/
#ifndef HAL_TCP_H
#define HAL_TCP_H

extern bool TCP_Init(void);
extern bool TCP_IsConnected(void);
extern void TCP_CloseSocket(void);
extern uint16_t TCP_Recv(void * buffer, size_t size);
extern void TCP_ModbusPoll(void);
extern int16_t TCP_Bringup(void);
extern void TCP_ModbusServerStart(void);
extern int16_t TCP_NetGetStat(void);

#endif /* HAL_TCP_H */
