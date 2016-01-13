/**
******************************************************************************
* @file     zc_hf_adpter.h
* @authors  cxy
* @version  V1.0.0
* @date     10-Sep-2014
* @brief    HANDSHAKE
******************************************************************************
*/

#ifndef  __ZC_HF_ADPTER_H__ 
#define  __ZC_HF_ADPTER_H__

#include <zc_common.h>
#include <zc_protocol_controller.h>
#include <zc_module_interface.h>
#include "os_type.h"


typedef struct 
{
    os_timer_t timer;
}ESP_TimerInfo;


#define HF_MAX_SOCKET_LEN    (1000)


//#ifdef WIFI_ON_MCU
typedef struct 
{
    u8 u8CloudKey[36];
    u8 u8PrivateKey[112];
    u8 u8DeviciId[ZC_HS_DEVICE_ID_LEN + ZC_DOMAIN_LEN];
    u8 u8CloudAddr[20];
    u8 u8EqVersion[ZC_EQVERSION_LEN];
    u8 u8TokenKey[16];
}HF_StaInfo;
//#endif

typedef enum{
    STATUS_BIT_NWP_INIT = 0, // If this bit is set: Network Processor is                           
    STATUS_BIT_CONNECTION,   // If this bit is set: the device is connected to                          
    STATUS_BIT_IP_LEASED,    // If this bit is set: the device has leased IP to 
    STATUS_BIT_IP_AQUIRED,   // If this bit is set: the device has acquired an IP
    STATUS_BIT_SMARTCONFIG_START, // If this bit is set: the SmartConfiguration 
    STATUS_BIT_P2P_DEV_FOUND,    // If this bit is set: the device (P2P mode) 
    STATUS_BIT_P2P_REQ_RECEIVED, // If this bit is set: the device (P2P mode) 
    STATUS_BIT_CONNECTION_FAILED, // If this bit is set: the device(P2P mode)
    STATUS_BIT_PING_DONE,         // If this bit is set: the device has completed
    STATUS_BIT_DISCONNECTED
}e_StatusBits;

#define GET_STATUS_BIT(status_variable, bit)    (0 != (status_variable & (1<<(bit))))
#define TI_IS_CONNECTED(status_variable)         GET_STATUS_BIT(status_variable, 1)
#define TI_IS_IP_ACQUIRED(status_variable)       GET_STATUS_BIT(status_variable, 3)
#define TI_IS_DISCONNECTED(status_variable)      GET_STATUS_BIT(status_variable, STATUS_BIT_DISCONNECTED)

#define SET_STATUS_BIT(status_variable, bit)     status_variable |= (1<<(bit))
#define CLR_STATUS_BIT(status_variable, bit)     status_variable &= ~(1<<(bit))


#ifdef __cplusplus
extern "C" {
#endif
int ESP_Init(void);
void ESP_WakeUp(void);
void ESP_Sleep(void);
void ESP_ReadDataFormFlash(void);
void ESP_WriteDataToFlash(u8 *pu8Data, u16 u16Len);
void ESP_GotIp(void);
void ESP_WakeUp(void);
void ESP_UdpBroadcast(void);
void ESP_CreateTaskTimer(void);

#ifdef __cplusplus
}
#endif
#endif
/******************************* FILE END ***********************************/

