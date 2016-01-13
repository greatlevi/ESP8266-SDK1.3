/**
******************************************************************************
* @file     ac_hal.h
* @authors  cxy
* @version  V1.0.0
* @date     10-Sep-2014
* @brief    
******************************************************************************
*/

#ifndef  __AC_HAL_H__ 
#define  __AC_HAL_H__

#include <zc_common.h>
#include <zc_protocol_interface.h>
#include <ac_api.h>
#define MSG_SERVER_CLIENT_SET_LED_ONOFF_REQ  (4)
//#define MSG_SERVER_CLIENT_GET_LED_STATUS_RSP (203) 
#define MSG_SERVER_CLIENT_SET_POWER_ONOFF_REQ	(1)
#define MSG_SERVER_CLIENT_SET_SPEED_ONOFF_REQ 	(2)
#define MSG_SERVER_CLIENT_SET_BUZZER_ONOFF_REQ	(3)
#define MSG_SERVER_CLIENT_SET_CAPACITY_ONOFF_REQ	(5)

#define MSG_SERVER_CLIENT_GET_MOUDLE_STATUS_RSP  (0)

#define ACCORD_RELPY_DATA (210)
#define CLIENT_SERVER_OK  (102)   
#define RESPOND_SERVER_DATA	(68)
#define CLOUDSTATUS 0
#define WIFIPOWERSTATUS 1

#define CLOUDCONNECT 1 
#define CLOUDDISCONNECT 0

#define AC_Printf UARTprintf
#define WIFIPOWERON 1
#define WIFIPOWEROFF 0
#ifdef __cplusplus
extern "C" {
#endif
void AC_SendMessage(u8 *pu8Msg, u16 u16DataLen);
void AC_DealNotifyMessage(ZC_MessageHead *pstruMsg, AC_OptList *pstruOptList, u8 *pu8Playload);
void AC_DealOtaMessage(ZC_MessageHead *pstruMsg, AC_OptList *pstruOptList,  u8 *pu8Playload);
void AC_DealEvent(ZC_MessageHead *pstruMsg, AC_OptList *pstruOptList,u8 *pu8Playload);
u32  AC_GetStoreStatus(u32 u32Type);
unsigned short crc16_ccitt(const unsigned char *buf, unsigned int len);	
void AC_StoreStatus(u32 u32Type , u32 u32Data);
void AC_DealLed(ZC_MessageHead *pstruMsg, AC_OptList *pstruOptList, u8 *pu8Playload);
#ifdef __cplusplus
}
#endif
#endif
/******************************* FILE END ***********************************/



