/**
******************************************************************************
* @file     zc_hf_adpter.c
* @authors  cxy
* @version  V1.0.0
* @date     10-Sep-2014
* @brief    Event
******************************************************************************
*/
#include <zc_protocol_controller.h>
#include <zc_timer.h>
#include <zc_module_interface.h>
#include <zc_hf_adpter.h>
#include <stdlib.h>
#include <Ac_cfg.h>
#include <ac_api.h>
#include "os_type.h"
#include "espconn.h"
#include "user_interface.h"
#include "flash_api.h"
#include "ets_sys.h"
#include "osapi.h"
#include "user_config.h"

#define  FLASH_ADDRESS            0x200000

#define  HF_SUCCESS            0


#define AcTaskPrio        1
#define AcTaskQueueLen    10

os_event_t    AcTaskQueue[AcTaskQueueLen];


u8  g_u8EqVersion[]={0,0,0,0};      
u8  g_u8ModuleKey[ZC_MODULE_KEY_LEN] =DEFAULT_IOT_PRIVATE_KEY;
u64  g_u64Domain;
u8  g_u8DeviceId[ZC_HS_DEVICE_ID_LEN];

//u64 SUB_DOMAIN_ID	=	0xFFFFFFFFFFFFFFFF;//子域id
vu32 g_u32DomainMagicFlag = 0xFFFFFFFF;

extern PTC_ProtocolCon  g_struProtocolController;
PTC_ModuleAdapter g_struHfAdapter;

MSG_Buffer g_struRecvBuffer;
MSG_Buffer g_struRetxBuffer;
MSG_Buffer g_struClientBuffer;


MSG_Queue  g_struRecvQueue;
MSG_Buffer g_struSendBuffer[MSG_BUFFER_SEND_MAX_NUM];
MSG_Queue  g_struSendQueue;

u8 g_u8MsgBuildBuffer[MSG_BULID_BUFFER_MAXLEN];
u8 g_u8ClientSendLen = 0;


u16 g_u16TcpMss;
u16 g_u16LocalPort;


u8 g_u8recvbuffer[HF_MAX_SOCKET_LEN];
ZC_UartBuffer g_struUartBuffer;
ESP_TimerInfo g_struEspTimer[ZC_TIMER_MAX_NUM];

u8  g_u8BcSendBuffer[100];
u8  g_u8JdRcvBuffer[256];
u32 g_u32BcSleepCount = 800;//800


LOCAL struct espconn tcp_client_conn;    /* 做server */
struct espconn tcp_server_conn;          /* 做client */

LOCAL esp_tcp esptcp;
LOCAL struct espconn user_udp_espconn;

LOCAL os_timer_t task_timer;

ip_addr_t tcp_server_ip;   /* 服务器ip*/

extern volatile unsigned long  g_ulStatus;
/*************************************************
* Function: ESP_ReadDataFormFlash
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void ICACHE_FLASH_ATTR
ESP_ReadDataFormFlash(void)
{
    u32 u32MagicFlag = 0xFFFFFFFF;

    flash_safe_read(FLASH_ADDRESS, (unsigned int *)(&u32MagicFlag), 4);
    if (ZC_MAGIC_FLAG == u32MagicFlag)
    {   
    	flash_safe_read(FLASH_ADDRESS, (unsigned int  *)(&g_struZcConfigDb), sizeof(ZC_ConfigDB));
    }
    else
    {
        ZC_Printf("no para, use default\n");
    }
}
/*************************************************
* Function: ESP_WriteDataToFlash
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void ICACHE_FLASH_ATTR
ESP_WriteDataToFlash(u8 *pu8Data, u16 u16Len)
{
	if(SPI_FLASH_RESULT_OK != flash_safe_erase_sector(512))
    {   
	    ZC_Printf("eraser sector fail\n");
        return;
    }
	if(SPI_FLASH_RESULT_OK != flash_safe_write(FLASH_ADDRESS, (uint32 *)pu8Data, u16Len))
    {   
		ZC_Printf("flash_safe_write sector fail\n");
        return;
    }
}
/*************************************************
* Function: ESP_timer_callback
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void ICACHE_FLASH_ATTR
ESP_timer_callback(u8 u8TimeId)
{
    TIMER_TimeoutAction(u8TimeId);
    TIMER_StopTimer(u8TimeId);
}
/*************************************************
* Function: ESP_timer_hook
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void ICACHE_FLASH_ATTR
ESP_timer_hook(void *arg)
{
    u8 u8TimeId;
    u8TimeId = *(u8 *)arg;
    ESP_timer_callback(u8TimeId);
}
/*************************************************
* Function: ESP_StopTimer
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void ICACHE_FLASH_ATTR
ESP_StopTimer(u8 u8TimerIndex)
{
    os_timer_disarm(&g_struEspTimer[u8TimerIndex].timer);
}
/*************************************************
* Function: ESP_SetTimer
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 ICACHE_FLASH_ATTR
ESP_SetTimer(u8 u8Type, u32 u32Interval, u8 *pu8TimeIndex)
{
    u8 u8TimerIndex;
    u32 u32Retval;
    os_timer_t* timer;
    u32Retval = TIMER_FindIdleTimer(&u8TimerIndex);
    if (ZC_RET_OK == u32Retval)
    {
        TIMER_AllocateTimer(u8Type, u8TimerIndex, (u8*)&g_struEspTimer[u8TimerIndex]);
        *pu8TimeIndex = u8TimerIndex;
        timer = &g_struEspTimer[u8TimerIndex].timer;
    	os_timer_disarm(timer);
        os_timer_setfn(timer, (os_timer_func_t *)ESP_timer_callback, pu8TimeIndex);
        os_timer_arm(timer, u32Interval, 0);
    }
    
    return u32Retval;
}
/*************************************************
* Function: ESP_FirmwareUpdateFinish
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 ICACHE_FLASH_ATTR
ESP_FirmwareUpdateFinish(u32 u32TotalLen)
{
    ZC_RET_OK;
}
/*************************************************
* Function: ESP_FirmwareUpdate
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 ICACHE_FLASH_ATTR
ESP_FirmwareUpdate(u8 *pu8FileData, u32 u32Offset, u32 u32DataLen)
{
    return ZC_RET_OK;
}
/*************************************************
* Function: ESP_SendDataToMoudle
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 ICACHE_FLASH_ATTR
ESP_SendDataToMoudle(u8 *pu8Data, u16 u16DataLen)
{
    AC_RecvMessage((ZC_MessageHead *)pu8Data);
    return ZC_RET_OK;
}
/*************************************************
* Function: ESP_Rest
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void ICACHE_FLASH_ATTR
ESP_Rest(void)
{
    os_printf("ESP_Rest\n");
    g_struZcConfigDb.struSwitchInfo.u32ServerAddrConfig = 0;
    g_struZcConfigDb.struSwitchInfo.u32SecSwitch = 1;
    ESP_WriteDataToFlash((u8 *)&g_struZcConfigDb, sizeof(ZC_ConfigDB));

	g_struProtocolController.u8MainState = PCT_STATE_INIT;
	g_struProtocolController.u16SendBcNum = 0;
    TIMER_Init();
	SamrtLink();
}
/*************************************************
* Function: ESP_SendTcpData
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void ICACHE_FLASH_ATTR
ESP_SendTcpData(u32 u32Fd, u8 *pu8Data, u16 u16DataLen, ZC_SendParam *pstruParam)
{
    /* client/server */
    if (PCT_SERVER_TCP_SOCKET == u32Fd)
    {
        espconn_send(&tcp_server_conn, pu8Data, u16DataLen);
        ZC_Printf("Send to cloud dataLen is :%d\n",u16DataLen);
    }
    else if (PCT_CLIENT_TCP_SOCKET == u32Fd)
    {
    	espconn_send(&tcp_client_conn, pu8Data, u16DataLen);
        ZC_Printf("Send to app dataLen is :%d\n",u16DataLen);
    }
}
/*************************************************
* Function: ESP_SendUdpData
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void ICACHE_FLASH_ATTR
ESP_SendUdpData(u32 u32Fd, u8 *pu8Data, u16 u16DataLen, ZC_SendParam *pstruParam)
{
    espconn_send(&user_udp_espconn, pu8Data, u16DataLen);
}
/*************************************************
* Function: ESP_GetMac
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void ICACHE_FLASH_ATTR
ESP_GetMac(u8 *pu8Mac)
{
    unsigned char mac[6] = {0};
    unsigned char mac_string[ZC_SERVER_MAC_LEN] = {0};
    wifi_get_macaddr(STATION_IF, mac);
    ZC_HexToString(mac_string, mac, 6);
    os_memcpy(pu8Mac, mac_string, ZC_SERVER_MAC_LEN);
    ZCHEX_Printf(pu8Mac,ZC_SERVER_MAC_LEN);
}
/*************************************************
* Function: ESP_Reboot
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void ICACHE_FLASH_ATTR
ESP_Reboot(void)
{
    ZC_Printf("Reboot system !!!\n");
    system_restart();
}
/*************************************************
* Function: ESP_SendToCloudSuccess
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void ICACHE_FLASH_ATTR
ESP_SendToCloudSuccess(void *arg)
{
    ZC_Printf("ESP_SendToCloudSuccess success!!! \n");
}
/*************************************************
* Function: ESP_DisconFromCloud
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void ICACHE_FLASH_ATTR
ESP_DisconFromCloud(void *arg)
{
    struct espconn *pespconn = arg;
    if(PCT_STATE_WAIT_SMARTLINK == g_struProtocolController.u8MainState)
    {
        return;
    }
	ZC_Printf("ESP_DisconFromCloud !!! %d\n",g_struProtocolController.u8MainState);
    if(tcp_server_conn.proto.tcp->remote_port == pespconn->proto.tcp->remote_port)
    {
		//tcp_server_conn.proto.tcp->local_port = espconn_port();
		espconn_connect(&tcp_server_conn);
		ZC_Printf("reconnect port %d \n",tcp_server_conn.proto.tcp->local_port);
    }
}
/*************************************************
* Function: ESP_RecvFromCloud
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void ICACHE_FLASH_ATTR
ESP_RecvFromCloud(void *arg, char *pusrdata, unsigned short length)
{
    //received some data from tcp connection
    os_memcpy(g_u8recvbuffer, pusrdata, length);
    MSG_RecvDataFromCloud(g_u8recvbuffer, length);
    ZC_Printf("recv from cloud\n");
}
/*************************************************
* Function: ESP_ConnectedCloud
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void ICACHE_FLASH_ATTR
ESP_ConnectedCloud(void *arg)
{
    struct espconn *pespconn = arg;
    espconn_regist_recvcb(pespconn, ESP_RecvFromCloud);
    espconn_regist_sentcb(pespconn, ESP_SendToCloudSuccess);
    if (ZC_CLOUD_PORT != pespconn->proto.tcp->remote_port)
    {
        espconn_regist_disconcb(pespconn, ESP_DisconFromCloud);
    }

	g_struProtocolController.u8MainState = PCT_STATE_WAIT_ACCESS;
	g_struProtocolController.u8keyRecv = PCT_KEY_UNRECVED;

    ZC_Printf("connect succeed !!! \r\n");
}
/*************************************************
* Function: ESP_ReconnectCloud
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void ICACHE_FLASH_ATTR
ESP_ReconnectCloud(void *arg, sint8 err)
{
    ZC_Printf("user error code %d !!! \r\n",err);
    //if(g_struProtocolController.u8MainState == PCT_STATE_INIT)return;
	PCT_ReconnectCloud(&g_struProtocolController, 1000);
	g_struUartBuffer.u32Status = MSG_BUFFER_IDLE;
	g_struUartBuffer.u32RecvLen = 0;
	g_struProtocolController.u8MainState = PCT_STATE_ACCESS_NET;
}
/*************************************************
* Function: ESP_DnsFoundHook
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void ICACHE_FLASH_ATTR
ESP_DnsFoundHook(const char *name, ip_addr_t *ipaddr, void *arg)
{
    struct espconn *pespconn = (struct espconn *)arg;
    sint8 temp;

    if (NULL == ipaddr) 
    {
        ZC_Printf("ESP_DnsFoundHook NULL \r\n");
        return;
    }

   //dns got ip
    ZC_Printf("ESP_DnsFoundHook %d.%d.%d.%d \r\n",
            *((uint8 *)&ipaddr->addr), *((uint8 *)&ipaddr->addr + 1),
            *((uint8 *)&ipaddr->addr + 2), *((uint8 *)&ipaddr->addr + 3));

    if (tcp_server_ip.addr == 0 && ipaddr->addr != 0) 
    {
        tcp_server_ip.addr = ipaddr->addr;
        os_memcpy(tcp_server_conn.proto.tcp->remote_ip, &ipaddr->addr, 4); // remote ip of tcp server which get by dns
        tcp_server_conn.proto.tcp->remote_port = ZC_CLOUD_PORT; // remote port of tcp server
        tcp_server_conn.proto.tcp->local_port = espconn_port(); //local port of ESP8266

        espconn_regist_connectcb(&tcp_server_conn, ESP_ConnectedCloud); // register connect callback
        espconn_regist_reconcb(&tcp_server_conn, ESP_ReconnectCloud); // register reconnect callback as error handler
        temp = espconn_connect(&tcp_server_conn);
        if(temp == ESPCONN_ISCONN)
        {
            g_struProtocolController.u8MainState = PCT_STATE_WAIT_ACCESS;
            return;
        }
        ZC_Printf("tcp connect %d, %d.%d.%d.%d, %d,%d\n",temp,
                tcp_server_conn.proto.tcp->remote_ip[0],
                tcp_server_conn.proto.tcp->remote_ip[1],
                tcp_server_conn.proto.tcp->remote_ip[2],
                tcp_server_conn.proto.tcp->remote_ip[3],
                tcp_server_conn.proto.tcp->remote_port,
				tcp_server_conn.proto.tcp->local_port);

    }

}

/*************************************************
* Function: ESP_ConnectToCloud
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 ICACHE_FLASH_ATTR
ESP_ConnectToCloud(PTC_Connection *pstruConnection)
{
    struct ip_addr struIp;
    int retval = 255;
    u16 port;
    
    if (1 == g_struZcConfigDb.struSwitchInfo.u32ServerAddrConfig)
    {
        g_struProtocolController.u8SendToCloudFlag = 1;
        port = g_struZcConfigDb.struSwitchInfo.u16ServerPort;
        struIp.addr = ZC_HTONL(g_struZcConfigDb.struSwitchInfo.u32ServerIp);
        retval = HF_SUCCESS;
    }
    else
    {
        port = ZC_CLOUD_PORT;
        retval = espconn_gethostbyname(&tcp_server_conn,
                                        (const char *)g_struZcConfigDb.struCloudInfo.u8CloudAddr,
                                         &tcp_server_ip, ESP_DnsFoundHook);
        return ZC_RET_OK;
    }
    
    ZC_Printf("cloud_default ip %d.%d.%d.%d %d\r\n",
            *((uint8 *)&struIp.addr), *((uint8 *)&struIp.addr + 1),
            *((uint8 *)&struIp.addr + 2), *((uint8 *)&struIp.addr + 3), port);

    os_memcpy(tcp_server_conn.proto.tcp->remote_ip, &struIp.addr, 4); 
    
    tcp_server_conn.proto.tcp->remote_port = port;
    tcp_server_conn.proto.tcp->local_port = espconn_port(); //local port of ESP8266
    
    espconn_regist_connectcb(&tcp_server_conn, ESP_ConnectedCloud); // register connect callback
    espconn_regist_reconcb(&tcp_server_conn, ESP_ReconnectCloud); // register reconnect callback as error handler
    
    if (0 != espconn_connect(&tcp_server_conn))
    {
        ZC_Printf("Connect not ok\n");
    }
    g_struProtocolController.struCloudConnection.u32ConnectionTimes = 0;

    ZC_Rand(g_struProtocolController.RandMsg);

    return ZC_RET_OK;

}
/*************************************************
* Function: ESP_SendToClient
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void ICACHE_FLASH_ATTR
ESP_SendToClient(void *arg)
{
    ZC_Printf("tcp sent cb \r\n");
}
/*************************************************
* Function: ESP_RecvFromClient
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void ICACHE_FLASH_ATTR
ESP_RecvFromClient(void *arg, char *pusrdata, unsigned short length)
{
    struct espconn *pespconn = arg;
    ZC_Printf("ZC_RecvDataFromClient :\r\n");
//    ZCHEX_Printf(pusrdata, length);
    os_memcpy(g_u8recvbuffer, pusrdata, length);
    RecvDataFromClient(0, g_u8recvbuffer, length);   /* 第一个参数没意义，传0进去即可 */
}
/*************************************************
* Function: ESP_DisconCliet
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void ICACHE_FLASH_ATTR
ESP_DisconCliet(void *arg)
{
    ZC_Printf("tcp disconnect succeed !!! \r\n");
}
/*************************************************
* Function: ESP_ReconClient
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void ICACHE_FLASH_ATTR
ESP_ReconClient(void *arg, sint8 err)
{
    ZC_Printf("reconnect callback, error code %d !!! \r\n",err);
}
/*************************************************
* Function: ESP_AcceptSuccessed
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void ICACHE_FLASH_ATTR
ESP_AcceptSuccessed(void *arg)
{
    struct espconn *pesp_conn = arg;
    os_memcpy(tcp_client_conn.proto.tcp->remote_ip, pesp_conn->proto.tcp->remote_ip,4);
    tcp_client_conn.proto.tcp->remote_port = pesp_conn->proto.tcp->remote_port;

    ZC_Printf("tcp_server_listen ip %d.%d.%d.%d port %d localport %d!!! \r\n"
    		,pesp_conn->proto.tcp->remote_ip[0]
			,pesp_conn->proto.tcp->remote_ip[1]
			,pesp_conn->proto.tcp->remote_ip[2]
			,pesp_conn->proto.tcp->remote_ip[3]
			,pesp_conn->proto.tcp->remote_port
			,pesp_conn->proto.tcp->local_port);
}
/*************************************************
* Function: ESP_ListenClient
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
u32 ICACHE_FLASH_ATTR
ESP_ListenClient(PTC_Connection *pstruConnection)
{
    tcp_client_conn.proto.tcp->local_port = pstruConnection->u16Port;
    ZC_Printf("Tcp Listen Port = %d\n", pstruConnection->u16Port);

    s8 ret = espconn_accept(&tcp_client_conn);
    ZC_Printf("espconn_accept [%d] !!! \r\n", ret);

    return ZC_RET_OK;
}
/*************************************************
* Function: ESP_RecvFromBroadcast
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void ICACHE_FLASH_ATTR
ESP_RecvFromBroadcast(void *arg, char *pusrdata, unsigned short length)
{
    ZC_Printf("recv udp data: \n");
    ZCHEX_Printf(pusrdata,length);
    ZC_SendClientQueryReq(pusrdata,length);
}
/*************************************************
* Function: ESP_SendBroadcast
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
LOCAL void ICACHE_FLASH_ATTR
ESP_SendBroadcast(void *arg)
{
//    struct espconn *pespconn = arg;
}
/*************************************************
* Function: ESP_BcInit
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void ICACHE_FLASH_ATTR
ESP_BcInit(void)
{
    user_udp_espconn.type = ESPCONN_UDP;
    user_udp_espconn.proto.udp = (esp_udp *)os_zalloc(sizeof(esp_udp));

    tcp_server_conn.type = ESPCONN_TCP;
    tcp_server_conn.proto.tcp = (esp_tcp *)os_zalloc(sizeof(esp_tcp));
    g_struProtocolController.struCloudConnection.u32Socket = PCT_SERVER_TCP_SOCKET;

    tcp_client_conn.type = ESPCONN_TCP;
    //tcp_client_conn.state = ESPCONN_NONE;
    tcp_client_conn.proto.tcp = (esp_tcp *)os_zalloc(sizeof(esp_tcp));
      
    g_struProtocolController.struClientConnection.u32Socket = PCT_CLIENT_TCP_SOCKET;
    tcp_client_conn.proto.tcp->local_port = ZC_SERVER_PORT;
    
    espconn_regist_connectcb(&tcp_client_conn, ESP_AcceptSuccessed);
    espconn_regist_recvcb(&tcp_client_conn, ESP_RecvFromClient);
    espconn_regist_reconcb(&tcp_client_conn, ESP_ReconClient);
    espconn_regist_disconcb(&tcp_client_conn, ESP_DisconCliet);
    espconn_regist_sentcb(&tcp_client_conn, ESP_SendToClient);

    g_struProtocolController.u16SendBcNum = 0;
    g_struProtocolController.u8MainState = PCT_STATE_INIT;
    g_u32BcSleepCount = 250000;

    return;
}
/*************************************************
* Function: ESP_GotIp
* Description:
* Author: cxy
* Returns:
* Parameter:
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_GotIp(void)
{
	struct ip_info info;
	wifi_get_ip_info(0, &info);
	os_memcpy(user_udp_espconn.proto.udp->local_ip, (u8*)&(info.ip.addr), 4);
	g_u32GloablIp = info.ip.addr;
    ZC_Printf("8266 ip  %d,%d,%d,%d\n",
    		user_udp_espconn.proto.udp->local_ip[0],
    		user_udp_espconn.proto.udp->local_ip[1],
    		user_udp_espconn.proto.udp->local_ip[2],
    		user_udp_espconn.proto.udp->local_ip[3]);
}
/*************************************************
* Function: ESP_UdpBroadcast
* Description:
* Author: cxy
* Returns:
* Parameter:
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_UdpBroadcast(void)
{
    user_udp_espconn.proto.udp->remote_ip[3] = 255;
    user_udp_espconn.proto.udp->remote_ip[2] = 255;
    user_udp_espconn.proto.udp->remote_ip[1] = 255;
    user_udp_espconn.proto.udp->remote_ip[0] = 255;

    user_udp_espconn.proto.udp->local_port = ZC_MOUDLE_PORT;
    user_udp_espconn.proto.udp->remote_port = ZC_MOUDLE_BROADCAST_PORT;  // ESP8266 udp remote port

    espconn_regist_recvcb(&user_udp_espconn, ESP_RecvFromBroadcast); // 收到数据回调
    espconn_regist_sentcb(&user_udp_espconn, ESP_SendBroadcast); // 发送数据成功回调
    
    espconn_create(&user_udp_espconn);   // create udp
}
/*************************************************
* Function: ESP_Init
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
int ICACHE_FLASH_ATTR
ESP_Init(void)
{
	char mac_buf[8];
    char mac_string[16];
	g_u64Domain = ((((u64)((SUB_DOMAIN_ID & 0xff00) >> 8)) << 48) + (((u64)(SUB_DOMAIN_ID & 0xff)) << 56) + (((u64)MAJOR_DOMAIN_ID & 0xff) << 40) + ((((u64)MAJOR_DOMAIN_ID & 0xff00) >> 8) << 32)
	+ ((((u64)MAJOR_DOMAIN_ID & 0xff0000) >> 16) << 24)
	+ ((((u64)MAJOR_DOMAIN_ID & 0xff000000) >> 24) << 16)
	+ ((((u64)MAJOR_DOMAIN_ID & 0xff00000000) >> 32) << 8)
	+ ((((u64)MAJOR_DOMAIN_ID & 0xff0000000000) >> 40) << 0));

    os_printf("ESP Init\n");

    g_struHfAdapter.pfunConnectToCloud = ESP_ConnectToCloud;
    g_struHfAdapter.pfunListenClient = ESP_ListenClient;
    g_struHfAdapter.pfunSendTcpData = ESP_SendTcpData;   
    g_struHfAdapter.pfunUpdate = ESP_FirmwareUpdate;     
    g_struHfAdapter.pfunUpdateFinish = ESP_FirmwareUpdateFinish;
    g_struHfAdapter.pfunSendToMoudle = ESP_SendDataToMoudle;  
    g_struHfAdapter.pfunSetTimer = ESP_SetTimer;   
    g_struHfAdapter.pfunStopTimer = ESP_StopTimer;
    g_struHfAdapter.pfunRest = ESP_Rest;
    g_struHfAdapter.pfunWriteFlash = ESP_WriteDataToFlash;
    g_struHfAdapter.pfunSendUdpData = ESP_SendUdpData;   
    g_struHfAdapter.pfunGetMac = ESP_GetMac;
    g_struHfAdapter.pfunReboot = ESP_Reboot;

    g_u16TcpMss = 1000;

    PCT_Init(&g_struHfAdapter);

    g_struUartBuffer.u32Status = MSG_BUFFER_IDLE;
    g_struUartBuffer.u32RecvLen = 0;
    ZC_Printf("u32SecSwitch %d\n",g_struZcConfigDb.struSwitchInfo.u32SecSwitch);
    //ESP_ReadDataFormFlash();
    ESP_BcInit();

    os_memset(g_u8DeviceId, '0', 16);
    os_memset(mac_string, '\0', 12);
    
    wifi_get_macaddr(STATION_IF, mac_buf);
    mac_buf[6] = spi_flash_get_id();
    mac_buf[7] = spi_flash_get_id() >> 8;
    ZCHEX_Printf(mac_buf,8);
    ZC_HexToString(mac_string, mac_buf, 8);
    os_memcpy(g_u8DeviceId, mac_string, ZC_HS_DEVICE_ID_LEN);

    os_memcpy(g_struRegisterInfo.u8PrivateKey, g_u8ModuleKey, ZC_MODULE_KEY_LEN);
    os_memcpy(g_struRegisterInfo.u8DeviciId, g_u8DeviceId, ZC_HS_DEVICE_ID_LEN);
    os_memcpy(g_struRegisterInfo.u8DeviciId + ZC_HS_DEVICE_ID_LEN, &g_u64Domain, ZC_DOMAIN_LEN);
    os_memcpy(g_struRegisterInfo.u8EqVersion, g_u8EqVersion, ZC_EQVERSION_LEN);
    
    ESP_CreateTaskTimer();
    return 1;

}

/*************************************************
* Function: ESP_WakeUp
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_WakeUp()
{
    PCT_WakeUp();
}
/*************************************************
* Function: ESP_Sleep
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR
ESP_Sleep()
{
    u32 u32Index;
    PCT_Sleep();
    
    g_struUartBuffer.u32Status = MSG_BUFFER_IDLE;
    g_struUartBuffer.u32RecvLen = 0;
}

/*************************************************
* Function: ESP_Cloudfunc
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
static void ICACHE_FLASH_ATTR
ESP_Cloudfunc(void)
{
    u32 u32Timer;
    //disarm timer first
    os_timer_disarm(&task_timer);

	if (TI_IS_IP_ACQUIRED(g_ulStatus))
    {
        ZC_Printf("ESP_Cloudfunc ip aquired\n");
        ESP_GotIp();
        ESP_WakeUp();
    	ESP_UdpBroadcast();
        CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);
    }
    if (TI_IS_DISCONNECTED(g_ulStatus))
    {
        ZC_Printf("ESP_Cloudfunc dis\n");
        ESP_Sleep();
        CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_DISCONNECTED);
    }

    PCT_Run();
    
    if (PCT_STATE_DISCONNECT_CLOUD == g_struProtocolController.u8MainState)
    {
        if (0 == g_struProtocolController.struCloudConnection.u32ConnectionTimes)
        {
            u32Timer = 1000;
        }
        else
        {
            u32Timer = rand();
            u32Timer = (PCT_TIMER_INTERVAL_RECONNECT) * (u32Timer % 10 + 1);
        }
        PCT_ReconnectCloud(&g_struProtocolController, u32Timer);
        g_struUartBuffer.u32Status = MSG_BUFFER_IDLE;
        g_struUartBuffer.u32RecvLen = 0;
    }
    else
    {
        MSG_SendDataToCloud((u8*)&g_struProtocolController.struCloudConnection);
    }
    ZC_SendBc();
    os_timer_setfn(&task_timer, (os_timer_func_t *)ESP_Cloudfunc, NULL);
    os_timer_arm(&task_timer, 100, 0);

}
/*************************************************
* Function: ESP_CreateTaskTimer
* Description:
* Author: cxy
* Returns:
* Parameter:
* History:
*************************************************/
void ESP_CreateTaskTimer(void)
{
    os_timer_disarm(&task_timer);
    os_timer_setfn(&task_timer, (os_timer_func_t *)ESP_Cloudfunc, NULL);
    os_timer_arm(&task_timer, 100, 0);

}
/******************************* FILE END ***********************************/


