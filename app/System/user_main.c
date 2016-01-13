/******************************************************************************
 * Copyright 2013-2014 Espressif Systems (Wuxi)
 *
 * FileName: user_main.c
 *
 * Description: entry file of user application
 *
 * Modification history:
 *     2014/1/1, v1.0 create this file.
*******************************************************************************/
#include "ets_sys.h"
#include "osapi.h"
#include "uart.h"
#include "user_interface.h"
#include "smartlink.h"
#include "SysLayerInit.h"
#include "zc_hf_adpter.h"

volatile unsigned long  g_ulStatus = 0;

extern u32 Update_Flash_addr;
extern int ESP_Init(void);

/*************************************************
* Function: wifi_handle_event_cb
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void wifi_handle_event_cb(System_Event_t *evt) 
{  
    os_printf("event %x\n", evt->event);  
    switch (evt->event) 
    {      
        case EVENT_STAMODE_CONNECTED:
            os_printf("connect to ssid %s, channel %d\n", 
                                evt->event_info.connected.ssid, 
                                evt->event_info.connected.channel);
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            break;      
        case EVENT_STAMODE_DISCONNECTED:
            os_printf("disconnect from ssid %s, reason %d\n",
                                evt->event_info.disconnected.ssid,
                                evt->event_info.disconnected.reason);
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_DISCONNECTED);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);
            //ESP_Sleep();
            break;      
        case EVENT_STAMODE_AUTHMODE_CHANGE:
            os_printf("mode: %d -> %d\n",
                            evt->event_info.auth_change.old_mode,
                            evt->event_info.auth_change.new_mode);
            break;      
        case EVENT_STAMODE_GOT_IP:
            os_printf("Got ip:" IPSTR ",mask:" IPSTR ",gw:" IPSTR,
                        IP2STR(&evt->event_info.got_ip.ip),
                        IP2STR(&evt->event_info.got_ip.mask), 
                        IP2STR(&evt->event_info.got_ip.gw));
            os_printf("\n");
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);
            os_printf("Got ip finished!\n");
            //ESP_GotIp();
            //ESP_WakeUp();
    		//ESP_UdpBroadcast();
            break;
        default:
            break;
    } 
} 
/*************************************************
* Function: uart_pre_init
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void  uart_pre_init(void)
{
	uart_init(BIT_RATE_115200, BIT_RATE_115200);
    os_printf("\r\n%s from 0x%x \r\n", SYS_VERSION, system_get_userbin_addr());
    os_delay_us(100);
}
/*************************************************
* Function: user_pre_init
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void //ICACHE_FLASH_ATTR 
user_pre_init(void)
{
    //os_printf("Enter user_pre_init\n");
	Sys_LayerInit();
    ESP_Init();
}
/*************************************************
* Function: user_init
* Description: 
* Author: cxy 
* Returns: 
* Parameter: 
* History:
*************************************************/
void user_init(void)
{
	//uart_pre_init();
    //wifi_set_opmode(STATION_MODE);
	uart_init(BIT_RATE_115200, BIT_RATE_115200);
    os_printf("\r\n%s from 0x%x \r\n", SYS_VERSION, system_get_userbin_addr());
    os_delay_us(100);
    wifi_set_event_handler_cb(wifi_handle_event_cb);
    system_init_done_cb(user_pre_init);
}
