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

#include "user_config.h"
#ifndef MakeUSER2
#include "ets_sys.h"
#include "osapi.h"

#include "user_interface.h"
#include "smartconfig.h"
#include "smartlink.h"
#include "SysLayerInit.h"
// Select smartconfig method.
#define USE_ESPTOUCH 1

LOCAL os_timer_t smartlink_timer;
LOCAL smartlink_success_callback_t smartlink_success_callback_handle = NULL;
LOCAL smartlink_timeout_callback_t smartlink_timeout_callback_handle = NULL;



void ICACHE_FLASH_ATTR
smartlink_timeout(void)
{
    if(smartlink_timeout_callback_handle)
    {
        smartlink_timeout_callback_handle(NULL);
    }
    os_timer_disarm(&smartlink_timer);
}

LOCAL void ICACHE_FLASH_ATTR
smartlink_done(sc_status status, void *pdata)
{
    switch (status) {
    case SC_STATUS_WAIT:
        os_printf("SC_STATUS_WAIT\n");
        break;
    case SC_STATUS_FIND_CHANNEL:
        os_printf("SC_STATUS_FIND_CHANNEL\n");
        break;
    case SC_STATUS_GETTING_SSID_PSWD:
        os_printf("SC_STATUS_GETTING_SSID_PSWD\n");
        break;
    case SC_STATUS_LINK:
        os_printf("SC_STATUS_LINK\n");
        struct station_config *sta_conf = pdata;

        wifi_station_set_config(sta_conf);
        wifi_station_disconnect();
        wifi_station_connect();
        break;
    case SC_STATUS_LINK_OVER:
        os_printf("SC_STATUS_LINK_OVER\n");

        if (pdata != NULL)
        {
            uint8 phone_ip[4] = {0};

            os_memcpy(phone_ip, (uint8*)pdata, 4);
            os_printf("Phone ip: %d.%d.%d.%d\n", phone_ip[0], phone_ip[1], phone_ip[2], phone_ip[3]);

        }
        if (smartlink_success_callback_handle)
        {
            smartlink_success_callback_handle(pdata);
        }
        smartconfig_stop();
        break;
    }

}

void ICACHE_FLASH_ATTR
smartlink_start(void)
{
    wifi_set_opmode(STATION_MODE);

#if defined(USE_ESPTOUCH)
    // USE ESPTOUCH
    smartconfig_start(smartlink_done, 255);
#endif

#if defined(USE_AIRKISS)
    // USE AIRKISS
    smartconfig_start(smartlink_done, 255);
#endif

    os_timer_disarm(&smartlink_timer);
    os_timer_setfn(&smartlink_timer, (os_timer_func_t *)smartlink_timeout, 1);
    os_timer_arm(&smartlink_timer, 1000 * 1000, 1);
}

void ICACHE_FLASH_ATTR
smartlink_stop(void)
{
    os_timer_disarm(&smartlink_timer);

#if defined(USE_ESPTOUCH)
    // USE ESPTOUCH
    smartconfig_stop();
#endif

#if defined(USE_AIRKISS)
    // USE AIRKISS
    smartconfig_stop();
#endif
}

void ICACHE_FLASH_ATTR
smartlink_success_callback_register(smartlink_success_callback_t smartlink_success_callback)
{
    smartlink_success_callback_handle = smartlink_success_callback;
}

void smartlink_timeout_callback_register(smartlink_timeout_callback_t smartlink_timeout_callback)
{
    smartlink_timeout_callback_handle = smartlink_timeout_callback;
}

/******************************************************************************
 * FunctionName : SmartLink
 * Description  : ��������
 * Parameters   :
 * Returns      :
*******************************************************************************/
//extern u8 TcpConnect;
void ICACHE_FLASH_ATTR
SmartLink(void)
{
	wifi_set_opmode(STATION_MODE);
    smartconfig_stop();
	smartconfig_start(smartlink_done);
}
void ICACHE_FLASH_ATTR
SmartLinkInter(void)
{
//	SysTask.m_WifiState = WIFI_SMARTLINK;
//#ifdef Network_Test
//	Le_tcpclient_discon(3);
//#endif
//	SamrtLink();
	ESP_Rest();
}


/******************************************************************************
 * FunctionName : WifiInit
 * Description  : wifi��ʼ��
 * Parameters   :
 * Returns      :
*******************************************************************************/
void ICACHE_FLASH_ATTR
WifiInit(void)
{

	struct station_config s_staconf;

	if(wifi_get_opmode_default() != 1)
	{
		os_printf("not STATION_MODE \r\n");
		wifi_set_opmode(1);//STATION_MODE
	}

	wifi_station_get_config_default(&s_staconf);
	if (os_strlen(s_staconf.ssid))
	{
		wifi_station_set_config(&s_staconf);
		wifi_station_connect();
	} else
	{
		SmartLinkInter();
	}
}

////////////////////////////////////////////////////////////////////////////////////////////IO�ж�
/******************************************************************************
 * FunctionName : InterruptIOHandle
 * Description  : InterruptIO����
 * Parameters   :
 * Returns      :
*******************************************************************************/
void ICACHE_FLASH_ATTR
InterruptIOHandle(void)
{
	uint32 gpio_status;
	gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
	if(gpio_status&BIT4)
	{
	    smartconfig_stop();
	    SmartLinkInter();
	}
	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);
}

/******************************************************************************
 * FunctionName : InterruptIOInit
 * Description  : InterruptIOInit��ʼ��
 * Parameters   :
 * Returns      :
*******************************************************************************/
void ICACHE_FLASH_ATTR
InterruptIOInit(void)
{
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U , FUNC_GPIO4);
	PIN_PULLUP_EN(PERIPHS_IO_MUX_GPIO4_U);
	//���ֵ�Ч������Ϊ�궨��
	gpio_output_set(0, 0, 0, BIT4);
//	GPIO_DIS_OUTPUT(BIT13);
//GPIO_PIN_INTR_POSEDGE=1,GPIO_PIN_INTR_NEGEDGE=2,_ANYEGDE=3,_LOLEVEL=4,_HILEVEL=5
	gpio_pin_intr_state_set(GPIO_ID_PIN(4), 2);
	ETS_GPIO_INTR_ATTACH(InterruptIOHandle, NULL);
	ETS_GPIO_INTR_ENABLE();

}


#endif


