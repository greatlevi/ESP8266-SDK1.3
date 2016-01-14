/*
 * File	: uart.c
 * This file is part of Espressif's AT+ command set program.
 * Copyright (C) 2013 - 2016, Espressif Systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of version 3 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "ets_sys.h"
#include "osapi.h"
#include "uart.h"
#include "osapi.h"
#include "uart_register.h"
#include "user_interface.h"
#include "user_config.h"
#include "zc_hf_adpter.h"


#ifdef Flash_Test
#include "spi_flash.h"
#endif

// UartDev is defined and initialized in rom code.
extern UartDevice    UartDev;
UARTStruct UART0Port;


LOCAL void uart0_rx_intr_handler(void *para);
void uart0_handler(void);

/******************************************************************************
 * FunctionName : uart_config
 * Description  : Internal used function
 *                UART0 used for data TX/RX, RX buffer size is 0x100, interrupt enabled
 *                UART1 just used for debug output
 * Parameters   : uart_no, use UART0 or UART1 defined ahead
 * Returns      : NONE
*******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
uart_config(uint8 uart_no)
{
    if (uart_no == UART1)
    {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_U1TXD_BK); /* used for debug */
    }
    else
    {
        /* rcv_buff size if 0x100 */
        ETS_UART_INTR_ATTACH(uart0_rx_intr_handler,  &(UartDev.rcv_buff));
        PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_U0RTS);
    }

    uart_div_modify(uart_no, UART_CLK_FREQ / (UartDev.baut_rate));

    WRITE_PERI_REG(UART_CONF0(uart_no), UartDev.exist_parity
                 | UartDev.parity
                 | (UartDev.stop_bits << UART_STOP_BIT_NUM_S)
                 | (UartDev.data_bits << UART_BIT_NUM_S));

    //clear rx and tx fifo,not ready
    SET_PERI_REG_MASK(UART_CONF0(uart_no), UART_RXFIFO_RST | UART_TXFIFO_RST);
    CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_RXFIFO_RST | UART_TXFIFO_RST);

    //set rx fifo trigger
//  WRITE_PERI_REG(UART_CONF1(uart_no),
//                 ((UartDev.rcv_buff.TrigLvl & UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S) |
//                 ((96 & UART_TXFIFO_EMPTY_THRHD) << UART_TXFIFO_EMPTY_THRHD_S) |
//                 UART_RX_FLOW_EN);
    if (uart_no == UART0)
    {
        //set rx fifo trigger
        WRITE_PERI_REG(UART_CONF1(uart_no),
                       ((0x10 & UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S) |
                       ((0x10 & UART_RX_FLOW_THRHD) << UART_RX_FLOW_THRHD_S) |
                       UART_RX_FLOW_EN |
                       (0x02 & UART_RX_TOUT_THRHD) << UART_RX_TOUT_THRHD_S |
                       UART_RX_TOUT_EN);
        SET_PERI_REG_MASK(UART_INT_ENA(uart_no), UART_RXFIFO_TOUT_INT_ENA |
                          UART_FRM_ERR_INT_ENA);
    }
    else
    {
        WRITE_PERI_REG(UART_CONF1(uart_no),
                   ((UartDev.rcv_buff.TrigLvl & UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S));
    }

    //clear all interrupt
    WRITE_PERI_REG(UART_INT_CLR(uart_no), 0xffff);
    //enable rx_interrupt
    SET_PERI_REG_MASK(UART_INT_ENA(uart_no), UART_RXFIFO_FULL_INT_ENA);
}

/******************************************************************************
 * FunctionName : uart1_tx_one_char
 * Description  : Internal used function
 *                Use uart1 interface to transfer one char
 * Parameters   : uint8 TxChar - character to tx
 * Returns      : OK
*******************************************************************************/
LOCAL STATUS
uart_tx_one_char(uint8 uart, uint8 TxChar)
{
    while (true)
    {
      uint32 fifo_cnt = READ_PERI_REG(UART_STATUS(uart)) & (UART_TXFIFO_CNT<<UART_TXFIFO_CNT_S);
      if ((fifo_cnt >> UART_TXFIFO_CNT_S & UART_TXFIFO_CNT) < 126) {
        break;
      }
    }

    WRITE_PERI_REG(UART_FIFO(uart) , TxChar);
    return OK;
}

/******************************************************************************
 * FunctionName : uart1_write_char
 * Description  : Internal used function
 *                Do some special deal while tx char is '\r' or '\n'
 * Parameters   : char c - character to tx
 * Returns      : NONE
*******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
uart1_write_char(char c)
{
    if (c == '\n')
    {
        uart_tx_one_char(UART1, '\r');
        uart_tx_one_char(UART1, '\n');
    }
    else if (c == '\r')
    {
    }
    else
    {
        uart_tx_one_char(UART1, c);
    }
}
/******************************************************************************
 * FunctionName : uart0_tx_buffer
 * Description  : use uart0 to transfer buffer
 * Parameters   : uint8 *buf - point to send buffer
 *                uint16 len - buffer len
 * Returns      :
*******************************************************************************/
void ICACHE_FLASH_ATTR
uart0_tx_buffer(uint8 *buf, uint16 len)
{
    uint16 i;

    for (i = 0; i < len; i++)
    {
        uart_tx_one_char(UART0, buf[i]);
    }
}
/******************************************************************************
 * FunctionName : uart0_tx_buffer
 * Description  : use uart0 to transfer buffer
 * Parameters   : uint8 *buf - point to send buffer
 *                uint16 len - buffer len
 * Returns      :
*******************************************************************************/
void ICACHE_FLASH_ATTR
uart1_tx_buffer(uint8 *buf, uint16 len)
{
    uint16 i;

    for (i = 0; i < len; i++)
    {
        uart_tx_one_char(UART1, buf[i]);
    }
}

/******************************************************************************
 * FunctionName : uart0_sendStr
 * Description  : use uart0 to transfer buffer
 * Parameters   : uint8 *buf - point to send buffer
 *                uint16 len - buffer len
 * Returns      :
*******************************************************************************/
void ICACHE_FLASH_ATTR
uart0_sendStr(const char *str)
{
	while(*str)
	{
		uart_tx_one_char(UART0, *str++);
	}
}

/******************************************************************************
 * FunctionName : uart0_rx_intr_handler
 * Description  : Internal used function
 *                UART0 interrupt handler, add self handle code inside
 * Parameters   : void *para - point to ETS_UART_INTR_ATTACH's arg
 * Returns      : NONE
*******************************************************************************/
LOCAL void
uart0_rx_intr_handler(void *para)
{
  /* uart0 and uart1 intr combine togther, when interrupt occur, see reg 0x3ff20020, bit2, bit0 represents
    * uart1 and uart0 respectively
    */
  uint8 uart_no = UART0;//UartDev.buff_uart_no;

  if(UART_FRM_ERR_INT_ST == (READ_PERI_REG(UART_INT_ST(uart_no)) & UART_FRM_ERR_INT_ST))
  {
    os_printf("FRM_ERR\r\n");
    WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_FRM_ERR_INT_CLR);
  }

  if(UART_RXFIFO_FULL_INT_ST == (READ_PERI_REG(UART_INT_ST(uart_no)) & UART_RXFIFO_FULL_INT_ST)
    || UART_RXFIFO_TOUT_INT_ST == (READ_PERI_REG(UART_INT_ST(uart_no)) & UART_RXFIFO_TOUT_INT_ST))
  {
	  uart0_handler();
  }

}

/******************************************************************************
 * FunctionName : uart_init
 * Description  : user interface for init uart
 * Parameters   : UartBautRate uart0_br - uart0 bautrate
 *                UartBautRate uart1_br - uart1 bautrate
 * Returns      : NONE
*******************************************************************************/
void ICACHE_FLASH_ATTR
uart_init(UartBautRate uart0_br, UartBautRate uart1_br)
{
    // rom use 74880 baut_rate, here reinitialize
    UartDev.baut_rate = uart0_br;
    uart_config(UART0);
    UartDev.baut_rate = uart1_br;
    uart_config(UART1);
    ETS_UART_INTR_ENABLE();

    // install uart1 putc callback
    //os_install_putc1((void *)uart1_write_char);
}

/******************************************************************************
 * FunctionName : PrintfHex
 * Description  : PrintfHex
 * Parameters   : UartBautRate uart0_br - uart0 bautrate
 *                UartBautRate uart1_br - uart1 bautrate
 * Returns      : NONE
*******************************************************************************/
void ICACHE_FLASH_ATTR
PrintfHex(u8* data,u16 len)
{
	u16 i;
	for(i=0;i<len;i++)
		os_printf("%02x",*(data+i));
	os_printf(" len:%d\r\n",len);
}
/******************************************************************************
 * FunctionName : uart0_handler
 * Description  : ���ݽ��մ���
 * Parameters   : UartBautRate uart0_br - uart0 bautrate
 *                UartBautRate uart1_br - uart1 bautrate
 * Returns      : NONE
*******************************************************************************/
void ICACHE_FLASH_ATTR
uart0_handler(void)
{
    u16   roomleft = 0;
    PKT_FIFO     *infor;
    PKT_FIFO     *temp_info;
    u8           ch = 0;
    u8           LastCh;
    PKT_DESC     *rx_desc = &(UART0Port.Rx_desc);
    BUFFER_INFO  *rx_ring = &(UART0Port.Rx_Buffer); 
    static u16 AMBodyLen =0;
    static u8  PDMatchNum = 0;
    unsigned long ulStatus; 

    ETS_UART_INTR_DISABLE();

    Buf_GetRoomLeft(rx_ring, roomleft);
	while(READ_PERI_REG(UART_STATUS(UART0)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S))
	{
	    WRITE_PERI_REG(0X60000914, 0x73); //WTD
	    ch = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF;
        os_printf("ch is %02x\n", ch);            //���Դ�ӡ����
        switch (rx_desc->cur_type)
        {
            case PKT_UNKNOWN:
                if (STAND_HEADER == ch)
                {
                    PDMatchNum = 1;
                }
                else
                {
                    PDMatchNum = 0;
                }
                if (HEADER_LEN == PDMatchNum)   /* find header */
                {   
                    rx_desc->cur_num = rx_desc->pkt_num;                  
                    infor = &(rx_desc->infor[rx_desc->cur_num]);
                    infor->pkt_len = 0;
                             
                    rx_desc->cur_type = PKT_PUREDATA;           //match case 2:iwpriv ra0
                    if(roomleft < 8)
                    {
                        rx_desc->cur_type= PKT_UNKNOWN;
                    }
                    else
                    {
                        Buf_Push(rx_ring, STAND_HEADER);
                        roomleft -= 1;
                        infor = &(rx_desc->infor[rx_desc->cur_num]);
                        infor->pkt_len += 1;
                    }                                      
    				
                    PDMatchNum = 0;
                    continue;
                }           
                break;
            
            case PKT_PUREDATA:
                infor = &(rx_desc->infor[rx_desc->cur_num]);
                Buf_Push(rx_ring, ch);
                roomleft--;
                infor->pkt_len++;
                if(infor->pkt_len == AC_PAYLOADLENOFFSET)
                {
                    AMBodyLen = (LastCh << 8) + ch;
                }
                else if(infor->pkt_len == AMBodyLen)
                {
                    //if task has consumed some packets
                    if (rx_desc->cur_num != rx_desc->pkt_num)
                    {   
                        temp_info = infor;
                        infor     = &(rx_desc->infor[rx_desc->pkt_num]);
                        infor->pkt_len = temp_info->pkt_len;
                        temp_info->pkt_len = 0;
                        temp_info->pkt_type = PKT_UNKNOWN;
                    }
                    
                    infor->pkt_type = rx_desc->cur_type;  // PKT_ATCMD / PKT_IWCMD;
                    rx_desc->pkt_num++;
                    rx_desc->cur_type = PKT_UNKNOWN;
                    AMBodyLen = 0;
                }   

                LastCh = ch;
                /*
                * if overflow,we discard the current packet
                * example1:packet length > ring size
                * example2:rx ring buff can no be freed by task as quickly as rx interrupt coming
                */    
                if ((!roomleft) || (rx_desc->pkt_num >= NUM_DESCS))
                {   
                    //rollback
                    Buff_RollBack(rx_ring,infor->pkt_len);
                    
                    roomleft += infor->pkt_len;
                    
                    infor->pkt_type = PKT_UNKNOWN;
                    infor->pkt_len = 0;
                    rx_desc->cur_type = PKT_UNKNOWN;
                    
                    if (rx_desc->pkt_num >= NUM_DESCS)
                    {
                        rx_desc->pkt_num--;
                    }
                    
                }      

            	break;
            default:
            	break;
        }
	}

    if(UART_RXFIFO_FULL_INT_ST == (READ_PERI_REG(UART_INT_ST(UART0)) & UART_RXFIFO_FULL_INT_ST))
    {
        WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_FULL_INT_CLR);
    }
    else if(UART_RXFIFO_TOUT_INT_ST == (READ_PERI_REG(UART_INT_ST(UART0)) & UART_RXFIFO_TOUT_INT_ST))
    {
        WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_TOUT_INT_CLR);
    }
    ETS_UART_INTR_ENABLE();
}



