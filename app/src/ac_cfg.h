/**
******************************************************************************
* @file     ac_hal.h
* @authors  cxy
* @version  V1.0.0
* @date     10-Sep-2014
* @brief    
******************************************************************************
*/

#ifndef  __AC_CFG_H__ 
#define  __AC_CFG_H__




#define DEVICE_ID "6666666666666666" //必须填写8个字节，不足的前面补0

#define MAJOR_DOMAIN_ID 411//主域id

#define SUB_DOMAIN_ID  703 //子域id

#define DEFAULT_IOT_CLOUD_KEY {\
    0xb1, 0x2c, 0xca, 0x70,\
    0xd6, 0xf8, 0x29, 0x9e,\
    0x15, 0xc7, 0x99, 0xf0,\
    0xe1, 0xcc, 0x2b, 0x6c,\
    0x26, 0xb5, 0xcf, 0x02,\
    0xca, 0xfb, 0xaf, 0x70,\
    0x78, 0x04, 0x8f, 0x00,\
    0x81, 0x8e, 0xf7, 0x79,\
    0x01, 0x00, 0x01\
}

#define DEFAULT_IOT_PRIVATE_KEY {\
    0xC4,0xFC,0xD7,0xC2,\
    0x80,0x01,0x53,0x59,\
    0x3A,0xE6,0x19,0x14,\
    0x1D,0xDE,0x72,0xCF,\
    0x13,0x39,0x1C,0x07,\
    0x94,0x2E,0x01,0x35,\
    0x63,0x5C,0xA5,0x69,\
    0x0A,0x19,0x53,0x07,\
    0xE9,0x07,0x33,0x64,\
    0xC7,0x0E,0x4E,0xED,\
    0xB3,0x70,0xCD,0x80,\
    0x4F,0xE8,0xF1,0xDF,\
    0xD8,0x68,0x1B,0xFB,\
    0x63,0x90,0x8D,0xF2,\
    0xDF,0xE3,0x14,0x0C,\
    0x59,0xE7,0x53,0xD9,\
    0x33,0x7B,0x8A,0xC8,\
    0x36,0x0E,0x1C,0x33,\
    0xF9,0xD3,0xEF,0x22,\
    0xED,0x04,0xAB,0x03,\
    0x5C,0xD5,0x7C,0x7B,\
    0x81,0x20,0xB7,0xE4,\
    0x22,0xAB,0x35,0x24,\
    0x7F,0x44,0xBF,0x31,\
    0x38,0xAD,0x61,0x71,\
    0xAA,0x87,0x6B,0x53,\
    0x18,0x6A,0x16,0x84,\
    0x4F,0xD7,0xC5,0x1E \
}

#endif
/******************************* FILE END ***********************************/

