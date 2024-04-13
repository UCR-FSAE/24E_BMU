/*
 *  @file bq79616.h
 *
 *  @author Vince Toledo - Texas Instruments Inc.
 *  @date 20-April-2020
 *  @version 1.0 beta version
 *  @note Built with CCS for Hercules Version: 8.1.0.00011
 *  @note Built for TMS570LS1224 (LAUNCH XL2)
 */

/*****************************************************************************
**
**  Copyright (c) 2011-2017 Texas Instruments
**
******************************************************************************/

#ifndef BQ79616_H_
#define BQ79616_H_

#include "datatypes.h"
// #include "main.h"
//#include <stdio.h>
#include "stm32f7xx_hal.h"
// #include "hal_stdtypes.h"
// #include "spi.h"

//****************************************************
// ***Register defines, choose one of the following***
// ***based on your device silicon revision:       ***
//****************************************************
// #include "A0_reg.h"
#include "B0_reg.h"

// USER DEFINES
#define TOTALBOARDS 2     // boards in stack, including base device
#define ACTIVECHANNELS 16 // channels to activate
#define BRIDGEDEVICE 1    //
#define MAXBYTES (16 * 2) // maximum number of bytes to be read from the devices (for array creation)
#define BAUDRATE 1000000  // device + uC baudrate

#define FRMWRT_SGL_R 0x00     // single device READ
#define FRMWRT_SGL_W 0x10     // single device WRITE
#define FRMWRT_STK_R 0x20     // stack READ
#define FRMWRT_STK_W 0x30     // stack WRITE
#define FRMWRT_ALL_R 0x40     // broadcast READ
#define FRMWRT_ALL_W 0x50     // broadcast WRITE
#define FRMWRT_REV_ALL_W 0x60 // broadcast WRITE reverse direction

// FUNCTION PROTOTYPES
void SpiAutoAddress();

int SpiWriteReg(BYTE bID, uint16_t wAddr, uint64_t dwData, BYTE bLen, BYTE bWriteType);
int SpiWriteFrame(uint16_t bID, uint16_t wAddr, uint16_t *pData, uint16_t bLen, uint8_t bWriteType);
int SpiReadReg(BYTE bID, uint16_t wAddr, uint16_t *pData, BYTE bLen, uint32_t dwTimeOut, BYTE bWriteType);

uint32_t SpiCRC16(uint16_t *pBuf, int nLen);

void delayus(uint16_t us);
void delayms(uint16_t ms);

void SpiDisableTimeout_600_616(void);

float Complement(uint16_t rawData, float multiplier);
BOOL GetFaultStat();
uint16_t volt2Byte(float volt);

//unsigned printConsole(const char *_format, ...);

// SPI variables
// spiDAT1_t dataconfig1_t; // FROM SL: Take out?
extern uint16_t FFBuffer[128];

#endif /* BQ79616_H_ */
// EOF
