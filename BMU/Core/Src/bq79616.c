/*
 *  @file bq79616.c
 *
 *  @author Vince Toledo - Texas Instruments Inc.
 *  @date 20-April-2020
 *  @version 1.0
 *  @note Built with CCS for Hercules Version: 8.1.0.00011
 */

/*****************************************************************************
 **
 **  Copyright (c) 2011-2017 Texas Instruments
 **
 ******************************************************************************/

/* From SL
    TODO:
    - DONE?Redo spiwritereg to have STM HAL functions
    - Redo spireadreg to have STM HAL functions
    - DONE? Redo spiwriteframe to have STM HAL functions (How do you access things from spi handler from main?)
    - Redo autoaddress function to have STM HAL functions and fit the amount of boards we have
    - DONE Replace all instances of delayms with STM HAL versions
    - DONE Make new version of delayus
    - DONE Change all uints to match STM versions
    - DONE Replace all instances of gioGetBit with STM HAL versions/figure out how pin works with BQ79600 datasheet
    - Get rid of uneccesary functions, variables and includes from TI microcontroller
    - Remake "PINGS" functions
    - DONE Add GPIO for SPI_RDY
*/

#include <bq79616.h>
#include <bq79600.h>
#include "string.h"
// #include "sci.h" // FROM SL: TI functions used in printConsole function, may take out
// #include "spi.h" // FROM SL: TI functions used in SPI functions, may take out
// #include "rti.h" // FROM SL: TI function used in delay functions, may take out
// #include "gio.h" // FROM SL: TI functions used in wake functions, may take out
#include "datatypes.h"
#include "stdarg.h"
#include "main.h"

// GLOBAL VARIABLES (use these to avoid stack overflows by creating too many function variables)
// avoid creating variables/arrays in functions, or you will run out of stack space quickly
uint16_t response_frame2[(MAXBYTES + 6) * TOTALBOARDS]; // response frame to be used by every read
uint16_t fault_frame[39 * TOTALBOARDS];                 // hold fault frame if faults are printed
int currentBoard = 0;
int currentCell = 0;
BYTE bReturn = 0;
int bRes = 0;
int count = 10000;
BYTE bBuf[8];
uint8_t pFrame[64];
static volatile unsigned int delayval = 0; // for delayms and delayus functions
extern int UART_RX_RDY;
extern int RTI_TIMEOUT;
int topFoundBoard = 0;
int baseCommunicating = 0;
int otpPass = 0;
BYTE *currCRC;
int crc_i = 0;
uint16_t wCRC2 = 0xFFFF;
int crc16_i = 0;
uint16_t autoaddr_response_frame[(1 + 6) * TOTALBOARDS]; // response frame for auto-addressing sequence
int numReads = 0;
int channel = 0;

// SpiWriteFrame
uint16_t spiBuf[8];
uint16_t spiFrame[64];
int spiPktLen = 0;
uint16_t *spiPBuf = spiFrame;
uint16_t spiWCRC;

// SpiReadReg
uint16_t spiReturn = 0;
int M = 0; // expected total response bytes
int i = 0; // number of groups of 128 bytes
int K = 0; // number of bytes remaining in the last group of 128

//******
// PINGS
//******

// FROM SL: REDO OR Make alternative of these

void SpiWake79600(void)
{
    spiREG3->PC0 &= ~(uint32_t)((uint32_t)(1U << 10U | 1U << 1U)); // disable MOSI and nCS - now GPIO
    gioToggleBit(spiPORT3, 1U);
    delayus(0.5);
    gioSetBit(spiPORT3, 10U, 0);
    delayus(2750); // WAKE ping = 2.5ms to 3ms
    gioSetBit(spiPORT3, 10U, 1);
    delayus(0.5);
    gioToggleBit(spiPORT3, 1U);
    spiREG3->PC0 |= (uint32_t)((uint32_t)(1U << 10U | 1U << 1U)); // re-enable MOSI and nCS - now SPI
}
void SpiSD79600(void)
{
    spiREG3->PC0 &= ~(uint32_t)((uint32_t)(1U << 10U | 1U << 1U)); // disable MOSI and nCS - now GPIO
    gioToggleBit(spiPORT3, 1U);
    delayus(0.5);
    gioSetBit(spiPORT3, 10U, 0);
    HAL_Delay(13); // Shutdown ping = >12.5ms
    gioSetBit(spiPORT3, 10U, 1);
    delayus(0.5);
    gioToggleBit(spiPORT3, 1U);
    spiREG3->PC0 |= (uint32_t)((uint32_t)(1U << 10U | 1U << 1U)); // re-enable MOSI and nCS - now SPI
}
void SpiStA79600(void)
{
    spiREG3->PC0 &= ~(uint32_t)((uint32_t)(1U << 10U | 1U << 1U)); // disable MOSI and nCS - now GPIO
    gioToggleBit(spiPORT3, 1U);
    delayus(0.5);
    gioSetBit(spiPORT3, 10U, 0);
    delayus(275); // Sleep to Active ping = 250us to 300us
    gioSetBit(spiPORT3, 10U, 1);
    delayus(0.5);
    gioToggleBit(spiPORT3, 1U);
    spiREG3->PC0 |= (uint32_t)((uint32_t)(1U << 10U | 1U << 1U)); // re-enable MOSI and nCS - now SPI
}

void SpiCommClear79600(void)
{
    spiTransmitData(spiREG3, &dataconfig1_t, 1, 0x00);
}
//**********
// END PINGS
//**********

//**********************
// AUTO ADDRESS SEQUENCE
//**********************
void SpiAutoAddress()
{
    // DUMMY WRITE TO SNCHRONIZE ALL DAISY CHAIN DEVICES DLL (IF A DEVICE RESET OCCURED PRIOR TO THIS)
    SpiWriteReg(0, OTP_ECC_DATAIN1, 0X00, 1, FRMWRT_STK_W);
    SpiWriteReg(0, OTP_ECC_DATAIN2, 0X00, 1, FRMWRT_STK_W);
    SpiWriteReg(0, OTP_ECC_DATAIN3, 0X00, 1, FRMWRT_STK_W);
    SpiWriteReg(0, OTP_ECC_DATAIN4, 0X00, 1, FRMWRT_STK_W); // Don't know what to replace "OTP_ECC_DATAINX" with
    SpiWriteReg(0, OTP_ECC_DATAIN5, 0X00, 1, FRMWRT_STK_W);
    SpiWriteReg(0, OTP_ECC_DATAIN6, 0X00, 1, FRMWRT_STK_W);
    SpiWriteReg(0, OTP_ECC_DATAIN7, 0X00, 1, FRMWRT_STK_W);
    SpiWriteReg(0, OTP_ECC_DATAIN8, 0X00, 1, FRMWRT_STK_W);

    // ENABLE AUTO ADDRESSING MODE
    SpiWriteReg(0, CONTROL1, 0X01, 1, FRMWRT_ALL_W);

    // SET ADDRESSES FOR EVERY BOARD
    for (currentBoard = 0; currentBoard < TOTALBOARDS; currentBoard++)
    {
        SpiWriteReg(0, DIR0_ADDR, currentBoard, 1, FRMWRT_ALL_W);
    }

    // BROADCAST WRITE TO SET ALL DEVICES AS STACK DEVICE
    SpiWriteReg(0, COMM_CTRL, 0x02, 1, FRMWRT_ALL_W);

    // SET THE HIGHEST DEVICE IN THE STACK AS BOTH STACK AND TOP OF STACK
    SpiWriteReg(TOTALBOARDS - 1, COMM_CTRL, 0x03, 1, FRMWRT_SGL_W);

    // SYNCRHONIZE THE DLL WITH A THROW-AWAY READ
    SpiReadReg(0, OTP_ECC_DATAIN1, autoaddr_response_frame, 1, 0, FRMWRT_STK_R);
    SpiReadReg(0, OTP_ECC_DATAIN2, autoaddr_response_frame, 1, 0, FRMWRT_STK_R);
    SpiReadReg(0, OTP_ECC_DATAIN3, autoaddr_response_frame, 1, 0, FRMWRT_STK_R);
    SpiReadReg(0, OTP_ECC_DATAIN4, autoaddr_response_frame, 1, 0, FRMWRT_STK_R);
    SpiReadReg(0, OTP_ECC_DATAIN5, autoaddr_response_frame, 1, 0, FRMWRT_STK_R);
    SpiReadReg(0, OTP_ECC_DATAIN6, autoaddr_response_frame, 1, 0, FRMWRT_STK_R);
    SpiReadReg(0, OTP_ECC_DATAIN7, autoaddr_response_frame, 1, 0, FRMWRT_STK_R);
    SpiReadReg(0, OTP_ECC_DATAIN8, autoaddr_response_frame, 1, 0, FRMWRT_STK_R);

    // OPTIONAL: read back all device addresses
    for (currentBoard = 0; currentBoard < TOTALBOARDS; currentBoard++)
    {
        SpiReadReg(currentBoard, DIR0_ADDR, autoaddr_response_frame, 1, 0, FRMWRT_SGL_R);
    }

    // OPTIONAL: read register address 0x2001 and verify that the value is 0x14
    SpiReadReg(0, 0x2001, autoaddr_response_frame, 1, 0, FRMWRT_SGL_R);

    return;
}

//**************************
// END AUTO ADDRESS SEQUENCE
//**************************

//************************
// WRITE AND READ FUNCTIONS
//************************
int SpiWriteReg(BYTE bID, uint16_t wAddr, uint64_t dwData, BYTE bLen, BYTE bWriteType)
{
    // device address, register start address, data bytes, data length, write type (single, broadcast, stack)
    bRes = 0;
    memset(spiBuf, 0, sizeof(spiBuf));
    while (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1) == 0) // FROM SL: IT IS SPI_RDY
        delayus(5);                                  // wait until SPI_RDY is ready
    switch (bLen)
    {
    case 1:
        spiBuf[0] = dwData & 0x00000000000000FF;
        bRes = SpiWriteFrame(bID, wAddr, spiBuf, 1, bWriteType);
        break;
    case 2:
        spiBuf[0] = (dwData & 0x000000000000FF00) >> 8;
        spiBuf[1] = dwData & 0x00000000000000FF;
        bRes = SpiWriteFrame(bID, wAddr, spiBuf, 2, bWriteType);
        break;
    case 3:
        spiBuf[0] = (dwData & 0x0000000000FF0000) >> 16;
        spiBuf[1] = (dwData & 0x000000000000FF00) >> 8;
        spiBuf[2] = dwData & 0x00000000000000FF;
        bRes = SpiWriteFrame(bID, wAddr, spiBuf, 3, bWriteType);
        break;
    case 4:
        spiBuf[0] = (dwData & 0x00000000FF000000) >> 24;
        spiBuf[1] = (dwData & 0x0000000000FF0000) >> 16;
        spiBuf[2] = (dwData & 0x000000000000FF00) >> 8;
        spiBuf[3] = dwData & 0x00000000000000FF;
        bRes = SpiWriteFrame(bID, wAddr, spiBuf, 4, bWriteType);
        break;
    case 5:
        spiBuf[0] = (dwData & 0x000000FF00000000) >> 32;
        spiBuf[1] = (dwData & 0x00000000FF000000) >> 24;
        spiBuf[2] = (dwData & 0x0000000000FF0000) >> 16;
        spiBuf[3] = (dwData & 0x000000000000FF00) >> 8;
        spiBuf[4] = dwData & 0x00000000000000FF;
        bRes = SpiWriteFrame(bID, wAddr, spiBuf, 5, bWriteType);
        break;
    case 6:
        spiBuf[0] = (dwData & 0x0000FF0000000000) >> 40;
        spiBuf[1] = (dwData & 0x000000FF00000000) >> 32;
        spiBuf[2] = (dwData & 0x00000000FF000000) >> 24;
        spiBuf[3] = (dwData & 0x0000000000FF0000) >> 16;
        spiBuf[4] = (dwData & 0x000000000000FF00) >> 8;
        spiBuf[5] = dwData & 0x00000000000000FF;
        bRes = SpiWriteFrame(bID, wAddr, spiBuf, 6, bWriteType);
        break;
    case 7:
        spiBuf[0] = (dwData & 0x00FF000000000000) >> 48;
        spiBuf[1] = (dwData & 0x0000FF0000000000) >> 40;
        spiBuf[2] = (dwData & 0x000000FF00000000) >> 32;
        spiBuf[3] = (dwData & 0x00000000FF000000) >> 24;
        spiBuf[4] = (dwData & 0x0000000000FF0000) >> 16;
        spiBuf[5] = (dwData & 0x000000000000FF00) >> 8;
        spiBuf[6] = dwData & 0x00000000000000FF;
        bRes = SpiWriteFrame(bID, wAddr, spiBuf, 7, bWriteType);
        break;
    case 8:
        spiBuf[0] = (dwData & 0xFF00000000000000) >> 56;
        spiBuf[1] = (dwData & 0x00FF000000000000) >> 48;
        spiBuf[2] = (dwData & 0x0000FF0000000000) >> 40;
        spiBuf[3] = (dwData & 0x000000FF00000000) >> 32;
        spiBuf[4] = (dwData & 0x00000000FF000000) >> 24;
        spiBuf[5] = (dwData & 0x0000000000FF0000) >> 16;
        spiBuf[6] = (dwData & 0x000000000000FF00) >> 8;
        spiBuf[7] = dwData & 0x00000000000000FF;
        bRes = SpiWriteFrame(bID, wAddr, spiBuf, 8, bWriteType);
        break;
    default:
        break;
    }
    return bRes;
}

int SpiWriteFrame(uint16_t bID, uint16_t wAddr, uint16_t *pData, uint16_t bLen, uint8_t bWriteType)
{
    spiPktLen = 0;
    spiPBuf = spiFrame;
    memset(spiFrame, 0x7F, sizeof(spiFrame));
    *spiPBuf++ = 0x80 | (bWriteType) | ((bWriteType & 0x10) ? bLen - 0x01 : 0x00); // Only include blen if it is a write; Writes are 0x90, 0xB0, 0xD0
    if (bWriteType == FRMWRT_SGL_R || bWriteType == FRMWRT_SGL_W)
    {
        *spiPBuf++ = (bID & 0x00FF);
    }
    *spiPBuf++ = (wAddr & 0xFF00) >> 8;
    *spiPBuf++ = wAddr & 0x00FF;

    while (bLen--)
        *spiPBuf++ = *pData++;

    spiPktLen = spiPBuf - spiFrame;

    spiWCRC = SpiCRC16(spiFrame, spiPktLen);
    *spiPBuf++ = spiWCRC & 0x00FF;
    *spiPBuf++ = (spiWCRC & 0xFF00) >> 8;
    spiPktLen += 2;

    // spiTransmitData(spiREG3, &dataconfig1_t, spiPktLen, spiFrame); // Replace all instances of this with STM HAL ver
    HAL_SPI_Transmit(&hspi3, spiFrame, spiPktLen, HAL_MAX_DELAY); // FROM SL: Is spiPktLen in correct units??? Number of bytes?

    return spiPktLen;
}

// GENERATE READ COMMAND FRAME AND THEN WAIT FOR RESPONSE DATA (INTERRUPT MODE FOR SCIRX)
int SpiReadReg(BYTE bID, uint16_t wAddr, uint16_t *pData, BYTE bLen, uint32_t dwTimeOut, BYTE bWriteType)
{
    // device address, register start address, byte frame pointer to store data, data length, read type (single, broadcast, stack)

    bRes = 0; // total bytes received

    while (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1) == 0)
        delayus(1); // wait until SPI_RDY is ready

    // send the read request to the 600
    spiReturn = bLen - 1;
    SpiWriteFrame(bID, wAddr, &spiReturn, 1, bWriteType); // send the read request command frame    // Replace all instances of this with STM HAL ver
    delayus(5 * TOTALBOARDS);                             // wait propagation time for each board

    uint16_t *movingPointer = pData;

    // prepare the correct number of bytes for the device to read
    if (bWriteType == FRMWRT_SGL_R)
    {
        M = bLen + 6;
    }
    else if (bWriteType == FRMWRT_STK_R)
    {
        M = (bLen + 6) * (TOTALBOARDS - 1);
    }
    else if (bWriteType == FRMWRT_ALL_R)
    {
        M = (bLen + 6) * TOTALBOARDS;
    }
    else
    {
        while (1)
            ; // infinite loop to catch error
    }

    // prepare the number of loops of 128-byte reads that need to occur
    i = (int)(M / 128);
    // prepare the remainder that is left over after the last full 128-byte read
    K = M - i * 128;

    // loop until we've read all data bytes
    while (i > (-1))
    {
        while (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1) == 0)
            delayus(100); // wait until SPI_RDY is ready

        // if there is more than 128 bytes remaining
        if (i > 0)
        {
            if (bWriteType == FRMWRT_SGL_R)
            {
                spiTransmitAndReceiveData(spiREG3, &dataconfig1_t, 128, FFBuffer, movingPointer); // Replace all instances of this with STM HAL ver
            }
            else if (bWriteType == FRMWRT_STK_R)
            {
                spiTransmitAndReceiveData(spiREG3, &dataconfig1_t, 128, FFBuffer, movingPointer); // Replace all instances of this with STM HAL ver
            }
            else if (bWriteType == FRMWRT_ALL_R)
            {
                spiTransmitAndReceiveData(spiREG3, &dataconfig1_t, 128, FFBuffer, movingPointer); // Replace all instances of this with STM HAL ver
            }
            movingPointer += 128;
        }

        // else if there is less than 128 bytes remaining
        else
        {
            if (bWriteType == FRMWRT_SGL_R)
            {
                spiTransmitAndReceiveData(spiREG3, &dataconfig1_t, K, FFBuffer, movingPointer); // Replace all instances of this with STM HAL ver
                bRes = bLen + 6;
            }
            else if (bWriteType == FRMWRT_STK_R)
            {
                spiTransmitAndReceiveData(spiREG3, &dataconfig1_t, K, FFBuffer, movingPointer); // Replace all instances of this with STM HAL ver
                bRes = (bLen + 6) * (TOTALBOARDS - 1);
            }
            else if (bWriteType == FRMWRT_ALL_R)
            {
                spiTransmitAndReceiveData(spiREG3, &dataconfig1_t, K, FFBuffer, movingPointer); // Replace all instances of this with STM HAL ver
                bRes = (bLen + 6) * TOTALBOARDS;
            }
        }

        i--; // decrement the number of groups of 128 bytes
    }
    numReads++;
    return bRes;
}

// CRC16 TABLE
// ITU_T polynomial: x^16 + x^15 + x^2 + 1
const uint16_t crc16_table[256] = {0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301,
                                   0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1,
                                   0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81,
                                   0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
                                   0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00,
                                   0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1,
                                   0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641, 0xD201, 0x12C0, 0x1380,
                                   0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 0x3180, 0xF141,
                                   0x3300, 0xF3C1, 0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501,
                                   0x35C0, 0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0,
                                   0x3E80, 0xFE41, 0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881,
                                   0x3840, 0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
                                   0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401,
                                   0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1,
                                   0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041, 0xA001, 0x60C0, 0x6180,
                                   0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740,
                                   0xA501, 0x65C0, 0x6480, 0xA441, 0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01,
                                   0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1,
                                   0xA881, 0x6840, 0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80,
                                   0xBA41, 0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
                                   0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200,
                                   0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1,
                                   0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241, 0x9601, 0x56C0, 0x5780,
                                   0x9741, 0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01, 0x5CC0, 0x5D80, 0x9D41,
                                   0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901,
                                   0x59C0, 0x5880, 0x9841, 0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1,
                                   0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80,
                                   0x8C41, 0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
                                   0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

uint32_t SpiCRC16(uint16_t *pBuf, int nLen)
{
    uint32_t wCRC = 0xFFFF;
    int i;

    for (i = 0; i < nLen; i++)
    {
        wCRC ^= (uint16_t)(*pBuf++) & 0x00FF;
        wCRC = crc16_table[wCRC & 0x00FF] ^ (wCRC >> 8);
    }

    return wCRC;
}
//****************************
// END WRITE AND READ FUNCTIONS
//****************************

//************************
// MISCELLANEOUS FUNCTIONS
//************************
void SpiDisableTimeout_600_616(void)
{
    // Disable timeout 600
    SpiWriteReg(0, 0x2005, 0x00, 1, FRMWRT_SGL_W);
    // Disable timeout 616
    SpiWriteReg(0, COMM_CTRL, 0x00, 1, FRMWRT_STK_W);
}

float Complement(uint16_t rawData, float multiplier)
{
    return -1 * (~rawData + 1) * multiplier;
}

// BOOL GetFaultStat()
// {

//     if (!gioGetBit(gioPORTA, 0))
//         return 0;
//     return 1;
// }

void delayus(uint16_t us) // FROM SL: New version of delayus
{
    __HAL_TIM_SET_COUNTER(&htim1, 0); // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim1) < us)
        ; // wait for the counter to reach the us input in the parameter
}

// void delayus(uint16_t us) // Make new version of this, may need to redo ioc file
// {
//     if (us == 0)
//         return;
//     else
//     {
//         // CHANGE THE INTERRUPT COMPARE VALUES (PERIOD OF INTERRUPT)
//         // Setup compare 0 value.
//         rtiREG1->CMP[0U].COMPx = 10 * us; // 10 ticks of clock per microsecond, so multiply by 10
//         // Setup update compare 0 value.
//         rtiREG1->CMP[0U].UDCPx = 10 * us;

//         // ENABLE THE NOTIFICATION FOR THE PERIOD WE SET
//         rtiEnableNotification(rtiNOTIFICATION_COMPARE0);

//         // START THE COUNTER
//         rtiStartCounter(rtiCOUNTER_BLOCK0);

//         // WAIT IN LOOP UNTIL THE INTERRUPT HAPPENS (HAPPENS AFTER THE PERIOD WE SET)
//         // WHEN INTERRUPT HAPPENS, RTI_NOTIFICATION GETS SET TO 1 IN THAT INTERRUPT
//         // GO TO notification.c -> rtiNotification() to see where RTI_TIMEOUT is set to 1
//         while (RTI_TIMEOUT == 0)
//             ;

//         // RESET THE VARIABLE TO 0, FOR THE NEXT TIME WE DO A DELAY
//         RTI_TIMEOUT = 0;

//         // DISABLE THE INTERRUPT NOTIFICATION
//         rtiDisableNotification(rtiNOTIFICATION_COMPARE0);

//         // STOP THE COUNTER
//         rtiStopCounter(rtiCOUNTER_BLOCK0);

//         // RESET COUNTER FOR THE NEXT TIME WE DO A DELAY
//         rtiResetCounter(rtiCOUNTER_BLOCK0);
//     }
// }

// void delayms(uint16_t ms) // Make new version of this
// {
//     if (ms == 0)
//         return;
//     else
//     {
//         rtiREG1->CMP[0U].COMPx = 10000 * ms;
//         rtiREG1->CMP[0U].UDCPx = 10000 * ms;
//         rtiEnableNotification(rtiNOTIFICATION_COMPARE0);
//         rtiStartCounter(rtiCOUNTER_BLOCK0);
//         while (RTI_TIMEOUT == 0)
//             ;
//         RTI_TIMEOUT = 0;
//         rtiDisableNotification(rtiNOTIFICATION_COMPARE0);
//         rtiStopCounter(rtiCOUNTER_BLOCK0);
//         rtiResetCounter(rtiCOUNTER_BLOCK0);
//     }
// }

uint16_t volt2Byte(float volt) // FROM SL: Take out? May be uneccesary
{
    return (uint16_t) ~((int16_t)((-volt / 0.00019073) - 1.0));
}

unsigned printConsole(const char *_format, ...) // FROM SL: Take out? May be uneccesary
{
    char str[128];
    int length = -1, k = 0;

    va_list argList;
    va_start(argList, _format);

    length = vsnprintf(str, sizeof(str), _format, argList);

    va_end(argList);

    //   if (length > 0)
    //   {
    //      for(k=0; k<length; k++)
    //      {
    //          HetUART1PutChar(str[k]);
    //      }
    //   }
    sciSend(scilinREG, length, str);

    return (unsigned)length;
}

//***************************
// END MISCELLANEOUS FUNCTIONS
//***************************

// EOF
