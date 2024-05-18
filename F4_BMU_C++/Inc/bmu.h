/*
 * bmu.h
 *
 *  Created on: May 15, 2024
 *      Author: aryabhattacharjee
 */

#ifndef INC_BMU_H_
#define INC_BMU_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "spi.h"
#include "tim.h"

// Frame format constants | See Software Design reference for source

typedef uint8_t FWRITETYPE;

static const FWRITETYPE FRMWRT_SGL_R = 0x00;     // single device READ
static const FWRITETYPE FRMWRT_SGL_W = 0x10;     // single device WRITE
static const FWRITETYPE FRMWRT_STK_R = 0x20;     // stack READ
static const FWRITETYPE FRMWRT_STK_W = 0X30;     // stack WRITE
static const FWRITETYPE FRMWRT_ALL_R = 0x40;     // broadcast READ
static const FWRITETYPE FRMWRT_ALL_W = 0x50;     // broadcast WRITE
static const FWRITETYPE FRMWRT_REV_ALL_W = 0x60; // broadcast WRITE reverse direction

class BMU{
public:
	// Pings
	void Wake79600(void);
	void ShutDown79600(void);
	void SleepToActive79600(void);

	// Read & Write Functions
	void ClearComms(void);
	size_t WriteFrame(uint8_t deviceID, uint16_t regAddr, uint8_t *data,
			size_t length, FWRITETYPE writeType);

protected:
	void MosiGPIOInit(uint8_t pin);                   // Initialize pin as GPIO Output
	void Delay_us(uint16_t us);                       // Microsecond delay function (for pings)
	uint16_t CRC16(uint8_t *pBuffer, size_t dataLen); // Used for error checking with CRC
													  // CHECK: CRC result is slightly different from SDR
	// Write Frame
	uint8_t spiBuffer[16];
	uint8_t spiFrame[32];
	uint8_t *spiBuffPtr;
	size_t spiPktLength;
	uint16_t spiWordCRC; // Used for Cyclic redundancy check 16

	// Read Register
	uint16_t TxRxBuffer[128]; // Used in HAL_SPI_TransmitReceive (Dummy data)
	uint16_t spiReturn;       // ** ^ Essentially tells peripheral to send 128 bytes of data
	int responseBytes;
	int byteGroups;           // Groups of 128 bytes
	int bytesRemaining;       // Bytes remaining in last group of 128

public:
	BMU(): spiPktLength(0), spiReturn(0){
		spiBuffPtr = spiFrame;
	}
	~BMU();
};



#endif /* INC_BMU_H_ */
