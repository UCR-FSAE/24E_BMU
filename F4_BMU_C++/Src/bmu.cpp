/*
 * bmu.cpp
 *
 *  Created on: May 15, 2024
 *      Author: Arya Bhattacharjee
 *      Derived from: Vince Toledo - TI Inc.
 */

#include "bmu.h"

#include <stdint.h>
#include <string.h>

// PINGS -> SPI1 Mosi pin used to send pings of varied length to indicate certain behaviors

/*
 * @brief Wake up the BQ79600 with a ping through the SPI1 Mosi pin
 */
void BMU::Wake79600(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // Set chip select high

	HAL_Delay(2);
	HAL_SPI_DeInit(&hspi1); // DeInit SPI1 while sending ping
	MosiGPIOInit(7); 		// Enable PA7 (Mosi GPIO pin) as GPIO output

	GPIOA->BSRR |= (0b01 << 7);                           // Set MOSI pin high
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Set chip select low
	Delay_us(2);

	GPIOA->BSRR |= (0b01 << 23); // Set pin PA7 low
	Delay_us(2750);				 // Wake ping (2.5 - 3ms) low active
	GPIOA->BSRR |= (0b01 << 7);  // Set pin PA7 high
	Delay_us(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // Set chip select high
	MX_SPI1_Init(); // Reinitialize SPI1
}

/*
 * @brief Shut down BQ79600 with a ping through the SPI1 Mosi pin
 */
void BMU::ShutDown79600(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // Set chip select high

	HAL_Delay(2);
	HAL_SPI_DeInit(&hspi1); // DeInit SPI1 while sending ping
	MosiGPIOInit(7); 		// Enable PA7 (Mosi GPIO pin) as GPIO output

	GPIOA->BSRR |= (0b01 << 7);                           // Set MOSI pin high
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Set chip select low
	Delay_us(2);

	GPIOA->BSRR |= (0b01 << 23); // Set pin PA7 low
	HAL_Delay(13);		         // Shutdown ping (>12.5ms) low active
	GPIOA->BSRR |= (0b01 << 7);  // Set pin PA7 high
	Delay_us(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // Set chip select high
	MX_SPI1_Init(); // Reinitialize SPI1
}

/*
 * @brief Sleep to active BQ79600 with a ping through SPI1 Mosi pin
 */
void BMU::SleepToActive79600(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // Set chip select high

	HAL_Delay(2);
	HAL_SPI_DeInit(&hspi1); // DeInit SPI1 while sending ping
	MosiGPIOInit(7); 		// Enable PA7 (Mosi GPIO pin) as GPIO output

	GPIOA->BSRR |= (0b01 << 7);                           // Set MOSI pin high
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Set chip select low
	Delay_us(2);

	GPIOA->BSRR |= (0b01 << 23); // Set pin PA7 low
	Delay_us(275);				 // Sleep to Active ping (250 - 300us) low active
	GPIOA->BSRR |= (0b01 << 7);  // Set pin PA7 high
	Delay_us(2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // Set chip select high
	MX_SPI1_Init(); // Reinitialize SPI1
}

// SPI Communication Functions (BQ79600 Specific)

/*
 * SPI Write frame needs
 */

/*
 * @brief Clear SPI communication line by sending 0x00
 */
void BMU::ClearComms(){
	HAL_SPI_Transmit(&hspi1, 0x00, 1, HAL_MAX_DELAY);
}

/*
 * @brief Writes a full frame that is then transmitted using HAL_SPI_Transmit
 *
 * ** READ BQ79600 Software reference: Write Command Frame
 *
 * @param deviceID Address of device to be written to
 * @param regAddr Address of register to be written to
 * @param data Data to be written
 * @param length Length of data
 * @param writeType write type of data (enum)
 *
 * @retval length of transmitted data
 */
size_t BMU::WriteFrame(uint8_t deviceID, uint16_t regAddr, uint8_t *data,
		size_t length, FWRITETYPE writeType){
	/*
	 * Formats a command frame for the BQ79600.
	 * Reduced from source code in order to take up potentially less stack space
	 */

	// Initialize frame buffer
	spiPktLength = 3; // 1 byte for init byte and 2 for regAddr
	spiBuffPtr = spiFrame;
	memset(spiFrame, 0x00, sizeof(spiFrame)); // Clear Frame with 0x00
	// Initialization Byte
	*spiBuffPtr++ = 0x80 | writeType | ((writeType & 0x10) ? length - 1 : 0); // Only include length on a write

	// If writing to single device, include device ID
	if(writeType == FRMWRT_SGL_R || writeType == FRMWRT_SGL_W){
		*spiBuffPtr++ = deviceID; spiPktLength++;
	}

	*spiBuffPtr++ = (regAddr & 0xFF00) >> 8;
	*spiBuffPtr++ = (regAddr & 0x00FF);

	for(uint8_t i = 0; i < length; i++){
		*spiBuffPtr++ = *data++;
	}

	spiPktLength += length; //Update Pkt Length

	spiWordCRC = CRC16(spiFrame, spiPktLength);
	*spiBuffPtr++ = spiWordCRC & 0x00FF;
	*spiBuffPtr++ = (spiWordCRC & 0xFF00) >> 8;

	spiPktLength += 2; // Add length of CRC

	// At this point frame looks something as follows:
	// ** [InitByte, DeviceID(if Single Device Write), RegAddrHi, RegAddrLo, Data Begin, ... , Data End, CRChi, CRClo]

	HAL_SPI_Transmit(&hspi1, spiFrame, spiPktLength, HAL_MAX_DELAY);
	return spiPktLength;
}

// Helper functions

/*
 * @brief Manually configure Mosi pin as a GPIO output pin
 */
void BMU::MosiGPIOInit(uint8_t pin){
	// Enable clock for GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //Enables clock on GPIOA

	// Configure GPIO Pin
	GPIOA->MODER |= (0b01 << pin*2);   // MODER7[1:0]   = 01: Sets to general purpose out
	GPIOA->OTYPER &= ~(0b01 << pin);  // OT7           =  0: Sets to output push-pull
	GPIOA->OSPEEDR |= (0b01 << pin*2); // OSPEEDR7[1:0] = 01: Sets to Medium Speed
	GPIOA->PUPDR &= ~(0b11 << pin*2);  // PUPDR7[1:0]   = 00: Sets to no pull up, pull down
}

void BMU::Delay_us(uint16_t us){
    __HAL_TIM_SET_COUNTER(&htim1, 0); // Set the counter value as 0
    while (__HAL_TIM_GET_COUNTER(&htim1) < us)
        ; // Wait for the counter to reach the us input in the parameter
}

// Cyclic Redundancy Check 16 TABLE
// 		Uses a polynomial hash function to check for potential noise errors in data
// 		ITU_T polynomial: x^16 + x^15 + x^2 + 1
static const uint16_t crc16_table[256] = {0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301,
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

/*
 * @brief Calculate Cyclic Redundancy Check 16 of data. (Used for error checking)
 *
 * @param pBuffer Pointer to data
 * @param dataLen Length of data
 *
 * @retval CRC16 Value
 */
uint16_t BMU::CRC16(uint8_t *pBuffer, size_t dataLen){
	uint16_t wordCRC = 0xFFFF;

	for (uint8_t i = 0; i < dataLen; i++){
		wordCRC ^= (uint16_t)(*pBuffer++) & 0x00FF;
		wordCRC = crc16_table[wordCRC & 0x00FF] ^ (wordCRC >> 8);
	}

	return wordCRC; // *Note for HR: No idea how this actually works tbh.
					// Someone should read up on how this hashing works.
}
