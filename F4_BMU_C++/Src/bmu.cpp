/*
 * bmu.cpp
 *
 *  Created on: May 15, 2024
 *      Author: aryabhattacharjee
 */

#include "bmu.h"

static volatile unsigned int delayval = 0;

// Wake BQ79600 with a ping
void BMU::Wake79600(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // Set chip select high

	HAL_Delay(2);
	HAL_SPI_DeInit(&hspi1); // Deinit SPI1 while sending ping
	MosiGPIOInit(7); 		// Enable PA7 (Mosi GPIO pin) as GPIO output

	GPIOA->BSRR |= (0b01 << 7);                           // Set MOSI pin high
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Set chip select low
	Delay_us(0.5);

	GPIOA->BSRR |= (0b01 << 23);
	Delay_us(2750);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	MX_SPI1_Init();
}

/*
 * Manually configure
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
