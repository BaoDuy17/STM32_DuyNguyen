/*
 * main.c
 *
 *  Created on: Mar 23, 2025
 *      Author: Dell
 */
#include "main.h"
#include <string.h>
#define GPIOD_BASE_ADDR  0x40020C00
int cnt = 1000;
void LedInit()
{
	__HAL_RCC_GPIOD_CLK_ENABLE();
	uint32_t* GPIOD_MODER = (uint32_t*)(GPIOD_BASE_ADDR + 0x00);
	*GPIOD_MODER &= ~(0xff << 24);
	*GPIOD_MODER |= (0b01 << 24)| (0b01 << 26)|(0b01 << 28)|(0b01 << 30);
}
#define GPIOA_BASE_ADDR  0x40020000

void ButtonInit()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
	*GPIOA_MODER &= ~(0b11 << 0);
}
typedef enum
{
	LED_GREEN = 12,
	LED_ORANGE,
	LED_RED,
	LED_BLUE
}LED_t;

void LedCtrl(LED_t led, int on_off)
{
#if 0
	uint32_t* GPIOD_ODR = (uint32_t*)(GPIOD_BASE_ADDR + 0x14);
	if(on_off == 1)
	{
		*GPIOD_ODR |= (1 << led);
	}
	else
	{
		*GPIOD_ODR &= ~(1<<led);
	}
#else
	uint32_t* GPIOD_BSRR = (uint32_t*)(GPIOD_BASE_ADDR + 0x18);
	if(on_off == 1)
	{
		*GPIOD_BSRR |= (1<<led);
	}
	else
	{
		*GPIOD_BSRR |= (1<<(led+16));
	}
#endif
}

char ButtonState()
{
	uint32_t* GPIOA_IDR = (uint32_t*)(GPIOA_BASE_ADDR + 0x10);
	return (*GPIOA_IDR >> 0) & 1;
}

void function()
{
	if(ButtonState())
		LedCtrl(LED_RED, 1);
	else
		LedCtrl(LED_RED, 0);
}

#define EXTI_BASE_ADDR	0x40013C00
void EXTI0Init()
{
	/*
	- Làm sao để EXTI0 gửi interrupt signal lên ARM?
		+ chọn cạnh (rising/falling/cả hai)
			+ set trong thanh ghi EXTI_RTSR và EXTI_FTSR
		+ enable exti0(set mark)
			+ set trong thanh ghi EXTI_IMR
	- ARM (NVIC) phải chấp nhận interrupt signal từ EXTI gửi lên?
		+ bước 1: xác định EXTI0 nằm ở position bao nhiêu trong vector table? (mở vector table ở chapter "10: interrupts and events" trong reference manual) --> 6
		+ bước 2: enable interrupt cho position 6
	*/
	uint32_t* EXTI_RTSR = (uint32_t*)(EXTI_BASE_ADDR + 0x08);
	*EXTI_RTSR |= (1<<0);
	uint32_t* EXTI_FTSR = (uint32_t*)(EXTI_BASE_ADDR + 0x0C);
	*EXTI_FTSR |= (1<<0);
	uint32_t* EXTI_IMR = (uint32_t*)(EXTI_BASE_ADDR + 0x00);
	*EXTI_IMR |= (1<<0);

	uint32_t* NVIC_ISER0 = (uint32_t*)0xE000E100;
	*NVIC_ISER0 |= 1<<6;

	// Move vector table lên RAM (0x20000000)
	uint8_t* src = 0;
	uint8_t* dis = (uint8_t*)0x20000000;
	for(int i = 0; i < 0x198; i++)
	{
		//dis[i] = src[i];
		*(dis+i) = *(src+i);
	}
	//Báo ARM vector table đã được offset lên RAM
	uint32_t* VTOR = (uint32_t*)0xE000ED08;
	*VTOR = 0x20000000;
	//
	int* ptr;
	ptr= (int*)0x20000058;
	*ptr = (int)function;


}

void EXTI0_IRQHandler()
{
	if(ButtonState())
		LedCtrl(LED_RED, 1);
	else
		LedCtrl(LED_RED, 0);

	//clear interrupt flag
	uint32_t* EXTI_PR = (uint32_t*)(EXTI_BASE_ADDR + 0x14);
	*EXTI_PR |= (1<<0);
}

#define UART1_BASE_ADDR	0x40011000
#define GPIOB_BASE_ADDR	0x40020400
void UART_Init()
{

	/* 	CONFIG GPIOB
		set PB6 as UART1_Tx, PB7 as UART1_Rx
		PB6 alternate function 07
		PB7 alternate function 07
	*/
	__HAL_RCC_GPIOB_CLK_ENABLE();
	uint32_t* GPIOB_MODER = (uint32_t*)(GPIOB_BASE_ADDR + 0x00);
	*GPIOB_MODER &= ~(0b1111 << 12);
	*GPIOB_MODER |= (0b10 << 12) | (0b10 << 14);

	uint32_t* GPIOB_AFLR = (uint32_t*)(GPIOB_BASE_ADDR + 0x20);
	*GPIOB_AFLR &= ~(0xff << 24);
	*GPIOB_AFLR |= (0b0111 << 24) | (0b0111 << 28);

	/* 	CONFIG UART
		- set baud rate: 9600 (bps)
		- data frame:
			+ data size: 8bit	-> CR1 bit 12 M =
			+ parity: even		-> CR1 bit 10 PCE = 1 (parity control enable)
						-> CR1 bit 9  PS = 0 (parity selection)
		- Enable transmitter, receiver	-> CR1 bit 3 TE, bit 2 RE
		- Enable UART			-> CR1 bit 13 UE
	*/
	__HAL_RCC_USART1_CLK_ENABLE();	// 16Mhz
	uint32_t* BRR = (uint32_t*)(UART1_BASE_ADDR + 0x08);
	*BRR = (104 << 4) | (3 << 0);

	uint32_t* CR1 = (uint32_t*)(UART1_BASE_ADDR + 0x0C);
	*CR1 |= (1 << 12) | (1 << 10) | (1 << 3) | (1 << 2) | (1 << 13);

	/* enable interrupt */
	*CR1 |= (1 << 5);
	uint32_t* NVIC_ISER1 = (uint32_t*)0xE000E104;
	*NVIC_ISER1 |= 1<<5;
}



void UART_Transmit(uint8_t data)
{
	uint32_t* DR = (uint32_t*)(UART1_BASE_ADDR + 0x04);
	uint32_t* SR = (uint32_t*)(UART1_BASE_ADDR + 0x00);
	while(((*SR >> 7) & 1) == 0);
	*DR = data;
	while(((*SR >> 6) & 1) == 0);
}

void UART_Print_Log(char* msg)
{
	int msg_len = strlen(msg);
	for(int i = 0; i < msg_len; i++)
	{
		UART_Transmit((uint8_t)msg[i]);
	}
}

char UART_Receive()
{
	uint32_t* DR = (uint32_t*)(UART1_BASE_ADDR + 0x04);
	uint32_t* SR = (uint32_t*)(UART1_BASE_ADDR + 0x00);
	while(((*SR >> 5) & 1) == 0);
	char data = *DR;
	return data;
}

char storeData[100];
int idx;

void USART1_IRQHandler()
{
	storeData[idx++] = UART_Receive();
	if(strstr(storeData, "\n"))
	{
		if(strstr(storeData, "blue led on"))
		{
			LedCtrl(LED_BLUE, 1);
			UART_Print_Log("--> ON LED OK\n");
		}
		else if(strstr(storeData, "blue led off"))
		{
			LedCtrl(LED_BLUE, 0);
			UART_Print_Log("--> OFF LED OK\n");
		}
		else if(strstr(storeData, "red led on"))
		{
			LedCtrl(LED_RED, 1);
			UART_Print_Log("--> OFF LED OK\n");
		}
		else if(strstr(storeData, "red led off"))
		{
			LedCtrl(LED_RED, 0);
			UART_Print_Log("--> OFF LED OK\n");
		}
		else
		{
			UART_Print_Log("--> COMMAND NOT FOUND\n");
		}

		memset(storeData, 0,  sizeof(storeData));
		idx = 0;
	}
}

//25/05/2025
#define I2C_BASE_ADDR  0x40005400
void I2C_Init()
{
	//Set mode Alternative Function
__HAL_RCC_GPIOB_CLK_ENABLE();
uint32_t* GPIOB_MODER = (uint32_t*)(GPIOB_BASE_ADDR + 0x00);
*GPIOB_MODER &=~ (0b1111 << 12);
*GPIOB_MODER |= (0b10 << 12);

*GPIOB_MODER &=~ (0b1111 << 18);
*GPIOB_MODER |= (0b10 << 18);

uint32_t* GPIOB_AFRL = (uint32_t*)(GPIOB_BASE_ADDR + 0x20);
*GPIOB_AFRL &=~ (0b1111 << 24);
*GPIOB_AFRL |= (0b0100 << 24);

uint32_t* GPIOB_AFRH = (uint32_t*)(GPIOB_BASE_ADDR + 0x24);
*GPIOB_AFRH &=~ (0b1111 << 4);
*GPIOB_AFRH |= (0b0100 << 4);

//Generate Clock 16Mhz
__HAL_RCC_I2C1_CLK_ENABLE();

uint32_t* I2C_CR2 = (uint32_t*)(I2C_BASE_ADDR + 0x04);
	*I2C_CR2 &=~ (0b11111 << 0);
	*I2C_CR2 |= (16 << 0);

uint32_t* I2C_CCR = (uint32_t*)(I2C_BASE_ADDR + 0x1C);
*I2C_CCR &=~ (0xFFF << 0);
*I2C_CCR |= (160 << 0);

//ENABLE I2C
uint32_t* I2C_CR1 = (uint32_t*)(I2C_BASE_ADDR + 0x00);
*I2C_CR1 |= (1 << 0);
}

void I2C_Write_Data (uint8_t slave_reg_addr, uint8_t slave_reg_val)
{
	//Generate Start Bit
	uint32_t* I2C_CR1 = (uint32_t*)(I2C_BASE_ADDR + 0x00);
	*I2C_CR1 |= (1 << 8);

	// Wait SB in SR1
	uint32_t* I2C_SR1 = (uint32_t*)(I2C_BASE_ADDR + 0x14);
	while (((*I2C_SR1 >> 0) & 1) == 0); // Formula Waiting bit
	uint32_t* I2C_DATA = (uint32_t*)(I2C_BASE_ADDR + 0x10);

	// Wait ADDR in SR1
	while (((*I2C_SR1 >> 1) & 1) == 0);
	uint32_t* I2C_SR2 = (uint32_t*)(I2C_BASE_ADDR + 0x18);
	uint32_t* temp  = *I2C_SR2;

	//Check ACK
	while (((*I2C_SR1 >> 10) & 1)== 1);

	//send command frame
	*I2C_DATA = slave_reg_addr; //address slave
	while (((*I2C_SR1 >> 2) & 1) == 1);

	//Check ACK
	while (((*I2C_SR1 >> 10) & 1)== 1);

	//send data
	*I2C_DATA = 0b11000000; //address slave
	while (((*I2C_SR1 >> 2) & 1) == 1);

	//Generate Stop bit
	*I2C_CR1 |= (1 << 9);
}

void I2C_Read_Data(uint8_t slave_reg_addr)
{
	uint32_t* I2C_CR1 = (uint32_t*)(I2C_BASE_ADDR + 0x00);
		*I2C_CR1 |= (1 << 8);

		// Wait Start Bit in SR1
		uint32_t* I2C_SR1 = (uint32_t*)(I2C_BASE_ADDR + 0x14);
		while (((*I2C_SR1 >> 0) & 1) == 0); // Formula Waiting bit
		uint32_t* I2C_DATA = (uint32_t*)(I2C_BASE_ADDR + 0x10);

		// Wait ADDR in SR1
		while (((*I2C_SR1 >> 1) & 1) == 0);
		uint32_t* I2C_SR2 = (uint32_t*)(I2C_BASE_ADDR + 0x18);
		uint32_t* temp  = *I2C_SR2;

		//Check ACK
		while (((*I2C_SR1 >> 10) & 1)== 1);

		//send command frame
		*I2C_DATA = slave_reg_addr; //address slave
		while (((*I2C_SR1 >> 2) & 1) == 1);

		//Check ACK
		while (((*I2C_SR1 >> 10) & 1)== 1);

		//Generate Repeat Start Bit
		*I2C_CR1 |= (1 << 8);

		// Wait Start Bit in SR1
		while (((*I2C_SR1 >> 0) & 1) == 0); // Formula Waiting bit

		//send data
		*I2C_DATA = 0b11000000; //address slave
		while (((*I2C_SR1 >> 1) & 1) == 1);
		temp  = *I2C_SR2;

		//Check ACK
		while (((*I2C_SR1 >> 10) & 1)== 1);

		//READ DATA
		while (((*I2C_SR1 >> 6) & 1) == 0);
		uint32_t* data = *I2C_DATA;

		//Generate Stop bit
		*I2C_CR1 |= (1 << 9);
		return data;
}

//SPI
#define SPI_BASE_ADDR 0x40013000
#define GPIOE_BASE_ADDR 0x40021000
void SPI_Init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
	*GPIOA_MODER &=~ (0b1111 << 10);
	*GPIOA_MODER |= (0b10 << 10);

	*GPIOA_MODER &=~ (0b1111 << 12);
	*GPIOA_MODER |= (0b10 << 12);

	*GPIOA_MODER &=~ (0b1111 << 14);
	*GPIOA_MODER |= (0b10 << 14);

// set alternate function
	uint32_t* GPIOA_AFRL = (uint32_t*)(GPIOA_BASE_ADDR + 0x20);
	*GPIOA_AFRL &=~ (0b11 << 20); //PA5 CLK
	*GPIOA_AFRL |= (0b0101 << 20);

	*GPIOA_AFRL &=~ (0b1111 << 24); //PA6 MISO
	*GPIOA_AFRL |= (0b0101 << 24);

	*GPIOA_AFRL &=~ (0b1111 << 28); //PA7 MOSI
	*GPIOA_AFRL |= (0b0101 << 28);

	uint32_t* GPIOE_MODER = (uint32_t*)(GPIOE_BASE_ADDR + 0x00);
	*GPIOE_MODER &=~ (0b1111 << 6); // PE3 Output
	*GPIOE_MODER |= (0b01 << 6);


	__HAL_RCC_SPI1_CLK_ENABLE();
	// Configuration STM32 Master
	uint32_t* SPI_CR1 = (uint32_t*)(SPI_BASE_ADDR + 0x00);
	*SPI_CR1 &=~ (01111 << 2);
	*SPI_CR1 |= (1 << 2);

	// Use software slave management
	*SPI_CR1 &=~ (0b1111 << 9);
	*SPI_CR1 |= (1 << 9);

	//Set Clock
	*SPI_CR1 &=~ (0b1111 << 3);
	*SPI_CR1 |= (0b011 << 3); //1MHz

	// ENABLE SPI
	*SPI_CR1 &=~ (0b1111 << 6);
	*SPI_CR1 |= (1 << 6);
}

char Read_SPI (char reg_add_slave)
{
	// Set PE3 (CS) to low -> active slave
	uint32_t* GPIOE_ODR = (uint32_t*)(GPIOE_BASE_ADDR + 0x14);
	*GPIOE_ODR &=~ (1 << 3);

	//Send register address of slave (write register address to DR)
	uint32_t* SPI_DR = (uint32_t*)(SPI_BASE_ADDR + 0x0C);
	*SPI_DR |= reg_add_slave;
	uint32_t* SPI_SR = (uint32_t*)(SPI_BASE_ADDR + 0x08);
	while (((*SPI_SR>> 7) & 1) == 1);
	// Wait until TXE
	while (((*SPI_SR >> 1) & 1) == 0); //đợi rỗng để ghi dữ liệu vào
	// Wait until RXNE
	while (((*SPI_SR >> 0) & 1) == 1); // Đọc dữ liệu từ buffer
	//Read DR -> Clear Garbage data
	*SPI_DR = 0xFF;
	//Set PE3 (CS) to HIGH -> inactive slave
	*GPIOE_ODR |= (1 << 3);
}

int main()
{
	HAL_Init();
	LedInit();
	ButtonInit();
	EXTI0Init();
	UART_Init();
	while(1)
	{
		LedCtrl(LED_BLUE, 1);
		HAL_Delay(1000);
		LedCtrl(LED_BLUE, 0);
		HAL_Delay(1000);
	}
	return 0;
}
