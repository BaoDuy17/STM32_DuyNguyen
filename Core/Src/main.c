#include "main.h"
#define GPIOD_Base_ADDR 0x40020C00
#define GPIOA_Base_ADDR 0x40020000

void LedInit()
{
	__HAL_RCC_GPIOD_CLK_ENABLE();
uint32_t* GPIOD_Moder = (uint32_t*)(GPIOD_Base_ADDR + 0x00);
*GPIOD_Moder &=~(0b11111111 << 24);
*GPIOD_Moder |= (0b01010101 << 24);

}



void buttonInit()
{
__HAL_RCC_GPIOA_CLK_ENABLE();
uint32_t* GPIOA_Moder =(uint32_t*)(GPIOA_Base_ADDR + 0x00);
*GPIOA_Moder &=~(0b11 << 0);
}



void Ledcontrol(int state)
{
	uint32_t* GPIOD_BSRR = (uint32_t*)(GPIOD_Base_ADDR + 0x18);
	if (state)
	{
		*GPIOD_BSRR = (0b1111 << 12); // bat led
	}
	else
	{
		*GPIOD_BSRR = (0b1111 << 28); // tat led
	}

}



int main()
{
	HAL_Init();
	LedInit();
	buttonInit();
	uint32_t* GPIOA_IDR = (uint32_t*)(GPIOA_Base_ADDR + 0x10);
	while(1)
	{
		if (*GPIOA_IDR & (1 << 0))
		{
		 Ledcontrol(1);
		}
		else
		{
		 Ledcontrol(0);
		}
	}
	return (0);
}
