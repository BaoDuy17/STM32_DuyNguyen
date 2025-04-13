#include "main.h"
#define GPIOD_Base_ADDR 0x40020C00
#define GPIOA_Base_ADDR 0x40020000
#define EXTI_Base_ADDR 0x40013C00
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


typedef enum
{
	LED_GREEN = 12,
	LED_ORANGE,
	LED_RED,
	LED_BLUE
}LED_t;
void Ledcontrol(LED_t led, int on_off)
{
	uint32_t* GPIOD_BSRR = (uint32_t*)(GPIOD_Base_ADDR + 0x18);
	if (on_off == 1)
	{
		*GPIOD_BSRR |= (1 << led); // bat led
	}
	else
	{
		*GPIOD_BSRR |= (1 << (led + 16)); // tat led
	}

}

char Button_state()
{
	uint32_t* GPIOA_IDR = (uint32_t*)(GPIOA_Base_ADDR + 0x10);
	return (*GPIOA_IDR >> 0) & 1;
}

void EXTI0Init()
{
	uint32_t* EXTI_RTSR = (uint32_t*)(EXTI_Base_ADDR + 0x08);
	uint32_t* EXTI_FTST = (uint32_t*)(EXTI_Base_ADDR + 0x0C);
	uint32_t* EXTI_IMR = (uint32_t*)(EXTI_Base_ADDR + 0x00);

	*EXTI_RTSR |= (1 << 0);
	*EXTI_FTST |= (1 << 0);
	*EXTI_IMR |= (1 << 0);

	uint32_t* NVIC_ISER0 = (uint32_t*)0xE000E100;
	*NVIC_ISER0 |= (1 << 6);
}
void EXTI0_IRQHandler ()
{
	if(Button_state())
	{
		Ledcontrol(LED_RED, 1);
	}
	else
	{
		Ledcontrol(LED_RED,0);
	}
	uint32_t* EXTI_PR = (uint32_t*)(EXTI_Base_ADDR + 0x14);
	*EXTI_PR |= (1 << 0); // clear flag interrupt
}
int main()
{
	HAL_Init();
	LedInit();
	buttonInit();
	EXTI0Init();
	while(1)
	{
		Ledcontrol(LED_BLUE,1);
		HAL_Delay(1000);
		Ledcontrol(LED_BLUE,0);
		HAL_Delay(1000);
	}
	return (0);
}
