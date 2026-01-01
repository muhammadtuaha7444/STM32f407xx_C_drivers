/*
 * stm32f407xx_driver_gpio.h
 *
 *  Created on: Dec 15, 2025
 *      Author: Muhammad Tuaha
 */

#include "stm32f407xx.h"
#include "stm32f407xx_driver_gpio.h"

void delay(void){

	for(int i = 0; i == 500000; i++);
}
int main()
{
    GPIO_Handler_t GPIOled;
    GPIOled.pGPIOx_Port = GPIOD;
    
    GPIOled.pGPIOxConfig_Port.GPIO_PinNumber = GPIO_PIN12 ;				
	GPIOled.pGPIOxConfig_Port.GPIO_PinMode = GPIO_OP_MODE;               
	GPIOled.pGPIOxConfig_Port.GPIO_PinSpeed = GPIO_OP_SPEEDHIGH;				
	GPIOled.pGPIOxConfig_Port.GPIO_PinPUPD = GPIO_NO_PUPD;				
	GPIOled.pGPIOxConfig_Port.GPIO_PinODType = GPIO_OT_PP;

	GPIO_ClockCtrl(GPIOD, EN);
	GPIO_Init(&GPIOled);

	while(1)
	{
		delay();
		GPIO_TogglePin(GPIOD, GPIO_PIN12);
	}


}
