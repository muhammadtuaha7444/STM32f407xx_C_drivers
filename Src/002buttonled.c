/*
 * stm32f407xx_driver_gpio.h
 *
 *  Created on: Dec 15, 2025
 *      Author: Muhammad Tuaha
 */

#include "stm32f407xx.h"
#include "stm32f407xx_driver_gpio.h"

void delay(void)
{
    for(uint32_t i = 0; i <= 10000; i++);
}

int main()
{
    GPIO_Handler_t GPIObutt;

    GPIObutt.pGPIOx_Port = GPIOA;
	GPIObutt.pGPIOxConfig_Port.GPIO_PinNumber = GPIO_PIN0;				
	GPIObutt.pGPIOxConfig_Port.GPIO_PinMode = GPIO_IN_MODE;               
	GPIObutt.pGPIOxConfig_Port.GPIO_PinSpeed = GPIO_OP_SPEEDHIGH;				
	GPIObutt.pGPIOxConfig_Port.GPIO_PinPUPD = GPIO_NO_PUPD;				
	//GPIObutt.pGPIOxConfig_Port.GPIO_PinODType;				
	//GPIObutt.pGPIOxConfig_Port.GPIO_PinAF;

    GPIO_Handler_t GPIOled;
    GPIOled.pGPIOx_Port = GPIOD;
    GPIOled.pGPIOxConfig_Port.GPIO_PinNumber = GPIO_PIN12;				
	GPIOled.pGPIOxConfig_Port.GPIO_PinMode = GPIO_OP_MODE;               
	GPIOled.pGPIOxConfig_Port.GPIO_PinSpeed = GPIO_OP_SPEEDHIGH;				
	GPIOled.pGPIOxConfig_Port.GPIO_PinPUPD = GPIO_OP_MODE;				
	GPIOled.pGPIOxConfig_Port.GPIO_PinODType = GPIO_OT_PP;				
	//GPIObutt.pGPIOxConfig_Port.GPIO_PinAF;

    GPIO_ClockCtrl(GPIOA, EN);
    GPIO_ClockCtrl(GPIOD, EN);

    GPIO_Init(&GPIObutt);
    GPIO_Init(&GPIOled);

    while(1)
    {
        delay();

        if( GPIO_ReadInputPin(GPIOA, GPIO_PIN0) == 1)
        {
            GPIO_TogglePin(GPIOD, GPIO_PIN12 );
        }
    }
}
