#include<stm32f407xx.h>
#include<stm32f407xx_driver_gpio.h>

#include <string.h>




void Delay()
{
    for(int i = 0 ; i <= 50000 ; i++)
    {

    }
}

int main()
{
    
    GPIO_Handler_t GpioLed, GpioButt;

    // Garbage values corrupt the registers of not memset()
    memset(&GpioLed, 0, sizeof(GpioLed));
    memset(&GpioButt, 0, sizeof(GpioButt));

    //Enabling Clocks
    GPIO_ClockCtrl(GPIOA, EN);
    GPIO_ClockCtrl(GPIOD, EN);

    GpioButt.pGPIOx_Port = GPIOA;

    GpioButt.pGPIOxConfig_Port.GPIO_PinNumber = GPIO_PIN0;
    GpioButt.pGPIOxConfig_Port.GPIO_PinMode = GPIO_IT_FE; 
    GpioButt.pGPIOxConfig_Port.GPIO_PinSpeed  = GPIO_OP_SPEEDHIGH;
    GpioButt.pGPIOxConfig_Port.GPIO_PinPUPD = GPIO_PD;
    GpioButt.pGPIOxConfig_Port.GPIO_PinODType = 0;
    GpioButt.pGPIOxConfig_Port.GPIO_PinAF = 0 ;

    GPIO_Init(&GpioButt);

    GpioLed.pGPIOx_Port = GPIOD;
    GpioLed.pGPIOxConfig_Port.GPIO_PinNumber = GPIO_PIN12;
    GpioLed.pGPIOxConfig_Port.GPIO_PinMode = GPIO_OP_MODE; 
    GpioLed.pGPIOxConfig_Port.GPIO_PinSpeed  = GPIO_OP_SPEEDHIGH;
    GpioLed.pGPIOxConfig_Port.GPIO_PinPUPD = GPIO_NO_PUPD;
    GpioLed.pGPIOxConfig_Port.GPIO_PinODType = GPIO_OT_PP;
    GpioLed.pGPIOxConfig_Port.GPIO_PinAF = 0 ;

    GPIO_Init(&GpioLed);

    //IRQ Configuration in the Processor
    GPIO_IRQPriorityConfig(IRQNUMBER_EXTI0, PRIORITY_14 );
    GPIO_IRQConfig(IRQNUMBER_EXTI0, EN);

    while(1)
    {

    }
}

void EXTI0_IRQHandler()
{
    Delay();
    GPIO_IRQHandler(GPIO_PIN0);
    GPIO_TogglePin(GPIOD, GPIO_PIN12);
}
