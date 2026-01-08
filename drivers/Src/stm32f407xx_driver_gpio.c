/*
 * stm32f407xx_driver_gpio.c
 *
 *  Created on: Dec 10, 2025
 *      Author: Muhammad Tuaha Yasin
 */

#include "stm32f407xx.h"
#include "stm32f407xx_driver_gpio.h"



/********************************************************************************
 * 						GPIO APIs Definations are written here
 *******************************************************************************/

/*******************************************************************************
 * @ Function : GPIO_ClockCtrl
 * @ Brief    : Enable or Disable the Clock of the GPIO Port
 * @ Para[1]  :	Pointer to the GPIO Register struct(GPIO base address)
 * @ Para[1]  : Enable or Disable Macro
 * @ Note	  :
 */
void GPIO_ClockCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnDis)
{
	if(EnDis == EN)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}

}

/*******************************************************************************
 * @ Function : GPIO_Init
 * @ Brief    : This Function Configure the GPIO port using the GPIO handler
 * @ Para[1]  : Pointer to GPIO Handler Struct(GPIO base address)
 * @ Note	  :
 */
void GPIO_Init(GPIO_Handler_t *pGPIO_Handle)
{
	uint32_t temp = 0;
	/* GPIOx Mode Configuration of the port*/
	if (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinMode <= GPIO_AN_MODE)
	{
		temp = ((pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinMode) << (2 * (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinNumber)));
		pGPIO_Handle->pGPIOx_Port->MODER &= ~(0x3 << pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinNumber); // clearing before setting
		(pGPIO_Handle->pGPIOx_Port->MODER) |= temp; //Setting 
		temp = 0;


	}else if (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinMode > GPIO_AN_MODE) // GPIO Interrupt Logic is here
	{
		//1. Configure the edge trigger
		if(pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinMode == GPIO_IT_FE)
		{
			
			EXTI->FTSR |= (1 << (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinNumber));
			EXTI->RTSR &= ~(1 << (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinNumber));

		}else if(pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinMode == GPIO_IT_RE)
		{
			EXTI->RTSR |= (1 << (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinNumber));
			EXTI->FTSR &= ~(1 << (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinNumber));

		}else if(pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinMode == GPIO_IT_RFE)
		{
			EXTI->RTSR |= (1 << (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinNumber));
			EXTI->FTSR |= (1 << (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinNumber));
		}
		//2. Configure the GPIO port selection in SYS CFGR
		SYSCFG_PCLK_EN();
		uint8_t temp1 = (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinNumber) / 4;
		uint8_t temp2 = (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinNumber) % 4;
		uint8_t gpio_port_code = GPIOPORTCODE(pGPIO_Handle->pGPIOx_Port);
		SYSCONFIG->EXTICR[temp1] |= gpio_port_code << ( temp2 * 4); 

		//3. Configure the interrupt EXTI interrupt delivery using IMR
			EXTI->IMR |=  (1 << (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinNumber));
	}


	/*Configure the output type of the pin*/
	if(pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinMode == GPIO_OP_MODE)
	{
		temp = (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinODType) << (1 * (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx_Port->OTYPER &= ~(1 << pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinNumber); // clearing before setting
		(pGPIO_Handle->pGPIOx_Port->OTYPER) |= temp;
		temp = 0;
	}

	/*Configure the pull up or pull down of the pin*/
	temp = (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinPUPD) << (2 * (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx_Port->PUPDR &= ~(0x3 << pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinNumber); // clearing before setting
	pGPIO_Handle->pGPIOx_Port->PUPDR |= temp;
	temp = 0;

	/*Configure the Output speed of the pin*/
	temp = (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinSpeed) << (2 * (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinSpeed));
	pGPIO_Handle->pGPIOx_Port->OSPEEDR &= ~(0x3 << pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinNumber); // clearing before setting
	pGPIO_Handle->pGPIOx_Port->OSPEEDR |= temp;
	temp = 0;
	
	/*Configre the alternate pin functionality of the pin*/
	if(pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinMode == GPIO_AF_MODE)
	{
		uint32_t temp1, temp2;
		temp1 = (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinNumber) / 8;
		temp2 = (pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinNumber) %  8;

		pGPIO_Handle->pGPIOx_Port->AFR[temp1] &=  ~(0xF<< (4 * temp2)); 
		pGPIO_Handle->pGPIOx_Port->AFR[temp1] |=  pGPIO_Handle->pGPIOxConfig_Port.GPIO_PinAF<< (4 * temp2);
		temp1 = 0;
		temp2 = 0;
	}

	temp = 0;

}

/*******************************************************************************
 * @ Function : GPIO_DeInit
 * @ Brief    : Reset the register value of the GPIO Port
 * @ Para[1]  : Pointer to GPIO Register Defination struct(GPIO base address)
 * @ Note	  :
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

	if(pGPIOx == GPIOA)
	{
		GPIOA_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOH_RESET();
	}else if(pGPIOx == GPIOI)
	{
		GPIOI_RESET();
	}

}

/*******************************************************************************
 * @ Function :
 * @ Brief    :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Note	  :
 */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *GPIOx, uint8_t GPIOPin )
{
	uint8_t value;
	value = (uint8_t)((GPIOx->IDR >> GPIOPin) & 0x00000001);
	return value;
}
/*******************************************************************************
 * @ Function :
 * @ Brief    :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Note	  :
 */
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *GPIOx)
{
	uint16_t value;
	value = (uint16_t) (GPIOx->IDR);
	return value;

}

/*******************************************************************************
 * @ Function :
 * @ Brief    :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Note	  :
 */
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIOPin, uint8_t value )
{
if(value == SET)
{
	pGPIOx->ODR |= (1 << GPIOPin);
}else if(value == RESET)
{
	pGPIOx->IDR &= ~(1 << GPIOPin);
}

}

/*******************************************************************************
 * @ Function :
 * @ Brief    :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Note	  :
 */
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{

pGPIOx->ODR = value;

}

/*******************************************************************************
 * @ Function :
 * @ Brief    :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Note	  :
 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t GPIOPin)
{

	pGPIOx->ODR ^= (1 << GPIOPin);
}

/*******************************************************************************
 * @ Function :
 * @ Brief    :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Note	  :
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnDis)
{
	if(EnDis == EN)
	{
		if(IRQNumber <= 31)
		{
			// Set the ICRER0 
			*NVIC_ISER0_ADDR	|= (1 << IRQNumber );
		}else if(IRQNumber >= 32 && ( IRQNumber < (2 * 32) ))
		{
			// Set the ICRER1 
			*NVIC_ISER0_ADDR	|= (1 << ( IRQNumber % 32 ) );
		}else if(IRQNumber >= (2 * 32) && ( IRQNumber < (3 * 32)) )
		{
			// Set the ICRER2
			*NVIC_ISER0_ADDR	|= (1 << ( IRQNumber % (2*32) ) ); 
		}

		/* todo: Set the other IRQ Enable of needed in other MCU
		else if(IRQNumber < 31)
		{
			// Set the ICRER0 
		}
		*/
	}


}

/*******************************************************************************
 * @ Function :
 * @ Brief    :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Note	  :
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t IPR_Reg_Select = IRQNumber / 4;
	uint8_t IRP_Number_Byte_Offset = IRQNumber % 4;
	uint8_t Shift_Amount =   (IRP_Number_Byte_Offset * 8) + (8 - NVIC_PRIORITY_SHIFT);
	*(NVIC_IPR_BASE_ADDR + (IPR_Reg_Select * 4)) |= (IRQPriority << Shift_Amount);  
}

/*******************************************************************************
 * @ Function :
 * @ Brief    :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Note	  :
 */
void GPIO_IRQHandler(uint8_t GPIOPin)
{
	// Clearing the pending register to stop the IRQ. We have to set it in order to so.

	if(EXTI->PR & (1 << GPIOPin))
	{
		EXTI->PR |= (1 << GPIOPin);
	}
}
