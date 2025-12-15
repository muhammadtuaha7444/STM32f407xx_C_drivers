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
void GPIO_ClockCtrl(GPIO_RegDef_t *GPIOx, uint8_t EnDis)
{
	if(EnDis == EN)
	{
		if(GPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(GPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(GPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(GPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(GPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(GPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(GPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(GPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if(GPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}else
	{
		if(GPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(GPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(GPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if(GPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if(GPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if(GPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if(GPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if(GPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if(GPIOx == GPIOI)
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
	if (pGPIO_Handle->pGPIOxConfig_Port->GPIO_PinMode <= GPIO_AN_MODE)
	{
		temp = ((pGPIO_Handle->pGPIOxConfig_Port->GPIO_PinMode) << (2 * (pGPIO_Handle->pGPIOxConfig_Port->GPIO_PinNumber)));
		pGPIO_Handle->pGPIOx_Port->MODER &= ~(0x3 << pGPIO_Handle->pGPIOxConfig_Port->GPIO_PinNumber); // clearing before setting
		(pGPIO_Handle->pGPIOx_Port->MODER) |= temp; //Setting 
		temp = 0;
	}else if (pGPIO_Handle->pGPIOxConfig_Port->GPIO_PinMode >> GPIO_AN_MODE)
	{
		//write the IT logic here
	}


	/*Configure the output type of the pin*/
	temp = (pGPIO_Handle->pGPIOxConfig_Port->GPIO_PinODType) << (1 * (pGPIO_Handle->pGPIOxConfig_Port->GPIO_PinNumber));
	pGPIO_Handle->pGPIOx_Port->OTYPER &= ~(1 << pGPIO_Handle->pGPIOxConfig_Port->GPIO_PinNumber); // clearing before setting
	(pGPIO_Handle->pGPIOx_Port->OTYPER) |= temp;
	temp = 0;

	/*Configure the pull up or pull down of the pin*/
	temp = (pGPIO_Handle->pGPIOxConfig_Port->GPIO_PinPUPD) << (2 * (pGPIO_Handle->pGPIOxConfig_Port->GPIO_PinNumber));
	pGPIO_Handle->pGPIOx_Port->PUPDR &= ~(0x3 << pGPIO_Handle->pGPIOxConfig_Port->GPIO_PinNumber); // clearing before setting
	pGPIO_Handle->pGPIOx_Port->PUPDR |= temp;
	temp = 0;

	/*Configure the Output speed of the pin*/
	temp = (pGPIO_Handle->pGPIOxConfig_Port->GPIO_PinSpeed) << (2 * (pGPIO_Handle->pGPIOxConfig_Port->GPIO_PinSpeed));
	pGPIO_Handle->pGPIOx_Port->OSPEEDR &= ~(0x3 << pGPIO_Handle->pGPIOxConfig_Port->GPIO_PinNumber); // clearing before setting
	pGPIO_Handle->pGPIOx_Port->OSPEEDR |= temp;
	temp = 0;
	
	/*Configre the alternate pin functionality of the pin*/
	if(pGPIO_Handle->pGPIOxConfig_Port->GPIO_PinMode == GPIO_AF_MODE)
	{
		uint32_t temp1, temp2;
		temp1 = (pGPIO_Handle->pGPIOxConfig_Port->GPIO_PinNumber) / 8;
		temp2 = (pGPIO_Handle->pGPIOxConfig_Port->GPIO_PinNumber) %  8;

		pGPIO_Handle->pGPIOx_Port->AFR[temp1] &=  ~(0xF<< (4 * temp2)); 
		pGPIO_Handle->pGPIOx_Port->AFR[temp1] |=  pGPIO_Handle->pGPIOxConfig_Port->GPIO_PinAF<< (4 * temp2);
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
void GPIO_DeInit(GPIO_RegDef_t *GPIOx)
{

	if(GPIOx == GPIOA)
	{
		GPIOA_RESET();
	}else if(GPIOx == GPIOB)
	{
		GPIOB_RESET();
	}else if(GPIOx == GPIOC)
	{
		GPIOC_RESET();
	}else if(GPIOx == GPIOD)
	{
		GPIOD_RESET();
	}else if(GPIOx == GPIOE)
	{
		GPIOE_RESET();
	}else if(GPIOx == GPIOF)
	{
		GPIOF_RESET();
	}else if(GPIOx == GPIOG)
	{
		GPIOG_RESET();
	}else if(GPIOx == GPIOH)
	{
		GPIOH_RESET();
	}else if(GPIOx == GPIOI)
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
value = (uint8_t) ((GPIOx->IDR >> GPIOPin) & (0x00000001));

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
void GPIO_WriteOutputPin(GPIO_RegDef_t *GPIOx, uint8_t GPIOPin, uint8_t Value )
{

}

/*******************************************************************************
 * @ Function :
 * @ Brief    :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Note	  :
 */
void GPIO_WriteOutputPort(GPIO_RegDef_t *GPIOx, uint16_t Value)
{

}

/*******************************************************************************
 * @ Function :
 * @ Brief    :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Note	  :
 */
void GPIO_TogglePin(GPIO_RegDef_t *GPIOx, uint8_t GPIOPin)
{

}

/*******************************************************************************
 * @ Function :
 * @ Brief    :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Para[1]  :
 * @ Note	  :
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnDis)
{


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

}
