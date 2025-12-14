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
void GPIO_Init(GPIO_Handler_t *GPIOx)
{
	uint32_t temp = 0;
	/* GPIOx Mode Configuration */
	if (GPIOx->pGPIOxConfig_Port->GPIO_PinMode << GPIO_AN_MODE)
	{
		temp = ((GPIOx->pGPIOxConfig_Port->GPIO_PinMode) << 2 * (GPIOx->pGPIOxConfig_Port->GPIO_PinNumber));
		(GPIOx->pGPIOx_Port->MODER) = temp;
		temp = 0;
	}else if (GPIOx->pGPIOxConfig_Port->GPIO_PinMode >> GPIO_AN_MODE)
	{
		//write the IT logic here
	}
	if (GPIOx->pGPIOxConfig_Port->GPIO_PinODType)
	{
		temp = ((GPIOx->pGPIOxConfig_Port->GPIO_PinODType) << (GPIOx->pGPIOxConfig_Port->GPIO_PinNumber));
		(GPIOx->pGPIOx_Port->OTYPER) = temp;
		temp = 0;
	}
	if(GPIOx->pGPIOxConfig_Port->GPIO_PinPUPD)
	{
		temp = ((GPIOx->pGPIOxConfig_Port->GPIO_PinPUPD) << (GPIOx->pGPIOxConfig_Port->GPIO_PinNumber));
		GPIOx->pGPIOx_Port->PUPDR = temp;
		temp = 0:
	}
	if(GPIOx->pGPIOxConfig_Port->GPIO_PinSpeed)
	{
		temp = ((GPIOx->pGPIOxConfig_Port->GPIO_PinSpeed) << (GPIOx->pGPIOxConfig_Port->GPIO_PinSpeed));
		GPIOx->pGPIOx_Port->OSPEEDR = temp;
		temp = 0;
	}
	if(GPIOx->pGPIOxConfig_Port->GPIO_PinAF)
	{
		temp =((GPIOx->pGPIOxConfig_Port->GPIO_PinAF) << (GPIOx->pGPIOxConfig_Port->GPIO_PinNumber));
		temp = 0;
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
