/*
 * stm32f407xx_driver_gpio.h
 *
 *  Created on: Dec 10, 2025
 *      Author: Dell
 */

#ifndef INC_STM32F407XX_DRIVER_GPIO_H_
#define INC_STM32F407XX_DRIVER_GPIO_H_

#include <stdint.h>
#include "stm32f407xx.h"



/*
 * Here we have defined a structure to for a GPIO pin configuration
 */
typedef struct {
	uint8_t GPIO_PinNumber;				/* !<Defined @GPIO PIN NUMBERS>   */
	uint8_t GPIO_PinMode;               /* !<Defined @GPIO MODE TYPES>    */
	uint8_t GPIO_PinSpeed;				/* !<Defined @GPIO SPEED TYPES>   */
	uint8_t GPIO_PinPUPD;				/* !<Defined @GPIO OUTPUT PU PD>  */
	uint8_t GPIO_PinODType;				/* !<Defined @GPIO OUTPUT TYPES>  */
	uint8_t GPIO_PinAF;
}GPIO_Pin_Config_t;
/*
 * Here we have defined the GPIOx Handler Structure
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx_Port;
	GPIO_Pin_Config_t pGPIOxConfig_Port;

}GPIO_Handler_t;


/*
 * GPIO Pin Configuration macros are defined here
 * 1. GPIOx Pin Numbers
 * 2. GPIO Mode Macros
 */

/* @GPIO PIN NUMBERS  */

#define GPIO_PIN0				0
#define GPIO_PIN1				1
#define GPIO_PIN2				2
#define GPIO_PIN3				3
#define GPIO_PIN4				4
#define GPIO_PIN5				5
#define GPIO_PIN6				6
#define GPIO_PIN7				7
#define GPIO_PIN8				8
#define GPIO_PIN9				9
#define GPIO_PIN10				10
#define GPIO_PIN11				11
#define GPIO_PIN12				12
#define GPIO_PIN13				13
#define GPIO_PIN14				14
#define GPIO_PIN15				15

/* @GPIO PIN MODE  */
#define GPIO_IN_MODE			0									// ->00: Input (reset state)
#define GPIO_OP_MODE			1									// ->01: General purpose output mode
#define GPIO_AF_MODE			2									// ->10: Alternate function mode
#define GPIO_AN_MODE			3									// ->11: Analog mode
#define GPIO_IT_FE				4
#define GPIO_IT_RE				5
#define GPIO_IT_RFE				6


/* @GPIO OUTPUT TYPES */
#define GPIO_OT_PP 				0									// ->0: Output push-pull (reset state)
#define GPIO_OT_OD				1									// ->1: Output open-drain


/* @GPIO SPEED TYPES  */
#define GPIO_OP_SPEEDLOW		0									// ->00: Low speed
#define GPIO_OP_SPEEDMED		1								    // ->01: Medium speed
#define GPIO_OP_SPEEDHIGH		2									// ->10: High speed
#define GPIO_OP_SPEEDVHIGH		3									// ->11: Very high speed

/* @GPIO PULL UP PULL DOWN REGISTERS */
#define GPIO_NO_PUPD			0									// ->00: No pull-up, pull-down
#define GPIO_PU					1									// ->01: Pull-up
#define GPIO_PD					2									// ->10: Pull-down





/********************************************************************************
 * 						GPIO APIs are written here
 *******************************************************************************/
void GPIO_ClockCtrl(GPIO_RegDef_t *GPIOx, uint8_t EnDis);
void GPIO_Init(GPIO_Handler_t *GPIOx);
void GPIO_DeInit(GPIO_RegDef_t *GPIOx);
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *GPIOx, uint8_t GPIOPin );
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *GPIOx);
void GPIO_WriteOutputPin(GPIO_RegDef_t *GPIOx, uint8_t GPIOPin, uint8_t Value );
void GPIO_WriteOutputPort(GPIO_RegDef_t *GPIOx, uint16_t Value);
void GPIO_TogglePin(GPIO_RegDef_t *GPIOx, uint8_t GPIOPin);
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnDis);
void GPIO_IRQHandler(uint8_t GPIOPin);

#endif /* INC_STM32F407XX_DRIVER_GPIO_H_ */
