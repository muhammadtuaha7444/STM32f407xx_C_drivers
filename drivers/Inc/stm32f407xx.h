/*
 * stm32f407xx.h
 *
 *  Created on: Dec 7, 2025
 *      Author: Muhammad Tuaha
 */




#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_


#include <stdint.h>
#include "stm32f407xx.h"

/*Base  addresses of FLASH and SRAM*/
#define FLASH_BASEADDR 			0x08000000U
#define SRAM1_BASEADDR  		0x20000000U
#define SRAM2_BASEADDR			0x2001C000U
#define SRAM_BASEADDR			SRAM_ADDRESS
#define ROM_BASEADDR			0x1FFF0000U

/*APB Peripheral and AHB Peripheral Base Addresses*/
#define AHB1PERI_BASEADDR		0x40020000U
#define AHB2PERI_BASEAADR		0x50000000U
#define APB1PERI_BASEADDR		0x40000000U
#define APB2PERI_BASEADDR		0x40010000U



/*
 * Base address of the Peripherals hanging on AHB1
 * 1. GPIOx Base addresses
 * TODO : Add the remaining peripherals
 * */
/*GPIO BASE addresses*/
#define GPIOA_BASEADDR			(AHB1PERI_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERI_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERI_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERI_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERI_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERI_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERI_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERI_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERI_BASEADDR + 0x2000)
#define RCC_BASEADDR			(AHB1PERI_BASEADDR + 0x3800)


/*
 * Base addresses of the Peripherals hanging on APB1
 * 1. I2C
 * 2. SPI
 * 3. USART
 * 4. UART
 * TODO : Add the remaining peripherals
 * */
#define I2C1_BASEADDR			(APB1PERI_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERI_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERI_BASEADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1PERI_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERI_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERI_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERI_BASEADDR + 0x4800)

#define UART4_BASEADDR 			(APB1PERI_BASEADDR + 0x4C00)
#define UART5_BASEADDR 			(APB1PERI_BASEADDR + 0x5000)

/*
 * Base addresses of the Peripherals hanging on APB2
 * 1. USART
 * 2. SPI
 * 3. SYS_Cofiguration
 * 4. EXTI
 * TODO : Add the remaining peripherals
 * */
#define USART1_BASEADDR 		(APB2PERI_BASEADDR + 0x1000)
#define USART6_BASEADDR 		(APB2PERI_BASEADDR + 0x1400)

#define SPI1_BASEADDR			(APB2PERI_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2PERI_BASEADDR + 0x3400)

#define SYSCONF_BASEADDR		(APB2PERI_BASEADDR + 0x3800)

#define EXTI_BASEADDR			(APB2PERI_BASEADDR + 0x3C00)

/*
 * Structs definations of peripheral registers
 * 1. GPIOx
 * 2. RCCx
 */
/******************* Peripherals Register Structures*******************/
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];

}GPIO_RegDef_t;


typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t Reserved_0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t Reserved_1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t Reserved_2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t Reserved_3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t Reserved_4[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t Reserved_5[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;

}RCC_RegDef_t;

/*
 * Defining Macros as base address of structures
 * 1. ALL GPIOx Register Structure Macros
 */

/******************** GPIOx RegDef_t Address Pointer ************************/
#define GPIOA 		((GPIO_RegDef_t*)GPIOA_BASEADDR )
#define GPIOB 		((GPIO_RegDef_t*)GPIOB_BASEADDR )
#define GPIOC 		((GPIO_RegDef_t*)GPIOC_BASEADDR )
#define GPIOD 		((GPIO_RegDef_t*)GPIOD_BASEADDR )
#define GPIOE 		((GPIO_RegDef_t*)GPIOE_BASEADDR )
#define GPIOF 		((GPIO_RegDef_t*)GPIOF_BASEADDR )
#define GPIOG 		((GPIO_RegDef_t*)GPIOG_BASEADDR )
#define GPIOH 		((GPIO_RegDef_t*)GPIOH_BASEADDR )
#define GPIOI 		((GPIO_RegDef_t*)GPIOI_BASEADDR )

/******************* RCC RegDef_t Addresses Pointers *************************/
#define RCC 		((RCC_RegDef_t*)RCC_BASEADDR )


/*
 * Defining the Macros to enable peripheral clock
 * 1. GPIOx AHB1
 * 2. I2C APB1
 * 3. SPI APB1
 * 4. USART APB1
 * 5. UART  APB1
 */

/******************* GPIO Peripheral Clock Enable  ***************************/
#define GPIOA_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 8))

/*************************** I2C Clock Enable  ********************************/
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

/*************************** SPI Clock Enable **********************************/
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))

/*************************** USART Clock Enable *******************************/
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define USART1_PCKL_EN()	(RCC->APB2ENR |= (1 <<  4))
#define USART6_PCKL_EN()	(RCC->APB2ENR |= (1 <<  5))

/**************************** UART Clock Enable ********************************/
#define UART4_PCKL_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCKL_EN()		(RCC->APB1ENR |= (1 << 20))

/**************************** SYSCFG Clock Enable ******************************/
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))



/*
 * Defining the Macros to disable peripheral clock
 * 1. GPIOx AHB1
 * 2. I2C APB1
 * 3. SPI APB1
 * 4. USART APB1
 * 5. UART  APB1
 */

/******************* GPIO Peripheral Clock Enable  ***************************/
#define GPIOA_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 8))

/*************************** I2C Clock Enable  ********************************/
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

/*************************** SPI Clock Enable **********************************/
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))

/*************************** USART Clock Enable *******************************/
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define USART1_PCKL_DI()	(RCC->APB2ENR &= ~(1 <<  4))
#define USART6_PCKL_DI()	(RCC->APB2ENR &= ~(1 <<  5))

/**************************** UART Clock Enable ********************************/
#define UART4_PCKL_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCKL_DI()		(RCC->APB1ENR &= ~(1 << 20))

/**************************** SYSCFG Clock Enable ******************************/
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))

/*
 * Defining the Macros to reset the GPIO port
 * 1. GPIOx AHB1
 */

#define GPIOA_RESET()		do{RCC->AHB1RSTR |= ~(1<<0); RCC->AHB1RSTR &= ~(1<<0);}while(0)
#define GPIOB_RESET()		do{RCC->AHB1RSTR |= ~(1<<1); RCC->AHB1RSTR &= ~(1<<1);}while(0)
#define GPIOC_RESET()		do{RCC->AHB1RSTR |= ~(1<<2); RCC->AHB1RSTR &= ~(1<<2);}while(0)
#define GPIOD_RESET()		do{RCC->AHB1RSTR |= ~(1<<3); RCC->AHB1RSTR &= ~(1<<3);}while(0)
#define GPIOE_RESET()		do{RCC->AHB1RSTR |= ~(1<<4); RCC->AHB1RSTR &= ~(1<<4);}while(0)
#define GPIOF_RESET()		do{RCC->AHB1RSTR |= ~(1<<5); RCC->AHB1RSTR &= ~(1<<5);}while(0)
#define GPIOG_RESET()		do{RCC->AHB1RSTR |= ~(1<<6); RCC->AHB1RSTR &= ~(1<<6);}while(0)
#define GPIOH_RESET()		do{RCC->AHB1RSTR |= ~(1<<7); RCC->AHB1RSTR &= ~(1<<7);}while(0)
#define GPIOI_RESET()		do{RCC->AHB1RSTR |= ~(1<<8); RCC->AHB1RSTR &= ~(1<<8);}while(0)

/*
 * Generic global Macros are defined here
 * 1. GPIOx
 */

#define EN 					1
#define DIS					0
#define SET					EN
#define RESET				DIS



#endif /* INC_STM32F407XX_H_ */
