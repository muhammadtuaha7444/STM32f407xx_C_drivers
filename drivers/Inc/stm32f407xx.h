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

/*
 * Macro address of the NVIC Peripheral of ARM Cortex F4 Processor
 * 1. NVIC Base addresses
 * TODO : Add the remaining peripherals and the regiters
*/

/*Processor addresses of NVIC Peripheral*/
#define NVIC_ISER0_ADDR		    ( (volatile uint32_t*)0xE000E100 )
#define NVIC_ISER1_ADDR		    ( (volatile uint32_t*)0xE000E104 )
#define NVIC_ISER2_ADDR		    ( (volatile uint32_t*)0xE000E108 )
#define NVIC_ISER3_ADDR		    ( (volatile uint32_t*)0xE000E10C )

/*Processor addresses of NVIC Peripheral*/
#define NVIC_ICER0_ADDR		    ( (volatile uint32_t*)0XE000E180 )
#define NVIC_ICER1_ADDR		    ( (volatile uint32_t*)0XE000E184 )
#define NVIC_ICER2_ADDR		    ( (volatile uint32_t*)0XE000E188 )
#define NVIC_ICER3_ADDR		    ( (volatile uint32_t*)0xE000E18C )

/*Processor NVIC Interrupt Priority register base address*/
#define NVIC_IPR_BASE_ADDR		    ( (volatile uint32_t*)0xE000E400 )
#define NVIC_PRIORITY_SHIFT				4

/*Priority Number for the Processor Settings*/
#define PRIORITY_0						0
#define PRIORITY_1						1
#define PRIORITY_2						2		
#define PRIORITY_3						3
#define PRIORITY_4						4
#define PRIORITY_5						5
#define PRIORITY_6						6
#define PRIORITY_7						7
#define PRIORITY_8						8
#define PRIORITY_9						9
#define PRIORITY_10						10
#define PRIORITY_11						11
#define PRIORITY_12						12
#define PRIORITY_13						13
#define PRIORITY_14						14

/*
 * Macro address of the Dicovery MCU Peripheral of ARM Cortex F4 
 * 1. NVIC Base addresses
 * TODO : Add the remaining peripherals and the regiters
*/
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
*/
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
*/
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
*/
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
 * 3. EXTIx
 * 4. SYSCFGx
 * 5. SPIx
*/


/******************* GPIOx Register Structure*******************/
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


/******************* RCC Register Structure*******************/
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

/******************* EXTI Register Structure*******************/
typedef struct
{

	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;

/******************* SYSCONFIG Register Structure*******************/
typedef struct 
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t Reserved[2];
	volatile uint32_t CMPCR;

}SYSCFG_RegDef_t;

/******************* SPI Register Structure*******************/
typedef struct
{
	volatile uint32_t SPI_CR1;
	volatile uint32_t SPI_CR2;
	volatile uint32_t  SPI_SR;
	volatile uint32_t  SPI_DR;
	volatile uint32_t SPI_CRCPR;
	volatile uint32_t SPI_RXCRCR;
	volatile uint32_t SPI_TXCRCR;
	volatile uint32_t SPI_I2SCFGR;
	volatile uint32_t SPI_I2SPR;

}SPI_RegDef_t;



/*
 * Defining Macros as base address of structures
 * 1. ALL GPIOx Register Structure Macros
 * 2. RCCx Registers Macros
 * 3. EXTI Registers Macros
 * 4. SysConfig Registers Macros
 * 5. SPIx Register Macros
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

/******************* RCC RegDef_t Addresses Pointer *************************/
#define RCC 		((RCC_RegDef_t*)RCC_BASEADDR )

/******************* EXTI RegDef_t Addresses Pointers *************************/
#define EXTI        ((EXTI_RegDef_t*)EXTI_BASEADDR)

/******************* SYSCONFG RegDef_t Addresses Pointer *************************/
#define SYSCONFIG   ((SYSCFG_RegDef_t*)SYSCONF_BASEADDR)

/******************* SPI Addresses Pointer *************************/
#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)


/*
 * Defining the Macros to enable peripheral clock
 * 1. GPIOx 
 * 2. I2C 
 * 3. SPI
 * 4. USART 
 * 5. UART  
 * 6. SYSCONFIG 
 * 7. SPIx
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
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))
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
 * 1. GPIOx 
 * 2. I2C 
 * 3. SPI 
 * 4. USART
 * 5. UART
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
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13))
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
 * Defining the Macros to reset Peripherals
 * 1. GPIOx AHB1
 
*/

/**************************** GPIO Reset Peripheral ******************************/
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
 * 2. GPIOx Port Code
 * 3. IRQ Numbers
*/


/**************************** GPIO General Macros ******************************/
#define EN 					1
#define DIS					0
#define SET					EN
#define RESET				DIS

/**************************** GPIO Port Code General Macros ********************/
#define GPIOPORTCODE(x) ( \
    /* 0000: PA[x] pin */ 	(x ==GPIOA) ? 0 : \
    /* 0001: PB[x] pin */ 	(x ==GPIOB) ? 1 : \
    /* 0010: PC[x] pin */ 	(x ==GPIOC) ? 2 : \
    /* 0011: PD[x] pin */ 	(x ==GPIOD) ? 3 : \
    /* 0100: PE[x] pin */ 	(x ==GPIOE) ? 4 : \
    /* 0101: PF[x] pin */ 	(x ==GPIOF) ? 5 : \
    /* 0110: PG[x] pin */ 	(x ==GPIOG) ? 6 : \
    /* 0111: PH[x] pin */ 	(x ==GPIOH) ? 7 : \
    /* 1000: PI[x] pin */ 	(x ==GPIOI) ? 8 : 0 \
)

/**************************** EXTIx IRQNumbers Macros ********************/
#define IRQNUMBER_EXTI0 		6
#define IRQNUMBER_EXTI1			7
#define IRQNUMBER_EXTI2			8
#define IRQNUMBER_EXTI3			9
#define IRQNUMBER_EXTI4			10
#define IRQNUMBER_EXTI9_5		23
#define IRQNUMBER_EXTI15_10		40



#include "stm32f407xx_driver_gpio.h"
#include "stm32f407xx_driver_spi.h"

#endif /* INC_STM32F407XX_H_ */
