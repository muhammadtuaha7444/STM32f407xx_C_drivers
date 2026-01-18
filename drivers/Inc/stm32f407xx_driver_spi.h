/*
 * stm32f07xx_driver_spi.h
 *
 *  Created on: Jan 13, 2026
 *      Author: Dell
 */

#ifndef INC_STM32F407XX_DRIVER_SPI_H_
#define INC_STM32F407XX_DRIVER_SPI_H_


#include <stdint.h>
#include "stm32f407xx.h"


/*
 * SPI type definations
 * 1. SPI Configuration struct
 * 2. SPI Handler Struct
 */

/*
 * Here we have defined a structure to for a SPIconfiguration
 */

typedef struct 
{
   uint8_t SPI_DeviceMode;
   uint8_t SPI_BusConfig;
   uint8_t SPI_DFF;
   uint8_t SPI_CPHA;
   uint8_t SPI_CPOL;
   uint8_t SPI_SSM;
   uint8_t SPI_Speed;
}SPI_Config_t;

typedef struct 
{
    SPI_RegDef_t *pSPI_reg;
    SPI_Config_t pSPI_config;
}SPI_Handler_t;


/********************************************************************************
 * 						GPIO APIs Decarations are written here
 *******************************************************************************/

void SPI_ClockCtrl(SPI_RegDef_t* pSPIx, uint8_t EnDis);
void SPI_Init(SPI_Handler_t *pSPIHandler);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
uint8_t SPI_Tx(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len );
uint8_t SPI_Rx(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len );
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnDis);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandler(SPI_RegDef_t *SPIx);



#endif /* INC_STM32F407XX_DRIVER_SPI_H_ */
