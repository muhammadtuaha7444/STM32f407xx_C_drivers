/*
 * stm32f07xx_driver_spi.h
 *
 *  Created on: Jan 13, 2026
 *      Author: Dell
 */

#ifndef INC_STM32F07XX_DRIVER_SPI_H_
#define INC_STM32F07XX_DRIVER_SPI_H_


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
   uint8_t SPI_DFF
   uint8_t SPI_CPHA;
   uint8_t SPI_CPOL;
   uint8_t SPI_SSM;
   uint8_t SPI_Speed;
}SPI_Config_t;

typedef struct 
{
    SPI_RegDef_t* pSPI_reg;
    SPI_Config_t pSPI_config;
}SPI_Handler_t;





#endif /* INC_STM32F07XX_DRIVER_SPI_H_ */
