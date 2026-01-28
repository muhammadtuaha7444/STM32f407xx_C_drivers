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
   uint8_t SPI_DeviceMode;      /* !<Defined @SPI_DeviceMode>   */
   uint8_t SPI_BusConfig;       /* !<Defined @SPI_Bus_Config>   */
   uint8_t SPI_DFF;             /* !<Defined @SPI_DFF>   */
   uint8_t SPI_CPHA;            /* !<Defined @SPI_CPHA>   */
   uint8_t SPI_CPOL;            /* !<Defined @SPI_CPOL>   */
   uint8_t SPI_SSM;             /* !<Defined @SPI_SSM>   */
   uint8_t SPI_Speed;           /* !<Defined @SPI_Speed>   */    
}SPI_Config_t;

typedef struct 
{
    SPI_RegDef_t *pSPI_reg;
    SPI_Config_t SPI_config;
}SPI_Handler_t;


/*
 * SPI MACRO definations
 * @brief: Reffer the to confgiure the SPI
 * 1. SPI Control Register 1 Bit Defination Values
 * 2. SPI Control Register 1 Bit Place Definations
 */


/* 1. SPI Control Register 1 Bit Defination Values */
/*
*@SPI_DeviceMode
*/
#define SPI_SLAVE_CONFIG        0           //0: Slave configuration
#define SPI_MASTER_CONFIG       1           //1: Master configuration

/*
*@SPI_Bus_Config
*/
#define SPI_F_DUPLEX_EN           0         //0: 2-line unidirectional data mode selected
#define SPI_H_DUPLEX_EN           1         //1: 1-line bidirectional data mode selected
#define SPI_SIMPLEX_TX_EN         3         //3: 1-line simplex mode enabled

/*
*@SPI_DFF
*/
#define SPI_8_BIT_EN              0         //0: 8-bit data frame format transmission/reception
#define SPI_16_BIT_E1             1         //1: 16-bit data frame format transmission/reception

/*
*@SPI_CPHA
*/
#define SPI_CPHA_HIGH             0         //0: CK to 0 when idle
#define SPI_CPHA_LOW              1         //1: CK to 1 when idle


/*
*@SPI_CPOL
*/
#define SPI_CPOL_HIGH             0         //0: The first clock transition is the first data capture edge
#define SPI_CPOL_LOW              1         //1: The second clock transition is the first data capture edge


/*
*@SPI_SSM
*/
#define SPI_SSM_DIS               0         //0: Software slave management disabled
#define SPI_SSM_EN                1         //1: Software slave management enabled


/*
*@SPI_Speed
*/
#define SPI_BAUD_RATE_2           0         //0000: fPCLK/2
#define SPI_BAUD_RATE_4           1         //0001: fPCLK/4
#define SPI_BAUD_RATE_8           2         //0010: fPCLK/8
#define SPI_BAUD_RATE_16          3         //0011: fPCLK/16
#define SPI_BAUD_RATE_32          4         //0100: fPCLK/32
#define SPI_BAUD_RATE_64          5         //0101: fPCLK/64
#define SPI_BAUD_RATE_128         6         //0110: fPCLK/128
#define SPI_BAUD_RATE_256         7         //0111: fPCLK/256

/* 2. SPI Control Register 1 Bit Place Defination */
#define SPI_BIDI_MODE_BIT             15
#define SPI_BIDI_OE_BIT               14
#define SPI_CRC_EN_BIT                13
#define SPI_CRC_NEXT_BIT              12
#define SPI_DFF_BIT                   11
#define SPI_RX_ONLY_BIT               10
#define SPI_SSM_BIT                   9
#define SPI_SSI_BIT                   8
#define SPI_LSB_FIRST_BIT             7
#define SPI_SPE_BIT                   6
#define SPI_SPE_BR_BIT                3
#define SPI_MSTR_BIT                  2
#define SPI_CPOL_BIT                  1
#define SPI_CPHA_BIT                  0

/* 3. SPI Status Register Bit Place Defination */
#define SPI_RXNE                      0
#define SPI_TXE                       1
#define SPI_CHSIDE                    2
#define SPI_UDR                       3
#define SPI_CRC_ERR                   4
#define SPI_MODF                      5
#define SPI_OVR                       6
#define SPI_BSY                       7
#define SPI_FRE                       8

/* 4. SPI Status Register Flag Bit Place Defination */
#define SPI_TX_EMPTY_STATUS_FLAG      (1 << SPI_TXE)
#define SPI_RXNE_N_EMPTY_FLAG         (1 << SPI_RXNE)
#define SPI_BSY_FLAG                  (1 << SPI_BSY)
#define SPI_TX_EMPTY                  1
#define SPI_TX_NON_EMPTY              0
/********************************************************************************
 * 						GPIO APIs Decarations are written here
 *******************************************************************************/

void SPI_ClockCtrl(SPI_RegDef_t* pSPIx, uint8_t EnDis);
void SPI_Init(SPI_Handler_t *pSPIHandler);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_Tx(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len );
uint8_t SPI_Rx(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len );
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnDis);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandler(SPI_RegDef_t *SPIx);
uint8_t SPI_Status_Register_Status(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_Control(SPI_Handler_t *pSPIx, uint8_t ENorDIS);


#endif /* INC_STM32F407XX_DRIVER_SPI_H_ */
