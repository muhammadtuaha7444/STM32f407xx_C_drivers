/*
 * stm32f407xx_driver_spi.c
 *
 *  Created on: Jan 13, 2026
 *      Author: Dell
 */


#include "stm32f407xx_driver_spi.h"

/********************************************************************************
 * 						GPIO APIs Decarations are written here
 *******************************************************************************/

/*******************************************************************************
 * @ Function : GPIO_Init
 * @ Brief    : This Function Configure the GPIO port using the GPIO handler
 * @ Para[1]  : Pointer to GPIO Handler Struct(GPIO base address)
 * @ Note	  :
 */
void SPI_ClockCtrl(SPI_RegDef_t* pSPIx, uint8_t EnDis)
{
    if(EnDis == EN)
    {
        if(pSPIx == SPI1)
        { 
            SPI1_PCLK_EN();
        }else if(pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }else if(pSPIx ==SPI3)
        {
            SPI3_PCLK_EN();
        }
        else if(pSPIx ==SPI4)
        {
            SPI4_PCLK_EN();
        }
    }
    else if(EnDis == DIS)
    {    
        if(pSPIx == SPI1)
        { 
            SPI1_PCLK_DI();
        }else if(pSPIx == SPI2)
        {
            SPI2_PCLK_DI();
        }else if(pSPIx ==SPI3)
        {
            SPI3_PCLK_DI();
        }else if(pSPIx ==SPI4)
        {
            SPI4_PCLK_DI();
        }
        
    }
}
/*******************************************************************************
 * @ Function : GPIO_Init
 * @ Brief    : This Function Configure the GPIO port using the GPIO handler
 * @ Para[1]  : Pointer to GPIO Handler Struct(GPIO base address)
 * @ Note	  :
 */
void SPI_Init(SPI_Handler_t *pSPIHandler)
{

}

/*******************************************************************************
 * @ Function : GPIO_Init
 * @ Brief    : This Function Configure the GPIO port using the GPIO handler
 * @ Para[1]  : Pointer to GPIO Handler Struct(GPIO base address)
 * @ Note	  :
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}

/*******************************************************************************
 * @ Function : GPIO_Init
 * @ Brief    : This Function Configure the GPIO port using the GPIO handler
 * @ Para[1]  : Pointer to GPIO Handler Struct(GPIO base address)
 * @ Note	  :
 */
uint8_t SPI_Tx(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len )
{

}

/*******************************************************************************
 * @ Function : GPIO_Init
 * @ Brief    : This Function Configure the GPIO port using the GPIO handler
 * @ Para[1]  : Pointer to GPIO Handler Struct(GPIO base address)
 * @ Note	  :
 */
uint8_t SPI_Rx(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len )
{

}

/*******************************************************************************
 * @ Function : GPIO_Init
 * @ Brief    : This Function Configure the GPIO port using the GPIO handler
 * @ Para[1]  : Pointer to GPIO Handler Struct(GPIO base address)
 * @ Note	  :
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnDis)
{

}

/*******************************************************************************
 * @ Function : GPIO_Init
 * @ Brief    : This Function Configure the GPIO port using the GPIO handler
 * @ Para[1]  : Pointer to GPIO Handler Struct(GPIO base address)
 * @ Note	  :
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{

}

/*******************************************************************************
 * @ Function : GPIO_Init
 * @ Brief    : This Function Configure the GPIO port using the GPIO handler
 * @ Para[1]  : Pointer to GPIO Handler Struct(GPIO base address)
 * @ Note	  :
 */
void SPI_IRQHandler(SPI_RegDef_t *SPIx)
{

}
