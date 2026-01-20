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
    //1. Configure the SPI Device Mode
    uint32_t temp = 0; 
    temp|= (pSPIHandler->SPI_config.SPI_DeviceMode) << (MSTR_BIT);
    

    //2. Configure the Bus Speed
    if(pSPIHandler->SPI_config.SPI_BusConfig == SPI_F_DUPLEX_EN)
    {
        temp&= ~(1 << SPI_BIDI_MODE_BIT);
    }else if(pSPIHandler->SPI_config.SPI_BusConfig == SPI_H_DUPLEX_EN)
    {
        temp|= (1 << SPI_BIDI_OE_BIT);
    }else if(pSPIHandler->pSPI_reg->SPI_CR1 == SPI_SIMPLEX_TX_EN)
    {

        temp|= (1 << SPI_BIDI_OE_BIT);
        pSPIHandler->pSPI_reg->SPI_CR1 |=(temp);
    }

    //3. Configure the Baud Rate
    temp|= (pSPIHandler->SPI_config.SPI_Speed) << (SPE_BR_BIT);
    //4. Confgiure the SPI data frame
    temp|= (pSPIHandler->SPI_config.SPI_DFF)   << (DFF_BIT);
    //5. Configure the CPHA 
    temp|= (pSPIHandler->SPI_config.SPI_CPHA)  << (DFF_BIT);
    //6. Configure the CPOL
    temp|= (pSPIHandler->SPI_config.SPI_CPOL)  << (CPOL_BIT);
    //7. COnfigure the SSM
    temp|= (pSPIHandler->SPI_config.SPI_SSM)   << (SSM_BIT);

    pSPIHandler->pSPI_reg->SPI_CR1 = temp;
    
}

/*******************************************************************************
 * @ Function : GPIO_Init
 * @ Brief    : This Function Configure the GPIO port using the GPIO handler
 * @ Para[1]  : Pointer to GPIO Handler Struct(GPIO base address)
 * @ Note	  :
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    if(SPI1)
    {
        RCC->APB2RSTR|= (1 << 12);
    }else if(SPI2)
    {
        RCC->APB1RSTR|= (1 << 14);
    }else if(SPI3)
    {
        RCC->APB1RSTR|= (1 << 15);
    }else if(SPI4)
    {
        RCC->APB2RSTR|= (1 << 13);
    }

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
