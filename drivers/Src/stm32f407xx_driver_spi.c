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

    SPI_ClockCtrl(pSPIHandler->pSPI_reg, EN);
    //1. Configure the SPI Device Mode
    uint32_t temp = 0; 
    temp|= (pSPIHandler->SPI_config.SPI_DeviceMode) << (SPI_MSTR_BIT);
    

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
    temp|= (pSPIHandler->SPI_config.SPI_Speed) << (SPI_SPE_BR_BIT);
    //4. Confgiure the SPI data frame
    temp|= (pSPIHandler->SPI_config.SPI_DFF)   << (SPI_DFF_BIT);
    //5. Configure the CPHA 
    temp|= (pSPIHandler->SPI_config.SPI_CPHA)  << (SPI_DFF_BIT);
    //6. Configure the CPOL
    temp|= (pSPIHandler->SPI_config.SPI_CPOL)  << (SPI_CPOL_BIT);
    //7. COnfigure the SSM
    temp|= (pSPIHandler->SPI_config.SPI_SSM)   << (SPI_SSM_BIT);

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

uint8_t SPI_Status_Register_Status(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
    if(FlagName == SPI_TX_EMPTY_STATUS_FLAG)
    {
        if((pSPIx->SPI_SR) & SPI_TX_EMPTY_STATUS_FLAG)
        {
            return SPI_TX_EMPTY;
        }else
        {
            return SPI_TX_NON_EMPTY;
        }
    }
}

/*******************************************************************************
 * @ Function : SPI_Tx
 * @ Brief    : This Function Sends the data on SPI In blocking or polling mode
 * @ Para[1]  : SPI 
 * @ Para[2]  : Pointer to data buffer
 * @ Para[3]  : Length of the data to be send
 * @ Note	  :
 */
void SPI_Tx(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len )
{

    while(Len > 0)
    {
        //1. Check if the TX buffer is empty or not.
        while(SPI_Status_Register_Status(pSPIx, SPI_TX_EMPTY_STATUS_FLAG ) == SPI_TX_NON_EMPTY );

        //2. check the for the dataframe
        if(((pSPIx->SPI_CR1) & (1 << SPI_DFF_BIT)))
        {
            //3. Transfer the 16 bit frame
            pSPIx->SPI_DR = (*((uint16_t*)pTxBuffer));
            Len--;
            Len--;
            (uint16_t*)pTxBuffer++;
        }else
        {
            //3. Transfer the 8 bit frame
            pSPIx->SPI_DR = *(pTxBuffer);
            Len--;
            pTxBuffer++;
            
        }

    }
}

/*******************************************************************************
 * @ Function : SPI_Control
 * @ Brief    : This Function Enables or Disables the SPI using the SPE bit
 * @ Para[1]  : Pointer to GPIO Handler Struct(GPIO base address)
 * @ Para[2]  : Enable or Disable Macro 
 * @ Note	  :
 */
void SPI_Control(SPI_Handler_t *pSPIx, uint8_t ENorDIS)
{
    if(ENorDIS == EN)
    {
        pSPIx->pSPI_reg->SPI_CR1 |= (1 << SPI_SPE_BIT);
    }else
    {
        pSPIx->pSPI_reg->SPI_CR1 &= ~(1 << SPI_SPE_BIT);
    }

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
