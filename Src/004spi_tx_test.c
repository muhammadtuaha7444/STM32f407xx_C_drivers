
#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_driver_spi.h"


void SPI2_GPIO_Init()
{
    //PB12->SPI2_NSS 
    //PB13->SPI2_SCK
    //PB14->SPI2_MISO
    //PB15->SPI2_MOSI


    GPIO_Handler_t spi2_gpio_init;

    spi2_gpio_init.pGPIOx_Port = GPIOB;   
    spi2_gpio_init.pGPIOxConfig_Port.GPIO_PinMode   = GPIO_AF_MODE;
    spi2_gpio_init.pGPIOxConfig_Port.GPIO_PinSpeed  = GPIO_OP_SPEEDHIGH;
    spi2_gpio_init.pGPIOxConfig_Port.GPIO_PinPUPD   = GPIO_NO_PUPD;
    spi2_gpio_init.pGPIOxConfig_Port.GPIO_PinODType = GPIO_OT_PP;
    spi2_gpio_init.pGPIOxConfig_Port.GPIO_PinAF     = 5;

    //PB12 Set
    spi2_gpio_init.pGPIOxConfig_Port.GPIO_PinNumber = GPIO_PIN12;
    GPIO_Init( &spi2_gpio_init );
    //PB13 Set
    spi2_gpio_init.pGPIOxConfig_Port.GPIO_PinNumber = GPIO_PIN13;
    GPIO_Init( &spi2_gpio_init );
    //PB14 Set
    spi2_gpio_init.pGPIOxConfig_Port.GPIO_PinNumber = GPIO_PIN14;
    GPIO_Init( &spi2_gpio_init );
    //PB15 Set
    spi2_gpio_init.pGPIOxConfig_Port.GPIO_PinNumber = GPIO_PIN12;

}

void SPI2_Init()
{
    SPI_Handler_t spi2_init;

    spi2_init.pSPI_reg = SPI2;
    spi2_init.SPI_config.SPI_DeviceMode = SPI_MASTER_CONFIG;
    spi2_init.SPI_config.SPI_BusConfig  = SPI_F_DUPLEX_EN;
    spi2_init.SPI_config.SPI_DFF        = SPI_8_BIT_EN;
    spi2_init.SPI_config.SPI_CPHA       = SPI_CPHA_LOW;
    spi2_init.SPI_config.SPI_CPOL       = SPI_CPOL_LOW;
    spi2_init.SPI_config.SPI_SSM        = SPI_SSM_EN;
    spi2_init.SPI_config.SPI_Speed      = SPI_BAUD_RATE_2;

    SPI_Init(&spi2_init);

}

int main()
{

    SPI2_GPIO_Init();
    SPI2_Init();
    SPI_Control(SPI2, EN);

    char tx_data[] = "Hello Iam Tuaha/n";
    SPI_Tx(SPI2, (uint8_t*)tx_data, strlen(tx_data));

    while(1)
    {

    }
}


