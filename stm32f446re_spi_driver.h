/*
 * stm32f446re_spi_driver.h
 *
 *  Created on: Jun 21, 2024
 *      Author: Atharv Pachankar
 */

#ifndef INC_STM32F446RE_SPI_DRIVER_H_
#define INC_STM32F446RE_SPI_DRIVER_H_

#include "stm32f44643xx.h"
#include "stm32f446re_gpio_driver.h"

// Device Modes
#define SPI_DEVICE_MODE_MASTER      1
#define SPI_DEVICE_MODE_SLAVE       0

// Bus Configuration
#define SPI_BUS_CONFIG_FD           1
#define SPI_BUS_CONFIG_HD           2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 3
#define SPI_BUS_CONFIG_SIMPLEX_TXONLY 4

// Clock Speed
#define SPI_CLK_DIV2    0     // Default Baud Rate
#define SPI_CLK_DIV4    1
#define SPI_CLK_DIV8    2
#define SPI_CLK_DIV16   3
#define SPI_CLK_DIV32   4
#define SPI_CLK_DIV64   5
#define SPI_CLK_DIV128  6
#define SPI_CLK_DIV256  7

// Data Frame Formats
#define SPI_DFF_8BITS  0     // Default
#define SPI_DFF_16BITS 1

// Clock Polarity
#define SPI_CPOL_HIGH  1
#define SPI_CPOL_LOW   0

// Clock Phase
#define SPI_CPHA_HIGH  1
#define SPI_CPHA_LOW   0

// Software Slave Management
#define SPI_SSM_EN     1
#define SPI_SSM_DI     0

typedef struct
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SCLK_Speed;  // Baud Rate
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
} SPI_Config_t;

typedef struct
{
    SPI_RegDef_t *pSPIx;
    SPI_Config_t SPIConfig;
} SPI_Handle_t;


void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_Peri_Clk_Ctrl(SPI_RegDef_t* pSPIx, uint8_t EnORDi);
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t* pRxBuffer, uint32_t len);

void SPI_PeriClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnORDi)
{
    if (EnORDi == ENABLE)
    {
        if (pSPIx == SPI1)
        {
            SPI1_CLK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_CLK_EN();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_CLK_EN();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_CLK_EN();
        }
    }
    else
    {
        if (EnORDi == DISABLE)
        {
            if (pSPIx == SPI1)
            {
                SPI1_CLK_DI();
            }
            else if (pSPIx == SPI2)
            {
                SPI2_CLK_DI();
            }
            else if (pSPIx == SPI3)
            {
                SPI3_CLK_DI();
            }
            else if (pSPIx == SPI4)
            {
                SPI4_CLK_DI();
            }
        }
    }
}

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    uint32_t tempreg = 0;

    // Configure Device Mode
    tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

    // Configure Bus Config
    if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        tempreg &= ~(1 << 15);  // Bi-directional mode should be cleared
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        tempreg |= (1 << 15);   // Bi-directional mode should be set
    }

    // Configure Clock Speed
    tempreg |= pSPIHandle->SPIConfig.SPI_SCLK_Speed << 3;

    // Configure Data Frame Format
    tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

    // Configure Clock Phase
    tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << 1;

    // Configure Clock Polarity
    tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << 0;

    // Write configuration to CR1 register
    pSPIHandle->pSPIx->SPI_CR1 = tempreg;
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
    while (len > 0)
    {
        // Wait until TXE (Transmit Buffer Empty) flag is set
        while (!(pSPIx->SR & (1 << 1)));

        // Check if Data Frame Format is 16-bit
        if (pSPIx->CR1 & (1 << 11))
        {
            pSPIx->DR = *((uint16_t *)pTxBuffer); // Send 16-bit data
            len -= 2;
            pTxBuffer += 2;
        }
        else
        {
            pSPIx->DR = *pTxBuffer; // Send 8-bit data
            len--;
            pTxBuffer++;
        }
    }
}

#define RXNE_BIT_POS 0

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
    while (len > 0)
    {
        // Wait until RXNE (Receive Buffer Not Empty) flag is set
        while (!(pSPIx->SR & (1 << RXNE_BIT_POS)));

        // Check if Data Frame Format is 16-bit
        if (pSPIx->CR1 & (1 << 11))
        {
            *((uint16_t *)pRxBuffer) = pSPIx->DR; // Receive 16-bit data
            len -= 2;
            pRxBuffer += 2;
        }
        else
        {
            *pRxBuffer = pSPIx->DR; // Receive 8-bit data
            len--;
            pRxBuffer++;
        }
    }
}

#define SPI_CR1_SPE 6

void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx, uint8_t EnORDi)
{
    if (EnORDi == ENABLE)
    {
        pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
    }
    else
    {
        pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

#endif /* INC_STM32F446RE_SPI_DRIVER_H_ */
