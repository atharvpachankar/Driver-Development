/*
 * stm32f446re_i2c_driver.h
 *
 * Created on: Jun 28, 2024
 * Author: Atharv Pachankar
 */

#ifndef INC_STM32F446RE_I2C_DRIVER_H_
#define INC_STM32F446RE_I2C_DRIVER_H_

#include "stm32f44643xx.h"
#include "i2c_Driver.c"

typedef struct
{
    uint32_t I2C_SCLSPEED;      // I2C SCL Speed
    uint8_t I2C_DEVICEADDR;     // I2C Device Address
    uint8_t I2C_ACKCTRL;        // I2C Acknowledgement Control
    uint8_t I2C_FMDutyCycle;    // I2C Fast Mode Duty Cycle
} I2C_Config_t;

typedef struct
{
    I2C_RegDef_t *pI2Cx;        // Pointer to I2C peripheral register
    I2C_Config_t I2CConfig;     // I2C configuration settings
} I2C_Handle_t;

// I2C Clock Speed Definitions
#define I2C_SCLK_SPEED_SM     100000   // Standard Mode: 100kHz
#define I2C_SCL_SPEED_FM4K    400000   // Fast Mode: 400kHz
#define I2C_SCL_SPEED_FM2K    200000   // Fast Mode: 200kHz

// I2C Acknowledgement Control
#define I2C_ACK_EN            1       // Enable Acknowledgement
#define I2C_ACK_DI            0       // Disable Acknowledgement

// I2C Fast Mode Duty Cycle
#define I2C_FM_DUTY_2        0        // Duty Cycle 2
#define I2C_FM_DUTY_16_9    1        // Duty Cycle 16/9

/******************************************************************************************
 * APIs supported by this driver
 * For more information about the APIs, check the function definitions
 ******************************************************************************************/

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_Master_Send_Data(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr);
uint32_t RCC_GetPCLKValue(void);
void I2C_Master_Receive_Data(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr);

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    uint32_t tempreg = 0;

    // Configure ACK Control Bit
    tempreg |= pI2CHandle->I2CConfig.I2C_ACKCTRL << 10;
    pI2CHandle->pI2Cx->CR1 = tempreg;

    // Configure Frequency field of CR2
    tempreg = RCC_GetPCLKValue() / 1000000U;
    pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F); // Masking first six bits

    // Configure Device Address and other settings
    tempreg = pI2CHandle->I2CConfig.I2C_DEVICEADDR << 1;
    tempreg |= (1 << 14); // Should always be set to 1
    pI2CHandle->pI2Cx->OAR1 = tempreg;

    uint16_t ccr_val = 0;
    ccr_val = (RCC_GetPCLKValue() / (2 * pI2CHandle->I2CConfig.I2C_SCLSPEED));
    tempreg |= (ccr_val & 0xFFF);
    pI2CHandle->pI2Cx->CCR = tempreg;
}

uint32_t RCC_GetPCLKValue(void)
{
    uint32_t pclk;
    uint8_t clksrc;
    uint32_t sysclk;

    clksrc = ((RCC->CFGR) & 0x03);

    if (clksrc == 0)
    {
        sysclk = 16000000; // 16 MHz
    }
    else if (clksrc == 1)
    {
        sysclk = 8000000;  // 8 MHz
    }

    return sysclk; // Return the system clock value
}

void I2C_Master_Send_Data(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr)
{
    pI2CHandle->pI2Cx->CR1 |= (1 << 8); // Generate Repeated Start
    while (!(pI2CHandle->pI2Cx->SR1 & 0x1)); // Wait for SB flag

    uint8_t slaveaddr = SlaveAddr << 1;
    slaveaddr &= ~(1 << 0); // Clear the read/write bit
    pI2CHandle->pI2Cx->DR = slaveaddr;

    while (!(pI2CHandle->pI2Cx->SR1 & (1 << 1))); // Wait for ADDR flag
    pI2CHandle->pI2CHandle->SR1 &= ~(1 << 1);

    while (len > 0)
    {
        while (!(pI2CHandle->pI2Cx->SR1 & (1 << 7))); // Wait until TXE is set
        pI2CHandle->pI2CHandle->DR = *pTxBuffer++;
        len--;
    }

    while (!(pI2CHandle->pI2Cx->SR1 & (1 << 7))); // Wait until TXE is set
    while (!(pI2CHandle->pI2CHandle->SR1 & (1 << 2))); // Wait until BTF is set

    pI2CHandle->pI2CHandle->CR1 |= (1 << 9); // Generate STOP
}

void I2C_Master_Receive_Data(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr)
{
    pI2CHandle->pI2Cx->CR1 |= (1 << 8); // Generate Repeated Start
    while (!(pI2CHandle->pI2Cx->SR1 & 0x1)); // Wait for SB flag

    uint8_t slaveaddr = SlaveAddr << 1;
    slaveaddr |= (1 << 0); // Set the read bit
    pI2CHandle->pI2CHandle->DR = slaveaddr;

    while (!(pI2CHandle->pI2CHandle->SR1 & (1 << 1))); // Wait for ADDR flag

    *pRxBuffer = pI2CHandle->pI2CHandle->DR; // Read data
}

#endif /* INC_STM32F446RE_I2C_DRIVER_H_ */
