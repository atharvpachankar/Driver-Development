#ifndef INC_STM32F446RE_GPIO_DRIVER_H_
#define INC_STM32F446RE_GPIO_DRIVER_H_

#include "stm32f44643xx.h"

// GPIO Pin Numbers
#define GPIO_PIN_NO_0   0
#define GPIO_PIN_NO_1   1
#define GPIO_PIN_NO_2   2
#define GPIO_PIN_NO_3   3
#define GPIO_PIN_NO_4   4
#define GPIO_PIN_NO_5   5
#define GPIO_PIN_NO_6   6
#define GPIO_PIN_NO_7   7
#define GPIO_PIN_NO_8   8
#define GPIO_PIN_NO_9   9
#define GPIO_PIN_NO_10  10
#define GPIO_PIN_NO_11  11
#define GPIO_PIN_NO_12  12
#define GPIO_PIN_NO_13  13
#define GPIO_PIN_NO_14  14
#define GPIO_PIN_NO_15  15

// GPIO Modes
#define GPIO_MODE_IN      0
#define GPIO_MODE_OUT     1
#define GPIO_MODE_ALTN    2
#define GPIO_MODE_ANALOG  3
#define GPIO_MODE_IT_FT   4
#define GPIO_MODE_IT_RT   5
#define GPIO_MODE_IT_RFT  6

// GPIO Output Types
#define OP_TYPE_PushPull   0
#define OP_TYPE_OpenDrain  1

// GPIO Speeds
#define GPIO_SPEED_LOW      1
#define GPIO_SPEED_MEDIUM   2
#define GPIO_SPEED_FAST     3
#define GPIO_SPEED_HIGH     4

// GPIO Pull-up/Pull-down
#define N0_PULL_UP   0
#define PULL_UP      1
#define PULL_DOWN    2

typedef struct
{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltMode;
} GPIO_PinConfig_t;

typedef struct
{
    GPIO_RegDef *    pGPIOx;
    GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

// API Functions

void GPIO_PeriClockControl(GPIO_RegDef * pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pGPIOx == GPIOA) { GPIOA_PCLK_EN(); }
        else if (pGPIOx == GPIOB) { GPIOB_PCLK_EN(); }
        else if (pGPIOx == GPIOC) { GPIOC_PCLK_EN(); }
        else if (pGPIOx == GPIOD) { GPIOD_PCLK_EN(); }
        else if (pGPIOx == GPIOE) { GPIOE_PCLK_EN(); }
        else if (pGPIOx == GPIOF) { GPIOF_PCLK_EN(); }
        else if (pGPIOx == GPIOG) { GPIOG_PCLK_EN(); }
        else if (pGPIOx == GPIOH) { GPIOH_PCLK_EN(); }
        else if (pGPIOx == GPIOI) { GPIOI_PCLK_EN(); }
    }
    else
    {
        if (pGPIOx == GPIOA) { GPIOA_PCLK_DI(); }
        else if (pGPIOx == GPIOB) { GPIOB_PCLK_DI(); }
        else if (pGPIOx == GPIOC) { GPIOC_PCLK_DI(); }
        else if (pGPIOx == GPIOD) { GPIOD_PCLK_DI(); }
        else if (pGPIOx == GPIOE) { GPIOE_PCLK_DI(); }
        else if (pGPIOx == GPIOF) { GPIOF_PCLK_DI(); }
        else if (pGPIOx == GPIOG) { GPIOG_PCLK_DI(); }
        else if (pGPIOx == GPIOH) { GPIOH_PCLK_DI(); }
        else if (pGPIOx == GPIOI) { GPIOI_PCLK_DI(); }
    }
}

void GPIO_Init(GPIO_Handle_t * pGPIOHandle)
{
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
       pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    }
    else
    {
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            EXTI->FTSR |= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
            EXTI->RTSR &= ~(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            EXTI->RTSR |= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
            EXTI->FTSR &= ~(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
        {
            EXTI->FTSR |= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
            EXTI->RTSR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    }
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber)
{
    return (uint8_t)((pGPIO->IDR >> PinNumber) & 0x00000001);
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    return (uint16_t)pGPIOx->IDR;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if (Value == GPIO_PIN_SET)
    {
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else
    {
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}

void GPIO_IRQConfig(uint32_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (IRQNumber <= 31)
        {
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
    else
    {
        if (IRQNumber <= 31)
        {
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
        }
    }
}

#endif /* INC_STM32F446RE_GPIO_DRIVER_H_ */
