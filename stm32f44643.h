/*
 * stm32f44643xx.h
 *
 * Created on: Jun 5, 2024
 * Author: Atharv Pachankar
 */

#ifndef INC_STM32F44643XX_H_
#define INC_STM32F44643XX_H_

#include "stm32f446re_spi_driver.h"

/* Base addresses for Memories */
#define FLASH_BASEADDR   0x08000000U
#define SRAM1_BASEADDR   0x20000000U
#define SRAM_BASEADDR    SRAM1_BASEADDR
#define ROM_BASEADDR     0x1FFF0000U

/* Base addresses for AHB and APB Peripherals */
#define PERI_BASE_ADDR   0x40000000U
#define APB1_PERIPH_BASE (PERI_BASE_ADDR)
#define APB2_PERIPH_BASE (0x40010000U)
#define AHB1_PERIPH_BASE (0x40020000U)
#define AHB2_PERIPH_BASE (0x50000000U)
#define AHB3_PERIPH_BASE (0x60000000U)

/* Base addresses for GPIOs */
#define GPIOA_BASEADDR   (AHB1_PERIPH_BASE + 0x0000U)
#define GPIOB_BASEADDR   (AHB1_PERIPH_BASE + 0x0400U)
#define GPIOC_BASEADDR   (AHB1_PERIPH_BASE + 0x0800U)
#define GPIOD_BASEADDR   (AHB1_PERIPH_BASE + 0x0C00U)
#define GPIOE_BASEADDR   (AHB1_PERIPH_BASE + 0x1000U)

/* Base addresses for RCC and EXTI */
#define RCC_BASEADDR     (AHB1_PERIPH_BASE + 0x3800U)
#define EXTI_BASEADDR    (APB2_PERIPH_BASE + 0x3C00U)

/* Base addresses for I2C */
#define I2C1_BASEADDR    (APB1_PERIPH_BASE + 0x05400U)
#define I2C2_BASEADDR    (APB1_PERIPH_BASE + 0x05800U)
#define I2C3_BASEADDR    (APB1_PERIPH_BASE + 0x05400U)  // Check if I2C3 is in the correct base address

/* Base addresses for USART/UART */
#define USART1_BASEADDR  (APB2_PERIPH_BASE + 0x1000U)
#define USART2_BASEADDR  (APB1_PERIPH_BASE + 0x4400U)
#define USART3_BASEADDR  (APB1_PERIPH_BASE + 0x4800U)
#define UART4_BASEADDR   (APB1_PERIPH_BASE + 0x4C00U)
#define UART5_BASEADDR   (APB1_PERIPH_BASE + 0x5000U)
#define USART6_BASEADDR  (APB2_PERIPH_BASE + 0x1400U)  // Check base address

/* Peripheral Definitions */
#define RCC              ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI             ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define GPIOA            ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB            ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC            ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD            ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE            ((GPIO_RegDef_t*)GPIOE_BASEADDR)

#define I2C1             ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2             ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3             ((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1           ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2           ((USART_RegDef_t*)USART2_BASEADDR)
#define USART3           ((USART_RegDef_t*)USART3_BASEADDR)
#define UART4            ((USART_RegDef_t*)UART4_BASEADDR)
#define UART5            ((USART_RegDef_t*)UART5_BASEADDR)
#define USART6           ((USART_RegDef_t*)USART6_BASEADDR)

/* Peripheral Clock Enable Macros */
#define GPIOA_PCLK_EN()  (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()  (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()  (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()  (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()  (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()  (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()  (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()  (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()  (RCC->AHB1ENR |= (1 << 8))

#define GPIOA_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()  (RCC->AHB1ENR &= ~(1 << 8))


#define USART1_PCLK_DI   (RCC->ABP2ENR&=~(1<<4))
#define USART2_PCLK_DI  (RCC->APB1ENR&=~(1<<17))
#define USART3_PCLK_DI  (RCC->APB1ENR&=~(1<<18))
#define USART4_PCLK_DI  (RCC->APB1ENR&=~(1<<19))
#define USART5_PCLK_DI  (RCC->APB1ENR&=~(1<<20))
#define USART6_PCLK_DI  (RCC->APB2ENR&=~(1<<5))


#define I2C1_PCLK_EN()   (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()   (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()   (RCC->APB1ENR |= (1 << 23))

/* Generic Macros */
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET

/* GPIO Register Definition */
typedef struct
{
    uint32_t MODER;   /*!< GPIO port mode register */
    uint32_t OTYPER;  /*!< GPIO port output type register */
    uint32_t OSPEEDR; /*!< GPIO port output speed register */
    uint32_t PUPDR;   /*!< GPIO port pull-up/pull-down register */
    uint32_t IDR;     /*!< GPIO port input data register */
    uint32_t ODR;     /*!< GPIO port output data register */
    uint32_t BSRR;    /*!< GPIO port bit set/reset register */
    uint32_t LCKR;    /*!< GPIO port configuration lock register */
} GPIO_RegDef_t;

/* I2C Register Definition */
typedef struct
{
    uint32_t I2C_CR1;    /*!< I2C control register 1 */
    uint32_t I2C_CR2;    /*!< I2C control register 2 */
    uint32_t I2C_OAR1;   /*!< I2C own address register 1 */
    uint32_t I2C_OAR2;   /*!< I2C own address register 2 */
    uint32_t I2C_DR;     /*!< I2C data register */
    uint32_t I2C_SR1;    /*!< I2C status register 1 */
    uint32_t I2C_SR2;    /*!< I2C status register 2 */
    uint32_t I2C_CCR;    /*!< I2C clock control register */
    uint32_t I2C_TRISE;  /*!< I2C rise time register */
    uint32_t I2C_FLTR;   /*!< I2C filter register */
} I2C_RegDef_t;

/* SPI Register Definition */
typedef struct
{
    uint32_t SPI_CR1;    /*!< SPI control register 1 */
    uint32_t SPI_CR2;    /*!< SPI control register 2 */
    uint32_t SPI_SR;     /*!< SPI status register */
    uint32_t SPI_DR;     /*!< SPI data register */
    uint32_t SPI_CRCPR;  /*!< SPI CRC polynomial register */
    uint32_t SPI_RXCRCR; /*!< SPI RX CRC register */
    uint32_t SPI_TXCRCR; /*!< SPI TX CRC register */
    uint32_t SPI_I2SCFGR;/*!< SPI I2S configuration register */
    uint32_t SPI_I2SPR;  /*!< SPI I2S prescaler register */
} SPI_RegDef_t;

/* USART Register Definition */
typedef struct
{
    uint32_t USART_SR;   /*!< USART status register */
    uint32_t USART_DR;   /*!< USART data register */
    uint32_t USART_BRR;  /*!< USART baud rate register */
    uint32_t USART_CR1;  /*!< USART control register 1 */
    uint32_t USART_CR2;  /*!< USART control register 2 */
    uint32_t USART_CR3;  /*!< USART control register 3 */
    uint32_t USART_GTPR; /*!< USART guard time and prescaler register */
} USART_RegDef_t;

/* RCC Register Definition */
typedef struct
{
    uint32_t RCC_CR;      /*!< RCC clock control register */
    uint32_t RCC_PLLCFGR; /*!< RCC PLL configuration register */
    uint32_t RCC_CFGR;    /*!< RCC clock configuration register */
    uint32_t RCC_CIR;     /*!< RCC clock interrupt register */
    uint32_t RCC_AHB1RSTR;/*!< RCC AHB1 peripheral reset register */
    uint32_t RCC_AHB2RSTR;/*!< RCC AHB2 peripheral reset register */
    uint32_t RCC_AHB3RSTR;/*!< RCC AHB3 peripheral reset register */
    uint32_t RCC_APB1RSTR;/*!< RCC APB1 peripheral reset register */
    uint32_t RCC_APB2RSTR;/*!< RCC APB2 peripheral reset register */
    uint32_t RCC_AHB1ENR; /*!< RCC AHB1 peripheral clock enable register */
    uint32_t RCC_AHB2ENR; /*!< RCC AHB2 peripheral clock enable register */
    uint32_t RCC_AHB3ENR; /*!< RCC AHB3 peripheral clock enable register */
    uint32_t RCC_APB1ENR; /*!< RCC APB1 peripheral clock enable register */
    uint32_t RCC_APB2ENR; /*!< RCC APB2 peripheral clock enable register */
    uint32_t RCC_AHB1LPENR; /*!< RCC AHB1 peripheral clock enable in low power mode register */
    uint32_t RCC_AHB2LPENR; /*!< RCC AHB2 peripheral clock enable in low power mode register */
    uint32_t RCC_AHB3LPENR; /*!< RCC AHB3 peripheral clock enable in low power mode register */
    uint32_t RCC_APB1LPENR; /*!< RCC APB1 peripheral clock enable in low power mode register */
    uint32_t RCC_APB2LPENR; /*!< RCC APB2 peripheral clock enable in low power mode register */
    uint32_t RCC_BDCR;    /*!< RCC Backup domain control register */
    uint32_t RCC_CSR;     /*!< RCC clock control & status register */
} RCC_RegDef_t;

/* EXTI Register Definition */
typedef struct
{
    uint32_t EXTI_IMR;    /*!< EXTI interrupt mask register */
    uint32_t EXTI_EMR;    /*!< EXTI event mask register */
    uint32_t EXTI_RTSR;   /*!< EXTI rising trigger selection register */
    uint32_t EXTI_FTSR;   /*!< EXTI falling trigger selection register */
    uint32_t EXTI_SWIER;  /*!< EXTI software interrupt event register */
    uint32_t EXTI_PR;     /*!< EXTI pending register */
} EXTI_RegDef_t;


#endif /* INC_STM32F44643XX_H_ */
