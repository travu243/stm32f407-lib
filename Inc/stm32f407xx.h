/*
 * stm32f407xx.h
 *
 *  Created on: August 4,2024
 *      Author: tra vu
 */
 
#ifndef STM32F407xx_H_
#define STM32F407xx_H_


#define ENABLE							1
#define DISABLE							0
#define SET								ENABLE
#define RESET							DISABLE
#define GPIO_PIN_SET					SET
#define GPIO_PIN_RESET					RESET
#define FLAG_SET						SET
#define FLAG_RESET						RESET
#define NO_PR_BITS_IMPLEMENTED			4

#define __vo 							volatile
#define __weak 							__attribute__((weak))

    /* exact-width signed integer types */
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;

    /* exact-width unsigned integer types */
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//								BASE ADDRESSES OF NESTED VECTORED INTERRUPT CONTROLLER								//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 

#define SCS_BASE            			(0xE000E000UL)                            /*!< System Control Space Base Address */

#define NVIC_BASE           			(SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address */

/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */	
#define NVIC_ISER0          			( (__vo uint32_t*)NVIC_BASE )
#define NVIC_ISER1          			( (__vo uint32_t*)NVIC_BASE + 0x4UL )
#define NVIC_ISER2          			( (__vo uint32_t*)NVIC_BASE + 0x8UL )
#define NVIC_ISER3         				( (__vo uint32_t*)NVIC_BASE + 0xCUL )

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 						( (__vo uint32_t*)NVIC_BASE + 0x80UL )
#define NVIC_ICER1						( (__vo uint32_t*)NVIC_BASE + 0x84UL )
#define NVIC_ICER2  					( (__vo uint32_t*)NVIC_BASE + 0x88UL )
#define NVIC_ICER3						( (__vo uint32_t*)NVIC_BASE + 0x8CUL )

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 				( (__vo uint32_t*)NVIC_BASE + 0x300UL )

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//											BASE ADDRESSES OF FLASH AND SRAM										//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#define FLASH_BASE						0x08000000U
#define SRAM1_BASE						0x20000000U									// 112kb
#define SRAM2_BASE						0x2001C000U									// 16kb
#define ROM_BASEADDR					0x1FFF0000U									// System memory
#define SRAM							SRAM1_BASEADDR

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//										AHBx AND APBx BUS PERIPHERAL BASE ADDRESSES									//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PERIPH_BASE						0x40000000UL
#define APB1PERIPH_BASE     	  		PERIPH_BASE
#define APB2PERIPH_BASE       			(PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       			(PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       			(PERIPH_BASE + 0x10000000UL)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//								BASE ADDRESSES OF PERIPHERAL WHICH IS ARE HANGING ON APB1							//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define TIM2_BASE             			(APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE             			(APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE             			(APB1PERIPH_BASE + 0x0800UL)
#define TIM5_BASE             			(APB1PERIPH_BASE + 0x0C00UL)
#define TIM6_BASE             			(APB1PERIPH_BASE + 0x1000UL)
#define TIM7_BASE             			(APB1PERIPH_BASE + 0x1400UL)
#define TIM12_BASE            			(APB1PERIPH_BASE + 0x1800UL)
#define TIM13_BASE            			(APB1PERIPH_BASE + 0x1C00UL)
#define TIM14_BASE            			(APB1PERIPH_BASE + 0x2000UL)

#define USART2_BASE           			(APB1PERIPH_BASE + 0x4400UL)
#define USART3_BASE           			(APB1PERIPH_BASE + 0x4800UL)
#define UART4_BASE            			(APB1PERIPH_BASE + 0x4C00UL)
#define UART5_BASE            			(APB1PERIPH_BASE + 0x5000UL)

#define SPI2_BASE             			(APB1PERIPH_BASE + 0x3800UL)
#define SPI3_BASE             			(APB1PERIPH_BASE + 0x3C00UL)

#define I2C1_BASE             			(APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE             			(APB1PERIPH_BASE + 0x5800UL)
#define I2C3_BASE             			(APB1PERIPH_BASE + 0x5C00UL)

#define CAN1_BASE             			(APB1PERIPH_BASE + 0x6400UL)
#define CAN2_BASE             			(APB1PERIPH_BASE + 0x6800UL)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//									BASE ADDRESSES OF PERIPHERAL WHICH IS ARE HANGING ON APB2	 				 	//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define TIM1_BASE             			(APB2PERIPH_BASE + 0x0000UL)
#define TIM8_BASE             			(APB2PERIPH_BASE + 0x0400UL)
#define TIM9_BASE             			(APB2PERIPH_BASE + 0x4000UL)
#define TIM10_BASE            			(APB2PERIPH_BASE + 0x4400UL)
#define TIM11_BASE            			(APB2PERIPH_BASE + 0x4800UL)

#define USART1_BASE           			(APB2PERIPH_BASE + 0x1000UL)
#define USART6_BASE           			(APB2PERIPH_BASE + 0x1400UL)

#define ADC1_BASE             			(APB2PERIPH_BASE + 0x2000UL)
#define ADC2_BASE             			(APB2PERIPH_BASE + 0x2100UL)
#define ADC3_BASE             			(APB2PERIPH_BASE + 0x2200UL)
#define ADC123_COMMON_BASE    			(APB2PERIPH_BASE + 0x2300UL)
#define ADC_BASE              			ADC123_COMMON_BASE

#define SPI1_BASE             			(APB2PERIPH_BASE + 0x3000UL)

#define SYSCFG_BASE           			(APB2PERIPH_BASE + 0x3800UL)
#define EXTI_BASE             			(APB2PERIPH_BASE + 0x3C00UL)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//								BASE ADDRESSES OF PERIPHERAL WHICH IS ARE HANGING ON AHB1							//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define GPIOA_BASE            			(AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE       			    (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE         			  	(AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            			(AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE            			(AHB1PERIPH_BASE + 0x1000UL)
#define GPIOF_BASE            			(AHB1PERIPH_BASE + 0x1400UL)
#define GPIOG_BASE            			(AHB1PERIPH_BASE + 0x1800UL)
#define GPIOH_BASE            			(AHB1PERIPH_BASE + 0x1C00UL)
#define GPIOI_BASE            			(AHB1PERIPH_BASE + 0x2000UL)

#define CRC_BASE              			(AHB1PERIPH_BASE + 0x3000UL)
#define RCC_BASE              			(AHB1PERIPH_BASE + 0x3800UL)

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/									PERIPHERAL REGISTER DEFINITIONS STRUCTURES									_/
//_/							SPI PERIPHERAL OF STM32F446XX FAMILY OF MCUs MAY BE DIFFERENT						_/
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

/*
 * GPIO Registers
 */
typedef struct
{
  __vo uint32_t MODER;    /*!< GPIO port mode register,               					Address offset: 0x00      */
  __vo uint32_t OTYPER;   /*!< GPIO port output type register,        					Address offset: 0x04      */
  __vo uint32_t OSPEEDR;  /*!< GPIO port output speed register,       					Address offset: 0x08      */
  __vo uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  					Address offset: 0x0C      */
  __vo uint32_t IDR;      /*!< GPIO port input data register,        					Address offset: 0x10      */
  __vo uint32_t ODR;      /*!< GPIO port output data register,        					Address offset: 0x14      */
  __vo uint32_t BSRR;     /*!< GPIO port bit set/reset register,      					Address offset: 0x18      */
  __vo uint32_t LCKR;     /*!< GPIO port configuration lock register, 					Address offset: 0x1C      */
  __vo uint32_t AFR[2];   /*!< GPIO alternate function registers(low and high),     	Address offset: 0x20-0x24 */
} GPIO_RegDef_t;

/*
 * RCC Registers
 */
typedef struct
{
  __vo uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  __vo uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  __vo uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  __vo uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  __vo uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  __vo uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  __vo uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  __vo uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  __vo uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  __vo uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  __vo uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  __vo uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  __vo uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  __vo uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  __vo uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  __vo uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  __vo uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  __vo uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __vo uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  __vo uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  __vo uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  __vo uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
} RCC_RegDef_t;

/*
 * EXTI Registers
 */
typedef struct
{
  __vo uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
  __vo uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
  __vo uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
  __vo uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
  __vo uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
  __vo uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_RegDef_t;

/*
 * SYSCFG Registers
 */
typedef struct
{
  __vo uint32_t MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  __vo uint32_t PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  __vo uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  uint32_t      RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
  __vo uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_RegDef_t;

/*
 * SPI Registers
 */
typedef struct
{
  __vo uint32_t CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
  __vo uint32_t CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
  __vo uint32_t SR;         /*!< SPI status register,                                Address offset: 0x08 */
  __vo uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
  __vo uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  __vo uint32_t RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
  __vo uint32_t TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
  __vo uint32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
  __vo uint32_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_RegDef_t;

/*
 * I2C Registers
 */
typedef struct
{
  __vo uint32_t CR1;        /*!< I2C Control register 1,     Address offset: 0x00 */
  __vo uint32_t CR2;        /*!< I2C Control register 2,     Address offset: 0x04 */
  __vo uint32_t OAR1;       /*!< I2C Own address register 1, Address offset: 0x08 */
  __vo uint32_t OAR2;       /*!< I2C Own address register 2, Address offset: 0x0C */
  __vo uint32_t DR;         /*!< I2C Data register,          Address offset: 0x10 */
  __vo uint32_t SR1;        /*!< I2C Status register 1,      Address offset: 0x14 */
  __vo uint32_t SR2;        /*!< I2C Status register 2,      Address offset: 0x18 */
  __vo uint32_t CCR;        /*!< I2C Clock control register, Address offset: 0x1C */
  __vo uint32_t TRISE;      /*!< I2C TRISE register,         Address offset: 0x20 */
} I2C_RegDef_t;

/*
 * USART Registers
 */
typedef struct
{
  __vo uint32_t SR;         /*!< USART Status register,                   Address offset: 0x00 */
  __vo uint32_t DR;         /*!< USART Data register,                     Address offset: 0x04 */
  __vo uint32_t BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
  __vo uint32_t CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
  __vo uint32_t CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
  __vo uint32_t CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
  __vo uint32_t GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_RegDef_t;

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/											 PERIPHERAL DEFINITIONS 											_/
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

/*
 * GPIOx Registers
 */
#define GPIOA							((GPIO_RegDef_t*)GPIOA_BASE)
#define GPIOB							((GPIO_RegDef_t*)GPIOB_BASE)
#define GPIOC							((GPIO_RegDef_t*)GPIOC_BASE)
#define GPIOD							((GPIO_RegDef_t*)GPIOD_BASE)
#define GPIOE							((GPIO_RegDef_t*)GPIOE_BASE)
#define GPIOF							((GPIO_RegDef_t*)GPIOF_BASE)
#define GPIOG							((GPIO_RegDef_t*)GPIOG_BASE)
#define GPIOH							((GPIO_RegDef_t*)GPIOH_BASE)
#define GPIOI               			((GPIO_RegDef_t*)GPIOI_BASE)

/*
 * RCC Registers
 */
#define RCC								((RCC_RegDef_t*)RCC_BASE)

/*
 * EXTI Registers
 */
#define EXTI							((EXTI_RegDef_t*)EXTI_BASE)

/*
 * SYSCFG Registers
 */
#define SYSCFG							((SYSCFG_RegDef_t*)SYSCFG_BASE)

/*
 * I2C Registers
 */
#define I2C1							((I2C_RegDef_t*)I2C1_BASE)
#define I2C2							((I2C_RegDef_t*)I2C2_BASE)
#define I2C3							((I2C_RegDef_t*)I2C3_BASE)

/*
 * SPI Registers
 */
#define SPI1							((SPI_RegDef_t*)SPI1_BASE)
#define SPI2							((SPI_RegDef_t*)SPI2_BASE)
#define SPI3							((SPI_RegDef_t*)SPI3_BASE)

/*
 * USART Registers
 */
#define USART1  						((USART_RegDef_t*)USART1_BASE)
#define USART2  						((USART_RegDef_t*)USART2_BASE)
#define USART3  						((USART_RegDef_t*)USART3_BASE)
#define UART4  							((USART_RegDef_t*)UART4_BASE)
#define UART5  							((USART_RegDef_t*)UART5_BASE)
#define USART6  						((USART_RegDef_t*)USART6_BASE)

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/										 CLOCK ENABLE MACROs FOR PERIPHERAL 									_/
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

/*
 * Clock Enable Macros For GPIOx Peripheral
 */
#define GPIOA_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOF_PCLK_EN()					( RCC->AHB1ENR |=  (1 << 5 ) )
#define GPIOG_PCLK_EN()					( RCC->AHB1ENR |=  (1 << 6 ) )
#define GPIOH_PCLK_EN()					( RCC->AHB1ENR |=  (1 << 7 ) )
#define GPIOI_PCLK_EN()					( RCC->AHB1ENR |=  (1 << 8 ) )

/*
 * Clock Enable Macros For SYSCFG Peripheral
 */
#define SYSCFG_PCLK_EN()				( RCC->APB2ENR |= ( 1 << 14 ) )

/*
 * Clock Enable Macros For I2Cx Peripheral
 */
#define I2C1_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 21 ) )
#define I2C2_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 22 ) )
#define I2C3_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 23 ) )

/*
 * Clock Enable Macros For SPIx Peripheral
 */
#define SPI1_PCLK_EN()					( RCC->APB2ENR |= ( 1 << 12 ) )
#define SPI2_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 14 ) )
#define SPI3_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 15 ) )

/*
 * Clock Enable Macros For USARTx Peripheral
 */
#define USART1_PCCK_EN() 				( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCCK_EN() 				( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCCK_EN() 				( RCC->APB1ENR |= (1 << 18) )
#define UART4_PCCK_EN()  				( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCCK_EN()  				( RCC->APB1ENR |= (1 << 20) )
#define USART6_PCCK_EN() 				( RCC->APB2ENR |= (1 << 5) )

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/									 CLOCK DISABLE MACROs FOR PERIPHERAL										_/
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

/*
 * Clock Disable Macros For GPIOx Peripheral
 */
#define GPIOA_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOF_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 5 ) )
#define GPIOG_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 6 ) )
#define GPIOH_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 7 ) )
#define GPIOI_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 8 ) )

/*
 * Clock Disable Macros For I2Cx Peripheral
 */
#define I2C1_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 21 ) )
#define I2C2_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 22 ) )
#define I2C3_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 23 ) )

/*
 * Clock Disable Macros For SPIx Peripheral
 */
#define SPI1_PCLK_DI()					( RCC->APB2ENR &= ~( 1 << 12 ) )
#define SPI2_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 14 ) )
#define SPI3_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 15 ) )

/*
 * Clock Disable Macros For USARTx Peripheral
 */
#define USART1_PCCK_DI() 				( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCCK_DI() 				( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCCK_DI() 				( RCC->APB1ENR &= ~(1 << 18) )
#define UART4_PCCK_DI()  				( RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCCK_DI()  				( RCC->APB1ENR &= ~(1 << 20) )
#define USART6_PCCK_DI() 				( RCC->APB2ENR &= ~(1 << 5) )

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/								***** CLOCK RESET MACROs FOR PERIPHERAL *****									_/
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
/*
 * Macros Reset GPIOx Peripheral
 */
#define GPIOA_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

/*
 * Macros Reset I2Cx Peripheral
 */
#define I2C1_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); }while(0)

/*
 * Macros Reset SPIx Peripheral
 */
#define SPI1_REG_RESET()				do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)

/*
 * Macros Reset USARTx Peripheral
 */
#define USART1_REG_RESET()				do{ (RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define USART2_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 12)); }while(0)
#define USART3_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 12)); }while(0)
#define UART4_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 12)); }while(0)
#define UART5_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 12)); }while(0)
#define USART6_REG_RESET()				do{ (RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/											***** DEFINE MACRO *****											_/
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:					\
										(x == GPIOB)?1:					\
										(x == GPIOC)?2:					\
										(x == GPIOD)?3:					\
								        (x == GPIOE)?4:					\
								        (x == GPIOF)?5:					\
								        (x == GPIOG)?6:					\
								        (x == GPIOG)?7:					\
								        (x == GPIOH)?8:0)
								        
/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 */

#define IRQ_NO_EXTI0 					6
#define IRQ_NO_EXTI1 					7
#define IRQ_NO_EXTI2 					8
#define IRQ_NO_EXTI3 					9
#define IRQ_NO_EXTI4 					10
#define IRQ_NO_EXTI9_5 					23
#define IRQ_NO_EXTI15_10 				40

#define IRQ_NO_SPI1						35
#define IRQ_NO_SPI2         			36
#define IRQ_NO_SPI3         			51

#define IRQ_NO_I2C1_EV     				31
#define IRQ_NO_I2C1_ER     				32
#define IRQ_NO_I2C2_EV     				33
#define IRQ_NO_I2C2_ER     				34
#define IRQ_NO_I2C3_EV     				72
#define IRQ_NO_I2C3_ER     				73

#define IRQ_NO_USART1	    			37
#define IRQ_NO_USART2	    			38
#define IRQ_NO_USART3	    			39
#define IRQ_NO_UART4	    			52
#define IRQ_NO_UART5	    			53
#define IRQ_NO_USART6	    			71

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    				0
#define NVIC_IRQ_PRI1    				1
#define NVIC_IRQ_PRI2    				2
#define NVIC_IRQ_PRI3    				3
#define NVIC_IRQ_PRI4    				4
#define NVIC_IRQ_PRI5    				5
#define NVIC_IRQ_PRI6    				6
#define NVIC_IRQ_PRI7    				7
#define NVIC_IRQ_PRI8    				8
#define NVIC_IRQ_PRI9    				9
#define NVIC_IRQ_PRI15    				15


/*
 * peripheral register definition structure for RCC
 */
#define RCC_CR_HSION    0
#define RCC_CR_HSIRDY   1
#define RCC_CR_HSEON    16
#define RCC_CR_HSERDY   17
#define RCC_CR_HSYBYP   18
#define RCC_CR_CSSON    19
#define RCC_CR_PLLON    24
#define RCC_CR_PLLRDY   25
#define RCC_CR_PLL2SON  26
#define RCC_CR_PLL2SRDY 27

#define RCC_CFGR_SW0    0
#define RCC_CFGR_SW1    1
#define RCC_CFGR_SWS0   2
#define RCC_CFGR_SWS1   3

/**********  Pre-scaller options  ***********/

#define RCC_AHB_NO_PRE_SCAL            0
#define RCC_AHB_PRE_SCAL_BY_2          8
#define RCC_AHB_PRE_SCAL_BY_4          9
#define RCC_AHB_PRE_SCAL_BY_8          10
#define RCC_AHB_PRE_SCAL_BY_16         11
#define RCC_AHB_PRE_SCAL_BY_64         12
#define RCC_AHB_PRE_SCAL_BY_128        13
#define RCC_AHB_PRE_SCAL_BY_256        14
#define RCC_AHB_PRE_SCAL_BY_512        15

#define RCC_APB_NO_PRE_SCAL             0
#define RCC_APB_PRE_SCAL_BY_2           4
#define RCC_APB_PRE_SCAL_BY_4           5
#define RCC_APB_PRE_SCAL_BY_8           6
#define RCC_APB_PRE_SCAL_BY_16          7

/************************* system Clock Source ******************/

#define RCC_SYSCLK_HSE_CRYSTAL          0
#define RCC_SYSCLK_HSE_RC               1
#define RCC_SYSCLK_HSI                  2
#define RCC_SYSCLK_PLL                  3



#endif /* STM32F407xx_H_ */



