/*
 * stm32fxx.h
 *
 *  Created on: Oct, 2022
 *      Author: Olajumoke Aboderin
 */

#ifndef INC_STM32FXX_H_
#define INC_STM32FXX_H_

#define __vo volatile
#include <stdint.h>

/*************************Processor Specific Details*************************
 * ARM Cortex M4 Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0						((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1						((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2						((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3						((__vo uint32_t*)0xE000E10C)

#define NVIC_ICER0						((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1						((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2						((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3						((__vo uint32_t*)0xE000E18C)

#define NVIC_PR_BASEADDR				((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED			4

/*
 * Base address of flash and SRAM memories
 */

#define FLASH_BASEADDR					0x08000000U
#define SRAM1_BASEADDR 					0x20000000U
#define SRAM2_BASEADDR					0x2001C000U
#define ROM								0x1FFF0000U
#define SRAM 							SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR					0x40000000U
#define APB1PERIPH_BASEADDR				PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR				0x40010000U
#define AHB1PERIPH_BASEADDR				0x40020000U
#define AHB2PERIPH_BASEADDR				0x50000000U

/*
 * Base addresses of peripherals on AHB1 bus
 */
#define GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0X0C00)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR					(AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR					(AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR					(AHB1PERIPH_BASEADDR + 0x2800)

#define RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3800)


/*
 * Base addresses of peripherals on APB1 bus
 */
#define SPI2_BASEADDR					(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR					(APB1PERIPH_BASEADDR + 0x3C00)

#define I2C1_BASEADDR					(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR					(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR					(APB1PERIPH_BASEADDR + 0x5C00)

#define USART2_BASEADDR					(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR					(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR					(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR					(APB1PERIPH_BASEADDR + 0x5000)


/*
 * Base addresses of peripherals on APB2 bus
 */
#define SPI1_BASEADDR					(APB1PERIPH_BASEADDR + 0x3000)

#define EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0x3C00)

#define USART1_BASEADDR					(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR					(APB2PERIPH_BASEADDR + 0x1400)

#define SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR + 0x3800)

/*****************Peripheral Register Definition Structures*****************/
typedef struct {
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;
}RCC_RegDef_t;

typedef struct {
	__vo uint32_t MODER; /*GPIO port mode register,	address offset:	0x00	*/
	__vo uint32_t OTYPER; /*GPIO port output type register,	address offset:	0x04	*/
	__vo uint32_t OSPEEDR; /*GPIO port output speed register, address offset:	0x08	*/
	__vo uint32_t PUPDR; /*GPIO port pull-up/pull-down register, address offset:	0x0C	*/
	__vo uint32_t IDR; /*GPIO port input data register, address offset:	0x10	*/
	__vo uint32_t ODR; /*GPIO port output data register, address offset:	0x14	*/
	__vo uint32_t BSRRL; /*GPIO port bit set/reset register, address offset:	0x18	*/
	__vo uint32_t BSRRH; /*GPIO port bit set/reset register, address offset:	0x1A	*/
	__vo uint32_t LCKR; /*GPIO port configuration lock register, address offset:	0x1C	*/
	__vo uint32_t AFR[2]; /*GPIO alternate function register, address offset:	0x20-0X24	*/
} GPIO_RegDef_t;


typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWTER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

typedef struct{
	__vo uint32_t USART_SR;
	__vo uint32_t USART_DR;
	__vo uint32_t USART_BRR;
	__vo uint32_t USART_CR1;
	__vo uint32_t USART_CR2;
	__vo uint32_t USART_CR3;
	__vo uint32_t USART_GTPR;
}USART_RegDef_t;


/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI					((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ					((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK					((GPIO_RegDef_t*)GPIOK_BASEADDR)

#define USART1					((USART_RegDef_t*)USART1_BASEADDR)
#define USART2					((USART_RegDef_t*)USART2_BASEADDR)
#define USART3					((USART_RegDef_t*)USART3_BASEADDR)
#define UART4					((USART_RegDef_t*)UART4_BASEADDR)
#define UART5					((USART_RegDef_t*)UART5_BASEADDR)
#define USART6					((USART_RegDef_t*)USART6_BASEADDR)

#define RCC 					((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI 					((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * Clock Enable Macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_EN()	( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()	( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()	( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()	( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()	( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()	( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()	( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()	( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()	( RCC->AHB1ENR |= (1 << 8) )
#define GPIOJ_PCLK_EN()	( RCC->AHB1ENR |= (1 << 9) )
#define GPIOK_PCLK_EN()	( RCC->AHB1ENR |= (1 << 10))

/*
 * Clock Enable Macros for UARTx/USARTx peripherals
 */
#define USART1_PCLK_EN()	( RCC->APB2ENR |= (1 << 4)  )
#define USART2_PCLK_EN()	( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()	( RCC->APB1ENR |= (1 << 18) )
#define UART4_PCLK_EN()		( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN()		( RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN()	( RCC->APB2ENR |= (1 << 5)  )

/*
 * Clock Enable Macro for SYSCFG & IRQ
 */
#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= (1 << 14)  )


/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 8) )
#define GPIOJ_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 9) )
#define GPIOK_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 10))

/*
 * Reset GPIO Peripherals
 */
#define GPIOA_REG_RST()	do{( RCC->AHB1RSTR |= (1 << 0) ); ( RCC->AHB1RSTR &= ~(1 << 0) );}while(0)
#define GPIOB_REG_RST()	do{( RCC->AHB1RSTR |= (1 << 1) ); ( RCC->AHB1RSTR &= ~(1 << 1) );}while(0)
#define GPIOC_REG_RST()	do{( RCC->AHB1RSTR |= (1 << 2) ); ( RCC->AHB1RSTR &= ~(1 << 2) );}while(0)
#define GPIOD_REG_RST()	do{( RCC->AHB1RSTR |= (1 << 3) ); ( RCC->AHB1RSTR &= ~(1 << 3) );}while(0)
#define GPIOE_REG_RST()	do{( RCC->AHB1RSTR |= (1 << 4) ); ( RCC->AHB1RSTR &= ~(1 << 4) );}while(0)
#define GPIOF_REG_RST()	do{( RCC->AHB1RSTR |= (1 << 5) ); ( RCC->AHB1RSTR &= ~(1 << 5) );}while(0)
#define GPIOG_REG_RST()	do{( RCC->AHB1RSTR |= (1 << 6) ); ( RCC->AHB1RSTR &= ~(1 << 6) );}while(0)
#define GPIOH_REG_RST()	do{( RCC->AHB1RSTR |= (1 << 7) ); ( RCC->AHB1RSTR &= ~(1 << 7) );}while(0)
#define GPIOI_REG_RST()	do{( RCC->AHB1RSTR |= (1 << 8) ); ( RCC->AHB1RSTR &= ~(1 << 8) );}while(0)
#define GPIOJ_REG_RST()	do{( RCC->AHB1RSTR |= (1 << 9) ); ( RCC->AHB1RSTR &= ~(1 << 9) );}while(0)
#define GPIOK_REG_RST()	do{( RCC->AHB1RSTR |= (1 << 10) ); ( RCC->AHB1RSTR &= ~(1 << 10) );}while(0)


/*
 * Clock Disable Macros for UARTx/USARTx peripherals
 */
#define USART1_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 4)  )
#define USART2_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 18) )
#define UART4_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 20) )
#define USART6_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 5)  )

//IRQ Macros
#define IRQ_EXTI0			6
#define IRQ_EXTI1			7
#define IRQ_EXTI2			8
//#define IRQ_EXTI3			9
//#define IRQ_EXTI4			2
#define IRQ_EXTI9_5			23
#define IRQ_EXTI5_10		40



//Generic Macros

#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define HIGH			1
#define LOW				0

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_usart_driver.h"


#endif /* INC_STM32FXX_H_ */
