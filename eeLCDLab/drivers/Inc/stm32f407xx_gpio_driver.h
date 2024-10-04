
#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32fxx.h"

/*
 * This is the configuration structure for a GPIO pin
 */
typedef struct {
	uint8_t GPIO_PinNumber;			//Possible values @GPIO_PIN_NO
	uint8_t GPIO_PinMode;			//Possible values @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			//Possible values @GPIO_PIN_SPEED
	uint8_t PinPuPdControl;			//Possible values @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType;			//Possible values @GPIO_PIN_OPTYPE
	uint8_t GPIO_PinAltFuncMode;	//Possible values @GPIO_PIN_ALTFUNC
} GPIO_PinConfig_t;

/*
 * This is a Handle Structure for a GPIO pin
 */
typedef struct {
	GPIO_RegDef_t *pGPIOx;//this holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

/*
 * @GPIO_PIN_NO
 * GPIO pin numbers
 */
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/*
 * @GPIO_PIN_MODES
 * GPIO pin modes
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALT		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_PIN_OPTYPE
 * GPIO pin output types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * @GPIO_PIN_SPEED
 * GPIO output speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_HIGH		2
#define GPIO_SPEED_VERYHIGH	3

/*
 * @GPIO_PIN_PUPD
 * GPIO pull up or pull down
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

/*
 * General Macros
 */
#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0:\
									 (x == GPIOB) ? 1:\
									 (x == GPIOC) ? 2:\
									 (x == GPIOD) ? 3:\
									 (x == GPIOE) ? 4:\
									 (x == GPIOF) ? 5:\
									 (x == GPIOG) ? 6:\
									 (x == GPIOH) ? 7:\
									 (x == GPIOI) ? 8: 0)

/********************************************************
 * 					APIs supported by this driver
 *********************************************************/

/*
 * Peripheral clock control
 */
void GPIO_PClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t State);

/*
 * initializing
 */
void GPIO_init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Reading and writing data
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR
 */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t State);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
