/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: Nov 9, 2022
 *      Author: Olajumoke Aboderin
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32fxx.h"

/*

 * Configuration Structure for USARTx peripherals

typedef struct{
	uint8_t USART_Mode;					//Possible values @USART_Mode
	uint32_t USART_Baud;				//Possible values @USART_Baud
	uint8_t USART_NoOfStopBits;			//Possible values @USART_NoOfStopBits
	uint8_t USART_WordLength;			//Possible values @USART_WordLength
	uint8_t USART_ParityControl;		//Possible values @USART_ParityControl
	uint8_t USART_HWFlowControl;		//Possible values @USART_HWFlowControl
}USART_Config_t;


 * Handle structure for USARTx peripherals

typedef struct{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
}USART_Handle_t;


 * @USART_Mode
 * Possible options for USART_Mode

enum USART_Mode_Options{
		USART_MODE_ONLY_TX,
		USART_MODE_ONLY_RX,
		USART_MODE_TXRX
};


 * @USART_Baud
 * Possible options for USART_Baud

#define USART_BAUD_1200				1200
#define USART_BAUD_2400				2400
#define USART_BAUD_9600				9600
#define USART_BAUD_19200			19200
#define USART_BAUD_38400			38400
#define USART_BAUD_57600			57600
#define USART_BAUD_115200			115200
#define USART_BAUD_230400			230400
#define USART_BAUD_460800			460800
#define USART_BAUD_921600			921600
#define USART_BAUD_2M				2000000
#define USART_BAUD_3M				3000000



 * @USART_NoOfStopBits
 * Possible Options for USART_NoOfStopBits

enum USART_NoOfStopBits_Options{
	USART_STOPBITS_1,
	USART_STOPBITS_0_5,
	USART_STOPBITS_2,
	USART_STOPBITS_1_5
};


 * @USART_WordLength
 * Possible Options for USART_WordLength

enum USART_WordLength_Options{
	USART_WORDLENGTH_8,
	USART_WORDLENGTH_9,
};


 * @USART_ParityControl
 * possible Options for USART_ParityControl

enum USART_Parity_Options{
	USART_PARITY_DISABLE,
	USART_PARITY_EN_EVEN,
	USART_PARITY_EN_ODD
};


 * @USART_HWFlowControl
 * possible options for USART_HWFlowControl

enum USART_HWFlowControl_Options{
	USART_HW_FLOW_CTRL_NONE,
	USART_HW_FLOW_CTRL_CTS,
	USART_HW_FLOW_CTRL_RTS,
	USART_HW_FLOW_CTRL_CTS_RTS
};


*************************************************************************************
 * 							APIs supported by this driver
 * 			For more information about the APIs check the functions definitions
 *************************************************************************************

 * Peripheral Clock Control

void USART_PClkCtrl(USART_RegDef_t *pUSARTx, uint8_t State);


 * Init and De-init

void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);


 * Data send and receive

void USART_SendData(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t length);
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t length);
uint8_t USART_SendDataIT(USART_Handle_T *pUSARTHandle, _RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t length);
uint8_t USART_ReceiveDataIT(USART_Handle_T *pUSARTHandle, _RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t length);


 * IRQ Config and ISR handling

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
USART_IRQHandling(USART_Handle_t *pHandle);


 * Other Peripheral Control APIs

void USART_PCtrl(USART_RegDef_T *pUSARTx, uint8_t State);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_T *pUSARTx, uint16_t StatusFlagName);




*/


#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
