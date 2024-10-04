#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral clock control
 */
void GPIO_PClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t State) {
	if (State == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		} else if (pGPIOx == GPIOJ) {
			GPIOJ_PCLK_EN();
		} else if (pGPIOx == GPIOK) {
			GPIOK_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		} else if (pGPIOx == GPIOJ) {
			GPIOJ_PCLK_DI();
		} else if (pGPIOx == GPIOK) {
			GPIOK_PCLK_DI();
		}
	}
}

/*
 * initializing
 */
void GPIO_init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t temp = 0;
	/****Configure the pin mode****/
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		//No interrupts are being initialized
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //to populate the correct two bits in the mode register
		pGPIOHandle->pGPIOx->MODER &= ~(0x3
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
		//temp = 0;
	} else {
		//Interrupts are being initialized
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//Falling edge trigger interrupt
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//Rising edge trigger
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//Both rising and falling edge trigger interrupt
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);



		//Enable the EXTI interrupt delivery using the IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);



	}
	temp = 0;

	/****Configure the pin speed****/
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //to populate the correct two bits in the mode register
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	/****Configure pull up or pull dowm resistors****/
	temp = (pGPIOHandle->GPIO_PinConfig.PinPuPdControl
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //to populate the correct two bits in the mode register
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	/****Configure output type****/
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType
			<< (1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //to populate the correct two bits in the mode register
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1
			<< (1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	/****Configure alternate function type****/
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT) {
		//Alternate functions being initialized
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; //Determining which alternate function register to go with
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8; //the relative bit position in the register
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); //clear the 4 bits associated with the Pin Number
		pGPIOHandle->pGPIOx->AFR[temp1] |=
				(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * temp2));

	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RST();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RST();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RST();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RST();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RST();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RST();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RST();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RST();
	} else if (pGPIOx == GPIOI) {
		GPIOI_REG_RST();
	} else if (pGPIOx == GPIOJ) {
		GPIOJ_REG_RST();
	} else if (pGPIOx == GPIOK) {
		GPIOK_REG_RST();
	}
}

/*
 * Reading and writing data
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value;
	value = (uint16_t) (pGPIOx->IDR);
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t value) {
	if (value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t State){
	if(State == ENABLE){
		if(IRQNumber <= 31){
			//program iser0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){
			//program iser1 register
			*NVIC_ISER1 |= (1 << (IRQNumber% 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			//program iser2 register
			*NVIC_ISER1 |= (1 << (IRQNumber% 32));
		}
	}else{
		if (IRQNumber <= 31) {
			//program icer0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			//program icer1 register
			*NVIC_ICER1 |= (1 << (IRQNumber% 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			//program icer2 register
			*NVIC_ICER1 |= (1 << (IRQNumber% 32));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){
	//finding the ipr register
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}


void GPIO_IRQHandling(uint8_t PinNumber){
	if(EXTI->PR & (1 << PinNumber)){
		//clear
		EXTI->PR |= (1 << PinNumber); //clear the pending register to show that we have handled the ISR
	}
}
