/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Olajumoke Aboderin
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>

int main(void) {
	/* Set Up */
	uint32_t volatile *const pClkCtrlreg = (uint32_t*) 0x40023830; //initialize pointer to the RCC Clock Enable Register
	uint32_t volatile *const pPortDModeReg = (uint32_t*) 0x40020C00; //initialize pointer to the GPIO Port D Mode Register
	uint32_t volatile *const pPortDOutReg = (uint32_t*) 0x40020C14; //initialize pointer to the GPIO Port D Output Data Register

	*pClkCtrlreg |= (1 << 3) | 0x01; //enable GPIO Port D and Port A Peripherals
	*pPortDModeReg &= ~(0xFF); //clear PD
	*pPortDModeReg |= (0x44 << 24); //PD13,15 as outputs (LEDs)

	uint32_t volatile *const pPortAModeReg = (uint32_t*) 0x40020000; //initialize pointer to the GPIO Port A Mode Register
	uint32_t const volatile *const pPortAInReg = (uint32_t*) 0x40020010; //initialize pointer to the GPIO Port A Input Data Register

	*pPortAModeReg &= ~(0x3 << 2); //clear A0 and set as input

	*pPortDOutReg |= (1 << 15); //turn blue LED on
	*pPortDOutReg &= ~(1 << 13); //turn orange LED off

	/* Loop forever */
	while (1) {
		//Flashing blue LED
		*pPortDOutReg |= (1 << 15); //turn blue LED on
		for (uint32_t i = 0; i < 160000; i++) {
		}
		*pPortDOutReg ^= (1 << 15); //toggle state of blue LED
		for (uint32_t j = 0; j < 160000; j++) {
		}

		while ((*pPortAInReg & 0x00000001)) {
			*pPortDOutReg |= (1 << 13); //orange LED on
			*pPortDOutReg |= (1 << 15); //turn blue LED on
			for (uint32_t k = 0; k < 160000; k++) {
			}
			*pPortDOutReg ^= (1 << 15); //toggle state of blue LED
			for (uint32_t l = 0; l < 160000; l++) {
			}
		}
		*pPortDOutReg &= ~(1 << 13); //turn orange LED off
	}
}
