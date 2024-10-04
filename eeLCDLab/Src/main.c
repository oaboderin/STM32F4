#include <stdint.h>
#include<string.h>
#include "stm32fxx.h"

void delay(uint16_t mult) {
	for (uint32_t i = 0; i < (mult * 16000); i++)
		;
}

void delayMs(uint32_t n) {
	uint32_t i;
	for (; n > 0; n--)
		for (i = 0; i < 3195; i++)
			;
}

void LCD_GPIO_Init(void) {
	GPIO_Handle_t GpioDpins, GpioCpins;

	GPIO_PClkCtrl(GPIOD, ENABLE);
	GPIO_PClkCtrl(GPIOC, ENABLE);

	GpioDpins.pGPIOx = GPIOD; //PD0 to PD7 for the data lines
	GpioDpins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioDpins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	GpioDpins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioDpins.GPIO_PinConfig.PinPuPdControl = GPIO_NO_PUPD;

	GpioCpins.pGPIOx = GPIOC; // PC0 to PC2 for control lines
	GpioCpins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioCpins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	GpioCpins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioCpins.GPIO_PinConfig.PinPuPdControl = GPIO_NO_PUPD;

	GpioDpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIO_init(&GpioDpins);
	GpioDpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
	GPIO_init(&GpioDpins);
	GpioDpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_init(&GpioDpins);
	GpioDpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_init(&GpioDpins);
	GpioDpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
	GPIO_init(&GpioDpins);
	GpioDpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIO_init(&GpioDpins);
	GpioDpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_init(&GpioDpins);
	GpioDpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_init(&GpioDpins);

	GpioCpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIO_init(&GpioCpins);
	GpioCpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
	GPIO_init(&GpioCpins);
	GpioCpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_init(&GpioCpins);

	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_1, LOW); //RW set to Write


}

void LCD_command(unsigned char input) {
	GPIO_WriteToOutputPort(GPIOD, input);
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_0, LOW); //RS to 0
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_1, LOW);
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_2, LOW);
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_2, HIGH); //EN high
	delayMs(10);
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_2, LOW); //EN low
	delayMs(20);

}

void LCD_Init(void) {
	unsigned char commands[] = {0x38,0x06,0x01,0x0F};
	delayMs(45);
	LCD_command(0x30);
	delayMs(15);
	LCD_command(0x30);
	delayMs(10);
	LCD_command(0x30);
	delayMs(1);

	for(uint8_t i = 0; i <= 3; i++){
		LCD_command(commands[i]);
	}

}

void LCD_data(char data) {
	GPIO_WriteToOutputPort(GPIOD, data);
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_0, HIGH); //RS to 0
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_1, LOW);
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_2, HIGH); //EN high
	delayMs(10);
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_2, LOW); //EN low
	delayMs(10);
}

int main(void) {
	LCD_GPIO_Init();
	LCD_Init();
	delayMs(10);


	char msg[24] = "HELLO WORLD";

	while (1) {
		LCD_command(0x01); // clear screen
		delayMs(30);
		LCD_command(0x2); // return cursor home
		delayMs(100);
		for(uint8_t i = 0; i <= 10; i++){
		LCD_data(msg[i]);
		}
		delay(60);// wait for about 1 sec
	}
	return 0;
}
