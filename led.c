/*
 * led.c
 *
 *  Created on: Dec 4, 2013
 *      Author: guoger
 */


#include "LPC17xx.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_gpio.h"

volatile unsigned long SysTickCnt;
extern void SystemInit(void);


void SysTick_Handler(void){
	SysTickCnt++;
}

void delay(unsigned long tick) {
	unsigned long systickcnt;

	systickcnt = SysTickCnt;
	while ((SysTickCnt - systickcnt) < tick);
}

int main(void) {
	SystemInit();
	LPC_GPIO1->FIODIR |= 0x20040000;
	LPC_GPIO1->FIOSET |= (1 << 29);
	LPC_GPIO1->FIOCLR |= (1 << 18);


    //LPC_GPIO1->FIOSET ^= (1<<29);
	SysTick_Config(SystemCoreClock/1000 - 1);

	while(1) {
		LPC_GPIO1->FIOPIN ^= (1 << 29);
		delay(500);
	}
}
