/*
 * i2c_cfg.c
 *
 *  Created on: Dec 9, 2013
 *      Author: guoger
 */

/** EUI 64 address length is 8 bytes*/
#define MAC_ADDR_SIZE	0x8
/** In 24AA025E64, mac address is stored at 0xF8 ~ 0xFF */
#define EEPROM_ADDR		0xf8;
/** EUI64 chip address */
#define I2CDEV_S_ADDR	(0xA0 >> 1)

#include "lpc17xx_i2c.h"
#include "lpc17xx_libcfg.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_clkpwr.h"

void get_mac_addr(uint8_t *mac_buf) {
	PINSEL_CFG_Type PinCfg;
	I2C_M_SETUP_Type transferMCfg;

	uint8_t eeprom_addr = EEPROM_ADDR;

	/* Configure I2C0 */
	PinCfg.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.Funcnum = 1;
	PinCfg.Pinnum = 27;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 28;
	PINSEL_ConfigPin(&PinCfg);
	// Initialize Slave I2C peripheral
	/* Set up clock and power for I2C0 module */
	CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCI2C0, ENABLE);
	/* As default, peripheral clock for I2C0 module
	 * is set to FCCLK / 2 */
	CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_I2C0, CLKPWR_PCLKSEL_CCLK_DIV_2);


	transferMCfg.sl_addr7bit = I2CDEV_S_ADDR;
	transferMCfg.tx_data = &eeprom_addr;
	transferMCfg.tx_length = 1;
	transferMCfg.rx_data = mac_buf;
	transferMCfg.rx_length = MAC_ADDR_SIZE;
	transferMCfg.retransmissions_max = 3;
	I2C_MasterTransferData(LPC_I2C0, &transferMCfg, I2C_TRANSFER_POLLING);

	/** deinitialize I2C0 */
	LPC_I2C0->I2CONCLR = I2C_I2CONCLR_I2ENC;
	CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCI2C0, DISABLE);
}

int main(void) {
	uint8_t mac_buf[MAC_ADDR_SIZE];
	uint8_t i;
	uint8_t verify=0;

	SystemInit();
	/* init buffer */
	for(i=0; i<MAC_ADDR_SIZE; i++) {
		mac_buf[i] = 0;
	}

	LPC_GPIO1->FIODIR |= 0x20040000;
	LPC_GPIO1->FIOSET |= (1 << 29);
	LPC_GPIO1->FIOCLR |= (1 << 18);

	get_mac_addr(mac_buf);

	for(i=0; i<MAC_ADDR_SIZE; i++) {
		if(mac_buf[i] != 0){
			verify = 1;
			break;
		}
	}
	if (verify == 1) {
		LPC_GPIO1->FIOPIN |= (1<<29);
	} else {
		LPC_GPIO1->FIOPIN &= ~(1<<29);
	}

	while(1);
}
