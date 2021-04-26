#include "dali.h"
#include "boards.h"
#include "nrf_delay.h"

void dali_sendByte(uint8_t b);
void dali_sendBit(int b);
void dali_sendZero(void);
void dali_sendOne(void);

void dali_init(void)
{
	nrf_gpio_cfg_output(DALI_TX);

	nrf_gpio_pin_write(DALI_TX, 0);
}

void dali_tx(uint8_t cmd1, uint8_t cmd2) // transmit commands to DALI bus (address byte, command byte)
{
	dali_sendBit(1);
	dali_sendByte(cmd1);
	dali_sendByte(cmd2);
	nrf_gpio_pin_write(DALI_TX, 0);
}

void dali_sendByte(uint8_t b)
{
	for (int i = 7; i >= 0; i--)
	{
		dali_sendBit((b >> i) & 1);
	}
}

void dali_sendBit(int b)
{
	/*if(b)
		dali_sendOne();
	else
		dali_sendZero();*/
	if(b)
		dali_sendZero();
	else
		dali_sendOne();
}

void dali_sendZero(void)
{
	nrf_gpio_pin_write(DALI_TX, 1);
	nrf_delay_us(416);
	nrf_gpio_pin_write(DALI_TX, 0);
	nrf_delay_us(416);
}

void dali_sendOne(void)
{
	nrf_gpio_pin_write(DALI_TX, 0);
	nrf_delay_us(416);
	nrf_gpio_pin_write(DALI_TX, 1);
	nrf_delay_us(416);
}
