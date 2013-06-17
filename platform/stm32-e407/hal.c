/*
 * hal.c - Substitute for cpu/avr/radio/rf230bb/halbb.c
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>


#include "hal.h"


#define	AT86RF230_REG_READ	0x80
#define	AT86RF230_REG_WRITE	0xc0
#define	AT86RF230_BUF_READ	0x20
#define	AT86RF230_BUF_WRITE	0x60


/* ----- I/O pin definitions ----------------------------------------------- */


/*
 * SD/MMC pin	atben signal	GPIO (Olimex STM32-E407)
 * ----------	------------	----
 * DAT2		IRQ		PC10
 * DAT3		nSEL		PC11
 * CMD		MOSI		PD2
 * CLK		SLP_TR		PC12
 * DAT0		MISO		PC8
 * DAT1		SCLK		PC9
 */

#define	PORT_IRQ	GPIOC
#define	BIT_IRQ		GPIO10
#define	PORT_nSEL	GPIOC
#define	BIT_nSEL	GPIO11
#define	PORT_MOSI	GPIOD
#define	BIT_MOSI	GPIO2
#define	PORT_SLP_TR	GPIOC
#define	BIT_SLP_TR	GPIO12
#define	PORT_MISO	GPIOC
#define	BIT_MISO	GPIO8
#define	PORT_SCLK	GPIOC
#define	BIT_SCLK	GPIO9

#define	OUT(pin)	gpio_mode_setup(PORT_##pin, GPIO_MODE_OUTPUT, \
			    GPIO_PUPD_NONE, BIT_##pin)
#define	IN(pin)		gpio_mode_setup(PORT_##pin, GPIO_MODE_INPUT, \
			    GPIO_PUPD_NONE, BIT_##pin)
#define	SET(pin)	GPIO_ODR(PORT_##pin) |= BIT_##pin
#define	CLR(pin)	GPIO_ODR(PORT_##pin) &= ~BIT_##pin

#define	PIN(pin)	!!(GPIO_IDR(PORT_##pin) & BIT_##pin)


/* ----- Control signals --------------------------------------------------- */


void hal_set_rst_low(void)
{
	/* not supported by hardware */
}


void hal_set_rst_high(void)
{
	/* not supported by hardware */
}


void hal_set_slptr_high(void)
{
	SET(SLP_TR);
}


void hal_set_slptr_low(void)
{
	CLR(SLP_TR);
}


bool hal_get_slptr(void)
{
	return PIN(SLP_TR);
}


void hal_enable_trx_interrupt(void)
{
	//@@@
}


void hal_disable_trx_interrupt(void)
{
	//@@@
}


void hal_init(void)
{
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);

	CLR(SCLK);
	SET(nSEL);
	CLR(SLP_TR);

	OUT(MOSI);
	IN(MISO);
	OUT(SCLK);
	OUT(nSEL);
	OUT(SLP_TR);
	//@@@ IN(IRQ);
}


/* ----- SPI bit-banging --------------------------------------------------- */


static void spi_begin(void)
{
	CLR(nSEL);
}


static void spi_end(void)
{
	SET(nSEL);
}


static void spi_send(uint8_t v)
{
	uint8_t mask;

	for (mask = 0x80; mask; mask >>= 1) {
		if (v & mask)
			SET(MOSI);
		else
			CLR(MOSI);
		SET(SCLK);
		CLR(SCLK);
	}
}


static uint8_t spi_recv(void)
{
	uint8_t res = 0;
	uint8_t mask;

	for (mask = 0x80; mask; mask >>= 1) {
		if (PIN(MISO))
			res |= mask;
		SET(SCLK);
		CLR(SCLK);
	}
	return res;
}


/* ----- Register access --------------------------------------------------- */


uint8_t hal_register_read(uint8_t address)
{
	uint8_t res;

	spi_begin();
	spi_send(AT86RF230_REG_READ | address);
	res = spi_recv();
	spi_end();
	return res;
}


void hal_register_write(uint8_t address, uint8_t value)
{
	spi_begin();
	spi_send(AT86RF230_REG_WRITE | address);
	spi_send(value);
	spi_end();
}



uint8_t hal_subregister_read(uint8_t address, uint8_t mask, uint8_t position)
{
	return (hal_register_read(address) & mask) >> position;
}


void hal_subregister_write(uint8_t address, uint8_t mask, uint8_t position,
    uint8_t value)
{
	uint8_t reg;

	//@@@ lock
	reg = hal_register_read(address);
	reg = (reg & ~mask) | (value << position);
	hal_register_write(address, reg);
	//@@@ lock
}


/* ----- Buffer access ----------------------------------------------------- */


void hal_frame_read(hal_rx_frame_t *rx_frame)
{
	uint8_t *buf = rx_frame->data;
	uint8_t i;

	spi_begin();
	spi_send(AT86RF230_BUF_READ);
	rx_frame->length = spi_recv();
	if (rx_frame->length > HAL_MAX_FRAME_LENGTH)
		rx_frame->length = HAL_MAX_FRAME_LENGTH;
	for (i = 0; i != rx_frame->length; i++)
		*(uint8_t *) buf++ = spi_recv();
	rx_frame->lqi = spi_recv();
        spi_end();

}


void hal_frame_write(uint8_t *write_buffer, uint8_t length)
{
	spi_begin();
	spi_send(AT86RF230_BUF_WRITE);
	spi_send(length);
	while (length--)
		spi_send(*(uint8_t *) write_buffer++);
	spi_end();
}


void hal_sram_read(uint8_t address, uint8_t length, uint8_t *data)
{
	/* not used */
}


void hal_sram_write(uint8_t address, uint8_t length, uint8_t *data)
{
	/* not used */
}
