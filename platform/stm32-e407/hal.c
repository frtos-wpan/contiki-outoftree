/*
 * hal.c - Substitute for cpu/avr/radio/rf230bb/halbb.c (atben on STM32-E407)
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

#include <stdint.h>
#include <stdio.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>

#include "../../contiki/cpu/avr/radio/rf230bb/at86rf230_registermap.h"

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
#define	BIT_IRQ		10
#define	PORT_nSEL	GPIOC
#define	BIT_nSEL	11
#define	PORT_MOSI	GPIOD
#define	BIT_MOSI	2
#define	PORT_SLP_TR	GPIOC
#define	BIT_SLP_TR	12
#define	PORT_MISO	GPIOC
#define	BIT_MISO	8
#define	PORT_SCLK	GPIOC
#define	BIT_SCLK	9

#define	GPIO_CONCAT(n)	GPIO##n
#define	GPIO(n)		GPIO_CONCAT(n)

#define	OUT(pin)	gpio_mode_setup(PORT_##pin, GPIO_MODE_OUTPUT, \
			    GPIO_PUPD_NONE, GPIO(BIT_##pin))
#define	IN(pin)		gpio_mode_setup(PORT_##pin, GPIO_MODE_INPUT, \
			    GPIO_PUPD_NONE, GPIO(BIT_##pin))
#define	SET(pin)	GPIO_BSRR(PORT_##pin) = GPIO(BIT_##pin)
#define	CLR(pin)	GPIO_BSRR(PORT_##pin) = GPIO(BIT_##pin) << 16

#define	PIN(pin)	!!(GPIO_IDR(PORT_##pin) & GPIO(BIT_##pin))

#define	EXTI_CONCAT(n)	EXTI##n
#define	EXTI(n)		EXTI_CONCAT(n)


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
	rx_frame->crc = true;	/* checked by hardware */
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


/* ----- Interrupts -------------------------------------------------------- */


void hal_enable_trx_interrupt(void)
{
	exti_enable_request(EXTI(BIT_IRQ));
}


void hal_disable_trx_interrupt(void)
{
	exti_disable_request(EXTI(BIT_IRQ));
}


static volatile int handled = 0;

void exti15_10_isr(void)
{
handled = 1;
	exti_reset_request(EXTI(BIT_IRQ));
}


/* ----- Initialization ---------------------------------------------------- */


#include <libopencm3/stm32/syscfg.h>

static void dump(void)
{
printf("NVIC(1) 40 en 0x%08x pend 0x%08x act 0x%08x\n",
    (unsigned) NVIC_ISER(1), (unsigned) NVIC_ISPR(1), (unsigned) NVIC_IABR(1));

printf("exti = 0x%x port = 0x%x\n", EXTI(BIT_IRQ), PORT_IRQ);
printf("irqmask 0x%08x\nevtmask 0x%08x\nrtriggr 0x%08x\n",
    (unsigned) EXTI_IMR, (unsigned) EXTI_EMR, (unsigned) EXTI_RTSR);
printf("ftriggr 0x%08x\nswevent 0x%08x\npending 0x%08x\n",
    (unsigned) EXTI_FTSR, (unsigned) EXTI_SWIER, (unsigned) EXTI_PR);

printf("map1 0x%04x\nmap2 0x%04x\n",
    (unsigned) SYSCFG_EXTICR1, (unsigned) SYSCFG_EXTICR2);
printf("map3 0x%04x\nmap4 0x%04x\n",
    (unsigned) SYSCFG_EXTICR3, (unsigned) SYSCFG_EXTICR4);
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
	IN(IRQ);

	hal_subregister_write(RG_TRX_CTRL_1, 1, 0, 0);

	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SYSCFGEN);
	nvic_enable_irq(NVIC_EXTI15_10_IRQ);
	exti_select_source(EXTI(BIT_IRQ), PORT_IRQ);
	exti_set_trigger(EXTI(BIT_IRQ), EXTI_TRIGGER_RISING);
	hal_enable_trx_interrupt();

printf("syscfg_exticr3 %p\n", &SYSCFG_EXTICR3);
printf("before %d 0x%x handled %d\n", PIN(IRQ),
    (unsigned) exti_get_flag_status(EXTI(BIT_IRQ)), handled);
dump();
	hal_subregister_write(RG_TRX_CTRL_1, 1, 0, 1);
printf("after %d 0x%x handled %d\n", PIN(IRQ),
    (unsigned) exti_get_flag_status(EXTI(BIT_IRQ)), handled);
dump();

	/* @@@ try to force transceiver into TRX_OFF ? */
}


void hal_test(void)
{
	uint8_t pn, vn;
	uint8_t m0, m1;

	hal_init();
	pn = hal_register_read(RG_PART_NUM);
	vn = hal_register_read(RG_VERSION_NUM);
	m0 = hal_register_read(RG_MAN_ID_0);
	m1 = hal_register_read(RG_MAN_ID_1);
	printf("part 0x%02x revision 0x%02x manufacturer xxxx%02x%02x\n",
	    pn, vn, m1, m0);
}
