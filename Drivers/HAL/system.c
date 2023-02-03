/**
 * @file system.c
 * @author cy023
 * @date 2022.08.16
 * @brief
 */
#include <stdio.h>

#include "system.h"
#include "systick.h"
#include "same54p20a.h"

/*******************************************************************************
 * Static Functions
 ******************************************************************************/

static void system_clock_init(void)
{
    // XOSC1 PIN multiplxer set
    PORT_REGS->GROUP[1].PORT_PMUX[11] = PORT_PMUX_PMUXO_N | PORT_PMUX_PMUXE_N;
    PORT_REGS->GROUP[1].PORT_PINCFG[22] |= PORT_PINCFG_PMUXEN(1);
    PORT_REGS->GROUP[1].PORT_PINCFG[23] |= PORT_PINCFG_PMUXEN(1);

    // XOSC1 set
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] &= OSCCTRL_XOSCCTRL_ONDEMAND(0);  // oscillator always on
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] |= OSCCTRL_XOSCCTRL_IMULT(4);     // oscillator current multiplier
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] |= OSCCTRL_XOSCCTRL_IPTAT(3);     // oscillator current reference
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] |= OSCCTRL_XOSCCTRL_XTALEN(1);    // internal oscillator circuit enable
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] |= OSCCTRL_XOSCCTRL_ENABLE(1);    // oscillator enable
    while (!(OSCCTRL_REGS->OSCCTRL_STATUS & OSCCTRL_STATUS_XOSCRDY1_Msk)); // wait for XOSC1 ready

    // GCLK 0 set Internal oscillator 48MHz
    // GCLK_REGS->GCLK_GENCTRL[0] |= GCLK_GENCTRL_DIV(2);
    // GCLK_REGS->GCLK_GENCTRL[0] |= GCLK_GENCTRL_SRC(8);
    // GCLK_REGS->GCLK_GENCTRL[0] |= GCLK_GENCTRL_IDC(1);
    // GCLK_REGS->GCLK_GENCTRL[0] |= GCLK_GENCTRL_GENEN(1);
    // while (GCLK_REGS->GCLK_SYNCBUSY & GCLK_SYNCBUSY_GENCTRL_GCLK0); // wait gclk 0 sync

    // GCLK 2 set External oscillator 12MHz
    GCLK_REGS->GCLK_GENCTRL[2] |= GCLK_GENCTRL_DIV(1); // gclk 2 output = src clk / 1
    GCLK_REGS->GCLK_GENCTRL[2] |= GCLK_GENCTRL_SRC(1); // gclk 2 use xosc1 as source
    GCLK_REGS->GCLK_GENCTRL[2] |= GCLK_GENCTRL_IDC(1);
	GCLK_REGS->GCLK_GENCTRL[2] |= GCLK_GENCTRL_GENEN(1);
    while (GCLK_REGS->GCLK_SYNCBUSY & GCLK_SYNCBUSY_GENCTRL_GCLK2); // wait gclk 2 sync

    // Peripheral clock set
    GCLK_REGS->GCLK_PCHCTRL[7]  |= GCLK_PCHCTRL_GEN_GCLK2;   // select gclk 2 as source
    GCLK_REGS->GCLK_PCHCTRL[7]  |= GCLK_PCHCTRL_CHEN(1);     // open sercom 0 clock

    // MCLK set
    MCLK_REGS->MCLK_APBAMASK |= MCLK_APBAMASK_SERCOM0(1);    // sercom 0 bus open
}

static void system_clock_deinit(void)
{
    // GCLK reset
    GCLK_REGS->GCLK_CTRLA = GCLK_CTRLA_SWRST(1);
    while (GCLK_REGS->GCLK_SYNCBUSY & GCLK_SYNCBUSY_SWRST_Msk);
}

static void system_power_init(void)
{
    // PCONP = PCONP_UART0 | PCONP_GPIO | PCONP_ENET;
}

static void system_power_deinit(void)
{
    // PCONP = 0;
}

static void system_uart0_init(void)
{
    // UART0 PIN multiplexer set
    PORT_REGS->GROUP[0].PORT_PMUX[2] |= PORT_PMUX_PMUXE_D; // set PA4 as function D (SERCOM0)
    PORT_REGS->GROUP[0].PORT_PINCFG[4] |= PORT_PINCFG_PMUXEN(1); // set PA4 PMUXEN
    PORT_REGS->GROUP[0].PORT_PMUX[3] |= PORT_PMUX_PMUXE_D; // set PA6 as function D (SERCOM0)
    PORT_REGS->GROUP[0].PORT_PINCFG[6] |= PORT_PINCFG_PMUXEN(1); // set PA6 PMUXEN

    SERCOM0_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_MODE_USART_INT_CLK;
    SERCOM0_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_RXPO_PAD2;
    SERCOM0_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_DORD_LSB;
    SERCOM0_REGS->USART_INT.SERCOM_BAUD = 62180;

    SERCOM0_REGS->USART_INT.SERCOM_CTRLB = SERCOM_USART_INT_CTRLB_RXEN(1);
    SERCOM0_REGS->USART_INT.SERCOM_CTRLB = SERCOM_USART_INT_CTRLB_TXEN(1);
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_CTRLB_Msk);

    SERCOM0_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE(1);
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_ENABLE_Msk);
}

static void system_uart0_deinit(void)
{
    // uart reset
    SERCOM0_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_SWRST(1);
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_SWRST_Msk);
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

void system_init(void)
{
    system_clock_init();
    system_power_init();
    system_uart0_init();
    systick_enable();
}

void system_deinit(void)
{
    system_uart0_deinit();
    system_power_deinit();
    system_clock_deinit();
}
