/**
 * @file boot_system.c
 * @author cy023
 * @date 2023.02.10
 * @brief
 */

#include "same54p20a.h"
#include "device.h"
#include "boot_system.h"

/*******************************************************************************
 * Peripheral Driver
 ******************************************************************************/

void system_clock_init(void)
{
    // XOSC1 pin multiplxer set
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

void system_clock_deinit(void)
{
    // MCLK reset
    MCLK_REGS->MCLK_APBAMASK &= ~MCLK_APBAMASK_SERCOM0(1);    // sercom 0 bus close

    // GCLK reset
    GCLK_REGS->GCLK_CTRLA = GCLK_CTRLA_SWRST(1);
    while (GCLK_REGS->GCLK_SYNCBUSY & GCLK_SYNCBUSY_SWRST_Msk);

    // XOSC1 reset
    // The oscillator is running when a peripheral is requesting the oscillator to be used as a clock source.
    // The oscillator is not running if no peripheral is requesting the clock source.
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] |= OSCCTRL_XOSCCTRL_ONDEMAND(1); 
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] &= ~OSCCTRL_XOSCCTRL_IMULT(4);
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] &= ~OSCCTRL_XOSCCTRL_IPTAT(3);
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] &= ~OSCCTRL_XOSCCTRL_XTALEN(1);
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] &= ~OSCCTRL_XOSCCTRL_ENABLE(1);

    // XOSC1 pin multiplxer reset
    PORT_REGS->GROUP[1].PORT_PMUX[11] &= ~(PORT_PMUX_PMUXO_N | PORT_PMUX_PMUXE_N);
    PORT_REGS->GROUP[1].PORT_PINCFG[22] &= ~PORT_PINCFG_PMUXEN(1);
    PORT_REGS->GROUP[1].PORT_PINCFG[23] &= ~PORT_PINCFG_PMUXEN(1);
}

void system_gpio_init(void)
{
    // set PA7 as prog/run detect pin
    PORT_REGS->GROUP[0].PORT_DIRCLR |= PORT_DIRCLR_DIRCLR(1 << 7);  // clear DIR
    PORT_REGS->GROUP[0].PORT_CTRL |= PORT_CTRL_SAMPLING(1 << 7);    // Continuous sampling.
    PORT_REGS->GROUP[0].PORT_PINCFG[7] |= PORT_PINCFG_PMUXEN(1);    //
    PORT_REGS->GROUP[0].PORT_PINCFG[7] |= PORT_PINCFG_INEN(1);      // IN enable
}

void system_gpio_deinit(void)
{
    // PA7 prog/run dectect pin reset
    // PORT_REGS->GROUP[0].PORT_DIRCLR &= ~PORT_DIRCLR_DIRCLR(1 << 7); // no effect
    PORT_REGS->GROUP[0].PORT_CTRL &= ~PORT_CTRL_SAMPLING(1 << 7);
    PORT_REGS->GROUP[0].PORT_PINCFG[7] &= ~PORT_PINCFG_PMUXEN(1);
    PORT_REGS->GROUP[0].PORT_PINCFG[7] &= ~PORT_PINCFG_INEN(1);
}

void system_uart0_init(void)
{
    // uart0 pin multiplexer set
    PORT_REGS->GROUP[0].PORT_PMUX[2] |= PORT_PMUX_PMUXE_D; // set PA4 as function D (SERCOM0)
    PORT_REGS->GROUP[0].PORT_PINCFG[4] |= PORT_PINCFG_PMUXEN(1); // set PA4 PMUXEN
    PORT_REGS->GROUP[0].PORT_PMUX[3] |= PORT_PMUX_PMUXE_D; // set PA6 as function D (SERCOM0)
    PORT_REGS->GROUP[0].PORT_PINCFG[6] |= PORT_PINCFG_PMUXEN(1); // set PA6 PMUXEN

    // uart0 init
    SERCOM0_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_MODE_USART_INT_CLK;
    SERCOM0_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_RXPO_PAD2;
    SERCOM0_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_DORD_LSB;
    SERCOM0_REGS->USART_INT.SERCOM_BAUD = 62180;

    SERCOM0_REGS->USART_INT.SERCOM_CTRLB |= SERCOM_USART_INT_CTRLB_RXEN(1);
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_CTRLB_Msk);
    SERCOM0_REGS->USART_INT.SERCOM_CTRLB |= SERCOM_USART_INT_CTRLB_TXEN(1);
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_CTRLB_Msk);

    SERCOM0_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE(1);
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_ENABLE_Msk);
}

void system_uart0_deinit(void)
{
    // uart reset
    SERCOM0_REGS->USART_INT.SERCOM_CTRLA &= ~SERCOM_USART_INT_CTRLA_ENABLE(1);
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY & \
            (SERCOM_USART_INT_SYNCBUSY_SWRST_Msk | SERCOM_USART_INT_SYNCBUSY_ENABLE_Msk));

    SERCOM0_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_SWRST(1);
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_SWRST_Msk);

    // uart0 pin reset
    PORT_REGS->GROUP[0].PORT_PMUX[2] &= ~PORT_PMUX_PMUXE_D; // set PA4 as function D (SERCOM0)
    PORT_REGS->GROUP[0].PORT_PINCFG[4] &= ~PORT_PINCFG_PMUXEN(1); // set PA4 PMUXEN
    PORT_REGS->GROUP[0].PORT_PMUX[3] &= ~PORT_PMUX_PMUXE_D; // set PA6 as function D (SERCOM0)
    PORT_REGS->GROUP[0].PORT_PINCFG[6] &= ~PORT_PINCFG_PMUXEN(1); // set PA6 PMUXEN
}

void system_init(void)
{
    system_gpio_init();
    system_clock_init();
    system_uart0_init();
}

void system_deinit(void)
{
    system_uart0_deinit();
    system_clock_deinit();
    system_gpio_deinit();
}

/*******************************************************************************
 * Peripheral Driver
 ******************************************************************************/

__attribute__((always_inline)) static inline void jump2app(void)
{
    // Setting the stack pointer.
    __set_MSP(*(uint32_t *)USER_APP_START);

    // SP + 4: Reset Handler Offset.
    __ASM volatile ("BLX %0" : : "r" (*(uint32_t *)(USER_APP_START + 4)));
}

void system_jump_to_app(void)
{
    // TODO If not in Privileged Mode.
    // if (CONTROL_nPRIV_Msk & __get_CONTROL()) {
    //    // not in privileged mode
    //    __asm("SVC #0\n");
    // }

    system_deinit();

    __disable_irq();

    NVIC->ICER[0] = 0xFFFFFFFF;
    NVIC->ICER[1] = 0xFFFFFFFF;
    NVIC->ICER[2] = 0xFFFFFFFF;
    NVIC->ICER[3] = 0xFFFFFFFF;
    NVIC->ICER[4] = 0xFFFFFFFF;
    NVIC->ICER[5] = 0xFFFFFFFF;
    NVIC->ICER[6] = 0xFFFFFFFF;
    NVIC->ICER[7] = 0xFFFFFFFF;

    NVIC->ICPR[0] = 0xFFFFFFFF;
    NVIC->ICPR[1] = 0xFFFFFFFF;
    NVIC->ICPR[2] = 0xFFFFFFFF;
    NVIC->ICPR[3] = 0xFFFFFFFF;
    NVIC->ICPR[4] = 0xFFFFFFFF;
    NVIC->ICPR[5] = 0xFFFFFFFF;
    NVIC->ICPR[6] = 0xFFFFFFFF;
    NVIC->ICPR[7] = 0xFFFFFFFF;

    SysTick->CTRL = 0;
    SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk; // Removes the pending status of the SysTick exception
    SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk);

    __DSB();
    __ISB();

    SCB->VTOR = (USER_APP_START & SCB_VTOR_TBLOFF_Msk);

    __DSB();
    __ISB();

    __enable_irq();

    jump2app();
}

uint8_t system_is_prog_mode(void)
{
    return !((PORT_REGS->GROUP[0].PORT_IN >> 7) & 0x01);
}

void system_delay_ms(uint32_t ms)
{
    uint16_t delay;
    volatile uint32_t i;
    for (delay = ms; delay > 0; delay--) {
        // 1 ms loop with -O0 optimization.
        for (i = 2400; i > 0; i--) {;}
    }
}
