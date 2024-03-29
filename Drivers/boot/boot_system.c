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
 * Peripheral Driver - private function
 ******************************************************************************/
/**
 * @brief Clock peripheral initialization.
 *
 *  - XOSC1 PIN enable
 *  - GCLK2 12MHz
 *  - GCLK2 for SERCOM0 enable
 *  - MCLK for SERCOM0 enable
 */
static void system_clock_init(void)
{
    // XOSC1 pin multiplxer set
    PORT_REGS->GROUP[1].PORT_PMUX[11] = PORT_PMUX_PMUXO_N | PORT_PMUX_PMUXE_N;
    PORT_REGS->GROUP[1].PORT_PINCFG[22] |= PORT_PINCFG_PMUXEN(1);
    PORT_REGS->GROUP[1].PORT_PINCFG[23] |= PORT_PINCFG_PMUXEN(1);

    // XOSC1 set
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] &= OSCCTRL_XOSCCTRL_ONDEMAND(0); // oscillator always on
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] |= OSCCTRL_XOSCCTRL_IMULT(4);    // oscillator current multiplier
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] |= OSCCTRL_XOSCCTRL_IPTAT(3);    // oscillator current reference
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] |= OSCCTRL_XOSCCTRL_XTALEN(1);   // internal oscillator circuit enable
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] |= OSCCTRL_XOSCCTRL_ENABLE(1);   // oscillator enable
    while (!(OSCCTRL_REGS->OSCCTRL_STATUS & OSCCTRL_STATUS_XOSCRDY1_Msk))
        ; // wait for XOSC1 ready

    // GCLK 2 set External oscillator 12MHz
    GCLK_REGS->GCLK_GENCTRL[2] |= GCLK_GENCTRL_DIV(1);  // gclk 2 output = src clk / 1
    GCLK_REGS->GCLK_GENCTRL[2] |= GCLK_GENCTRL_SRC(1);  // gclk 2 use xosc1 as source
    GCLK_REGS->GCLK_GENCTRL[2] |= GCLK_GENCTRL_IDC(1);
    GCLK_REGS->GCLK_GENCTRL[2] |= GCLK_GENCTRL_GENEN(1);
    while (GCLK_REGS->GCLK_SYNCBUSY & GCLK_SYNCBUSY_GENCTRL_GCLK2)
        ;  // wait gclk 2 sync

    // Peripheral clock set
    GCLK_REGS->GCLK_PCHCTRL[7] |= GCLK_PCHCTRL_GEN_GCLK2; // select source
    GCLK_REGS->GCLK_PCHCTRL[7] |= GCLK_PCHCTRL_CHEN(1);   // open sercom 0 clock

    // MCLK set
    MCLK_REGS->MCLK_APBAMASK |= MCLK_APBAMASK_SERCOM0(1); // sercom 0 bus open
}

/**
 * @brief Clock peripheral initialization.
 */
static void system_clock_deinit(void)
{
    // MCLK reset
    MCLK_REGS->MCLK_APBAMASK &= ~MCLK_APBAMASK_SERCOM0(1); // sercom 0 bus close

    // GCLK reset
    GCLK_REGS->GCLK_CTRLA = GCLK_CTRLA_SWRST(1);
    while (GCLK_REGS->GCLK_SYNCBUSY & GCLK_SYNCBUSY_SWRST_Msk)
        ;

    // XOSC1 reset
    // The oscillator is running when a peripheral is requesting the oscillator
    // to be used as a clock source. The oscillator is not running if no
    // peripheral is requesting the clock source.
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] |= OSCCTRL_XOSCCTRL_ONDEMAND(1);
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] &= ~OSCCTRL_XOSCCTRL_IMULT(4);
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] &= ~OSCCTRL_XOSCCTRL_IPTAT(3);
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] &= ~OSCCTRL_XOSCCTRL_XTALEN(1);
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] &= ~OSCCTRL_XOSCCTRL_ENABLE(1);

    // XOSC1 pin multiplxer reset
    PORT_REGS->GROUP[1].PORT_PMUX[11] &=
        ~(PORT_PMUX_PMUXO_N | PORT_PMUX_PMUXE_N);
    PORT_REGS->GROUP[1].PORT_PINCFG[22] &= ~PORT_PINCFG_PMUXEN(1);
    PORT_REGS->GROUP[1].PORT_PINCFG[23] &= ~PORT_PINCFG_PMUXEN(1);
}

/**
 * @brief GPIO peripheral initialization.
 *
 *  - PA7 for prog/run dectect pin
 */
static void system_gpio_init(void)
{
    // set PA7 as prog/run detect pin
    PORT_REGS->GROUP[0].PORT_DIRCLR |= PORT_DIRCLR_DIRCLR(1 << 7);  // clear DIR
    PORT_REGS->GROUP[0].PORT_CTRL |=
        PORT_CTRL_SAMPLING(1 << 7);  // Continuous sampling.
    PORT_REGS->GROUP[0].PORT_PINCFG[7] |= PORT_PINCFG_PMUXEN(1);  //
    PORT_REGS->GROUP[0].PORT_PINCFG[7] |= PORT_PINCFG_INEN(1);    // IN enable
}

/**
 * @brief GPIO peripheral deinitialization.
 */
static void system_gpio_deinit(void)
{
    // PA7 prog/run dectect pin reset
    // PORT_REGS->GROUP[0].PORT_DIRCLR &= ~PORT_DIRCLR_DIRCLR(1 << 7); // no
    // effect
    PORT_REGS->GROUP[0].PORT_CTRL &= ~PORT_CTRL_SAMPLING(1 << 7);
    PORT_REGS->GROUP[0].PORT_PINCFG[7] &= ~PORT_PINCFG_PMUXEN(1);
    PORT_REGS->GROUP[0].PORT_PINCFG[7] &= ~PORT_PINCFG_INEN(1);
}

/**
 * @brief UART peripheral initialization.
 *
 *  - MSB first
 *  - TX -> PAD[0]
 *  - RX -> PAD[2]
 *  - no parity
 *  - Asychronous
 *  - Internal clock
 *  - BAUD = 12MHz / 16 * (1 - 62180/65535) = 38400
 */
static void system_uart0_init(void)
{
    // uart0 pin multiplexer set
    PORT_REGS->GROUP[0].PORT_PMUX[2] |= PORT_PMUX_PMUXE_D; // set PA4 as SERCOM0
    PORT_REGS->GROUP[0].PORT_PINCFG[4] |= PORT_PINCFG_PMUXEN(1); // PA4
    PORT_REGS->GROUP[0].PORT_PMUX[3] |= PORT_PMUX_PMUXE_D; // set PA6 as SERCOM0
    PORT_REGS->GROUP[0].PORT_PINCFG[6] |= PORT_PINCFG_PMUXEN(1); // PA6

    // uart0 init
    SERCOM0_REGS->USART_INT.SERCOM_CTRLA |=
        SERCOM_USART_INT_CTRLA_MODE_USART_INT_CLK;
    SERCOM0_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_RXPO_PAD2;
    SERCOM0_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_DORD_LSB;
    SERCOM0_REGS->USART_INT.SERCOM_BAUD = 62180;

    SERCOM0_REGS->USART_INT.SERCOM_CTRLB |= SERCOM_USART_INT_CTRLB_RXEN(1);
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY &
           SERCOM_USART_INT_SYNCBUSY_CTRLB_Msk)
        ;
    SERCOM0_REGS->USART_INT.SERCOM_CTRLB |= SERCOM_USART_INT_CTRLB_TXEN(1);
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY &
           SERCOM_USART_INT_SYNCBUSY_CTRLB_Msk)
        ;

    SERCOM0_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE(1);
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY &
           SERCOM_USART_INT_SYNCBUSY_ENABLE_Msk)
        ;
}

/**
 * @brief UART peripheral deinitialization.
 */
static void system_uart0_deinit(void)
{
    // uart reset
    SERCOM0_REGS->USART_INT.SERCOM_CTRLA &= ~SERCOM_USART_INT_CTRLA_ENABLE(1);
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY &
           (SERCOM_USART_INT_SYNCBUSY_SWRST_Msk |
            SERCOM_USART_INT_SYNCBUSY_ENABLE_Msk))
        ;

    SERCOM0_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_SWRST(1);
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY &
           SERCOM_USART_INT_SYNCBUSY_SWRST_Msk)
        ;

    // uart0 pin reset
    PORT_REGS->GROUP[0].PORT_PMUX[2] &= ~PORT_PMUX_PMUXE_D;// set PA4 as SERCOM0
    PORT_REGS->GROUP[0].PORT_PINCFG[4] &= ~PORT_PINCFG_PMUXEN(1); // PA4
    PORT_REGS->GROUP[0].PORT_PMUX[3] &= ~PORT_PMUX_PMUXE_D;// set PA6 as SERCOM0
    PORT_REGS->GROUP[0].PORT_PINCFG[6] &= ~PORT_PINCFG_PMUXEN(1); // PA6
}

/**
 * @brief For External Flash - w25q128jv
 *
 *  FLASH_MOSI  : PC04
 *  FLASH_SCK   : PC05
 *  FLASH_MISO  : PC07
 *  FLASH_CS    : PC06
 *
 */
static void system_spi_init(void)
{
    // SPI6 pin multiplexer set
    // PC4, PC5, PC6, PC7 as function C (SERCOM6)
    PORT_REGS->GROUP[2].PORT_PMUX[2] |= (PORT_PMUX_PMUXE_C | PORT_PMUX_PMUXO_C);
    PORT_REGS->GROUP[2].PORT_PMUX[3] |= (PORT_PMUX_PMUXE_C | PORT_PMUX_PMUXO_C);
    PORT_REGS->GROUP[2].PORT_PINCFG[4] |= PORT_PINCFG_PMUXEN(1); // PC4
    PORT_REGS->GROUP[2].PORT_PINCFG[5] |= PORT_PINCFG_PMUXEN(1); // PC5
    PORT_REGS->GROUP[2].PORT_PINCFG[7] |= PORT_PINCFG_PMUXEN(1); // PC7

    // Set PC6 as output, for FLASH_CS.
    PORT_REGS->GROUP[2].PORT_DIRSET |= PORT_DIRSET_DIRSET(1 << 6);
    PORT_REGS->GROUP[2].PORT_OUTSET |= PORT_OUTSET_OUTSET(1 << 6);

    // Peripheral clock set
    GCLK_REGS->GCLK_PCHCTRL[36] |= GCLK_PCHCTRL_GEN_GCLK2; // select source
    GCLK_REGS->GCLK_PCHCTRL[36] |= GCLK_PCHCTRL_CHEN(1);

    // MCLK set
    MCLK_REGS->MCLK_APBDMASK |= MCLK_APBDMASK_SERCOM6(1);  // sercom 6 bus open

    // SPI setting
    SERCOM6_REGS->SPIM.SERCOM_CTRLB |= SERCOM_SPIM_CTRLB_RXEN(1);
    SERCOM6_REGS->SPIM.SERCOM_CTRLA |= SERCOM_SPIM_CTRLA_MODE(3);
    SERCOM6_REGS->SPIM.SERCOM_CTRLA |= SERCOM_SPIM_CTRLA_DIPO(3);

    // SPI soft reset
    SERCOM6_REGS->SPIM.SERCOM_CTRLA |= SERCOM_SPIM_CTRLA_ENABLE(1);
}

static void system_spi_deinit(void)
{
    SERCOM6_REGS->SPIM.SERCOM_CTRLA &= ~SERCOM_SPIM_CTRLA_SWRST(1);

    SERCOM6_REGS->SPIM.SERCOM_CTRLA = 0;
    SERCOM6_REGS->SPIM.SERCOM_CTRLB = 0;

    MCLK_REGS->MCLK_APBDMASK &= ~MCLK_APBDMASK_SERCOM6(1);
    GCLK_REGS->GCLK_PCHCTRL[36] = 0;

    PORT_REGS->GROUP[2].PORT_OUTCLR |= PORT_OUTCLR_OUTCLR(1 << 6);
    PORT_REGS->GROUP[2].PORT_DIRCLR |= PORT_DIRCLR_DIRCLR(1 << 6);

    PORT_REGS->GROUP[2].PORT_PINCFG[4] &= ~PORT_PINCFG_PMUXEN(1);
    PORT_REGS->GROUP[2].PORT_PINCFG[5] &= ~PORT_PINCFG_PMUXEN(1);
    PORT_REGS->GROUP[2].PORT_PINCFG[7] &= ~PORT_PINCFG_PMUXEN(1);
    PORT_REGS->GROUP[2].PORT_PMUX[2] &=
        ~(PORT_PMUX_PMUXE_C | PORT_PMUX_PMUXO_C);
    PORT_REGS->GROUP[2].PORT_PMUX[3] &=
        ~(PORT_PMUX_PMUXE_C | PORT_PMUX_PMUXO_C);
}

/**
 * @brief Boot LED initialization
 *  BOOT_LED pin: PB02 (output)
 */
static void bootLED_init(void)
{
    // Set PB2 as output, for Boot LED.
    PORT_REGS->GROUP[1].PORT_DIRSET |= PORT_DIRSET_DIRSET(1 << 2);
    PORT_REGS->GROUP[1].PORT_OUTSET |= PORT_OUTSET_OUTSET(1 << 2);
}

/**
 * @brief Boot LED deinitialization
 */
static void bootLED_deinit(void)
{
    PORT_REGS->GROUP[1].PORT_OUTCLR |= PORT_OUTCLR_OUTCLR(1 << 2);
    PORT_REGS->GROUP[1].PORT_DIRCLR |= PORT_DIRCLR_DIRCLR(1 << 2);
}

/*******************************************************************************
 * Public Function
 ******************************************************************************/

void system_init(void)
{
#if defined(BOOT_GPIO_DRIVER_ENABLE) && (BOOT_GPIO_DRIVER_ENABLE + 0)
    system_gpio_init();
#endif

#if defined(BOOT_CLOCK_DRIVER_ENABLE) && (BOOT_CLOCK_DRIVER_ENABLE + 0)
    system_clock_init();
#endif

#if defined(BOOT_UART_DRIVER_ENABLE) && (BOOT_UART_DRIVER_ENABLE + 0)
    system_uart0_init();
#endif

#if defined(BOOT_SPI_DRIVER_ENABLE) && (BOOT_SPI_DRIVER_ENABLE + 0)
    system_spi_init();
#endif

    bootLED_init();
}

void system_deinit(void)
{
    system_spi_deinit();
    system_uart0_deinit();
    system_clock_deinit();
    system_gpio_deinit();
    bootLED_deinit();
}

/*******************************************************************************
 * Bootloader Operation
 ******************************************************************************/

__attribute__((always_inline)) static inline void jump2app(void)
{
    // Setting the stack pointer.
    __set_MSP(*(uint32_t *) USER_APP_START);

    // SP + 4: Reset Handler Offset.
    __ASM volatile("BLX %0" : : "r"(*(uint32_t *) (USER_APP_START + 4)));
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
    SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk;  // Removes the pending status of the
                                          // SysTick exception
    SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk |
                    SCB_SHCSR_MEMFAULTENA_Msk);

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
        for (i = 2400; i > 0; i--) {
            ;
        }
    }
}

void bootLED_on(void)
{
    PORT_REGS->GROUP[1].PORT_OUTCLR |= PORT_OUTCLR_OUTCLR(1 << 2);
}

void bootLED_off(void)
{
    PORT_REGS->GROUP[1].PORT_OUTSET |= PORT_OUTSET_OUTSET(1 << 2);
}
