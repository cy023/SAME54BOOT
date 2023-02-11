/**
 * @file boot_system.h
 * @author cy023
 * @date 2023.02.10
 * @brief
 */


#ifndef BOOT_SYSTEM_H
#define BOOT_SYSTEM_H

#include <stdint.h>

/**
 * @brief Clock peripheral initialization.
 *
 *  - XOSC1 PIN enable
 *  - GCLK2 12MHz
 *  - GCLK2 for SERCOM0 enable
 *  - MCLK for SERCOM0 enable
 */
void system_clock_init(void);

/**
 * @brief Clock peripheral initialization.
 */
void system_clock_deinit(void);

/**
 * @brief GPIO peripheral initialization.
 *
 *  - PA7 for prog/run dectect pin 
 */
void system_gpio_init(void);

/**
 * @brief GPIO peripheral deinitialization.
 */
void system_gpio_deinit(void);

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
void system_uart0_init(void);

/**
 * @brief UART peripheral deinitialization.
 */
void system_uart0_deinit(void);

/**
 * @brief System initialization.
 * 
 *  - system_gpio_init()
 *  - system_clock_init()
 *  - system_uart_init()
 */
void system_init(void);

/**
 * @brief System deinitialization.
 * 
 *  - system_gpio_deinit()
 *  - system_clock_deinit()
 *  - system_uart_deinit()
 */
void system_deinit(void);

/**
 * @brief Jump to APP section from bootloader section.
 * 
 *  - system_deinit()
 *  - Reset Core register
 *  - Change the vector table offset to application vector table
 *  - Set the MSP
 *  - Jump to application's Reset_Handler()
 */
void system_jump_to_app(void);

/**
 * @brief Check whether the MCU is in "Prog" mode.
 * @return uint8_t
 *      1: True, in PROG mode.
 *      0: False, in RUN mode.
 */
uint8_t system_is_prog_mode(void);

/**
 * @brief Delay by polling.
 * @param uint32_t ms   delay times in millisecond.
 */
void system_delay_ms(uint32_t ms);

#endif  /* BOOT_SYSTEM_H */
