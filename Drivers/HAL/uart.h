/**
 * @file uart.h
 * @author cy023
 * @date 2022.08.02
 * @brief
 */

#ifndef UART_H
#define UART_H

void uart0_init(void);
void uart0_deinit(void);
void uart0_putc(char data);
char uart0_getc(void);

#endif /* UART_H */
