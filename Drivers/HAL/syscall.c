/**
 * @file syscall.c
 * @author cy023
 * @date 2022.08.02
 * @brief
 *
 */

#include "uart.h"

int _write (__attribute__((unused)) int fd, char *ptr, int len)
{
    int i;
    for(i = 0; i< len; i++) {
        uart0_putc(*ptr++);
        if (*ptr == '\n')
            uart0_putc('\r');
    }
    return len;
}

int _read (__attribute__((unused)) int fd, char *ptr, int len)
{
    int i;
    for(i = 0; i< len; i++)
        *ptr++ = uart0_getc();
    return len;
}

void _ttywrch(int ch)
{
    uart0_putc(ch);
}
