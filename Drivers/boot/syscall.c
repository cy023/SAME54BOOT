/**
 * @file syscall.c
 * @author cy023
 * @date 2022.08.02
 * @brief
 *
 */

#include "commuch.h"

int _write(__attribute__((unused)) int fd, char *ptr, int len)
{
    for (int i = 0; i < len; i++) {
        com_channel_putc(*ptr++);
        if (*ptr == '\n')
            com_channel_putc('\r');
    }
    return len;
}

int _read(__attribute__((unused)) int fd, char *ptr, int len)
{
    for (int i = 0; i < len; i++)
        *ptr++ = com_channel_getc();
    return len;
}

void _ttywrch(int ch)
{
    com_channel_putc(ch);
}
