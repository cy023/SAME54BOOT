/**
 * @file test_00_uart.c
 * @author cy023
 * @date 2023.02.07
 * @brief 
 * 
 * TODO: scanf() testing.
 */

#include "same54p20a.h"
#include <stdio.h>
#include "boot_system.h"
#include "commuch.h"

int main()
{
    system_init();
    printf("System Boot.\n");
    printf("[test00]: uart ...\n");

    char c;
    while (1) {
        printf("Please input a character: \n");
        c = uart0_getc();
        printf("Your input character is %c\n\n", c);
    }
    return 0;
}
