/**
 * @file test_01_gpio.c
 * @author cy023
 * @date 2023.02.07
 * @brief 
 * 
 */

#include "same54p20a.h"
#include <stdio.h>
#include <stdint.h>
#include "boot_system.h"

int main()
{
    system_init();
    printf("System Boot.\n");
    printf("[test01]: gpio ...\n");

    if (!system_is_prog_mode())
        printf("run.\n");
    else
        printf("prog.\n");

    while (1) {;}
    return 0;
}
