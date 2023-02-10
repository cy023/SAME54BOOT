/**
 * @file test_03_delay.c
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
    printf("[test03]: delay_ms() ...\n");

    uint32_t sec = 0;
    while (1) {
        printf("Time: %ld sec\n", sec++);
        system_delay_ms(1000);
    }
    return 0;
}
