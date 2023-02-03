/**
 * @file test_01_sysnow.c
 * @author cy023
 * @date 2022.08.18
 * @brief 
 * 
 */

#include <stdio.h>

#include "system.h"
#include "systick.h"
#include "delay.h"

int main()
{
    system_init();
    printf("System Boot.\n");
    printf("[test01]: sysnow ...\n");

    uint32_t cur_time = 0;

    while (1) {
        cur_time = sys_now();
        printf("current systick is %ld\n", cur_time);
        delay_ms(1000);
    }

    return 0;
}
