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
    printf("Hello World\n");

    return 0;
}
