/**
 * @file test_02_bootjump.c
 * @author cy023
 * @date 2023.02.07
 * @brief 
 * 
 */

#include "same54p20a.h"
#include <stdio.h>
#include <stdint.h>
#include "boot_system.h"
#include "device.h"

__attribute__((always_inline)) static inline void jump2app(void)
{
    // Setting the stack pointer.
    __set_MSP(*(uint32_t *)USER_APP_START);

    // SP + 4: Reset Handler Offset.
    __ASM volatile ("BLX %0" : : "r" (*(uint32_t *)(USER_APP_START + 4)));
}

int main()
{
    system_init();
    printf("System Boot.\n");
    printf("[test02]: boot jump ...\n");

    jump2app();

    printf("jump failed ... \n");
    
    while (1) {;}
    return 0;
}


