/**
 * @file delay.c
 * @author cy023
 * @brief Provide delay functions
 * @date 2022.08.17
 * 
 */

#include "delay.h"
#include "systick.h"

void delay_ms(uint16_t ms)
{
    volatile uint32_t t = sys_now();
    while (1) {
        if (sys_now() - t > ms * 10)
            break;
    }
}
