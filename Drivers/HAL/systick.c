/**
 * @file systick.c
 * @author cy023
 * @brief Systick driver
 * @date 2022.08.17
 * 
 * // TODO : SysTick_Config() ?
 */

#include "systick.h"
#include "Drivers/SAME54_DFP/same54p20a.h"
#include "Drivers/SAME54_BSP/Config/peripheral_clk_config.h"
#include "Drivers/CMSIS/core_cm4.h"

static volatile uint32_t systick_timems = 0;

uint32_t sys_now(void)
{
    return systick_timems;
}

void SysTick_Handler(void)
{
    systick_timems++;
}

void systick_enable(void)
{
    systick_timems = 0;
    SysTick_Config(4800);
    // SysTick_Config((CONF_CPU_FREQUENCY) / 1000);
}
