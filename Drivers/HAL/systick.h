/**
 * @file systick.h
 * @author cy023
 * @brief Systick driver
 * @date 2022.08.16
 * 
 * // TODO : Why inttypes.h?
 */

#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

uint32_t sys_now(void);
void SysTick_Handler(void);
void systick_enable(void);

#endif  /* SYSTICK_H */
