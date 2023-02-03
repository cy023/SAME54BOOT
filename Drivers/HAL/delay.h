/**
 * @file delay.h
 * @author cy023
 * @brief Provide delay functions
 * @date 2022.08.17
 * 
 */

#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>

/**
 * @brief delay in ms (systick + polling)
 *
 * @param ms time in ms
 */
void delay_ms(uint16_t ms);

#endif  /* DELAY_H */
