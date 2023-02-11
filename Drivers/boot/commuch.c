/**
 * @file commuch.c
 * @author cy023
 * @date 2023.02.06
 * @brief
 */

#include "same54p20a.h"
#include "commuch.h"

void com_channel_putc(uint8_t data)
{
    while (!(SERCOM0_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk));
    SERCOM0_REGS->USART_INT.SERCOM_DATA = data;
    while (!(SERCOM0_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_TXC_Msk));
}

uint8_t com_channel_getc(void)
{
    while (!(SERCOM0_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk));
    return SERCOM0_REGS->USART_INT.SERCOM_DATA;
}
