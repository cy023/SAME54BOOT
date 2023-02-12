/**
 * @file flash.c
 * @author cy023
 * @date 2023.02.06
 * @brief
 * 
 * NOTICE: No buffer bounds checks are done here.
 */

#include "same54p20a.h"
#include <stdint.h>
#include "commuch.h"
#include "device.h"

/*******************************************************************************
 * Macro
 ******************************************************************************/
#define NVMCTRL_FLASH_PAGESIZE          (256U)
#define NVMCTRL_FLASH_BLOCKSIZE         (8192U)     // 32 * page

#define NVMCTRL_PROGE   ((NVMCTRL_REGS->NVMCTRL_INTFLAG & NVMCTRL_INTFLAG_PROGE_Msk) >> 2)

#define FAILED                          1
#define SUCCESSED                       0

/*******************************************************************************
 * Static inline function
 ******************************************************************************/
static inline void NVMCTRL_write_page(uint32_t *data, uint32_t *dest)
{
    while (!(NVMCTRL_REGS->NVMCTRL_STATUS & NVMCTRL_STATUS_READY_Msk));
    NVMCTRL_REGS->NVMCTRL_CTRLB |= NVMCTRL_CTRLB_CMD_PBC | NVMCTRL_CTRLB_CMDEX_KEY;

    while (!(NVMCTRL_REGS->NVMCTRL_STATUS & NVMCTRL_STATUS_READY_Msk));
    for (uint32_t i = 0; i < (NVMCTRL_FLASH_PAGESIZE / 4); i++)
        *dest++ = data[i];
    NVMCTRL_REGS->NVMCTRL_CTRLB |= NVMCTRL_CTRLB_CMD_WP | NVMCTRL_CTRLB_CMDEX_KEY;
}

static inline void NVMCTRL_erase_block(uint32_t address)
{
    while (!(NVMCTRL_REGS->NVMCTRL_STATUS & NVMCTRL_STATUS_READY_Msk));
    NVMCTRL_REGS->NVMCTRL_ADDR = address;
    NVMCTRL_REGS->NVMCTRL_CTRLB |= NVMCTRL_CTRLB_CMD_EB | NVMCTRL_CTRLB_CMDEX_KEY;
}

static inline void NVMCTRL_read_page(uint32_t *data, uint32_t *src)
{
    while (!(NVMCTRL_REGS->NVMCTRL_STATUS & NVMCTRL_STATUS_READY_Msk));
    for (uint32_t i = 0; i < (NVMCTRL_FLASH_PAGESIZE / 4); i++)
        data[i] = src[i];
}

/*******************************************************************************
 * Public function
 ******************************************************************************/
uint8_t flash_set_pgsz(uint16_t size)
{
    if (size != 256) // TODO: only support 256 bytes now
        return FAILED;
    else
        return SUCCESSED;
}

uint16_t flash_get_pgsz(void)
{
    return NVMCTRL_FLASH_PAGESIZE;
}

uint8_t flash_write_app_page(const uint32_t dest, uint8_t *buf)
{
    NVMCTRL_write_page((uint32_t *)buf, (uint32_t *)(USER_APP_START + dest));
    return NVMCTRL_PROGE;
}

uint8_t flash_read_app_page(const uint32_t src, uint8_t *buf)
{
    NVMCTRL_read_page((uint32_t *)buf, (uint32_t *)(USER_APP_START + src));
    return NVMCTRL_PROGE;
}

uint8_t flash_verify_app_page(const uint32_t src, uint8_t *buf)
{
    uint32_t readbuf[NVMCTRL_FLASH_PAGESIZE / 4] = {0};
    uint32_t *cmpbuf = (uint32_t *)buf;
    NVMCTRL_read_page(readbuf, (uint32_t *)(USER_APP_START + src));
    for (uint32_t i = 0; i < (NVMCTRL_FLASH_PAGESIZE / 4); i++) {
        if (cmpbuf[i] != readbuf[i])
            return FAILED;
    }
    return NVMCTRL_PROGE;
}

uint8_t flash_earse_sector(uint8_t sector_num)
{
    uint32_t addr = sector_num * NVMCTRL_FLASH_BLOCKSIZE;
    NVMCTRL_erase_block(addr);
    return NVMCTRL_PROGE;
}

// TODO: Erase more than 256 Bytes
uint8_t flash_earse_app_all(void)
{
    for (uint32_t addr = USER_APP_START; addr < USER_APP_END; addr += NVMCTRL_FLASH_BLOCKSIZE) 
        NVMCTRL_erase_block(addr);
    return NVMCTRL_PROGE;
}
