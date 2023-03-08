/**
 * @file main.c
 * @author cy023
 * @date 2023.02.10
 * @brief bootloader implementation.
 */

#include <stdio.h>
#include "same54p20a.h"
#include "device.h"
#include "boot_system.h"
#include "commuch.h"
#include "nvmctrl.h"
#include "bootprotocol.h"

void shell_start(void)
{
    printf("  ╔══════════════════════════════════════════════════════════╗\n");
    printf("  ║                                                          ║\n");
    printf("  ║     ____              _      __  __                      ║\n");
    printf("  ║    | __ )  ___   ___ | |_   |  \\/  | ___ _ __  _   _     ║\n");
    printf("  ║    |  _ \\ / _ \\ / _ \\| __|  | |\\/| |/ _ \\ '_ \\| | | |    ║\n");
    printf("  ║    | |_) | (_) | (_) | |_   | |  | |  __/ | | | |_| |    ║\n");
    printf("  ║    |____/ \\___/ \\___/ \\__|  |_|  |_|\\___|_| |_|\\__,_|    ║\n");
    printf("  ║                                                          ║\n");
    printf("  ║                                               by cy023.  ║\n");
    printf("  ╚══════════════════════════════════════════════════════════╝\n\n");
}

uint8_t select_boot_partition(void)
{
    char select;
    shell_start();
    printf("Boot partition select ...\n");
    printf("(1) Primary Slot (Boot from external flash)\n");
    printf("(2) Secondary Slot (Boot from external flash)\n");
    printf("> \n");
    select = com_channel_getc();

    printf("Waiting for boot up ...\n");
    switch (select)
    {
        case '1': {
            break;
        }
        case '2': {
            boot_from_fs();
            printf("Boot from secondary slot successed!\n");
            break;
        }
        default: {
            printf("Boot Failed. Please Reset the computer.\n");
            return FAILED;
        }
    }
    return SUCCESSED;
}

int main(void)
{
    system_init();
    if (!system_is_prog_mode()) {
        if (select_boot_partition())
            return 0;
        printf("\033[0;32;32m\x1B[1m=======================\033[m\n");
        system_jump_to_app();
    }
    while (1) {
        establish_connection();
        bl_command_process();
    }
}
