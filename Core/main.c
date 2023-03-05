/**
 * @file main.c
 * @author cy023
 * @date 2023.02.10
 * @brief bootloader implementation.
 */

#include "same54p20a.h"
#include "device.h"
#include "boot_system.h"
#include "commuch.h"
#include "flash.h"
#include "bootprotocol.h"

#include "lfs.h"
#include "lfs_port.h"

uint8_t buffer[260] = {0};

/* Waiting for 'CMD_CHK_PROTOCOL' command and response ACK or NACK */
void establish_connection(void);

/* Waiting for programmer's command and response ACK or NACK */
void bl_command_process(void);

/* Boot the program. Load the application image from /boot partition */
void boot_from_fs(void);

void shell_start(void);

int main(void)
{
    char select;
    system_init();
    if (!system_is_prog_mode()) {
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
                return 0;
            }
        }
        printf("\033[0;32;32m\x1B[1m=======================\033[m\n");
        system_jump_to_app();
    }
    while (1) {
        establish_connection();
        bl_command_process();
    }
}

void establish_connection(void)
{
    bl_packet_t pac = { .cmd = 0,
                        .length = 0,
                        .data = buffer };
    while (1) {
        if (get_packet(&pac))
            continue;

        if (pac.cmd == CMD_CHK_PROTOCOL) {
            pac.length = 2;
            pac.data[0] = SUCCESSED;
            pac.data[1] = 1;
            put_packet(&pac);
            return;
        }
        else {
            send_NACK(&pac);
        }
    }
}

void bl_command_process(void)
{
    bl_packet_t pac = { .cmd = 0,
                        .length = 0,
                        .data = buffer };
    uint16_t go_app_delay = 50;

    while (1) {
        if (get_packet(&pac))
            continue;

        switch (pac.cmd) {
            case CMD_CHK_PROTOCOL: {
                pac.length = 2;
                pac.data[0] = SUCCESSED; // ACK
                pac.data[1] = 1;
                put_packet(&pac);
                break;
            }
            case CMD_PROG_CHK_DEVICE: {
                pac.length = 2;
                pac.data[0] = SUCCESSED; // ACK
                pac.data[1] = (uint8_t)D_ATSAME54_DEVB;
                put_packet(&pac);
                break;
            }
            case CMD_PROG_END: {
                send_ACK(&pac);
                return;
            }
            case CMD_PROG_END_AND_GO_APP: {
                send_ACK(&pac);
                system_delay_ms(go_app_delay);
                system_jump_to_app();
                return;
            }
            case CMD_PROG_SET_GO_APP_DELAY: {
                uint16_t d = *((uint16_t *)pac.data);
                if (d > 10000) {  // delay to long.
                    send_NACK(&pac);
                } else {
                    go_app_delay = d;
                    send_ACK(&pac);
                }
                break;
            }

            case CMD_PROG_EXT_FLASH_BOOT: {
                boot_from_fs();
                send_ACK(&pac);
                break;
            }

            case CMD_FLASH_SET_PGSZ: {
                // TODO: only support 256 byte page size.
                if (flash_set_pgsz(*((uint16_t *)pac.data)))
                    send_NACK(&pac);
                else
                    send_ACK(&pac);
                break;
            }
            case CMD_FLASH_GET_PGSZ: {
                pac.length = 3;
                pac.data[0] = SUCCESSED; // ACK
                *((uint16_t *)(pac.data + 1)) = flash_get_pgsz();
                put_packet(&pac);
                break;
            }
            case CMD_FLASH_WRITE: {
                if (flash_write_app_page(*(uint32_t *)pac.data, (uint8_t *)(pac.data + 4)))
                    send_NACK(&pac);
                else
                    send_ACK(&pac);
                break;
            }
            case CMD_FLASH_READ: {
                if (flash_read_app_page(*(uint32_t *)pac.data, (uint8_t *)(pac.data + 1))) {
                    send_NACK(&pac);
                }
                else {
                    pac.length = 1 + flash_get_pgsz();
                    pac.data[0] = SUCCESSED; // ACK
                    put_packet(&pac);
                }
                break;
            }
            case CMD_FLASH_VERIFY: {
                if (flash_read_app_page(*(uint32_t *)pac.data, (uint8_t *)(pac.data + 4)))
                    send_NACK(&pac);
                else
                    send_ACK(&pac);
                break;
            }
            case CMD_FLASH_EARSE_SECTOR: { // TODO:
                if (flash_earse_sector(*(uint16_t *)pac.data))
                    send_NACK(&pac);
                else
                    send_ACK(&pac);
                break;
            }
            case CMD_FLASH_EARSE_ALL: {
                if (flash_earse_app_all())
                    send_NACK(&pac);
                else
                    send_ACK(&pac);
                break;
            }

            /******************************************************************/

            case CMD_EXT_FLASH_FOPEN: {
                // mount the filesystem
                int err = lfs_mount(&lfs_w25q128jv, &cfg);

                // reformat if we can't mount the filesystem, this should only happen on the first boot
                if (err) {
                    lfs_format(&lfs_w25q128jv, &cfg);
                    lfs_mount(&lfs_w25q128jv, &cfg);
                }

                lfs_remove(&lfs_w25q128jv, "/boot");

                // Open boot partition
                lfs_file_open(&lfs_w25q128jv, &lfs_file_w25q128jv, "/boot", LFS_O_WRONLY | LFS_O_APPEND | LFS_O_CREAT);

                send_ACK(&pac);
                break;
            }
            case CMD_EXT_FLASH_FCLOSE: {
                // Write boot image done!
                lfs_file_close(&lfs_w25q128jv, &lfs_file_w25q128jv);

                // release any resources we were using
                lfs_unmount(&lfs_w25q128jv);

                send_ACK(&pac);
                break;
            }
            case CMD_EXT_FLASH_WRITE: {
                lfs_file_rewind(&lfs_w25q128jv, &lfs_file_w25q128jv);
                lfs_file_write(&lfs_w25q128jv, &lfs_file_w25q128jv, (uint8_t *)pac.data, 256 + 4);
                send_ACK(&pac);
                break;
            }
            case CMD_EXT_FLASH_READ:            {break;}
            case CMD_EXT_FLASH_VERIFY:          {break;}
            case CMD_EXT_FLASH_EARSE_SECTOR:    {break;}
            case CMD_EXT_FLASH_HEX_DEL:         {break;}

            /******************************************************************/

            case CMD_EEPROM_SET_PGSZ:           {break;}
            case CMD_EEPROM_GET_PGSZ:           {break;}
            case CMD_EEPROM_WRITE:              {break;}
            case CMD_EEPROM_READ:               {break;}
            case CMD_EEPROM_EARSE_ALL:          {break;}

            /******************************************************************/

            default: { // NOT supported command
                send_NACK(&pac);
                break;
            }
        }
    }
}

void boot_from_fs(void)
{
    memset(buffer, 0, 260);
    flash_earse_app_all();

    // mount the filesystem
    int err = lfs_mount(&lfs_w25q128jv, &cfg);

    // reformat if we can't mount the filesystem, this should only happen on the first boot
    if (err) {
        lfs_format(&lfs_w25q128jv, &cfg);
        lfs_mount(&lfs_w25q128jv, &cfg);
    }

    // Open boot partition
    lfs_file_open(&lfs_w25q128jv, &lfs_file_w25q128jv, "/boot", LFS_O_RDONLY | LFS_O_CREAT);

    lfs_soff_t fsize = lfs_file_size(&lfs_w25q128jv, &lfs_file_w25q128jv);
    // printf("/boot size is %ld\n", fsize);

    while (fsize >= 260) {
        lfs_file_read(&lfs_w25q128jv, &lfs_file_w25q128jv, buffer, 256 + 4);
        flash_write_app_page(*(uint32_t *)buffer, (uint8_t *)(buffer + 4));
        fsize -= 260;
    }
    if (fsize) {
        memset(buffer, 0, 260);
        lfs_file_read(&lfs_w25q128jv, &lfs_file_w25q128jv, buffer, fsize);
        flash_write_app_page(*(uint32_t *)buffer, (uint8_t *)(buffer + 4));
    }

    // Read and verify boot image
    lfs_file_close(&lfs_w25q128jv, &lfs_file_w25q128jv);

    // release any resources we were using
    lfs_unmount(&lfs_w25q128jv);
}

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
