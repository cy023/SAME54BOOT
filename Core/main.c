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

uint8_t buffer[300] = {0};

/* Waiting for 'CMD_CHK_PROTOCOL' command and response ACK or NACK */
void establish_connection();

/* Waiting for programmer's command and response ACK or NACK */
void bl_command_process();

int main(void)
{
    system_init();
    if (!system_is_prog_mode())
        system_jump_to_app();
    while (1) {
        establish_connection();
        bl_command_process();
    }
}

void establish_connection()
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
            pac.data[1] = 2;
            put_packet(&pac);
            return;
        }
        else {
            send_NACK(&pac);
        }
    }
}

void bl_command_process()
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
                pac.data[1] = 2;
                put_packet(&pac);
                break;
            }
            case CMD_PROG_CHK_DEVICE: {
                pac.length = 2;
                pac.data[0] = SUCCESSED; // ACK
                pac.data[1] = (uint8_t)D_M4_V1;
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

            case CMD_PROG_EXT_TO_INT: {break;}

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

            case CMD_EXT_FLASH_FOPEN:           {break;}
            case CMD_EXT_FLASH_FCLOSE:          {break;}
            case CMD_EXT_FLASH_WRITE:           {break;}
            case CMD_EXT_FLASH_READ:            {break;}
            case CMD_EXT_FLASH_VERIFY:          {break;}
            case CMD_EXT_FLASH_EARSE_SECTOR:    {break;}
            case CMD_EXT_FLASH_HEX_DEL:         {break;}

            case CMD_EEPROM_SET_PGSZ:           {break;}
            case CMD_EEPROM_GET_PGSZ:           {break;}
            case CMD_EEPROM_WRITE:              {break;}
            case CMD_EEPROM_READ:               {break;}
            case CMD_EEPROM_EARSE_ALL:          {break;}
            default: { // NOT supported command
                send_NACK(&pac);
                break;
            }
        }
    }
}
