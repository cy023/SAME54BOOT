/**
 * @file device.h
 * @author cy023
 * @date 2023.02.06
 * @brief
 *      Code Section Plan for SAME54P20A.
 *
 *      Flash Size              : 1024 kB
 *      USER_APP Section Size   :  960 kB
 *      BOOTLOADER Section Size :   64 kB
 */

#ifndef DEVICE_H
#define DEVICE_H

#define USER_APP_START (0x00010000UL)
#define USER_APP_END   (0x00100000UL)
#define USER_APP_SIZE  (0x000F0000UL)

#define BOOTLOADER_START (0x00000000UL)
#define BOOTLOADER_END   (0x00001000UL)
#define BOOTLOADER_SIZE  (0x00001000UL)

enum device_table { D_ATSAME54_DEVB = 1, D_NUM487KM_DEVB };

#endif /* DEVICE_H */
