/**
 * @file test_06_fs_rw_image.c
 * @author cy023
 * @date 2023.02.22
 * @brief 
 * 
 */

#include <stdio.h>
#include "boot_system.h"
#include "lfs.h"
#include "lfs_port.h"

uint8_t image_buf[256] = {0};

void shell_start(void)
{
    printf("  ==============================================================================================\n");
    printf("  ||     ____    ____  ____   ____  ____    ____   ______    _____            __              ||\n");
    printf("  ||    |_   \\  /   _||_  _| |_  _||_   \\  /   _|.' ___  |  |_   _|          [  |             ||\n");
    printf("  ||      |   \\/   |    \\ \\   / /    |   \\/   | / .'   \\_|    | |      ,--.   | |.--.         ||\n");
    printf("  ||      | |\\  /| |     \\ \\ / /     | |\\  /| | | |           | |   _ `'_\\ :  | '/'`\\ \\       ||\n");
    printf("  ||     _| |_\\/_| |_     \\ ' /     _| |_\\/_| |_\\ `.___.'\\   _| |__/ |// | |, |  \\__/ | _     ||\n");
    printf("  ||    |_____||_____|     \\_/     |_____||_____|`.____ .'  |________|\\'-;__/[__;.__.' (_)    ||\n");
    printf("  ||                                                                                          ||\n");
    printf("  ==============================================================================================\n\n");
}

void prompt(void)
{
    printf("\033[0;32;32m\x1B[1m$ \033[m");
}

void printBuf(void)
{
    printf("==========        Buffer        ==========\n");
    for (int i = 0; i < 32; i++) {
        printf("\t");
        for (int j = 0; j < 8; j++)
            printf("%02x ", image_buf[8 * i + j]);
        printf("\n");
    }
    printf("==========================================\n");
}

int main(void)
{
    system_init();
    shell_start();
    
    printf("System Boot.\n");
    printf("[test05]: LittleFS ...\n\n");

    // ---------------------------------------------------------------------- //

    // mount the filesystem
    int err = lfs_mount(&lfs_w25q128jv, &cfg);

    // reformat if we can't mount the filesystem, this should only happen on the first boot
    if (err) {
        lfs_format(&lfs_w25q128jv, &cfg);
        lfs_mount(&lfs_w25q128jv, &cfg);
    }

    // ---------------------------------------------------------------------- //

    lfs_remove(&lfs_w25q128jv, "/boot");

    // ---------------------------------------------------------------------- //

    // Open boot partition
    lfs_file_open(&lfs_w25q128jv, &lfs_file_w25q128jv, "/boot", LFS_O_WRONLY | LFS_O_APPEND | LFS_O_CREAT);

    for (int i = 0; i < 256; i++) {
        image_buf[i] = 0;
    }
    lfs_file_rewind(&lfs_w25q128jv, &lfs_file_w25q128jv);
    lfs_file_write(&lfs_w25q128jv, &lfs_file_w25q128jv, image_buf, sizeof(image_buf));

    for (int i = 0; i < 256; i++) {
        image_buf[i] = 1;
    }
    lfs_file_rewind(&lfs_w25q128jv, &lfs_file_w25q128jv);
    lfs_file_write(&lfs_w25q128jv, &lfs_file_w25q128jv, image_buf, sizeof(image_buf));

    for (int i = 0; i < 256; i++) {
        image_buf[i] = 2;
    }
    lfs_file_rewind(&lfs_w25q128jv, &lfs_file_w25q128jv);
    lfs_file_write(&lfs_w25q128jv, &lfs_file_w25q128jv, image_buf, sizeof(image_buf));

    for (int i = 0; i < 256; i++) {
        image_buf[i] = 3;
    }
    lfs_file_rewind(&lfs_w25q128jv, &lfs_file_w25q128jv);
    lfs_file_write(&lfs_w25q128jv, &lfs_file_w25q128jv, image_buf, sizeof(image_buf));

    // Write boot image done!
    lfs_file_close(&lfs_w25q128jv, &lfs_file_w25q128jv);

    // ---------------------------------------------------------------------- //

    // Open boot partition
    lfs_file_open(&lfs_w25q128jv, &lfs_file_w25q128jv, "/boot", LFS_O_RDONLY | LFS_O_CREAT);

    lfs_file_read(&lfs_w25q128jv, &lfs_file_w25q128jv, image_buf, sizeof(image_buf));
    printBuf();

    lfs_file_read(&lfs_w25q128jv, &lfs_file_w25q128jv, image_buf, sizeof(image_buf));
    printBuf();

    lfs_file_read(&lfs_w25q128jv, &lfs_file_w25q128jv, image_buf, sizeof(image_buf));
    printBuf();

    lfs_file_read(&lfs_w25q128jv, &lfs_file_w25q128jv, image_buf, sizeof(image_buf));
    printBuf();

    lfs_file_read(&lfs_w25q128jv, &lfs_file_w25q128jv, image_buf, sizeof(image_buf));
    printBuf();

    // Read and verify boot image
    lfs_file_close(&lfs_w25q128jv, &lfs_file_w25q128jv);

    // ---------------------------------------------------------------------- //

    // Open boot partition
    lfs_file_open(&lfs_w25q128jv, &lfs_file_w25q128jv, "/boot", LFS_O_RDONLY | LFS_O_CREAT);

    lfs_soff_t fsize = lfs_file_size(&lfs_w25q128jv, &lfs_file_w25q128jv);
    printf("/boot size is %ld\n", fsize);

    // Read and verify boot image
    lfs_file_close(&lfs_w25q128jv, &lfs_file_w25q128jv);

    // ---------------------------------------------------------------------- //

    // release any resources we were using
    lfs_unmount(&lfs_w25q128jv);

    // print the boot count
    printf("Finish\n");
    return 0;
}
