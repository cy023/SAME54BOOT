/**
 * @file test_04_flash.c
 * @author cy023
 * @date 2023.02.07
 * @brief 
 * 
 * NOTICE: If run this test code in .App section, 
 *          the flash_earse_app_all() boundary should be modified.
 *         Otherwise, it will erase itself.
 */

#include "same54p20a.h"
#include <stdio.h>
#include "boot_system.h"
#include "flash.h"

uint8_t page_buffer[256] = {0};

void printPage(char *s)
{
    printf("\n\t[%s]\n", s);
    printf("\n\t========== printPage ==========\n");
    for (int i = 0; i < 32; i++) {
        printf("\t|   ");
        for (int j = 0; j < 8; j++)
            printf("%02x ", page_buffer[(i << 3) + j]);
        printf("  |\n");
    }
    printf("\t===============================\n");
}

int main()
{
    system_init();
    printf("System Boot.\n");
    printf("[test03]: flash ...\n");
    
    flash_set_pgsz(256);
    printf("flash_set_pgsz(256);\n");
    printf("flash_get_pgsz(): %d\n\n", flash_get_pgsz());

    printPage("Original page_buffer");
    flash_read_app_page(0x00010000UL, page_buffer);
    printPage("After flash_read_app_page()");

    for (int i = 0; i < 256; i++)
        page_buffer[i] = i;
    
    printPage("Set page_buffer from 0 to 255");
    flash_write_app_page(0x00010000UL, page_buffer);
    flash_read_app_page(0x00010000UL, page_buffer);
    printPage("Read after write.");

    printf("flash_verify_app_page: %d\n", \
        flash_verify_app_page(0x00010000UL, page_buffer));
    page_buffer[100] = 0;
    printf("flash_verify_app_page after change one byte: %d\n", \
        flash_verify_app_page(0x00010000UL, page_buffer));

    printf("\t========== earase all ==========\n");
    printPage("Before Erase all");
    // This action will erase all the app section !
    // Including this program !
    flash_earse_app_all();
    flash_read_app_page(0x00010000UL, page_buffer);
    printPage("Read after Erase all");

    while (1) {;}
    return 0;
}
