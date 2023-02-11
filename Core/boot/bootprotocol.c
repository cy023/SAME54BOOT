/**
 * @file bootprotocol.c
 * @author cy023
 * @brief Packet Handler
 * @date 2023.02.10
 */

#include "commuch.h"
#include "bootprotocol.h"

uint8_t get_packet(bl_packet_t *packet)
{
    uint8_t len_h, len_l;
    uint8_t cmd;
    uint8_t chksum = 0;

    for (int i = 0; i < 3; i++)
        if (com_channel_getc() != HEADER)
            return FAILED;

    cmd = com_channel_getc();

    if (com_channel_getc() != TOKEN)
        return FAILED;

    len_h = com_channel_getc();
    len_l = com_channel_getc();
    packet->length = (len_h << 8) + len_l;
    packet->cmd = cmd;

    for (uint16_t i = 0; i < packet->length; i++) {
        packet->data[i] = com_channel_getc();
        chksum += packet->data[i];
    }

    if (com_channel_getc() != chksum)   
        return FAILED;

    return SUCCESSED;
}

uint8_t put_packet(bl_packet_t *packet)
{
    uint8_t chksum = 0;
    
    com_channel_putc(HEADER);
    com_channel_putc(HEADER);
    com_channel_putc(HEADER);
    com_channel_putc(packet->cmd);
    com_channel_putc(TOKEN);
    com_channel_putc(packet->length >> 8);
    com_channel_putc(packet->length & 0xFF);

    for (uint16_t i = 0; i < packet->length; i++) {
        com_channel_putc(packet->data[i]);
        chksum += packet->data[i];
    }
    com_channel_putc(chksum);
    return SUCCESSED;
}

void send_ACK(bl_packet_t *packet)
{
    packet->length = 1;
    packet->data[0] = ACK;
    put_packet(packet);
}

void send_NACK(bl_packet_t *packet)
{
    packet->length = 1;
    packet->data[0] = NACK;
    put_packet(packet);
}
