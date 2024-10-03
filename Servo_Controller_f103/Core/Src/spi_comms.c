#include "spi_comms.h"
#include <stdio.h>

/* Master methods ---------------------------------------------------------*/
void master_reset_miso_packet(miso_packet_t *packet) {
    memset(packet, 0, sizeof(miso_packet_t));
}

void master_crc_packet(mosi_packet_t *packet) {
    uint32_t buffer[sizeof(mosi_packet_t) / 4];
    memcpy(buffer, packet, sizeof(mosi_packet_t) - sizeof(uint32_t));
    uint32_t crc_check = HAL_CRC_Calculate(&hcrc, buffer, (sizeof(mosi_packet_t) - sizeof(uint32_t)) / 4);
    packet->crc_res = crc_check;
}

uint8_t master_verify_miso_packet(miso_packet_t *packet) {
    uint32_t buffer[sizeof(miso_packet_t) / 4];
    memcpy(buffer, packet, sizeof(miso_packet_t) - sizeof(uint32_t));
    uint32_t crc_check = HAL_CRC_Calculate(&hcrc, buffer, (sizeof(miso_packet_t) - sizeof(uint32_t)) / 4);
    return (packet->crc_res == crc_check);
}

/* Slave methods ---------------------------------------------------------*/
void slave_reset_mosi_packet(mosi_packet_t *packet) {
    memset(packet, 0, sizeof(mosi_packet_t));
}

void slave_crc_packet(miso_packet_t *packet) {
    uint32_t buffer[sizeof(miso_packet_t) / 4];
    memcpy(buffer, packet, sizeof(miso_packet_t) - sizeof(uint32_t));
    uint32_t crc_check = HAL_CRC_Calculate(&hcrc, buffer, (sizeof(miso_packet_t) - sizeof(uint32_t)) / 4);
    packet->crc_res = crc_check;
}

uint8_t slave_verify_mosi_packet(mosi_packet_t *packet) {
    uint32_t buffer[sizeof(mosi_packet_t) / 4];
    memcpy(buffer, packet, sizeof(mosi_packet_t) - sizeof(uint32_t));
    uint32_t crc_check = HAL_CRC_Calculate(&hcrc, buffer, (sizeof(mosi_packet_t) - sizeof(uint32_t)) / 4);
    return (packet->crc_res == crc_check);
}
