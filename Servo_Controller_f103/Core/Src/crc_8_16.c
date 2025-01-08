#include "crc_8_16.h"

// CRC-8-CCITT-FALSE calculation
uint8_t crc8_ccitt(uint8_t *data, size_t length) {
    uint8_t crc = CRC8_INIT;

    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC8_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// CRC-16-CCITT-FALSE calculation
uint16_t crc16_ccitt(uint8_t *data, size_t length) {
    uint16_t crc = CRC16_INIT;

    for (size_t i = 0; i < length; ++i) {
        crc ^= (data[i] << 8);
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC16_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}