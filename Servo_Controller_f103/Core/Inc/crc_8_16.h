#ifndef CRC816_H
#define CRC816_H

#include <stdint.h>
#include <stddef.h>

// CRC-8-CCITT-FALSE parameters
#define CRC8_POLY 0x07 // Polynomial: x^8 + x^2 + x + 1
#define CRC8_INIT 0x00 // Initial value

// CRC-16-CCITT-FALSE parameters
#define CRC16_POLY 0x1021  // Polynomial: x^16 + x^12 + x^5 + 1
#define CRC16_INIT 0xFFFF  // Initial value

// CRC-8-CCITT-FALSE function
uint8_t crc8_ccitt(uint8_t *data, size_t length);

// CRC-16-CCITT-FALSE function
uint16_t crc16_ccitt(uint8_t *data, size_t length);

#endif // CRC816_H
