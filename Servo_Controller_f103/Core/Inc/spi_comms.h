#ifndef INC_SPI_COMMS_H_
#define INC_SPI_COMMS_H_

#include "main.h"
#include <stdlib.h>
#include <string.h>
#include "crc.h"

/* MOSI packet struct ---------------------------------------------------------*/
typedef struct {
    uint8_t mode;               // MODE register bytes
    float angle_task1;       // Velocity command for wheel 1
    float angle_task2;       // Velocity command for wheel 2
    float velocity_task1;       // Velocity command for wheel 1
    float velocity_task2;       // Velocity command for wheel 2
    uint8_t reset_servos;       // Command to reset servos
    uint32_t crc_res;           // CRC
} __attribute__((packed)) mosi_packet_t;

/* MISO packet struct ---------------------------------------------------------*/
typedef struct {
    uint8_t status;             // STATUS register bytes (enabled/disabled)
    float angle_feedback1;   // Velocity feedback for wheel 1
    float angle_feedback2;   // Velocity feedback for wheel 2
    float velocity_feedback1;   // Velocity feedback for wheel 1
    float velocity_feedback2;   // Velocity feedback for wheel 2
    float current_feedback1;    // Current feedback for wheel 1
    float current_feedback2;    // Current feedback for wheel 2
    uint8_t servos_reset;       // Notification that servos were reset
    uint32_t crc_res;           // CRC
} __attribute__((packed)) miso_packet_t;

/* Master methods ---------------------------------------------------------*/
void master_reset_miso_packet(miso_packet_t*);
void master_crc_packet(mosi_packet_t*);
uint8_t master_verify_miso_packet(miso_packet_t*);

/* Slave methods ---------------------------------------------------------*/
void slave_reset_mosi_packet(mosi_packet_t*);
void slave_crc_packet(miso_packet_t*);
uint8_t slave_verify_mosi_packet(mosi_packet_t*);

#endif /* INC_SPI_COMMS_H_ */
