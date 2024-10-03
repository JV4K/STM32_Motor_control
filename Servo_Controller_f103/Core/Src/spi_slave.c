#include "spi_slave.h"

void slave_system_enabled(miso_packet_t *packet_to_master) {
    packet_to_master->status |= 0x01;
}

void slave_system_disabled(miso_packet_t *packet_to_master) {
    packet_to_master->status &= ~0x01;
}

void slave_servo_enc_is_reset(miso_packet_t *packet_to_master) {
    packet_to_master->servos_reset = 1;
}

void slave_velocity_feedback(miso_packet_t *packet_to_master, float velocity1, float velocity2) {
    packet_to_master->velocity_feedback1 = velocity1;
    packet_to_master->velocity_feedback2 = velocity2;
}

void slave_current_feedback(miso_packet_t *packet_to_master, float current1, float current2) {
    packet_to_master->current_feedback1 = current1;
    packet_to_master->current_feedback2 = current2;
}
