#ifndef INC_SPI_SLAVE_H_
#define INC_SPI_SLAVE_H_

#include "spi_comms.h"

void slave_system_enabled(miso_packet_t*);
void slave_system_disabled(miso_packet_t*);
void slave_servo_enc_is_reset(miso_packet_t*);

void slave_velocity_feedback(miso_packet_t*, float velocity1, float velocity2);
void slave_current_feedback(miso_packet_t*, float current1, float current2);

#endif /* INC_SPI_SLAVE_H_ */
