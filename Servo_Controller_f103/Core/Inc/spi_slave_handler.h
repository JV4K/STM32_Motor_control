#ifndef INC_SPI_SLAVE_HANDLER_H_
#define INC_SPI_SLAVE_HANDLER_H_

#include "spi_slave.h"
#include "spi.h"
#include "gpio.h"
#include "global_flags.h"

extern miso_packet_t transmit_packet;
extern mosi_packet_t received_packet, valid_packet;
extern uint32_t interrupr_cnt, callback_cnt, invalid_cnt;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);



#endif /*INC_SPI_SLAVE_HANDLER_H_*/