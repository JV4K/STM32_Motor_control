#ifndef __POTAMON_BUS_HANDLE_LOWLVL_H__
#define __POTAMON_BUS_HANDLE_LOWLVL_H__

#include "main.h"
#include "usart.h"
#include "potamon_bus_protocol.h"
#include "crc_8_16.h"
#include "global_flags.h"

#define SERVO_DATA_ID 0x40
// #define SERVO_DATA_ID 0x41

#define TXOFF HAL_GPIO_WritePin(RS485_TX_EN_GPIO_Port, RS485_TX_EN_Pin, 0)

extern pack_data_ctrl_servo_t rx_packet, valid_packet;

#endif