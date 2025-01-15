#ifndef __POTAMON_BUS_HANDLE_LOWLVL_H__
#define __POTAMON_BUS_HANDLE_LOWLVL_H__

#include "main.h"
#include "usart.h"
#include "potamon_bus_protocol.h"
#include "crc_8_16.h"
#include "global_flags.h"
#include "settings.h"
#include "servo_iq18.h"
#include "servo_init.h"
#include "adc_current.h"

#define TXOFF HAL_GPIO_WritePin(RS485_TX_EN_GPIO_Port, RS485_TX_EN_Pin, GPIO_PIN_RESET)
#define TXON HAL_GPIO_WritePin(RS485_TX_EN_GPIO_Port, RS485_TX_EN_Pin, GPIO_PIN_SET)

extern pack_data_ctrl_servo_t data_ctrl;
extern pack_sync_t packet_sync;

#endif