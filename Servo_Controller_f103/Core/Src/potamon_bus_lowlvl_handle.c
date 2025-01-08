#include "potamon_bus_lowlvl_handle.h"

pack_data_ctrl_servo_t rx_packet, valid_packet;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (rx_packet.crc16 == crc16_ccitt((uint8_t*)&rx_packet, sizeof(rx_packet)-2))
    {
        if (rx_packet.ID == SERVO_DATA_ID)
        {
            memcpy(&rx_packet, &valid_packet, sizeof(valid_packet));
            system_enabled = valid_packet.mode;
        }
    }
    HAL_UART_Receive_IT(&huart3, (uint8_t*)&valid_packet, sizeof(valid_packet));
}