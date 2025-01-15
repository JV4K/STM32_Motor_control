#include "potamon_bus_lowlvl_handle.h"

// Buffer to dump packets not intended to use on this level of control
uint8_t uart_rx_dump_buffer[67];

// Struct to recieve SYNC packets
pack_sync_t packet_sync;

// Struct to recieve CTRL packets
pack_data_ctrl_servo_t data_ctrl_unchecked, data_ctrl;

// Struct to recieve n of trajectory chunks
pack_data_traj_n_t data_traj_n_unchecked, data_traj_n;

// Struct to pack feedback data
pack_data_encoder_t data_encoder;
pack_data_current_t data_current;

next_packet_t expected_packet; // Packet that is expected next
uint8_t chunk_counter;         // Counter of skipped chunks

// Counters for packets that failed crc check
uint32_t failed_sync_packets_count, failed_data_packets_count;

void pack_encoder_feedback(pack_data_encoder_t *packet) // Pack and crc encoder data
{
    packet->angle_1 = encoder_getAngle_iq18(&servo1_g.encoder) >> 9;
    packet->angle_1 = encoder_getAngle_iq18(&servo2_g.encoder) >> 9;
    packet->velocity_1 = (int16_t)(encoder_getVelocity_iq18(&servo1_g.encoder) >> 9);
    packet->velocity_2 = (int16_t)(encoder_getVelocity_iq18(&servo2_g.encoder) >> 9);
    packet->crc16 = crc16_ccitt((uint8_t*)packet, S_DATA_ENC_S1-2);
}

void pack_current_feedback(pack_data_current_t *packet) // Pack and crc encoder data
{
    packet->current1 = (int16_t)(current[0] >> 9);
    packet->current2 = (int16_t)(current[1] >> 9);
    packet->crc16 = crc16_ccitt((uint8_t*)packet, S_DATA_CUR_S1-2);
}

void sync_packet_handler(pack_sync_t *packet)
{
    switch (packet->ID)
    {
    case 0x00: // STATE_STBY
        system_enabled = 0;
        expected_packet = SYNC;
        HAL_UART_Receive_DMA(&huart3, (uint8_t *)&packet_sync, S_SYNC);
        break;
    case 0x01: // STATE_VEL_CTRL
        system_enabled = 1;
        expected_packet = SYNC;
        HAL_UART_Receive_DMA(&huart3, (uint8_t *)&packet_sync, S_SYNC);
        break;
    case 0x02: // STATE_POS_CTRL
        system_enabled = 1;
        expected_packet = SYNC;
        HAL_UART_Receive_DMA(&huart3, (uint8_t *)&packet_sync, S_SYNC);
        break;
    case 0x03: // STATE_IND_WH_CTRL
        system_enabled = 1;
        expected_packet = SYNC;
        HAL_UART_Receive_DMA(&huart3, (uint8_t *)&packet_sync, S_SYNC);
        break;
    case 0x06: // SYNC_CTRL_S1
        expected_packet = DATA_CTRL_S1;
        if (S_NUM == 1)
        {
            HAL_UART_Receive_DMA(&huart3, (uint8_t *)&data_ctrl_unchecked, S_DATA_CTRL_S1);
        }
        else
        {
            HAL_UART_Receive_DMA(&huart3, uart_rx_dump_buffer, S_DATA_CTRL_S1);
        }
        break;
    case 0x07: // SYNC_CTRL_S2
        expected_packet = DATA_CTRL_S2;
        if (S_NUM == 2)
        {
            HAL_UART_Receive_DMA(&huart3, (uint8_t *)&data_ctrl_unchecked, S_DATA_CTRL_S2);
        }
        else
        {
            HAL_UART_Receive_DMA(&huart3, uart_rx_dump_buffer, S_DATA_CTRL_S2);
        }
        break;
    case 0x08: // SYNC_VEL_VEC_TASK
        expected_packet = DATA_VEL_VEC_TASK;
        HAL_UART_Receive_DMA(&huart3, uart_rx_dump_buffer, S_DATA_VEL_VEC_TASK);
        break;
    case 0x09: // SYNC_ENC_S1
        expected_packet = DATA_ENC_S1;
        if (S_NUM == 1)
        {
            pack_encoder_feedback(&data_encoder);
            TXON; // Turn on transmission on Transceiver
            HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&data_encoder, S_DATA_ENC_S1);
        }
        else
        {
            HAL_UART_Receive_DMA(&huart3, uart_rx_dump_buffer, S_DATA_ENC_S1);
        }
        break;
    case 0x0A: // SYNC_ENC_S2
        expected_packet = DATA_ENC_S2;
        if (S_NUM == 2)
        {
            pack_encoder_feedback(&data_encoder);
            TXON; // Turn on transmission on Transceiver
            HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&data_encoder, S_DATA_ENC_S2);
        }
        else
        {
            HAL_UART_Receive_DMA(&huart3, uart_rx_dump_buffer, S_DATA_ENC_S2);
        }
        break;
    case 0x0B: // SYNC_CUR_S1
        expected_packet = DATA_CUR_S1;
        if (S_NUM == 1)
        {
            pack_current_feedback(&data_current);
            TXON; // Turn on transmission on Transceiver
            HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&data_current, S_DATA_CUR_S1);
        }
        else
        {
            HAL_UART_Receive_DMA(&huart3, uart_rx_dump_buffer, S_DATA_CUR_S1);
        }
        break;
    case 0x0C: // SYNC_CUR_S2
        expected_packet = DATA_CUR_S2;
        if (S_NUM == 2)
        {
            pack_current_feedback(&data_current);
            TXON; // Turn on transmission on Transceiver
            HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&data_current, S_DATA_CUR_S2);
        }
        else
        {
            HAL_UART_Receive_DMA(&huart3, uart_rx_dump_buffer, S_DATA_CUR_S2);
        }
        break;
    case 0x0D: // SYNC_ODO
        expected_packet = DATA_ODO;
        HAL_UART_Receive_DMA(&huart3, uart_rx_dump_buffer, S_DATA_ODO);
        break;
    case 0x0E: // SYNC_IMU
        expected_packet = DATA_IMU;
        HAL_UART_Receive_DMA(&huart3, uart_rx_dump_buffer, S_DATA_IMU);
        break;
    case 0x0F: // SYNC_TRAJ_N
        expected_packet = DATA_TRAJ_N;
        HAL_UART_Receive_DMA(&huart3, (uint8_t *)&data_traj_n_unchecked, S_DATA_IMU);
        break;
    case 0x10: // SYNC_RESET_ODO
        expected_packet = SYNC;
        HAL_UART_Receive_DMA(&huart3, (uint8_t *)&packet_sync, S_SYNC);
        break;
    default:
        break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    switch (expected_packet)
    {
    case SYNC:
        if (packet_sync.crc8 == crc8_ccitt((uint8_t *)&packet_sync, 1))
        {
            sync_packet_handler(&packet_sync);
            LED_OFF;
        }
        else
        {
            LED_ON;
            failed_sync_packets_count++;
            system_enabled = 0;
        }
        break;
    case DATA_CTRL_S1:
        if (S_NUM == 1)
        {
            if (data_ctrl_unchecked.crc16 == crc16_ccitt((uint8_t *)&data_ctrl_unchecked, S_DATA_CTRL_S1 - 2))
            {
                memcpy(&data_ctrl, &data_ctrl_unchecked, S_DATA_CTRL_S1);
                LED_OFF;
            }
            else
            {
                LED_ON;
                system_enabled = 0;
                failed_data_packets_count++;
            }
        }
        expected_packet = SYNC;
        HAL_UART_Receive_DMA(&huart3, (uint8_t *)&packet_sync, S_SYNC);
        break;
    case DATA_CTRL_S2:
        if (S_NUM == 2)
        {
            if (data_ctrl_unchecked.crc16 == crc16_ccitt((uint8_t *)&data_ctrl_unchecked, S_DATA_CTRL_S2 - 2))
            {
                memcpy(&data_ctrl, &data_ctrl_unchecked, S_DATA_CTRL_S2);
            }
            else
            {
                LED_ON;
                system_enabled = 0;
                failed_data_packets_count++;
            }
        }
        expected_packet = SYNC;
        HAL_UART_Receive_DMA(&huart3, (uint8_t *)&packet_sync, S_SYNC);
        break;
    case DATA_TRAJ_N:
        if (data_traj_n_unchecked.crc16 == crc16_ccitt((uint8_t *)&data_traj_n_unchecked, S_DATA_TRAJ_N - 2))
        {
            memcpy(&data_traj_n, &data_traj_n_unchecked, S_DATA_TRAJ_N);
            LED_OFF;
            expected_packet = DATA_TRAJ_CHUNK;
            HAL_UART_Receive_DMA(&huart3, uart_rx_dump_buffer, S_DATA_TRAJ_CHUNK);
        }
        else
        {
            LED_ON;
            system_enabled = 0;
            failed_data_packets_count++;
        }
        break;
    case DATA_TRAJ_CHUNK:
        chunk_counter++;
        if (chunk_counter < data_traj_n.no_of_chunks)
        {
            expected_packet = DATA_TRAJ_CHUNK;
            HAL_UART_Receive_DMA(&huart3, uart_rx_dump_buffer, S_DATA_TRAJ_CHUNK);
        }
        else
        {
            expected_packet = SYNC;
            HAL_UART_Receive_DMA(&huart3, (uint8_t *)&packet_sync, S_SYNC);
        }
        break;
    default:
        expected_packet = SYNC;
        HAL_UART_Receive_DMA(&huart3, (uint8_t *)&packet_sync, S_SYNC);
        break;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    TXOFF; // Turn of transmission on transceiver in order to not hang the bus
    expected_packet = SYNC;
    HAL_UART_Receive_DMA(&huart3, (uint8_t *)&packet_sync, S_SYNC);
}