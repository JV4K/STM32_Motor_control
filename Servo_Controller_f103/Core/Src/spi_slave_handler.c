#include "spi_slave_handler.h"

miso_packet_t transmit_packet;
mosi_packet_t received_packet, valid_packet;
uint32_t interrupr_cnt, callback_cnt, invalid_cnt;
uint32_t txrxthing;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// interrupr_cnt++;
	if (GPIO_Pin == NSS_Pin)
	{
		if (HAL_GPIO_ReadPin(NSS_GPIO_Port, NSS_Pin))
		{
			// interrupr_cnt++;
			hspi2.Instance->CR1 |= 0x100; // Set SSI bit
		}
		else
		{
			// interrupr_cnt++;
			slave_crc_packet(&transmit_packet);
			hspi2.Instance->CR1 &= 0xFEFF; // Reset SSI bit
			txrxthing = (uint32_t)HAL_SPI_TransmitReceive_IT(&hspi2, (uint8_t *)&transmit_packet,
															 (uint8_t *)&received_packet, sizeof(mosi_packet_t));
			if (transmit_packet.servos_reset)
			{
				// Todo: add servo reset code
				transmit_packet.servos_reset = 0;
			}
		}
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	callback_cnt++;

	if (slave_verify_mosi_packet(&received_packet))
	{
		memcpy(&valid_packet, &received_packet, sizeof(mosi_packet_t));
		system_enabled = valid_packet.mode;
		reset_flag = valid_packet.reset_servos;
	}
	else
	{
		invalid_cnt++;
		system_enabled = 0;
	}
	hspi2.Instance->CR1 |= 0x100; // Set SSI bit
}
