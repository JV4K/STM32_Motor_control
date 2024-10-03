#include "loops_handler.h"

uint16_t irq_counter9k, irq_counter250;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim3)
    {
        if (system_enabled)
        {

            // Allow powering motors by setting Enable pins of a driver HIGH
            HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_SET);
            if (!irq_counter9k)
            {
                servo_iq18_currentLoop(&servo1_g, current[0]);
            }
            else
            {
                servo_iq18_currentLoop(&servo2_g, current[1]);
                slave_current_feedback(&transmit_packet, _IQ18toF(current[0]), _IQ18toF(current[1]));
            }

            irq_counter9k = !irq_counter9k;

            irq_counter250++;

            if (irq_counter250 == 36)
            {
                servo_iq18_velocityLoop(&servo1_g);
            }

            if (irq_counter250 == 72)
            {
                servo_iq18_velocityLoop(&servo2_g);
                irq_counter250 = 0;

                slave_velocity_feedback(&transmit_packet, _IQ18toF(servo1_g.encoder.angularVelocity), _IQ18toF(servo2_g.encoder.angularVelocity));
                servo_iq18_controlVelocity(&servo1_g, valid_packet.velocity_task1);
                servo_iq18_controlVelocity(&servo2_g, valid_packet.velocity_task2);
            }
        }
        else
        {
            HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_RESET);
            servo_iq18_reset(&servo1_g);
            servo_iq18_reset(&servo2_g);
        }
        if (reset_flag)
        {
            servo_iq18_reset(&servo1_g);
            servo_iq18_reset(&servo2_g);
            slave_servo_enc_is_reset(&transmit_packet);
            // transmit_packet.servos_reset = 0;
        }
    }
}