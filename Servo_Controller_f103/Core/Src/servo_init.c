#include "servo_init.h"

// servo_iq18_t servo1_g, servo2_g;

void servo_periph_init()
{

    // Init timers

    // TIM1&2 for encoders
    MX_TIM1_Init();
    MX_TIM2_Init();

    // TIM3&4 for periodical tasks
    MX_TIM3_Init(); // PWM as well

    // Init & calibrate ADC
    MX_ADC1_Init();
    HAL_ADCEx_Calibration_Start(&hadc1);

    // Start timers

    // Clear CNT and start timers 1&2 in encoder mode
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    // Start TIM3 in PWM mode and also turn on interrupts
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    // HAL_TIM_Base_Start_IT(&htim3);
}

void servo_init()
{
    // Init servo1
    if (SERVO1_ENABLE)
    {
        servo_iq18_base_init(&servo1_g, Triple, MT_VELOCITY, MT_GEAR_RATIO, SERVO1_REVERSE);
        servo_iq18_encoder_init(&servo1_g, &htim2, ENC_TPR, SERVO1_ENC_REVERSE);

        servo_iq18_driver_init(&servo1_g, &htim3, 1, INA_GPIO_Port, INA_Pin,
                               INB_GPIO_Port,
                               INB_Pin, 0, 1000);

        servo_iq18_position_init(&servo1_g, ANG_KP, ANG_KI, ANG_KD, ANG_DT, ANG_KB);
        servo_iq18_velocity_init(&servo1_g, VEL_KP, VEL_KI,
                                 VEL_KD, VEL_DT, VEL_KB);
        servo_iq18_current_init(&servo1_g, CUR_LIMIT, CUR_KP, CUR_KI, CUR_KD,
                                CUR_DT, CUR_KB);
        servo_iq18_setPositionTolerance(&servo1_g, ANG_TOLERANCE);
    }

    // Init servo2
    if (SERVO2_ENABLE)
    {
        servo_iq18_base_init(&servo2_g, Triple, MT_VELOCITY, MT_GEAR_RATIO, SERVO2_REVERSE);
        servo_iq18_encoder_init(&servo2_g, &htim1, ENC_TPR, SERVO2_ENC_REVERSE);

        servo_iq18_driver_init(&servo2_g, &htim3, 2, INA2_GPIO_Port, INA2_Pin,
                               INB2_GPIO_Port,
                               INB2_Pin, 0, 1000);

        servo_iq18_position_init(&servo2_g, ANG_KP, ANG_KI, ANG_KD, ANG_DT, ANG_KB);
        servo_iq18_velocity_init(&servo2_g, VEL_KP, VEL_KI,
                                 VEL_KD, VEL_DT, VEL_KB);
        servo_iq18_current_init(&servo2_g, CUR_LIMIT, CUR_KP, CUR_KI, CUR_KD,
                                CUR_DT, CUR_KB);
        servo_iq18_setPositionTolerance(&servo2_g, ANG_TOLERANCE);
    }
}