#include "main_init.h"

void init_mtr_ctrl()
{
    // Init GPIO and set some pin states
    pin_init_mtcrl();

    // Init TIMERS and start them
    servo_periph_init();

    // Init servo structures
    servo_init();

    // Init ADC&DMA
    adc_cur_init();

    // Reset servos
    servo_iq18_reset(&servo1_g);
    servo_iq18_reset(&servo2_g);

    // Init finished
    system_enabled = 1;
}
