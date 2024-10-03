#include "pins_init.h"

void pin_init_mtcrl()
{
    // Init DMA
    MX_DMA_Init();
    // Initialize gpio; dma for adc
    MX_GPIO_Init();

    // Block powering motors by setting Enable pins of a driver LOW
	HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_RESET);
}