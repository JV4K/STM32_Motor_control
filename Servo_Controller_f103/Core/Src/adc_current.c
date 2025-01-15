#include "adc_current.h"

volatile uint16_t adc[2];
_iq18 current[2];
EMA_iq18 *filter1, *filter2;
uint32_t check;
int ch_order = 1;

void adc_cur_init()
{
    // Starting DMA for capturing current measures from ADC
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc, 2);

    // Initia;lization of filters
    filter1 = initEMA_iq18(0.005, _IQ18(2048));
    filter2 = initEMA_iq18(0.005, _IQ18(2048));
    adc[0] = 2048;
    adc[1] = 2048;
}

void adc_callback_handler()
{
    ch_order = !ch_order;

    if (system_enabled)
    {
        if (!ch_order)
        {
            _iq18 adc_filtered = updateEMA_iq18(filter1, _IQ18(adc[ch_order]));
            current[ch_order] = _IQ18mpy((adc_filtered - _IQ18(2048)), _IQ18(CUR_SENSOR_GAIN));
        }
        else
        {
            _iq18 adc_filtered = updateEMA_iq18(filter2, _IQ18(adc[ch_order]));
            current[ch_order] = _IQ18mpy((adc_filtered - _IQ18(2048)), _IQ18(CUR_SENSOR_GAIN));
        }
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // check++;
    adc_callback_handler();
}