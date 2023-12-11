## Концепт
Общая логика работы следующая:
Передача осуществляется периодически по прерыванию с таймера.

1. Master по прерыванию с таймера опускает соответствующий NSS пин и начинает прием/передачу.
2. Slave в прерывании по спаду сигнала на NSS, сбрасывает SSI бит (разрешает тактирование сдвигового регистра SPI). Начинает приём данных.
3. Master в колбеке RxTxCplt поднимает пин NSS
4. Slave в прерывании по фронту на NSS устанавливает SSI бит.

## Master
Привожу код (только то, что касается SPI).

файл main.c:
```c
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	HAL_GPIO_WritePin(SS1_GPIO_Port, SS1_Pin, GPIO_PIN_SET); // Поднимаем NSS пин
}
```

файл stm32f4xx_it.c:
```c
...
// Данные
uint8_t txbuffer[27] = { 127, 156, 184, 209, 229, 244, 252, 254, 249, 237, 219,
		197, 170, 142, 112, 84, 57, 35, 17, 5, 0, 2, 10, 25, 45, 70, 98 };
uint8_t rxbuffer[27];

...

// Периодическое прерывание по переполнению таймера
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	HAL_GPIO_WritePin(SS1_GPIO_Port, SS1_Pin, GPIO_PIN_RESET); // Опускаем NSS пин
	HAL_SPI_TransmitReceive_DMA(&hspi1, txbuffer, rxbuffer, 27);

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}
```

## Slave
main.c:

```c
// Данные
uint8_t rxbuffer[27];
uint8_t txbuffer[27] = { 127, 170, 209, 237, 252, 252, 237, 209, 170, 127, 84,
		45, 17, 2, 2, 17, 45, 84, 127, 170, 209, 237, 252, 252, 237, 209, 170 };

...

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == NSS_Pin) {
		if (HAL_GPIO_ReadPin(NSS_GPIO_Port, NSS_Pin)) { // Фронт сигнала на NSS
			hspi2.Instance->CR1 |= 0x100; // Устанавливаем SSI бит (software nss)

      			// При чтении из регистра DR, стек rx fifo обнуляется, помогает со сдвигом принимаемых данных.
			uint32_t clearFifo = hspi2.Instance->DR;

		} else { // Спад сигнала на NSS

			hspi2.Instance->CR1 &= 0xFEFF; // Сбрасываем SSI бит (software nss)
			HAL_SPI_TransmitReceive_DMA(&hspi2, txbuffer, rxbuffer, 27);
		}
	}
}
```
