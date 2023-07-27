# STM32_Motor_control
Модуль для регулирования коллекторного двигателя с энкодером.
## Возможности
 - Использование одного/двух/трех контуров управления (по положению, угловой скорости и току)
 - Регулирование как положения, так и угловой скорости (при наличии контура скорости)
 - Встроен алгоритм управления каналом таймера в режиме генерации ШИМ, а также выбором направления вращения мотора с помощью двух GPIO
 - Встроен опрос энкодера, расчет относительного положения и угловой скорости

**Внимание: данные для контура тока необходимо вводить самостоятельно, т.е. вам нужно самим настроить АЦП или получение данных о токе из внешнего источника**

## Содержание
- [Установка](#install)
- [Настройка проекта](#mxproj)
- [Объявление экземпляра структуры](#structure)
- [Инициализация](#init)
- [Опрос данных и расчёт контуров регулирования](#contours)


<a id="install"></a>
## Установка
Распаковать архив _servocontrol.zip из релиза в папку проекта.

В файлах main.c и stm32...xx_it.c включить следующий файл:
```c++
/* USER CODE BEGIN Includes */
#include <servocontroller.h>
/* USER CODE END Includes */
```

<a id="mxproj"></a>
## Настройка проекта
Для работы одного мотора нужно инициализировать (спойлеры разворачиваются):


<details>
<summary>1. Таймер в режиме энкодера, к каналам которого необходимо подключить энкодер.</summary>

------------

![Preview1](./images/EncoderMode.png)

------------

После этого в функцию `int main()` файла main.c нужно добавить следующее (пример для таймера TIM1):

```c++
/* USER CODE BEGIN 2 */
__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
/* USER CODE BEGIN 2 */
```
Убедитесь, что вызываете эти методы перед бесконечным циклом `while(1)` - в указанном выше плейсхолдере для пользовательского кода

------------
</details>

<details>
<summary>2. Канал другого таймера в режиме генерации ШИМ, к пину которого подключаем вывод PWM драйвера.</summary>

Частоту работы ШИМ необходимо выбирать исходя из характеристик драйвера. Например, если в характеристиках драйвера указано до 20 кГц, стоит установить 18 кГц (небольшой запас прочности). Частота от 18 кГц наиболее оптимальна, т.к. это за пределами порога слышимости большинства людей.

![Preview1](./images/PWMSettings.png)

------------

После этого в функции `int main()` файла main.c нужно включить ШИМ на используемых каналах другого таймера (пример для TIM3):

```c++
/* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
/* USER CODE BEGIN 2 */
```
Убедитесь, что вызываете эти методы перед бесконечным циклом `while(1)` - в указанном выше плейсхолдере для пользовательского кода

------------
</details>

3. Два GPIO в режиме output для управления направлением вращения мотора с помощью драйвера.

<a id="structure"></a>
## Объявление экземпляра структуры
Для каждого контролируемого привода необходимо объявить экземпляр типа `servocontrol_t` в файлах main.c:

```c++
/* USER CODE BEGIN PV */
servocontrol_t servo1;
/* USER CODE END PV */
```

и stm32...xx_it.c:
```c++
/* USER CODE BEGIN EV */
extern servocontrol_t servo1;
/* USER CODE END EV */
```
<a id="init"></a>
## Инициализация
Теперь необходимо проинициализировать отдельные компоненты модуля необходимыми настройками.

**Внимание:** `servocontrol_t *servo` во всех функциях - указатель на экземпляр структуры, т.е. первым аргументом вы подаете: `&yourServoName`. Далее это объясняться не будет.

<details>
<summary>Спойлер: инициализация компонентов</summary>

```c++
void servo_baseInit(servocontrol_t *servo, enum loops servoLoops, float motorSpeed, float gearRatio,
		uint8_t reverse);
// servoLoops - количество используемых контуров управления
//   Single - регулирование по углу положения вала
//   Double - подчиненное регулирование по положению и угловой скорости
//   Triple - подчиненное регулирование по положению, угловой скорости и току (пропорционален моменту)


// motorSpeed - скорость привода до редуктора в РАД/С
// gearRatio - передаточное число редуктора. Например, если передаточное число 1:21.3, передайте 21.3.
// 	Если редуктора нет, или хотите регулировать до привод без учета редукции
//	(бывает полезно при большом влиянии вязкого трения редуктора на работу привода), передайте 1.
// reverse - определяет направление вращения, передайте 0 или 1


void servo_encoderInit(servocontrol_t *servo, TIM_HandleTypeDef *htim, uint16_t CPR);
// htim - указатель на обработчик таймера, например &htim1, если используется TIM1
// CPR - количество счетов таймера за один оборот мотора (если использованы два канала, CPR=(PPR*4)-1.
//		PPR можно узнать из характеристик энкодера.


void servo_driverInit(servocontrol_t *servo, TIM_HandleTypeDef *htim, uint8_t timerChannel,
		GPIO_TypeDef *dir1_Port, uint32_t dir1_Pin, GPIO_TypeDef *dir2_Port, uint32_t dir2_Pin,
		uint16_t minDuty, uint16_t maxDuty);
// htim - обработчик таймера, генерирующего ШИМ-сигнал.
// timerChannel - номер канала таймера, который контролирует скорость данного привода (числом: 1/2/3/4)
// Далее пины, управляющие направлением вращения привода через драйвер (с указанием портов)
// minDuty - минимальное значение шим, отличное от нуля, которое будет выдавать микроконтроллер (обычно 0)
// maxDuty - максимальное значение шим, которое будет выдавать микроконтроллер.
//		Рекомендую взять значение, равное ARR-1, где ARR - arr регистр таймера
//		Стоит уменьшить его на единицу, так как при полном заполнении есть риск перегрева мосфетов.
```
</details>

<details>
<summary>Спойлер: инициализация контуров управления</summary>
	
```c++
//------------------------ Следующие инициализаторы - настройки контуров управления ------------------------
//------------ Рекомендуется инициализировать только те контуры, которые будут использоваться --------------

// kp, ki, kd - коэффициенты ПИД регулятора контура
// dt - период работы каждого контура в секундах (очень важно соблюдать эту величину)
// kt - коэффициент алгоритма anti-windup. При отсутствии интегральной составляющей оставить 0

void servo_positionInit(servocontrol_t *servo, float kp, float ki, float kd, float dt, float kt);
void servo_velocityInit(servocontrol_t *servo, float kp, float ki, float kd, float dt, float kt);
void servo_currentInit(servocontrol_t *servo, float ratedCurrent, float kp, float ki, float kd, float dt,
		float kt);
// ratedCurrent - номинальный ток мотора в амперах
```
</details>

<a id="contours"></a>
## Опрос данных и расчет контуров регулирования
Для опроса данных с энкодера и расчёта необходимых для регулирования величин, в модуле предусмотрено 3 метода:

```c++
void servo_positionLoop(servocontrol_t *servo); // Контур положения
void servo_velocityLoop(servocontrol_t *servo); // Контур угловой скорости
void servo_currentLoop(servocontrol_t *servo, float currentFeedback); // Контур тока. currentFeedback - текущий ток (А)
```
Используйте только те контуры, которые вы указали в `servo_baseInit`.

Каждый из вышеперечисленных методов должен вызываться с определенной частотой. Ранее в методах `servo_positionInit`, `servo_velocityInit` и `servo_currentInit` вы указали период интегрирования dt для каждого контура. Величина, равная 1/dt - и есть частота, с которой должен вызываться метод расчета соответствующего контура. Т.е. если для контура положения был указан dt = 0.01, то метод `servo_positionLoop` должен вызываться с частотой 100 Гц.

</details>

<details>
<summary>Совет 1: Выбор частот</summary>
	
### Выбор частоты опроса контуров
- **Ток:** функция для расчёта регулятора тока в идеале должна вычисляться с частотой соответствующей обновлению задания для ШИМ силовых ключей. Чтобы для каждой новой коммутации ШИМ уже было рассчитано обновлённое значение на выходе регулятора тока. Но допускается **кратно** снижать частоту в несколько раз. Чем больше частота расчета контура тока - тем лучше, но необходимо учитывать вычислительные возможности процессора, АЦП и другие факторы. Обычно частоты для вызова регулятора тока - несколько килогерц.
- **Угловая скорость:** - для контура скорости обычно достаточно частоты 50-200 Гц, но опять же, частота должна быть кратно меньше частоты обновления контура тока. При отстутствии контура тока, частота должна быть кратно меньше частоты ШИМ.
- **Положение** - контур положения стоит обновлять с той же частотой, что и контур скорости (при его наличии). При остутствии контура скорости, необходимо рассчитать частоту исходя из максимальной скорости вращения вала привода.
</details>

</details>

<details>
<summary>Совет 2: Возможная реализация</summary>
	
### Способ вызова функций с необходимой частотой
Самый простой способ обеспечить кратность частот ШИМ и контуров - использовать прерывания по переполнению счетчика таймера, генерирующего ШИМ и программного счётчика. Данный метод не самый "элегантный", если есть достаточное число таймеров для всех контуров, то лучше настроить прерывания с их помощью.
Но в случае нехватки таймеров, данный метод тоже сработает.
Сначала включаем 
</details>
