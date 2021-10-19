/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Timer Task, siehe Erklärung unten
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  *
  *
  * @ Erklärung für Herrn Kobelrausch:
  *
  * Die grüne LD3 blinkt gleichmaessig alle 4 Sekunden. Diese 4 Sekunden "delay"
  * entstehen durch den Aufruf der blocking method.
  * Immer nach dem auf- bzw. abdrehen der LD3 wird, parallel dazu, nach einer Sekunde die
  * rote RGB LED vom dev-board auf oder abgedreht - dies jedoch durch den Aufruf der
  * non-blocking method.
  * Da die non-blocking Methode eben nicht blockiert, verzögert sich der 4 Sekunden Rhythmus
  * der grünen ld3 NICHT (!) - es bleibt bei regelmaessigen 4 Sekunden - obwohl "parallel" dazu
  * die rote LED getoggelt wird.
  * Realisiert wird das Ganze mit einem globalen counter welcher in 50ms Schritten,
  * ausgelöst durch den TIM6 timer + interrupt, aufwärts zählt.
  *
  *
  *
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
unsigned int tim6_elapsed = 0; // Basically our global clock/counter; counts upwards in 50ms steps
unsigned int non_blocking_called_at_time = 0; // saves the "time" of the moment when the non-blocking method was called
unsigned int non_blocking_callback_timeout_ms = 0; // timeout after which the callback function passed to the non-blocking method should be called
void (*non_blocking_callback)(void) = NULL; // callback function reference
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void _tim_timeout_blocking(unsigned int);
void _tim_timeout_nonblocking_with_callback(unsigned int, void (*callback)(void));
void toggle_red_LED(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  // First, let's turn off the RED RGB LED
  HAL_GPIO_WritePin(RGB_LED_RED_GPIO_Port, RGB_LED_RED_Pin, GPIO_PIN_SET);

  // Turn on green LD3
  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

  // Start TIM6 with interrupt
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // First, we call the blocking method, which blocks for 4 seconds
    _tim_timeout_blocking(4000);
    // After 4 seconds, the green led (LD3) will be turned on or off
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    // Now we call the non-blocking method:
    // It returns immediately, so the green LD3 keeps "blinking" every 4 seconds
    // You will see that every time after 1 second after the green LD3 was turned off or on
    // the red LED will be turned off or on
    _tim_timeout_nonblocking_with_callback(1000, toggle_red_LED);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 32000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RGB_LED_RED_GPIO_Port, RGB_LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : VCP_TX_Pin */
  GPIO_InitStruct.Pin = VCP_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(VCP_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RGB_LED_RED_Pin */
  GPIO_InitStruct.Pin = RGB_LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RGB_LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VCP_RX_Pin */
  GPIO_InitStruct.Pin = VCP_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
  HAL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Blocks execution for a given amount of time (in milliseconds).
  * @param  timeout_milliseconds: Timeout in milliseconds for how long the method should block.
  *         Should be 50ms steps (E.g. 750, 900 or, 1050, but not 749, 816,..)
  * @retval None
  */
void _tim_timeout_blocking(unsigned int timeout_milliseconds) {
  unsigned int current_tim6_elapsed = tim6_elapsed;
  while (tim6_elapsed < current_tim6_elapsed + timeout_milliseconds) {
    // Here we block until the given timeout elapsed (measured with our own "clock" tim6_elapsed ;)
  }
}

/**
  * @brief  Runs a callback method after a given timeout has passed, but does that async with the help of an interrupt,
  *         and therefore does not block execution, which means it will return immediately.
  * @param  timeout_milliseconds: Timeout in milliseconds after which the callback method should be run.
  *         Should be 50ms steps (E.g. 750, 900 or, 1050, but not 749, 816,..)
  * @param  callback: Method reference used as callback method that will be run after the timeout has passed.
  * @retval None
  */
void _tim_timeout_nonblocking_with_callback(unsigned int timeout_milliseconds, void (*callback)(void)) {
  // Here you can see no work is done (= no blocking)
  // We just assign variables, which the interrupt routine will use by calling the callback method later
  non_blocking_callback = callback;
  non_blocking_callback_timeout_ms = timeout_milliseconds;
  non_blocking_called_at_time = tim6_elapsed;
}

/**
  * @brief  Turns on/off the red RDB led. Used as callback method for this task.
  *         Also unsets the callback method.
  * @retval None
  */
void toggle_red_LED() {
  HAL_GPIO_TogglePin(RGB_LED_RED_GPIO_Port, RGB_LED_RED_Pin);

  // After the red LED was toggled, let's reset the callback as proof that next time the non-blocking function
  // get's called the callback function will be set (again)
  non_blocking_callback = NULL;
  non_blocking_callback_timeout_ms = 0;
  non_blocking_called_at_time = 0;
}

/**
  * @brief  Interrupt routine which gets fired by timers.
  *         For our task we use it to increase our custom (global) counter and to call the callback method set by the
  *         non-blocking method.
  * @param  htim: Information about the timer.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Here the magic happens with our custom clock (which just is a counter)
  // Because we have 32MHZ and use a pre-scaler of 32000 that mans our clock speed is 1ms
  // However because I set the period to 49 the interrupt fires every 50ms and the counter therefore increased in 50ms steps
  tim6_elapsed += htim->Instance -> ARR + 1; // 49 + 1 = 50 ms to add to the counter

  if (non_blocking_callback != NULL && tim6_elapsed >= non_blocking_called_at_time + non_blocking_callback_timeout_ms) {
    // If a non-blocking callback is currently set and if its time to call it, call it ;)
    non_blocking_callback();
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
