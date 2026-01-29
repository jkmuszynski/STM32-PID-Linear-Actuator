/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Main program body
******************************************************************************
* @attention
*
* Copyright (c) 2026 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*
******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#include "lcd_i2c.h"
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
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

#define MIN_PWM 0
#define MAX_PWM 275 // Maksimum bo powyżej zasilacz/sterownik odcina
#define AS5600_ADDR (0x36 << 1)
#define RAW_ANGLE_REG 0x0C
#define IMPULSES_PER_MM   3296    // Ile impulsów enkodera na 1mm
#define MOTOR_DEADZONE    20      // Poniżej tego PWM silnik buczy a nie jedzie
#define HOMING_SPEED      -200    // Prędkość bazowania
#define HOMING_TIMEOUT_MS 10000   // 10 sekund na znalezienie bazy (inaczej BŁĄD)
#define MAX_TRAVEL_MM     255     // Maksymalny wysuw
#define PID_INTERVAL_MS   2       // Co ile działa PID (zgodnie z ustawieniami Timer10)
#define PID_DEADBAND 400

// --- ZMIENNE SYSTEMOWE ---

volatile int32_t total_position = 0;
volatile int32_t active_target_imp = 0;
volatile int motor_pwm = 0;
volatile uint16_t last_angle = 0;
volatile int is_first_read = 1;
int16_t knob_val = 0;
int32_t preview_position_mm = 0;
uint32_t last_button_time = 0;
char msg[128];
uint32_t uart_timer = 0;

// --- ZMIENNE PID ---
float Kp = 0.05;  // Wzmocnienie proporcjonalne
float Ki = 0.05;  // Całka
float Kd = 0.02; // Różniczka

int32_t error = 0;
int32_t last_error = 0;
int32_t integral = 0;

// Zmienne do komunikacji z PC
uint8_t rx_byte;
uint8_t rx_buffer[32];
uint8_t rx_index = 0;
volatile uint8_t cmd_received_flag = 0;
volatile uint8_t control_mode = 0;// Flaga sterowania: 0 = Gałka (Manual), 1 = PC (Automat)


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Funkcje pomocnicze

// Nastawienie prędkości
void Set_Motor_Speed(int speed) {
    if (abs(speed) < MOTOR_DEADZONE) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
        return;
    }

    int abs_speed = abs(speed);
    if (abs_speed < MIN_PWM) abs_speed = MIN_PWM;
    if (abs_speed > MAX_PWM) abs_speed = MAX_PWM;

    if (speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, abs_speed);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, abs_speed);
    }
}


// Odczyt z enkodera magnetycznego
uint16_t AS5600_ReadAngle(void) {
    uint8_t raw_data[2];
    // Timeout 2ms - musi być krótki, bo jesteśmy w przerwaniu!
    if (HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, RAW_ANGLE_REG, I2C_MEMADD_SIZE_8BIT, raw_data, 2, 2) == HAL_OK) {
        return ((raw_data[0] << 8) | raw_data[1]);
    }
    return last_angle;
}


// Update pozycji
void Update_Position() {
    uint16_t curr_angle = AS5600_ReadAngle();
    if (is_first_read) {
        last_angle = curr_angle;
        is_first_read = 0; return;
    }
    int32_t diff = curr_angle - last_angle;
    if (diff > 2048)       total_position -= (4096 - diff);
    else if (diff < -2048) total_position += (4096 + diff);
    else                   total_position += diff;
    last_angle = curr_angle;
}


// Regulator
int Calculate_PID(int32_t current_pos, int32_t target_pos) {
    // 1. Error
    error = current_pos - target_pos;

    if (abs(error) < PID_DEADBAND) {
            error = 0;
            last_error = 0;
            integral = 0;
            return 0;
        }

    // 2. Proportional
    float P = error * Kp;

    // 3. Integral (z Anti-Windup)
    integral += error;
    //if (integral > 2000) integral = 2000;
    //if (integral < -2000) integral = -2000;
    float I = integral * Ki;

    // 4. Derivative
    float D = (error - last_error) * Kd;
    last_error = error;


    int output = (int)(P + I + D);
    if (output > MAX_PWM) output = MAX_PWM;
    if (output < -MAX_PWM) output = -MAX_PWM;

    return output;
}

//  bazowanie z timeoutem
void Auto_Homing(void) {
    char info[] = "BAZOWANIE...\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)info, strlen(info), 100);

    Set_Motor_Speed(HOMING_SPEED);

    uint32_t start_tick = HAL_GetTick();


    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) != GPIO_PIN_RESET) {
        if (HAL_GetTick() - start_tick > HOMING_TIMEOUT_MS) {
            Set_Motor_Speed(0);
            char err[] = "BLAD: TIMEOUT KRANCOWKI!\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), 100);
            while(1);
        }
    }

    Set_Motor_Speed(0);
    HAL_Delay(500);
    // Reset systemu
    __disable_irq(); // Wyłączamy przerwania na moment zerowania
    total_position = 0;
    active_target_imp = 0;
    last_angle = AS5600_ReadAngle();
    is_first_read = 0;
    __enable_irq();
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    char ok[] = "BAZOWANIE OK. START.\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)ok, strlen(ok), 100);
}

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  // Bazowanie
  Auto_Homing();
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // --- 1. ODBIÓR KOMEND Z PC --
	  if (cmd_received_flag) {
		  cmd_received_flag = 0;
		  // A. Komenda "MAN" -> Tryb Ręczny (Gałka)
		  if (strncmp((char*)rx_buffer, "MAN", 3) == 0) {
			  control_mode = 0;
			  // Synchronizacja gałki z aktualną pozycją (żeby nie szarpnęło)
			  if (preview_position_mm < 0) preview_position_mm = 0;
			  __HAL_TIM_SET_COUNTER(&htim2, preview_position_mm);
	            }
		  // B. Komenda "SET:xxx" -> Tryb PC
		  else if (strncmp((char*)rx_buffer, "SET:", 4) == 0) {
			  control_mode = 1; // Przełącz na PC
			  int new_mm = atoi((char*)rx_buffer + 4);
			  active_target_imp = new_mm * -IMPULSES_PER_MM;
			  preview_position_mm = new_mm;
		  }
		  else if (strncmp((char*)rx_buffer, "PID:", 4) == 0) {
			  float p, i, d;
			  if (sscanf((char*)rx_buffer, "PID:%f:%f:%f", &p, &i, &d) == 3) {
				  Kp = p; Ki = i; Kd = d;
			  }
		  }
	  }
	  // --- 2. OBSŁUGA MANUALNA (GAŁKA) ---
	  // Działa tylko w trybie 0.
	  if (control_mode == 0) {
		  int16_t raw_knob = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
		  if (raw_knob < 0) { __HAL_TIM_SET_COUNTER(&htim2, 0); raw_knob = 0; }
		  if (raw_knob > MAX_TRAVEL_MM) { __HAL_TIM_SET_COUNTER(&htim2, MAX_TRAVEL_MM); raw_knob = MAX_TRAVEL_MM; }
		  preview_position_mm = raw_knob;
		  // Przycisk fizyczny
		  if (HAL_GPIO_ReadPin(KNOB_SW_GPIO_Port, KNOB_SW_Pin) == GPIO_PIN_RESET) {
			  if (HAL_GetTick() - last_button_time > 300) {
				  last_button_time = HAL_GetTick();
				  active_target_imp = preview_position_mm * -IMPULSES_PER_MM;
				  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			  }
		  }
	  }
	  // --- 3. WYSYŁANIE DANYCH (TELEMETRIA) ---
	  if (HAL_GetTick() - uart_timer >= 100) {
		  uart_timer = HAL_GetTick();
		  if (huart2.gState == HAL_UART_STATE_READY) {
			  int current_mm = total_position / -IMPULSES_PER_MM;
			  int active_mm = active_target_imp / -IMPULSES_PER_MM;
			  int err_mm = error / -IMPULSES_PER_MM;
			  // Format: DAT,czas,CEL,AKTUALNA,ERR,PWM,KP,KD,TRYB
			  int len = sprintf(msg, "DAT,%ld,%d,%d,%d,%d,%d,%d,%d\r\n",HAL_GetTick(),active_mm,current_mm,err_mm,motor_pwm,(int)(Kp*100), (int)(Kd*100), control_mode);
			  HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg, len);
		  }
	  }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */


  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 4;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 99;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 1999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, R_EN_Pin|L_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : KNOB_SW_Pin */
  GPIO_InitStruct.Pin = KNOB_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KNOB_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENSOR_R_Pin */
  GPIO_InitStruct.Pin = SENSOR_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SENSOR_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENSOR_L_Pin */
  GPIO_InitStruct.Pin = SENSOR_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SENSOR_L_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R_EN_Pin L_EN_Pin */
  GPIO_InitStruct.Pin = R_EN_Pin|L_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM10) {
    	Update_Position();
        motor_pwm = Calculate_PID(total_position, active_target_imp);
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET && motor_pwm < 0) motor_pwm = 0;
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET && motor_pwm > 0) motor_pwm = 0;
        Set_Motor_Speed(motor_pwm);
    }
}



// Callback od odbioru UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		if (rx_byte == '\n' || rx_byte == '\r') {
			rx_buffer[rx_index] = 0;
            cmd_received_flag = 1;
            rx_index = 0;
		}
		else {
			if (rx_index < 31) rx_buffer[rx_index++] = rx_byte;
		}
		HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
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
#ifdef USE_FULL_ASSERT
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
