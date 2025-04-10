/* USER CODE BEGIN Header */
/*
 * Torque Vectoring HIL Test firmware
 *
 * This project recieves CAN messages from the Simulink HIL simulation, and
 * runs the torque vectoring algorithm to validate its embedded implementation.
 * The STM32 expects a message with driver and vehicle state data, then computes
 * the Mz_tv and torque outputs, sending them back over CAN, where Simulink
 * updates the plant model simulation.
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_DATA_SCALING_FACTOR 1000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */

// CAN Data and message identifiers
CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];  // CAN max payload is 8 bytes
uint8_t received_message_yaw = 0;
uint8_t received_message_vehicle_state = 0;

// Vehicle Data
float ref_velocity = 0.0f;
float sw_angle = 0.0f;

// Yaw Controller Data
float input_data = 0.0f;
float output_data = 0.0f;
float time_step = 0.0f;

// PID Data
pid_t pid_data;
float sampling_period = 0.01f;
float tau = 0.0f;
gain_t sample_gains = {7000.0, 2000.0, 0.0};  // Sample gains for the HIL test model

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

void init_can_filter(CAN_FilterTypeDef canfilter);
void process_can_message(void);
void send_can_message(int16_t actuating_signal);

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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  uint32_t last_toggle = HAL_GetTick();

  // Enable the 5V Power rail
  HAL_GPIO_WritePin(GPIOC, RAIL_POWER_ENABLE_5V_Pin, GPIO_PIN_SET);

  // Pull The standby pin low to put the CAN tranceivers in normal mode
  HAL_GPIO_WritePin(GPIOC, CAN_1_STANDBY_Pin, GPIO_PIN_RESET);

  HAL_CAN_Start(&hcan1);

  // Set CAN Bus filter to find ID = 0x120 to 0x125 messages
  CAN_FilterTypeDef canfilter;
  (void)init_can_filter(canfilter);

  if (HAL_CAN_ConfigFilter(&hcan1, &canfilter) != HAL_OK) {
      Error_Handler();  // Or debug print
  }

  // Initialize PID controller and set gains
  (void)init_pid(&pid_data, sampling_period, tau);
  (void)update_gains(&pid_data, sample_gains);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	// Heatnbeat to ensure it is running
	if (HAL_GetTick() - last_toggle >= 500) {
		HAL_GPIO_TogglePin(GPIOC, DEBUG_LED_1_Pin);
	    last_toggle = HAL_GetTick();
	}

	// 1. Wait to recieve a CAN message. This is non-blocking so we only want to compute the PID if we receive a message
    (void)process_can_message();

    // Schedule Gains
    if(received_message_vehicle_state) {
      // Interpolate the lookup table to find the closes gain values
      gain_t new_gains = schedule_gains(ref_velocity, sw_angle);
      update_gains(&pid_data, new_gains);
      received_message_vehicle_state = 0;
    }

    // Compute PID controller
    if(received_message_yaw) {
    	// Note: this only works because this is not computed real-time. Simulation data between messages is sent every simulation 0.01s time step, which is what the sample_time is set to.

      // Calculate error signal
    	float error = input_data - output_data; 
    	update_pid(&pid_data, error);

    	// Send a CAN message out
    	float actuating_signal = pid_data.u;
    	(void)send_can_message((int16_t)(actuating_signal));

    	received_message_yaw = 0;
    }

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 4199;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DEBUG_LED_1_Pin|DEBUG_LED_2_Pin|DEBUG_LED_3_Pin|CAN_1_STANDBY_Pin
                          |CAN_2_STANDBY_Pin|RAIL_POWER_ENABLE_5V_Pin|PUMP_1_CONTROL_Pin|PUMP_2_CONTROL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FAN_1_CONTROL_Pin|FAN_2_CONTROL_Pin|VCU_FAULT_Pin|BRAKE_LIGHT_CONTROL_Pin
                          |BUZZER_CONTROL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DASH_INPUT_1_GPIO_Port, DASH_INPUT_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DEBUG_LED_1_Pin DEBUG_LED_2_Pin DEBUG_LED_3_Pin CAN_1_STANDBY_Pin
                           CAN_2_STANDBY_Pin RAIL_POWER_ENABLE_5V_Pin PUMP_1_CONTROL_Pin PUMP_2_CONTROL_Pin */
  GPIO_InitStruct.Pin = DEBUG_LED_1_Pin|DEBUG_LED_2_Pin|DEBUG_LED_3_Pin|CAN_1_STANDBY_Pin
                          |CAN_2_STANDBY_Pin|RAIL_POWER_ENABLE_5V_Pin|PUMP_1_CONTROL_Pin|PUMP_2_CONTROL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VCU_SHUTDOWN_LOOP_IN_Pin VCU_SHUTDOWN_LOOP_RESET_Pin START_BUTTON_Pin DASH_INPUT_2_Pin
                           DASH_INPUT_3_Pin DASH_INPUT_4_Pin */
  GPIO_InitStruct.Pin = VCU_SHUTDOWN_LOOP_IN_Pin|VCU_SHUTDOWN_LOOP_RESET_Pin|START_BUTTON_Pin|DASH_INPUT_2_Pin
                          |DASH_INPUT_3_Pin|DASH_INPUT_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FAN_1_CONTROL_Pin FAN_2_CONTROL_Pin VCU_FAULT_Pin BRAKE_LIGHT_CONTROL_Pin
                           BUZZER_CONTROL_Pin */
  GPIO_InitStruct.Pin = FAN_1_CONTROL_Pin|FAN_2_CONTROL_Pin|VCU_FAULT_Pin|BRAKE_LIGHT_CONTROL_Pin
                          |BUZZER_CONTROL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT_1_Pin */
  GPIO_InitStruct.Pin = BOOT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DASH_INPUT_1_Pin */
  GPIO_InitStruct.Pin = DASH_INPUT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DASH_INPUT_1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void init_can_filter(CAN_FilterTypeDef canfilter) {
  canfilter.FilterActivation = ENABLE;
  canfilter.FilterBank = 0;                         // Use filter bank 0
  canfilter.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilter.FilterScale = CAN_FILTERSCALE_32BIT;

  // Accept IDs from 0x120 to 0x125
  // Mask: match bits that define 0x120 to 0x125
  // ID bits 10:3 must match 0x24 (0b100100) -> bits 10:3 of 0x120 to 0x125 are the same
  // Bits 2:0 can vary, so mask those out

  canfilter.FilterIdHigh     = 0x120 << 5;          // Shifted left 5 to align with CAN ID format
  canfilter.FilterIdLow      = 0x0000;

  canfilter.FilterMaskIdHigh = 0x1F8 << 5;          // Mask bits 10:3 (0x1F8 = 0b1111111000)
  canfilter.FilterMaskIdLow  = 0x0000;

  canfilter.SlaveStartFilterBank = 14;              // Only needed if using CAN2
}

void process_can_message() {
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
      if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {

    	  // Yaw Controller Data Message
          if (rxHeader.StdId == 0x123 && rxHeader.DLC >= 6) {

        	  // Toggle pin if the right message was recieved
        	  HAL_GPIO_TogglePin(GPIOC, DEBUG_LED_2_Pin);

        	  // Tell everyone we recieved the message
        	  received_message_yaw = 1;

            // MSB first
        	  input_data  = (int16_t)((rxData[0] << 8) | rxData[1]) / CAN_DATA_SCALING_FACTOR;
        	  output_data = (int16_t)((rxData[2] << 8) | rxData[3]) / CAN_DATA_SCALING_FACTOR;
        	  time_step   = (int16_t)((rxData[4] << 8) | rxData[5]) / CAN_DATA_SCALING_FACTOR;
          }

        // Vehicle State Message
        if (rxHeader.StdId == 0x122 && rxHeader.DLC >= 6) {

          // Toggle pin if the right message was recieved
          HAL_GPIO_TogglePin(GPIOC, DEBUG_LED_2_Pin);

          // Tell everyone we recieved the message
          received_message_vehicle_state = 1;

           // MSB first
          ref_velocity  = (float)((rxData[0] << 8) | rxData[1]) / CAN_DATA_SCALING_FACTOR;
          sw_angle = (float)((rxData[2] << 8) | rxData[3]) / CAN_DATA_SCALING_FACTOR;
        }
      }
  }
}

void send_can_message(int16_t actuating_signal) {
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[2];
    uint32_t txMailbox;

    // Prepare CAN header
    txHeader.StdId = 0x124;
    txHeader.ExtId = 0;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.IDE = CAN_ID_STD;
    txHeader.DLC = 2;
    txHeader.TransmitGlobalTime = DISABLE;

    // Pack the 16-bit value into 2 bytes (little-endian)
    txData[0] = actuating_signal & 0xFF;
    txData[1] = (actuating_signal >> 8) & 0xFF;

    // Transmit the message
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox) != HAL_OK) {
        // Optionally handle error
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
