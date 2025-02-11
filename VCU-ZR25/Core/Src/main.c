/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"
#include "stdio.h"
#include "stdbool.h"
#include "usbd_cdc_if.h"
#include "driver_sensors.h"

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
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;

WWDG_HandleTypeDef hwwdg;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
osThreadId_t fsmTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

typedef enum {
  VEHICLE_OFF,
  LOW_VOLTAGE_STATE,
  TRACTIVE_SYSTEM_ACTIVE_STATE,
  READY_TO_DRIVE_STATE,
  LOCKOUT_STATE
} VCU_State_t;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_WWDG_Init(void);
void StartDefaultTask(void *argument);
void StartFSMTask(void *argument);
void TransitionState(VCU_State_t newState);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

VCU_State_t currentState = VEHICLE_OFF;
/* Interrupt flags */
volatile uint8_t GLVMS_Turned_On = 0;
volatile uint8_t Shutdown_Loop_Open = 0;
volatile uint8_t Shutdown_Loop_Open_Critical = 0;
volatile uint8_t External_Button_Pressed = 0;
volatile uint8_t Brake_Pressed = 0;
volatile uint8_t Start_Button_Pressed = 0;
volatile uint8_t Fault_Cleared = 0;
volatile uint8_t External_Reset_Pressed = 0;

void StartFSMTask(void *argument)
{
  for(;;)
  {
    switch(currentState)
    {
      case VEHICLE_OFF:
        if (GLVMS_Turned_On)
        {
          TransitionState(LOW_VOLTAGE_STATE);
        }
        break;

      case LOW_VOLTAGE_STATE:
		if (!GLVMS_Turned_On)
		{
		  TransitionState(VEHICLE_OFF);
		}
		else if (!Shutdown_Loop_Open && External_Button_Pressed)
        {
          TransitionState(TRACTIVE_SYSTEM_ACTIVE_STATE);
        }
        break;

      case TRACTIVE_SYSTEM_ACTIVE_STATE:
    	if (!GLVMS_Turned_On)
    	{
    	  TransitionState(VEHICLE_OFF);
    	}
        else if (Shutdown_Loop_Open_Critical)
		{
		  TransitionState(LOCKOUT_STATE);
		}
        else if (Shutdown_Loop_Open)
        {
          TransitionState(LOW_VOLTAGE_STATE);
        }
    	else if (Brake_Pressed && Start_Button_Pressed)
        {
          TransitionState(READY_TO_DRIVE_STATE);
        }
        break;

      case READY_TO_DRIVE_STATE:
      	if (!GLVMS_Turned_On)
      	{
      	  TransitionState(VEHICLE_OFF);
      	}
      	if (Shutdown_Loop_Open_Critical)
		{
		  TransitionState(LOCKOUT_STATE);
		}
        else if (Shutdown_Loop_Open)
        {
          TransitionState(LOW_VOLTAGE_STATE);
        }
        break;

      case LOCKOUT_STATE:
		if (!GLVMS_Turned_On)
		{
		  TransitionState(VEHICLE_OFF);
		}
        if (Fault_Cleared && External_Reset_Pressed)
        {
          TransitionState(TRACTIVE_SYSTEM_ACTIVE_STATE);
        }
        break;

      default:
        break;
    }

    osDelay(1);
  }
}

/* TODO: Determine Pin writes and reads for transitions and exceptions */
void TransitionState(VCU_State_t newState)
{
  currentState = newState;

  switch(newState)
  {
    case VEHICLE_OFF:
      HAL_GPIO_WritePin(GPIOA, STATUS_LED_1_Pin, GPIO_PIN_RESET);
      break;

    case LOW_VOLTAGE_STATE:
      HAL_GPIO_WritePin(GPIOA, STATUS_LED_1_Pin, GPIO_PIN_SET);
      break;

    case TRACTIVE_SYSTEM_ACTIVE_STATE:
      HAL_GPIO_WritePin(GPIOB, STATUS_LED_1_Pin, GPIO_PIN_SET);
      break;

    case READY_TO_DRIVE_STATE:
      HAL_GPIO_WritePin(GPIOA, STATUS_LED_1_Pin, GPIO_PIN_SET);
      break;

    case LOCKOUT_STATE:
      HAL_GPIO_WritePin(GPIOB, STATUS_LED_1_Pin, GPIO_PIN_RESET);
      break;

    default:
      break;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GLV_BATTERY_Pin)
  {
    GLVMS_Turned_On = 1;
  }
  else if (GPIO_Pin == VCU_SHUTDOWN_LOOP_Pin)
  {
    Shutdown_Loop_Open = 1;
  }
  else if (GPIO_Pin == VCU_SHUTDOWN_LOOP_Pin)
{
  Shutdown_Loop_Open_Critical = 1;
}
  else if (GPIO_Pin == STATUS_LED_1_Pin)
  {
    External_Button_Pressed = 1;
  }
  else if (GPIO_Pin == (BPS_FRONT_Pin | BPS_REAR_Pin))
  {
    Brake_Pressed = 1;
  }
  else if (GPIO_Pin == STATUS_LED_1_Pin)
  {
    Start_Button_Pressed = 1;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int _write(int file, char *ptr, int len)
{
  int i = 0;
  for (i = 0; i<len; i++)
  {
    ITM_SendChar((*ptr++));
  }
  return len;
}

uint16_t count;

int main(void)
{

  /* USER CODE BEGIN 1 */
  APPSSensor_t apps;
  BPSSensor_t bps_f;
  BPSSensor_t bps_r;
  SteeringAngleSensor_t steering_angle;
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
  MX_WWDG_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  fsmTaskHandle = osThreadNew(StartFSMTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
     /* USER CODE END WHILE */

     /* USER CODE BEGIN 3 */
 	  // Heart beat
	  HAL_GPIO_TogglePin(GPIOD, LD3_Pin);

 	  // Read APPS sensor
	  temp_channel = sConfig.Channel;
	  sConfig.Channel = ADC_CHANNEL_1;
 	  read_driver_input(&hadc1);
 	  apps = get_apps_data();
 	  bps_f = get_bps_front_data();
 	  bps_r = get_bps_rear_data();
 	  steering_angle = get_steering_angle_data();
 	  sConfig.Channel = temp_channel;

 	  // Format data to send over USB
 	  char msg_buffer[1024];

 	  uint16_t length = snprintf(msg_buffer, sizeof(msg_buffer),
 			  "\nAccelerator Pedal:\n"
 			  "- Raw Value 1: %u\n"
 			  "- Raw Value 2: %u\n"
 			  "- Voltage 1: %u mV\n"
 			  "- Voltage 2: %u mV\n"
 			  "- Pedal Percentage: %u percent * 10\n"
 			  "- Channel 1: %u percent * 10\n"
 			  "- Channel 2: %u percent * 10\n"
 			  "- APPS Plausibility: %d\n"
 			  "\n"
 			  "Brake Pressure:\n"
 			  "- Raw Value Front: %u\n"
 			  "- Raw Value Rear: %u\n"
 			  "- Voltage Front: %u mV\n"
 			  "- Voltage Rear: %u mV\n"
 			  "- Pressure Front: %u PSI\n"
 			  "- Pressure Rear: %u PSI\n"
 			  "- Plausibility Front: %d\n"
 			  "- Plausibility Rear: %d\n"
 			  "\n"
 			  "Steering Angle:\n"
 			  "- Device status: %u\n"
 			  "- Angle: %u radians * 1000\n"
 			  "- Plausibility Front: %d\n",
 			  apps.raw_value_1,
 			  apps.raw_value_2,
 			  apps.voltage_1,
 			  apps.voltage_2,
 			  apps.percent,
 			  apps.percent_1,
 			  apps.percent_2,
 			  (uint8_t)apps.plausible,

 			  bps_f.raw_value,
 			  bps_r.raw_value,
 			  bps_f.voltage,
 			  bps_r.voltage,
 			  bps_f.pressure,
 			  bps_r.pressure,
 			  (uint8_t)bps_f.plausible,
 			  (uint8_t)bps_r.plausible,

 			  steering_angle.i2c_device.device_status,
 			  steering_angle.angle,
 			  (uint8_t)steering_angle.plausible);

 	  // Ensure snprintf was successful and message length is valid
 	  if (length > 0 && length < sizeof(msg_buffer)) {
 	      // Send only the formatted message length over USB
 	      CDC_Transmit_FS((uint8_t *)msg_buffer, length);
 	  } else {
 	      // Handle error in formatting or length (optional)
 	  }

 	  HAL_Delay(50);
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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
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
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  * @brief WWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_WWDG_Init(void)
{

  /* USER CODE BEGIN WWDG_Init 0 */

  /* USER CODE END WWDG_Init 0 */

  /* USER CODE BEGIN WWDG_Init 1 */

  /* USER CODE END WWDG_Init 1 */
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
  hwwdg.Init.Window = 64;
  hwwdg.Init.Counter = 64;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG_Init 2 */

  /* USER CODE END WWDG_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HEARTBEAT_LED_Pin|STATUS_LED_1_Pin|STATUS_LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, WATER_PUMP_2_Pin|FAN_1_Pin|FAN_2_Pin|CAN2_STANDBY_Pin
                            |CAN1_STANDBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VCU_SHUTDOWN_LOOP_Pin|WATER_PUMP_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : HEARTBEAT_LED_Pin STATUS_LED_1_Pin STATUS_LED_2_Pin */
  GPIO_InitStruct.Pin = HEARTBEAT_LED_Pin|STATUS_LED_1_Pin|STATUS_LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : VCU_SHUTDOWN_LOOP_Pin WATER_PUMP_1_Pin */
  GPIO_InitStruct.Pin = VCU_SHUTDOWN_LOOP_Pin|WATER_PUMP_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : WATER_PUMP_2_Pin FAN_1_Pin FAN_2_Pin CAN2_STANDBY_Pin
                           CAN1_STANDBY_Pin */
  GPIO_InitStruct.Pin = WATER_PUMP_2_Pin|FAN_1_Pin|FAN_2_Pin|CAN2_STANDBY_Pin
                          |CAN1_STANDBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

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
