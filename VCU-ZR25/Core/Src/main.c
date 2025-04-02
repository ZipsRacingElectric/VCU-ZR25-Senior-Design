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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "driver_sensors.h"
#include "vehicle_data.h"
#include "vehicle_fsm.h"
#include "power_supply.h"
#include "fault_mgmt.h"
#include "can_db.h"
#include "can_messages.h"
#include "cooling_system.h"
#include "torque_ctrl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
typedef StaticEventGroup_t osStaticEventGroupDef_t;
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

TIM_HandleTypeDef htim11;

powSupTaskArgs_t powsupArgs;
DriverSensorTaskArgs_t driversensorArgs;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for fsmTask */
osThreadId_t fsmTaskHandle;
uint32_t fsmTaskBuffer[ 512 ];
osStaticThreadDef_t fsmTaskControlBlock;
const osThreadAttr_t fsmTask_attributes = {
  .name = "fsmTask",
  .cb_mem = &fsmTaskControlBlock,
  .cb_size = sizeof(fsmTaskControlBlock),
  .stack_mem = &fsmTaskBuffer[0],
  .stack_size = sizeof(fsmTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for powsupTask */
osThreadId_t powsupTaskHandle;
uint32_t powsupTaskBuffer[ 512 ];
osStaticThreadDef_t powsupTaskControlBlock;
const osThreadAttr_t powsupTask_attributes = {
  .name = "powsupTask",
  .cb_mem = &powsupTaskControlBlock,
  .cb_size = sizeof(powsupTaskControlBlock),
  .stack_mem = &powsupTaskBuffer[0],
  .stack_size = sizeof(powsupTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for driversensrTask */
osThreadId_t driversensrTaskHandle;
uint32_t driversensrTaskBuffer[ 2048 ];
osStaticThreadDef_t driversensrTaskControlBlock;
const osThreadAttr_t driversensrTask_attributes = {
  .name = "driversensrTask",
  .cb_mem = &driversensrTaskControlBlock,
  .cb_size = sizeof(driversensrTaskControlBlock),
  .stack_mem = &driversensrTaskBuffer[0],
  .stack_size = sizeof(driversensrTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for faultTask */
osThreadId_t faultTaskHandle;
uint32_t faultTaskBuffer[ 512 ];
osStaticThreadDef_t faultTaskControlBlock;
const osThreadAttr_t faultTask_attributes = {
  .name = "faultTask",
  .cb_mem = &faultTaskControlBlock,
  .cb_size = sizeof(faultTaskControlBlock),
  .stack_mem = &faultTaskBuffer[0],
  .stack_size = sizeof(faultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canDbTask */
osThreadId_t canDbTaskHandle;
uint32_t canDbTaskBuffer[ 512 ];
osStaticThreadDef_t canDbTaskControlBlock;
const osThreadAttr_t canDbTask_attributes = {
  .name = "canDbTask",
  .cb_mem = &canDbTaskControlBlock,
  .cb_size = sizeof(canDbTaskControlBlock),
  .stack_mem = &canDbTaskBuffer[0],
  .stack_size = sizeof(canDbTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for coolingTask */
osThreadId_t coolingTaskHandle;
uint32_t coolingTaskBuffer[ 512 ];
osStaticThreadDef_t coolingTaskControlBlock;
const osThreadAttr_t coolingTask_attributes = {
  .name = "coolingTask",
  .cb_mem = &coolingTaskControlBlock,
  .cb_size = sizeof(coolingTaskControlBlock),
  .stack_mem = &coolingTaskBuffer[0],
  .stack_size = sizeof(coolingTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for dashboardTask */
osThreadId_t dashboardTaskHandle;
uint32_t dashboardTaskBuffer[ 512 ];
osStaticThreadDef_t dashboardTaskControlBlock;
const osThreadAttr_t dashboardTask_attributes = {
  .name = "dashboardTask",
  .cb_mem = &dashboardTaskControlBlock,
  .cb_size = sizeof(dashboardTaskControlBlock),
  .stack_mem = &dashboardTaskBuffer[0],
  .stack_size = sizeof(dashboardTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for torquectrlTask */
osThreadId_t torquectrlTaskHandle;
uint32_t torquectrlTaskBuffer[ 512 ];
osStaticThreadDef_t torquectrlTaskControlBlock;
const osThreadAttr_t torquectrlTask_attributes = {
  .name = "torquectrlTask",
  .cb_mem = &torquectrlTaskControlBlock,
  .cb_size = sizeof(torquectrlTaskControlBlock),
  .stack_mem = &torquectrlTaskBuffer[0],
  .stack_size = sizeof(torquectrlTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for amkTask */
osThreadId_t amkTaskHandle;
uint32_t amkTaskBuffer[ 512 ];
osStaticThreadDef_t amkTaskControlBlock;
const osThreadAttr_t amkTask_attributes = {
  .name = "amkTask",
  .cb_mem = &amkTaskControlBlock,
  .cb_size = sizeof(amkTaskControlBlock),
  .stack_mem = &amkTaskBuffer[0],
  .stack_size = sizeof(amkTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canDbTxQueue */
osMessageQueueId_t canDbTxQueueHandle;
uint8_t canDbTxQueueBuffer[ 16 * 16 ];
osStaticMessageQDef_t canDbTxQueueControlBlock;
const osMessageQueueAttr_t canDbTxQueue_attributes = {
  .name = "canDbTxQueue",
  .cb_mem = &canDbTxQueueControlBlock,
  .cb_size = sizeof(canDbTxQueueControlBlock),
  .mq_mem = &canDbTxQueueBuffer,
  .mq_size = sizeof(canDbTxQueueBuffer)
};
/* Definitions for canDbRxQueue */
osMessageQueueId_t canDbRxQueueHandle;
uint8_t canDbRxQueueBuffer[ 16 * 16 ];
osStaticMessageQDef_t canDbRxQueueControlBlock;
const osMessageQueueAttr_t canDbRxQueue_attributes = {
  .name = "canDbRxQueue",
  .cb_mem = &canDbRxQueueControlBlock,
  .cb_size = sizeof(canDbRxQueueControlBlock),
  .mq_mem = &canDbRxQueueBuffer,
  .mq_size = sizeof(canDbRxQueueBuffer)
};
/* Definitions for vdb_sas_lock */
osMutexId_t vdb_sas_lockHandle;
osStaticMutexDef_t vdb_sas_lockControlBlock;
const osMutexAttr_t vdb_sas_lock_attributes = {
  .name = "vdb_sas_lock",
  .cb_mem = &vdb_sas_lockControlBlock,
  .cb_size = sizeof(vdb_sas_lockControlBlock),
};
/* Definitions for vdb_apps_lock */
osMutexId_t vdb_apps_lockHandle;
osStaticMutexDef_t vdb_apps_lockControlBlock;
const osMutexAttr_t vdb_apps_lock_attributes = {
  .name = "vdb_apps_lock",
  .cb_mem = &vdb_apps_lockControlBlock,
  .cb_size = sizeof(vdb_apps_lockControlBlock),
};
/* Definitions for vdb_bps_front_lock */
osMutexId_t vdb_bps_front_lockHandle;
osStaticMutexDef_t vdb_bps_front_lockControlBlock;
const osMutexAttr_t vdb_bps_front_lock_attributes = {
  .name = "vdb_bps_front_lock",
  .cb_mem = &vdb_bps_front_lockControlBlock,
  .cb_size = sizeof(vdb_bps_front_lockControlBlock),
};
/* Definitions for vdb_bps_rear_lock */
osMutexId_t vdb_bps_rear_lockHandle;
osStaticMutexDef_t vdb_bps_rear_lockControlBlock;
const osMutexAttr_t vdb_bps_rear_lock_attributes = {
  .name = "vdb_bps_rear_lock",
  .cb_mem = &vdb_bps_rear_lockControlBlock,
  .cb_size = sizeof(vdb_bps_rear_lockControlBlock),
};
/* Definitions for vdb_inverter_lock */
osMutexId_t vdb_inverter_lockHandle;
osStaticMutexDef_t vdb_inverter_lockControlBlock;
const osMutexAttr_t vdb_inverter_lock_attributes = {
  .name = "vdb_inverter_lock",
  .cb_mem = &vdb_inverter_lockControlBlock,
  .cb_size = sizeof(vdb_inverter_lockControlBlock),
};
/* Definitions for vdb_fsm_state_lock */
osMutexId_t vdb_fsm_state_lockHandle;
osStaticMutexDef_t vdb_fsm_state_lockControlBlock;
const osMutexAttr_t vdb_fsm_state_lock_attributes = {
  .name = "vdb_fsm_state_lock",
  .cb_mem = &vdb_fsm_state_lockControlBlock,
  .cb_size = sizeof(vdb_fsm_state_lockControlBlock),
};
/* Definitions for vdb_powsup_lock */
osMutexId_t vdb_powsup_lockHandle;
osStaticMutexDef_t vdb_powsup_lockControlBlock;
const osMutexAttr_t vdb_powsup_lock_attributes = {
  .name = "vdb_powsup_lock",
  .cb_mem = &vdb_powsup_lockControlBlock,
  .cb_size = sizeof(vdb_powsup_lockControlBlock),
};
/* Definitions for vdb_cooling_lock */
osMutexId_t vdb_cooling_lockHandle;
osStaticMutexDef_t vdb_cooling_lockControlBlock;
const osMutexAttr_t vdb_cooling_lock_attributes = {
  .name = "vdb_cooling_lock",
  .cb_mem = &vdb_cooling_lockControlBlock,
  .cb_size = sizeof(vdb_cooling_lockControlBlock),
};
/* Definitions for vdb_dashboard_lock */
osMutexId_t vdb_dashboard_lockHandle;
osStaticMutexDef_t vdb_dashboard_lockControlBlock;
const osMutexAttr_t vdb_dashboard_lock_attributes = {
  .name = "vdb_dashboard_lock",
  .cb_mem = &vdb_dashboard_lockControlBlock,
  .cb_size = sizeof(vdb_dashboard_lockControlBlock),
};
/* Definitions for vdb_torquectrl_lock */
osMutexId_t vdb_torquectrl_lockHandle;
osStaticMutexDef_t vdb_torquectrl_lockControlBlock;
const osMutexAttr_t vdb_torquectrl_lock_attributes = {
  .name = "vdb_torquectrl_lock",
  .cb_mem = &vdb_torquectrl_lockControlBlock,
  .cb_size = sizeof(vdb_torquectrl_lockControlBlock),
};
/* Definitions for vdb_faulttask_lock */
osMutexId_t vdb_faulttask_lockHandle;
osStaticMutexDef_t vdb_faulttask_lockControlBlock;
const osMutexAttr_t vdb_faulttask_lock_attributes = {
  .name = "vdb_faulttask_lock",
  .cb_mem = &vdb_faulttask_lockControlBlock,
  .cb_size = sizeof(vdb_faulttask_lockControlBlock),
};
/* Definitions for can_db_lock */
osMutexId_t can_db_lockHandle;
osStaticMutexDef_t can_db_lockControlBlock;
const osMutexAttr_t can_db_lock_attributes = {
  .name = "can_db_lock",
  .cb_mem = &can_db_lockControlBlock,
  .cb_size = sizeof(can_db_lockControlBlock),
};
/* Definitions for amkEventFlags */
osEventFlagsId_t amkEventFlagsHandle;
osStaticEventGroupDef_t amkEventFlagsControlBlock;
const osEventFlagsAttr_t amkEventFlags_attributes = {
  .name = "amkEventFlags",
  .cb_mem = &amkEventFlagsControlBlock,
  .cb_size = sizeof(amkEventFlagsControlBlock),
};
/* USER CODE BEGIN PV */
osThreadId_t fsmTaskHandle;
osThreadId_t powsupTaskHandle;
osThreadId_t driversensorTaskHandle;
osThreadId_t faultTaskHandle;
osThreadId_t canTaskHandle;
osThreadId_t coolingTaskHandle;
osThreadId_t dashboardTaskHandle;
osThreadId_t torquectrlTaskHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM11_Init(void);
void StartDefaultTask(void *argument);
extern void StartFsmTask(void *argument);
extern void StartPwrSupTask(void *argument);
extern void StartDriverSensorTask(void *argument);
extern void StartFaultTask(void *argument);
extern void StartCanDbTask(void *argument);
extern void StartCoolingTask(void *argument);
extern void StartDashboardTask(void *argument);
extern void StartTorqueCtrlTask(void *argument);
extern void StartAMKTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  FSM_GPIO_Callback(GPIO_Pin);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef pHeader;
	uint8_t aData[8];
	if (HAL_CAN_GetRxMessage(&hcan1, 0, &pHeader, aData) == HAL_OK) {
		CANIRQRxHandler(&pHeader, aData);
	}
	if (HAL_CAN_GetRxMessage(&hcan2, 0, &pHeader, aData) == HAL_OK) {
		CANIRQRxHandler(&pHeader, aData);
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef pHeader;
	uint8_t aData[8];
	if (HAL_CAN_GetRxMessage(&hcan1, 0, &pHeader, aData) == HAL_OK) {
		CANIRQRxHandler(&pHeader, aData);
	}
	if (HAL_CAN_GetRxMessage(&hcan2, 0, &pHeader, aData) == HAL_OK) {
		CANIRQRxHandler(&pHeader, aData);
	}
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  initVehicleData();

  powsupArgs = (powSupTaskArgs_t){.hadc1 = hadc1, .sConfig = {0}};
  driversensorArgs = (DriverSensorTaskArgs_t){.hadc1 = hadc1, .sConfig = {0}, .hi2c1 = hi2c1};

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of vdb_sas_lock */
  vdb_sas_lockHandle = osMutexNew(&vdb_sas_lock_attributes);

  /* creation of vdb_apps_lock */
  vdb_apps_lockHandle = osMutexNew(&vdb_apps_lock_attributes);

  /* creation of vdb_bps_front_lock */
  vdb_bps_front_lockHandle = osMutexNew(&vdb_bps_front_lock_attributes);

  /* creation of vdb_bps_rear_lock */
  vdb_bps_rear_lockHandle = osMutexNew(&vdb_bps_rear_lock_attributes);

  /* creation of vdb_inverter_lock */
  vdb_inverter_lockHandle = osMutexNew(&vdb_inverter_lock_attributes);

  /* creation of vdb_fsm_state_lock */
  vdb_fsm_state_lockHandle = osMutexNew(&vdb_fsm_state_lock_attributes);

  /* creation of vdb_powsup_lock */
  vdb_powsup_lockHandle = osMutexNew(&vdb_powsup_lock_attributes);

  /* creation of vdb_cooling_lock */
  vdb_cooling_lockHandle = osMutexNew(&vdb_cooling_lock_attributes);

  /* creation of vdb_dashboard_lock */
  vdb_dashboard_lockHandle = osMutexNew(&vdb_dashboard_lock_attributes);

  /* creation of vdb_torquectrl_lock */
  vdb_torquectrl_lockHandle = osMutexNew(&vdb_torquectrl_lock_attributes);

  /* creation of can_db_lock */
  can_db_lockHandle = osMutexNew(&can_db_lock_attributes);

  /* creation of faulttask_lock */
  vdb_faulttask_lockHandle = osMutexNew(&vdb_faulttask_lock_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  initCANDatabase();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of canDbTxQueue */
  canDbTxQueueHandle = osMessageQueueNew (16, 16, &canDbTxQueue_attributes);

  /* creation of canDbRxQueue */
  canDbRxQueueHandle = osMessageQueueNew (16, 16, &canDbRxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of fsmTask */
  fsmTaskHandle = osThreadNew(StartFsmTask, NULL, &fsmTask_attributes);

  /* creation of powsupTask */
  powsupTaskHandle = osThreadNew(StartPwrSupTask, (void*) &powsupArgs, &powsupTask_attributes);

  /* creation of driversensrTask */
  driversensrTaskHandle = osThreadNew(StartDriverSensorTask, (void*) &driversensorArgs, &driversensrTask_attributes);

  /* creation of faultTask */
  faultTaskHandle = osThreadNew(StartFaultTask, NULL, &faultTask_attributes);

  /* creation of canDbTask */
  canDbTaskHandle = osThreadNew(StartCanDbTask, (void*) &hcan1, &canDbTask_attributes);

  /* creation of coolingTask */
  coolingTaskHandle = osThreadNew(StartCoolingTask, NULL, &coolingTask_attributes);

  /* creation of dashboardTask */
  dashboardTaskHandle = osThreadNew(StartDashboardTask, NULL, &dashboardTask_attributes);

  /* creation of torquectrlTask */
  torquectrlTaskHandle = osThreadNew(StartTorqueCtrlTask, NULL, &torquectrlTask_attributes);

  /* creation of amkTask */
  amkTaskHandle = osThreadNew(StartAMKTask, NULL, &amkTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* creation of amkEventFlags */
  amkEventFlagsHandle = osEventFlagsNew(&amkEventFlags_attributes);

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
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

  int vcu_dbg_can = CANGetDbEntry(0x651);
  CANMessage_VCU_DEBUG_MESSAGE contents = {
		  .fields = {
				  .VCU_DEBUG_0 = 12300
		  }
  };

  // Wait for 5V rail to come up
  while (HAL_GPIO_ReadPin(RAIL_POWER_ENABLE_5V_GPIO_Port, RAIL_POWER_ENABLE_5V_Pin) == GPIO_PIN_RESET) {
    osDelay(1);
  }
  osDelay(10);

  CANQueueMessageToSend(vcu_dbg_can, contents.as_u64, &hcan2);
  AMKControllerEventFlags_t amk_flags = {.flagBits = {.start_motors=1}};
  osEventFlagsClear(amkEventFlagsHandle, (AMKControllerEventFlags_t){.flagBits = AMK_FLAGS_ALL}.flagInt);
  osEventFlagsSet(amkEventFlagsHandle, amk_flags.flagInt);

  /* Infinite loop */
  for(;;)
  {
    // Heart beat
    HAL_GPIO_TogglePin(GPIOC, DEBUG_LED_3_Pin);
    osDelay(50);
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
