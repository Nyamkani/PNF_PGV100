/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <Extinc/PNFPosSensor.h>
#include <Extinc/CommonSensor.h>
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "Extinc/api_init.h"
#include "Extinc/IntegratedSensorManager.h"

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
 CAN_HandleTypeDef hcan1;

NOR_HandleTypeDef hnor1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */

//extern bool Nyamkani::SensorManager::bDestroyed;
//extern Nyamkani::SensorManager* Nyamkani::SensorManager::pIns;

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
  //MX_GPIO_Init();
  //MX_CAN1_Init();
  //MX_FMC_Init();
  //MX_UART4_Init();
  //MX_USART1_UART_Init();
  //MX_USART3_UART_Init();
  //MX_USART6_UART_Init();
  //MX_UART5_Init();
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
	  Nyamkani::SensorManager::GetInstance().RegisterCommonSensor();
	  Nyamkani::SensorManager::GetInstance().RegisterPNFPosSensor();
	  Nyamkani::SensorManager::GetInstance().PNFPosSensorInitialize();

	  Nyamkani::SensorManager::GetInstance().CommonSensorsGetValue();
	  Nyamkani::SensorManager::GetInstance().PNFPosSensorGetValue();





	  //Nyamkani::SensorManager::GetInstance()->test();
    /* USER CODE END WHILE */
	//auto PGV100 = new Nyamkani::PNFPosSensor(1, 0, 5, 10.0, 10.0, 15.0);
	//if(PGV100->main_loop()==1) delete PGV100;
	//auto ORGSENSOR = new Nyamkani::CommonSensor(0x01, 1, 5, GPIOA, LL_GPIO_PIN_12);
	//ORGSENSOR->main_loop();
	//auto val = ORGSENSOR->GetSensorValue();
	//printf("%d", val);

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

