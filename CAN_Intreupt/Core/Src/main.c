/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
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
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
uint8_t DATA[8]={0};
uint32_t id=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void CANStart();
void CANTransmit(uint32_t,uint32_t,uint32_t,uint32_t,uint8_t*);
void CANReceive(void);
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
  MX_CAN_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  CANStart();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //uint8_t DATA[] = {5,4,3,2,1};
	  //CANTransmit((uint32_t)0x01, 0, 0, 5, DATA);

	 //CANReceive();
	 //HAL_Delay(100);
	 HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_12);
	//uint8_t DATA1[] = {5,4,3,2,1,12,5,4};
	// CANTransmit((uint32_t)0x02, 0, 0, 8, DATA1);
	 HAL_Delay(500);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* CEC_CAN_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CEC_CAN_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void CANStart()
{
  CAN_FilterTypeDef filter;
  filter.FilterActivation = ENABLE;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;

  // Allow two IDs per entry
  filter.FilterScale = CAN_FILTERSCALE_16BIT;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;

  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow  = 0x0000;

  filter.FilterIdLow  = 0x0000;
  filter.FilterIdHigh = 0x0000;

  filter.FilterBank = 0;

  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
	  Error_Handler();
  }

 HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN);

  if (HAL_CAN_Start(&hcan) != HAL_OK) {
	  Error_Handler();
  }
}

void CANTransmit(uint32_t id,uint32_t IDE,uint32_t RTR,uint32_t DLC,uint8_t *data)
{
  uint32_t mailbox= 0x00U;
  CAN_TxHeaderTypeDef Header;
  Header.StdId= id;
  Header.IDE= IDE;
  Header.RTR= RTR;
  Header.DLC= DLC;

  if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan)>0) {
	  if(HAL_CAN_AddTxMessage(&hcan, &Header, data , &mailbox)==HAL_OK){
		  printf("HALOK\n");

		  while(HAL_CAN_IsTxMessagePending(&hcan,mailbox));
		if(hcan.Instance->TSR & CAN_TSR_TXOK0) {
			HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_13);
		}
		  else if(hcan.Instance->TSR & CAN_TSR_ALST0) {
			  HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_14);
		  }

	  }
	  else {
		  printf("HALERROR\n");
	  }
  }
  else
  	  HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_12);
}

void CANReceive()
{
  CAN_RxHeaderTypeDef Header;
  if(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0){
	  HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_12);
	  if(HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &Header, DATA)==HAL_OK) {
	  	  id = Header.StdId;
	    }
  }


  else{
	  HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_13);
   }

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef Header;
  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Header, DATA)==HAL_OK) {
	  id = Header.StdId;
	  HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_12);
  }
  else {
	  HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_13);
  }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
	if(hcan->Instance->TSR & CAN_TSR_TXOK0) {
		  HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_14);
	  }
	  else if(hcan->Instance->TSR & CAN_TSR_ALST0) {
		  HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_13);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
