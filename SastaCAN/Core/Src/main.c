/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  */


/*
 ***************************
 *    Flow chart summary   *
 ***************************
 *
 * 1. Board powered
 * 2. TSMS bit will send to front board board
 * 3. Front board after receiving data sent data to rear board received using interrupt.
 * 4. After receiving message id is matched
 * 5. if id matched: flagAction takes placed()
 * 6. Data send to motor controller, brakeLight()
 * **************************
 * tsms value is received using GPIOinterrupt
 ****************************
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
CAN_HandleTypeDef hcan1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */
//uint8_t TxData[8];
//uint16_t RxData[4];
uint16_t id;
uint8_t analog;
uint32_t temp=0;

uint8_t tsms = 0;						//represent status of TS: false -> TS is off | true -> TS is on ||| feedback signal from RearECU
uint16_t APPS_range_err = 0;				//false -> in range | true -> out of range
uint16_t brk_range_err = 0;					//false -> in range | true -> out of range

uint16_t apps_avg = 0;
uint16_t brk_avg = 0;				//apps_avg -> APPS value | brk -> Brake value

//CAN vatiables
uint16_t front_rearid = 0x19;			//CAN message ID: front and rear comm.
uint8_t TxData[8];
uint16_t RxData[4];

uint16_t rtds_id = 0x10;
uint16_t rtds=0;

//Brakev vars
uint16_t brakelight_thresh = 1200;

uint32_t timestamp = 0;

//-------------------------------------------------------------------
uint16_t arr[4];
uint8_t brr[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
//void CANStart();
//void CANTransmit(uint32_t id,uint32_t IDE,uint32_t RTR,uint32_t DLC,uint8_t *data);
uint16_t flagsAction();
void transmitAPPSvalue();
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1);
void brakeLight();
void transmitAPPSvalue_DAC();
HAL_StatusTypeDef HAL_CAN_AddTxMessageType1(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint16_t aData[], uint8_t bData[], uint32_t *pTxMailbox);
void CANTransmit2(uint32_t id,uint32_t IDE,uint32_t RTR,uint32_t DLC,uint16_t *data1,uint16_t *data2);

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
  MX_TIM2_Init();
  MX_DAC_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

//  HAL_TIM_Base_Start(&htim2);
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//  //CANStart();
//
//  HAL_TIM_Base_Start(&htim4);
//  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
//
//  HAL_TIM_Base_Start(&htim9);
//  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
//
//  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
//  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
//
//  CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
//
	  TxData[1] = 123;
	  HAL_I2C_Slave_Transmit(&hi2c1, TxData, 8, 1000);
//	  arr[0] = 1235;
//	  arr[1] = 233;
//	  arr[2] = 0b111111011111;
//	  brr[0] = 34;
//	  brr[1] = 242;
//	  TxData[0] = 253;
	  //CAN_Transmit(&hcan1, front_rearid, 8, TxData);
	  //CANTransmit2(front_rearid, 0, 0, 8, arr, brr);
	  //CANTransmit(front_rearid, 0, 0, 8, TxData);
//	  transmitAPPSvalue();
//	  transmitAPPSvalue_DAC();
//	  brakeLight();
//	  HAL_Delay(10);
//	  //HAL_GPIO_TogglePin (GPIOG, GPIO_PIN_6);
//	  uint8_t DATA1[] = {1,4,3,2,1,12,5,4};
//
//	  //if(id==306)
//	 // CANTransmit((uint32_t)0x02, 0, 0, 8, DATA1);
//	  //HAL_Delay(500);
//	  //CANReceive();
//	  HAL_GPIO_TogglePin (GPIOG, GPIO_PIN_5);
//	  //HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 3000);
////	  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, temp);
////
////	  if (temp>360000) temp = 10000;
//	  //else temp = temp+2000;
//	  //HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_12);
//	  //HAL_GPIO_TogglePin (GPIOG, GPIO_PIN_5);
//
//	  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, analog/5*100);
//
//	  //HAL_Delay(5000);
//	  //HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 200);
//	  //transmitAPPSvalue_DAC();
//	  	  	  //Remove delay and switch to interrupts
//	  HAL_Delay(200);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = ENABLE;
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
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
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
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 15;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 99;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PF13 PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PG1 PG5 PG6 PG7
                           PG8 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
//void CANStart()
//{
//	  CAN_FilterTypeDef filter;
//	  filter.FilterActivation = ENABLE;
//	  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//
//	  // Allow two IDs per entry
//	  filter.FilterScale = CAN_FILTERSCALE_16BIT;
//	  filter.FilterMode = CAN_FILTERMODE_IDMASK;
//
//	  filter.FilterMaskIdHigh = 0x0000;
//	  filter.FilterMaskIdLow  = 0x0000;
//
//	  filter.FilterIdLow  = 0x0000;
//	  filter.FilterIdHigh = 0x0000;
//
//	  filter.FilterBank = 0;
//
//
//	  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
//		  Error_Handler();
//	  }
//
//	  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
//		  Error_Handler();
//	  }
//
//	  if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN) != HAL_OK){
//		  Error_Handler();
//	  }
//
//}
//
//void CANTransmit(uint32_t id,uint32_t IDE,uint32_t RTR,uint32_t DLC,uint8_t *data)
//{
//  uint32_t mailbox = 0x00U;
//  CAN_TxHeaderTypeDef Header;
//  Header.StdId= id;
//  Header.IDE= IDE;
//  Header.RTR= RTR;
//  Header.DLC= DLC;
//
//  if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)>0) {
//	  if(HAL_CAN_AddTxMessage(&hcan1, &Header, data , &mailbox) == HAL_OK){
//		  //printf("HALOK\n");
//		  HAL_GPIO_TogglePin (GPIOG, GPIO_PIN_6);
//		  HAL_GPIO_TogglePin (GPIOF, GPIO_PIN_14);
//		  while(HAL_CAN_IsTxMessagePending(&hcan1,mailbox));
//	  }
//	  else {
//		  //printf("HALERROR\n");
//	  }
//
//  }
//  else
//  	  HAL_GPIO_TogglePin (GPIOG, GPIO_PIN_5);
//}
//
//
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) {
//	CAN_RxHeaderTypeDef Header;
//	//if(hcan -> Instance == CAN1)
//	if(HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &Header, RxData)==HAL_OK) {
//		  HAL_GPIO_TogglePin (GPIOG, GPIO_PIN_7);
//		  HAL_GPIO_TogglePin (GPIOF, GPIO_PIN_14);
//	  }
//}
//
///*
//void CANReceive()
//{
//  CAN_RxHeaderTypeDef Header;
//  if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Header, RxData)==HAL_OK) {
//	  id = Header.StdId;
//	  HAL_GPIO_TogglePin (GPIOG, GPIO_PIN_6);
//	  HAL_GPIO_TogglePin (GPIOF, GPIO_PIN_14);
//  }
//}
//
//*/
//
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//  /* Prevent unused argument(s) compilation warning */
//  UNUSED(GPIO_Pin);
//  /* NOTE: This function Should not be modified, when the callback is needed,
//           the HAL_GPIO_EXTI_Callback could be implemented in the user file
//   */
//
//  //---------------- TSMS received using GPIO interrupt --------------------
//  if(GPIO_Pin == GPIO_PIN_11)
//  {
//	  HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_6);
//	  //HALDelay(10);
//  }
//  //------------------------------------------------------------------------
//}
//
//void transmitAPPSvalue_DAC(){
//	temp=3000/3300 *(1<<12);
//	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 200);
//}

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


	  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
		  Error_Handler();
	  }

	  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		  Error_Handler();
	  }


	  if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN) != HAL_OK){
		  Error_Handler();
	  }

}

void CANTransmit(uint32_t id,uint32_t IDE,uint32_t RTR,uint32_t DLC,uint8_t *data)
{
  uint32_t mailbox = 0x00U;
  CAN_TxHeaderTypeDef Header;
  Header.StdId= id;
  Header.IDE= IDE;
  Header.RTR= RTR;
  Header.DLC= DLC;

  if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)>0) {
	  HAL_CAN_AddTxMessage(&hcan1, &Header, data , &mailbox);
	  HAL_GPIO_TogglePin (GPIOG, GPIO_PIN_8);
  }
  else HAL_GPIO_TogglePin (GPIOG, GPIO_PIN_7);
}

void CANTransmit2(uint32_t id,uint32_t IDE,uint32_t RTR,uint32_t DLC,uint16_t *data1,uint16_t *data2)
{
  uint32_t mailbox = 0x00U;
  CAN_TxHeaderTypeDef Header;
  Header.StdId= id;
  Header.IDE= IDE;
  Header.RTR= RTR;
  Header.DLC= DLC;

  if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)>0) {
	  HAL_CAN_AddTxMessageType1(&hcan1, &Header, data1, data2, &mailbox);
	  HAL_GPIO_TogglePin (GPIOG, GPIO_PIN_8);
  }
  else HAL_GPIO_TogglePin (GPIOG, GPIO_PIN_7);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) {
	CAN_RxHeaderTypeDef Header;
	if(HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &Header, (uint8_t *)RxData)==HAL_OK) {
		timestamp = Header.Timestamp + HAL_GetTick();
		if(Header.StdId==0x14){
			  HAL_GPIO_TogglePin (GPIOG, GPIO_PIN_8);
			  apps_avg = RxData[0];
			  brk_avg = RxData[1];
			  APPS_range_err = RxData[2];
			  brk_range_err = RxData[3];
			  //flagsAction();
		}
		if(Header.StdId==0x10){
			  rtds=RxData[0];
			  if(rtds==1) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
		}
	  }
	  else {
		  HAL_GPIO_TogglePin (GPIOG, GPIO_PIN_7);
	  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */

  //---------------- TSMS received using GPIO interrupt --------------------
  if(GPIO_Pin == GPIO_PIN_7)
  {
	  if(!tsms) tsms = 1;
	  else tsms = 0;
	  HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_7);
  }
  //------------------------------------------------------------------------
}

//uint16_t flagsAction(){
//	/*
//	 ******************
//	 *  Flags Action  *
//	 ******************
//	 *  |          flag            |      Action      |
//	 *  | apps_range | brake_range | ShutDown Circuit |
//	 *  |     0      |      0      |         0        |
//	 *  |     0      |      1      |         0        |
//	 *  |     1      |      0      |         1        |
//	 *  |     1      |      1      |         1        |
//	 *
//	 * *****************
//	 */
//
//	if (APPS_range_err == 1){
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
//		return 0;
//	}
//	else{
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
//		return 1;
//	}
//}

void brakeLight(){
	if (brk_avg>brakelight_thresh)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, 1);
	else
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, 0);
}

void transmitAPPSvalue(){
	uint16_t analog_per = (apps_avg * 100)/((1<<12)-1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, analog_per);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, analog_per);
}

void transmitAPPSvalue_DAC(){;
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, apps_avg);
}

HAL_StatusTypeDef HAL_CAN_AddTxMessageType1(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint16_t aData[], uint8_t bData[], uint32_t *pTxMailbox)
{
  /*adata max length = 4 (12*4 = 48 bit)*/
  /*bdata max length = 2 (8**2 = 16 bit)*/

  /* Data in Regeister */
  /*
   *  TDHR --- |  12bit  |  12bit  |  8bit  |
   *  TDLR --- |  12bit  |  12bit  |  8bit  |
   */


  uint32_t transmitmailbox;
  HAL_CAN_StateTypeDef state = hcan->State;
  uint32_t tsr = READ_REG(hcan->Instance->TSR);

  /* Check the parameters */
  assert_param(IS_CAN_IDTYPE(pHeader->IDE));
  assert_param(IS_CAN_RTR(pHeader->RTR));
  assert_param(IS_CAN_DLC1(pHeader->DLC1));
  assert_param(IS_CAN_DLC2(pHeader->DLC2));
  if (pHeader->IDE == CAN_ID_STD)
  {
    assert_param(IS_CAN_STDID(pHeader->StdId));
  }
  else
  {
    assert_param(IS_CAN_EXTID(pHeader->ExtId));
  }
  assert_param(IS_FUNCTIONAL_STATE(pHeader->TransmitGlobalTime));

  if ((state == HAL_CAN_STATE_READY) ||
      (state == HAL_CAN_STATE_LISTENING))
  {
    /* Check that all the Tx mailboxes are not full */
    if (((tsr & CAN_TSR_TME0) != 0U) ||
        ((tsr & CAN_TSR_TME1) != 0U) ||
        ((tsr & CAN_TSR_TME2) != 0U))
    {
      /* Select an empty transmit mailbox */
      transmitmailbox = (tsr & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;

      /* Check transmit mailbox value */
      if (transmitmailbox > 2U)
      {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_INTERNAL;

        return HAL_ERROR;
      }

      /* Store the Tx mailbox */
      *pTxMailbox = (uint32_t)1 << transmitmailbox;

      /* Set up the Id */
      if (pHeader->IDE == CAN_ID_STD)
      {
        hcan->Instance->sTxMailBox[transmitmailbox].TIR = ((pHeader->StdId << CAN_TI0R_STID_Pos) |
                                                           pHeader->RTR);
      }
      else
      {
        hcan->Instance->sTxMailBox[transmitmailbox].TIR = ((pHeader->ExtId << CAN_TI0R_EXID_Pos) |
                                                           pHeader->IDE |
                                                           pHeader->RTR);
      }

      /* Set up the DLC */
      hcan->Instance->sTxMailBox[transmitmailbox].TDTR = (pHeader->DLC);

      /* Set up the Transmit Global Time mode */
      if (pHeader->TransmitGlobalTime == ENABLE)
      {
        SET_BIT(hcan->Instance->sTxMailBox[transmitmailbox].TDTR, CAN_TDT0R_TGT);
      }

      /* Set up the data field */
      WRITE_REG(hcan->Instance->sTxMailBox[transmitmailbox].TDHR, ((uint32_t)aData[3] << 20) | ((uint32_t)aData[2] << 8) | ((uint32_t)bData[1] << 0));
      WRITE_REG(hcan->Instance->sTxMailBox[transmitmailbox].TDLR, ((uint32_t)aData[1] << 20) | ((uint32_t)aData[0] << 8) | ((uint32_t)bData[0] << 0));

      /* Request transmission */
      SET_BIT(hcan->Instance->sTxMailBox[transmitmailbox].TIR, CAN_TI0R_TXRQ);

      /* Return function status */
      return HAL_OK;
    }
    else
    {
      /* Update error code */
      hcan->ErrorCode |= HAL_CAN_ERROR_PARAM;

      return HAL_ERROR;
    }
  }
  else
  {
    /* Update error code */
    hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;

    return HAL_ERROR;
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
