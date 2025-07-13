/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

	#define OFF			0
	#define ON			1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */

	uint8_t		recv_dat[14];
	uint8_t		recv_flg;
	uint8_t		bz_flg;

	uint8_t		mode;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

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

	mode = 0;
	bz_flg = OFF;
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

	// Led Init
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);			// Green Led-1 ON
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);			// Green Led-2 ON
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);			// Green Led-3 ON

	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);				// Red Led-1 OFF
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);				// Red Led-2 OFF
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);				// Red Led-3 OFF

	HAL_UART_Receive_DMA(&huart2,recv_dat,7);								// Pelco Cmd

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		if( HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) == GPIO_PIN_RESET )
		{
			bz_flg = OFF;
		}
		
		if( bz_flg == ON )
		{
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);										// Bz ON
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);			// Bz OFF			
		}
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Ch1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch1_IRQn);
  /* DMA1_Ch2_3_DMA2_Ch1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch2_3_DMA2_Ch1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch2_3_DMA2_Ch1_2_IRQn);
  /* DMA1_Ch4_7_DMA2_Ch3_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_7_DMA2_Ch3_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch3_5_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC6 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 PB3 PB4
                           PB5 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	recv_flg = ON;
}

void Main_1ms(void)
{
	if( recv_flg == ON )
	{
		recv_flg = OFF;
		
		HAL_UART_Transmit_DMA(&huart2,recv_dat,2);
		
		switch(recv_dat[1])
		{
			case 0x30:
			case 0x31:
			case 0x32:
			case 0x33:
			case 0x34:
			case 0x35:
			case 0x36:
			case 0x37:
			case 0x38:
			case 0x39:
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); 
				mode = recv_dat[1] - 0x30;																	//
				break;
			default:
				mode = 99;
				break;
		}
		
		if( recv_dat[2] & 0x01 )
		{
			bz_flg = ON;
		}
		else
		{
			bz_flg = OFF;			
		}
	
		switch(mode)
		{
			case 0x00:
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);			// Green Led-1 ON
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);			// Green Led-2 ON
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);			// Green Led-3 ON

				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);				// Red Led-1 OFF
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);				// Red Led-2 OFF
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);				// Red Led-3 OFF
				break;
			case 0x01:
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);				// Green Led-1 OFF
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);			// Green Led-2 ON
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);			// Green Led-3 ON

				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);			// Red Led-1 ON
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);				// Red Led-2 OFF
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);				// Red Led-3 OFF
				break;
			case 0x02:
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);			// Green Led-1 ON
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);				// Green Led-2 OFF
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);			// Green Led-3 ON

				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);				// Red Led-1 OFF
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);			// Red Led-2 ON
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);				// Red Led-3 OFF
				break;
			case 0x03:
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);				// Green Led-1 OFF
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);				// Green Led-2 OFF
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);			// Green Led-3 ON

				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);			// Red Led-1 ON
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);			// Red Led-2 ON
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);				// Red Led-3 OFF
				break;
			case 0x04:
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);			// Green Led-1 ON
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);			// Green Led-2 ON
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);				// Green Led-3 OFF

				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);				// Red Led-1 OFF
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);				// Red Led-2 OFF
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);			// Red Led-3 ON
				break;
			case 0x05:
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);				// Green Led-1 OFF
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);			// Green Led-2 ON
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);				// Green Led-3 OFF

				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);			// Red Led-1 ON
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);				// Red Led-2 OFF
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);			// Red Led-3 ON
				break;
			case 0x06:
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);			// Green Led-1 ON
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);				// Green Led-2 OFF
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);				// Green Led-3 OFF

				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);				// Red Led-1 OFF
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);			// Red Led-2 ON
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);			// Red Led-3 ON
				break;
			case 0x07:
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);				// Green Led-1 OFF
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);				// Green Led-2 OFF
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);				// Green Led-3 OFF

				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);			// Red Led-1 ON
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);			// Red Led-2 ON
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);			// Red Led-3 ON
				break;
			
			case 0x08:
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);			// Red Led-1 ON
				break;
			case 0x09:
				break;
			default:
				break;
		}

		HAL_UART_Receive_DMA(&huart2,recv_dat,7);									//
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
