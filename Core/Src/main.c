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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f446xx.h"
#include "Driver_1553.h"
#include "LCD_driver.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define count 10000


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
MIL_Error *MIL_Error_;
MIL_Handle_struc *mil_handle_struct;
MIL_Command_Word *mil_command_word;
MIL_Data_Word *mil_data_word;
MIL_Response_Word *mil_response_word;
MIL_Config_Device *mil_config_device;

uint8_t virtTact = 1;
uint16_t encodeTimerCnt = 1;
uint8_t TEST[32] = {
		0b01010111, 0b11111110, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10
};
uint8_t change_chenel = 0;

uint8_t RX_data_SPI[40];
bool flag;
bool flag123 = 0;
uint32_t counter_bit_capture = 0;
uint32_t num_byte = 0;
uint8_t logic_decode_start = 0;

uint8_t* array_receive1 = (uint32_t*)0x2001C000UL;
uint32_t* size_mss;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MIL_Init_Struct()
{
	uint32_t *mil_command_word_ptr;
	uint32_t *mil_data_word_ptr;
	uint32_t *mil_response_word_ptr;
	uint32_t *mil_config_device_ptr;
	mil_handle_struct = malloc(sizeof(MIL_Handle_struc));
	mil_command_word = malloc(sizeof(MIL_Command_Word));
	mil_data_word = malloc(sizeof(MIL_Data_Word));
	mil_response_word = malloc(sizeof(MIL_Response_Word));
	mil_config_device = malloc(sizeof(MIL_Config_Device));

	mil_command_word_ptr =(uint32_t*)mil_command_word;
	mil_data_word_ptr = (uint32_t*)mil_data_word;
	mil_response_word_ptr = (uint32_t*)mil_response_word;
	mil_config_device_ptr = (uint32_t*)mil_config_device;

	MIL_Error_ =  malloc(sizeof(MIL_Error));

	MIL_Error_->Message = 0;
	MIL_Error_->Error = 0;

	/*init MIL_Handle_struc Begin*/
	for( uint8_t i = 0; i < 20 ; i++)
	{
		mil_handle_struct->MIL_RegDef_struct.command_word[i] = 0;
	}

	for( uint8_t i = 0; i < 40 ; i++)
	{
		mil_handle_struct->command_word_manch[i] = 0;
	}

	for( uint8_t i = 0; i < 170 ; i++)
	{
		mil_handle_struct->array_receive_bit[i] = 0;
	}

	for(uint16_t i = 0; i < 1300 ; i++)
	{
		mil_handle_struct->array_receive_byte[i] = 0;
	}

	for(uint8_t t = 0; t < 31 ; t++)
	{
		for(uint8_t i = 0; i < 17 ; i++)
		{
			mil_handle_struct->decoder_array[t][i] = 0;
		}
	}

	for( uint8_t i = 0; i < 31 ; i++)
	{
		mil_handle_struct->data_complit[i] = 0;
	}

	for(uint8_t t = 0; t < 31 ; t++)
	{
		for(uint8_t i = 0; i < 20 ; i++)
		{
			mil_handle_struct->coder_array[t][i] = 0;
		}
	}


	mil_handle_struct->cnt_bit = 0;
	mil_handle_struct->data_word = 0;



	/*init MIL_Handle_struc End */

	/*init MIL_Command_Word Begin*/

	for( uint8_t i = 0; i < 20 ; i++)
	{
		*(mil_command_word_ptr + i) = 0;
	}

	/*init MIL_Command_Word End*/

	/*init MIL_Data_Word Begin*/

	for( uint8_t i = 0; i < 20 ; i++)
	{
		*(mil_data_word_ptr + i) = 0;
	}

	/*init MIL_Data_Word End*/

	/*init MIL_Response_Word Begin*/

	for( uint8_t i = 0; i < 20 ; i++)
	{
		*(mil_response_word_ptr + i) = 0;
	}

	/*init MIL_Response_Word End*/

//	switch(type_device) {

//	case (FINAL_DEVICE):

//	}



	 mil_handle_struct->flag_sync_c_RX = false;
	 mil_handle_struct->flag_sync_c_TX = false;
	 mil_handle_struct->flag_sync_d_RX = false;
	 mil_handle_struct->flag_sync_d_TX = false;

	 mil_command_word->addr_rt[0] = 0b0;
	 mil_command_word->addr_rt[1] = 0b0;
	 mil_command_word->addr_rt[2] = 0b0;
	 mil_command_word->addr_rt[3] = 0b1;
	 mil_command_word->addr_rt[4] = 0b0;
	 mil_command_word->wr = 1;
	 mil_command_word->subaddr_cl[0] = 0b0;
	 mil_command_word->subaddr_cl[1] = 0b0;
	 mil_command_word->subaddr_cl[2] = 0b1;
	 mil_command_word->subaddr_cl[3] = 0b1;
	 mil_command_word->subaddr_cl[4] = 0b1;
	 mil_command_word->n_com[0] = 0b0;
	 mil_command_word->n_com[1] = 0b1;
	 mil_command_word->n_com[2] = 0b1;
	 mil_command_word->n_com[3] = 0b1;
	 mil_command_word->n_com[4] = 0b0;

}
void MIL_ChangeMassege(MIL_Handle_struc *MIL_Addr, const uint8_t *pData)
{
	for(uint8_t i = 0; i < 20; i++)
	{
		MIL_Addr->MIL_RegDef_struct.command_word[i] = *(pData + i);
	}

	MIL_Addr->flag_sync_c_TX = true;
//	ParityBit(MIL_Addr);
//	ManchesterEncode(MIL_Addr);

}
void SPI_SendData(SPI_HandleTypeDef* hspi2,uint8_t *pTxBuffer, uint32_t Len)
{
	SPI2->CR1 |= SPI_CR1_SPE;
	while(Len > 0)
	{
		//1. wait until TXE is set
		while(!(SPI2->SR & SPI_SR_TXE) )
		{

		}
			//8 bit DFF
		    SPI2->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		while(SPI2->SR & SPI_SR_BSY);
	}

}
void SPI2_DMA_Transmit__(uint32_t size)
{
    /* Clear pending flag */
    DMA1->HIFCR = DMA_HIFCR_CTCIF4;

    DMA1_Stream4->CR |= (1 << 4);

    /* Set Peripheral Address to be SPI2->DR */
    DMA1_Stream4->PAR = (uint32_t)&SPI2->DR;

    /* Set Memory Address */
    DMA1_Stream4->M0AR = (uint32_t)&mil_handle_struct->data_complit;
//    DMA1_Stream4->M0AR = (uint32_t)&mil_handle_struct->array_receive[0];
//    DMA1_Stream4->M0AR = (uint32_t)&mil_handle_struct->array_receive[1];
    /* Set Number of Data to Transmit */
    DMA1_Stream4->NDTR = size;

//    DMA1_Stream4->CR &= ~(1 << 8);
    /* Configure the DMA Stream */
//    DMA1_Stream4->CR |= (1 << 18);
//    DMA1_Stream4->CR |= (1 << 0);
    /* Enable SPI2 DMA Request */
//    SPI2->CR2 |= SPI_CR2_TXDMAEN;

    SPI2->CR1 |= (1 << 6);

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
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  MIL_Init_Struct();

  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);

  // Заполняем массив значениями 0 или 1


  MIL_Handlig_Tx_data(mil_handle_struct, mil_command_word ,TEST,0);
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MIL_Transmit(mil_handle_struct);
	  for(uint32_t i = 0; i < 40000; i ++)
	  {
			while( !(TIM2->SR & TIM_SR_UIF) );
			TIM2->SR = 0;
	  }
	  /*
	  for(uint32_t i = 0; i < 39;i++)
	  {
		  printf(" %u", mil_handle_struct->array_receive[0][i]);
	  }
	  printf("\n");
	  for(uint32_t i = 0; i < 39;i++)
	  {
		  printf(" %u", mil_handle_struct->array_receive[1][i]);
	  }
	  printf("\n\n");
	  HAL_Delay(500);
*/
//	  SPI_SendData(&hspi2, init_values, 17);
    /*
	  lcd_display_clear();
	  lcd_send_command(LCD_CMD_DIS_RETURN_HOME);

	  for(uint8_t i = 0; i < 20 ;i++ )
	  {
		  lcd_print_char(mil_handle_struct->MIL_RegDef_struct.command_word[i]);
		  if(i == 15)
		  {
			  lcd_send_command(LCD_CMD_CURS_SECLINE);

		  }
	  }
	  lcd_send_command(LCD_CMD_INCADD);
	  lcd_send_command(LCD_CMD_INCADD);
	  lcd_print_char(RXdata);

	  HAL_Delay(500);
	  */
	  /*
	  if(!(GPIOC->IDR & (1 << 13)))
	  {
		  HAL_Delay(300);
		  flag ^= 1;
      }
	  if(flag)
	  {
		  MIL_ChangeMassege(mil_handle_struct,(uint8_t*)signal_changes_2);
	  }else{
    	  MIL_ChangeMassege(mil_handle_struct,(uint8_t*)signal_changes_1);
      }
	  MIL_Transmit(mil_handle_struct);
	  HAL_Delay(200);
*/


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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 89;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 44;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_10
                          |GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC6 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
	printf("Assertion failed: file %s on line %lu\n", file, line);
	  __disable_irq();
	  while (1)
	  {
	  }
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
