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
#include "main.h"
#include "pdm2pcm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PDM_BUFFER_SIZE 128
#define PCM_BUFFER_SIZE 256

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

//DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;

SAI_HandleTypeDef hsai_BlockB1;
SAI_HandleTypeDef hsai_BlockA4;
DMA_HandleTypeDef hdma_sai1_b;
DMA_HandleTypeDef hdma_sai4_a;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_BDMA_Init(void);
static void MX_SAI4_Init(void);
static void MX_CRC_Init(void);
static void MX_DMA_Init(void);
//static void MX_DFSDM1_Init(void);
static void MX_SAI1_Init(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  	MX_GPIO_Init();
//  MX_CRC_Init();

//  MX_SAI4_Init();
  //MX_SAI1_Init();
//  MX_BDMA_Init();
  //MX_DMA_Init();

  //MX_PDM2PCM_Init();
  //MX_DFSDM1_Init();

  /* USER CODE BEGIN 2 */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
  BSP_LED_Off(LED_GREEN);
  BSP_LED_Off(LED_RED);

  BSP_AUDIO_Init_t haudio_in;
  haudio_in.Device = AUDIO_IN_DEVICE_DIGITAL_MIC1;
  haudio_in.ChannelsNbr = 1;
  haudio_in.SampleRate = AUDIO_FREQUENCY_16K;
  haudio_in.BitsPerSample = AUDIO_RESOLUTION_8B;
  haudio_in.Volume = 50;
  int32_t in_init_status = BSP_AUDIO_IN_Init(PDM, &haudio_in);
  int32_t convert_status = BSP_AUDIO_IN_PDMToPCM_Init(PDM, SAI_AUDIO_FREQUENCY_16K, 1, 1);

  uint8_t mic_buffer[PDM_BUFFER_SIZE] = {0};
  // uint16_t speaker_buffer[PCM_BUFFER_SIZE] = {0};
  // @param  Instance  Audio IN instance: 0 for SAI, 1 for SAI PDM and 2 for DFSDM
  uint8_t record_status = BSP_AUDIO_IN_RecordPDM(PDM, mic_buffer, PDM_BUFFER_SIZE);
  //int32_t status_convert = BSP_AUDIO_IN_PDMToPCM(PDM, mic_buffer, speaker_buffer);
  /* USER CODE END 2 */

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 125;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_CRC_DR_RESET(&hcrc);
  /* USER CODE BEGIN CRC_Init 2 */

  __HAL_RCC_CRC_CLK_ENABLE();
  HAL_CRC_MspInit(&hcrc);

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_DFSDM1_Init(void)
//{
//
//  /* USER CODE BEGIN DFSDM1_Init 0 */
//////
//  /* USER CODE END DFSDM1_Init 0 */
//
//  /* USER CODE BEGIN DFSDM1_Init 1 */
//////
//  /* USER CODE END DFSDM1_Init 1 */
//  hdfsdm1_channel0.Instance = DFSDM1_Channel0;
//  hdfsdm1_channel0.Init.OutputClock.Activation = DISABLE;
//  hdfsdm1_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
//  hdfsdm1_channel0.Init.OutputClock.Divider = 2;
//  hdfsdm1_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_INTERNAL_REGISTER;
//  hdfsdm1_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
//  hdfsdm1_channel0.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
//  hdfsdm1_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
//  hdfsdm1_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
//  hdfsdm1_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
//  hdfsdm1_channel0.Init.Awd.Oversampling = 1;
//  hdfsdm1_channel0.Init.Offset = 0x00;
//  hdfsdm1_channel0.Init.RightBitShift = 0x00;
//  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN DFSDM1_Init 2 */
//////
//  /* USER CODE END DFSDM1_Init 2 */
//
//}
//
///**
//  * @brief SAI1 Initialization Function
//  * @param None
//  * @retval None
//  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockB1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockB1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SAI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI4_Init(void)
{

  /* USER CODE BEGIN SAI4_Init 0 */

  /* USER CODE END SAI4_Init 0 */

  /* USER CODE BEGIN SAI4_Init 1 */

  /* USER CODE END SAI4_Init 1 */

  /* USER CODE BEGIN SAI4_Init 2 */
  haudio_in_sai[PDM].Instance = AUDIO_IN_SAI_PDMx;
  haudio_in_sai[PDM].Init.Protocol = SAI_FREE_PROTOCOL;
  haudio_in_sai[PDM].Init.AudioMode = SAI_MODEMASTER_RX;
  haudio_in_sai[PDM].Init.DataSize = SAI_DATASIZE_16;
  haudio_in_sai[PDM].Init.FirstBit = SAI_FIRSTBIT_MSB;
  haudio_in_sai[PDM].Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  haudio_in_sai[PDM].Init.Synchro = SAI_ASYNCHRONOUS;
  haudio_in_sai[PDM].Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  haudio_in_sai[PDM].Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  haudio_in_sai[PDM].Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  haudio_in_sai[PDM].Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  haudio_in_sai[PDM].Init.MonoStereoMode = SAI_STEREOMODE;
  haudio_in_sai[PDM].Init.CompandingMode = SAI_NOCOMPANDING;

  haudio_in_sai[PDM].FrameInit.FrameLength = 16;
  haudio_in_sai[PDM].FrameInit.ActiveFrameLength = 1;
  haudio_in_sai[PDM].FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  haudio_in_sai[PDM].FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  haudio_in_sai[PDM].FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  haudio_in_sai[PDM].SlotInit.FirstBitOffset = 0;
  haudio_in_sai[PDM].SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  haudio_in_sai[PDM].SlotInit.SlotNumber = 0;
  haudio_in_sai[PDM].SlotInit.SlotActive = 0x0000FFFF;
  haudio_in_sai[PDM].Init.PdmInit.Activation = ENABLE;
  haudio_in_sai[PDM].Init.PdmInit.MicPairsNbr = 1;
  haudio_in_sai[PDM].Init.PdmInit.ClockEnable = SAI_PDM_CLOCK2_ENABLE;
  if (HAL_SAI_Init(&haudio_in_sai[PDM]) != HAL_OK)
  {
    Error_Handler();
  }


}

/**
  * Enable DMA controller clock
  */
static void MX_BDMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_BDMA_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMAMUX2_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX2_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX2_OVR_IRQn);
  /* BDMA_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BDMA_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(BDMA_Channel1_IRQn);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX1_OVR_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Register Bus IOs if component ID is OK
  * @retval error status
  */


/**
  * @brief  Playback initialization
  * @param  None
  * @retval None
  */

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

