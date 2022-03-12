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
#include "../Drivers/BSP/STM32H735G-DK/stm32h735g_discovery.h"
#include "../Drivers/BSP/Components/wm8994/wm8994.h"
#include "../Drivers/BSP/STM32H735G-DK/stm32h735g_discovery_audio.h"
#include "../Drivers/BSP/STM32H735G-DK/stm32h735g_discovery_bus.h"
#include "../Drivers/BSP/Components/Common/audio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BYTES_IN_AUDIO_WAV  NUM_BYTES
#define AUDIO_BUFFER_SIZE	NUM_BYTES
#if defined ( __ICCARM__ )
#pragma location=0x38000000
uint8_t  input_buffer[AUDIO_BUFFER_SIZE];
#elif defined ( __CC_ARM )
__attribute__((at(0x38000000))) uint8_t input_buffer[AUDIO_BUFFER_SIZE];
#elif defined ( __GNUC__ )
uint16_t __attribute__((section(".RAM_D3")))  input_buffer[AUDIO_BUFFER_SIZE];
#endif

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define	NUM_BYTES  8192
//#define PLAY_BUFF_SIZE 1024
#define SRAM4_BASE 0x38000000
#define SRAM2_BASE 0x30004000
#define CODEC_I2C  0x34U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

//DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;

I2C_HandleTypeDef hi2c4;

RAMECC_HandleTypeDef hramecc2_m1;
RAMECC_HandleTypeDef hramecc2_m2;
RAMECC_HandleTypeDef hramecc3_m1;

SAI_HandleTypeDef hsai_BlockB1;
SAI_HandleTypeDef hsai_BlockA4;
DMA_HandleTypeDef hdma_sai1_b;
DMA_HandleTypeDef hdma_sai4_a;

DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
/* USER CODE BEGIN PV */
extern PDM_Filter_Handler_t PDM1_filter_handler;
//
///* CODEC INIT */
static AUDIO_Drv_t *AudioDrv = NULL;
void *AudioCompObj;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_BDMA_Init(void);
static void MX_SAI4_Init(void);
static void MX_CRC_Init(void);
static void MX_DMA_Init(void);
static void MX_SAI1_Init(void);
static void MX_RAMECC_Init(void);
//static void MX_I2C4_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
static void Playback_Init(void);
static int32_t WM8994_Probe(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint8_t *input_buffer = (uint8_t*)SRAM4_BASE;
//uint8_t *pdm_buffer = (uint8_t*)SRAM2_BASE;
//uint8_t input_buffer[AUDIO_BUFFER_SIZE];
uint8_t pdm_buffer[AUDIO_BUFFER_SIZE] = {0};
uint16_t audio_wav[AUDIO_BUFFER_SIZE] = {0};


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Off(LED1);
  BSP_LED_Off(LED2);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_BDMA_Init();
  MX_SAI4_Init();
  MX_CRC_Init();
  MX_PDM2PCM_Init();
  MX_DMA_Init();
  MX_SAI1_Init();
  MX_RAMECC_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream0, input_buffer, pdm_buffer, NUM_BYTES);
  HAL_SAI_Receive_DMA(&hsai_BlockA4, input_buffer, NUM_BYTES);
//  while(!hsai_BlockA4.Ack) {}
  PDM_Filter(pdm_buffer, audio_wav, &PDM1_filter_handler);
  AudioPlay_demo();

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
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /*!< Supply configuration update enable */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 104;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }

/* Select PLL as system clock source and configure  bus clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | \
                                 RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }

/*
  Note : The activation of the I/O Compensation Cell is recommended with communication  interfaces
          (GPIO, SPI, FMC, OSPI ...)  when  operating at  high frequencies(please refer to product datasheet)
          The I/O Compensation Cell activation  procedure requires :
        - The activation of the CSI clock
        - The activation of the SYSCFG clock
        - Enabling the I/O Compensation Cell : setting bit[0] of register SYSCFG_CCCSR
*/


  __HAL_RCC_CSI_ENABLE() ;

  __HAL_RCC_SYSCFG_CLK_ENABLE() ;

  HAL_EnableCompensationCell();

}

//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Supply configuration update enable
//  */
//  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//  RCC_OscInitStruct.PLL.PLLM = 4;
//  RCC_OscInitStruct.PLL.PLLN = 24;
//  RCC_OscInitStruct.PLL.PLLP = 1;
//  RCC_OscInitStruct.PLL.PLLQ = 125;
//  RCC_OscInitStruct.PLL.PLLR = 2;
//  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
//  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
//  RCC_OscInitStruct.PLL.PLLFRACN = 0;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
//                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
//  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* BDMA_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BDMA_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(BDMA_Channel1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* DMAMUX2_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX2_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX2_OVR_IRQn);
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

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_I2C4_Init(void)
//{
//
//  /* USER CODE BEGIN I2C4_Init 0 */
//
//  /* USER CODE END I2C4_Init 0 */
//
//  /* USER CODE BEGIN I2C4_Init 1 */
//
//  /* USER CODE END I2C4_Init 1 */
//  hi2c4.Instance = I2C4;
//  hi2c4.Init.Timing = 0x10B0DCFB;
//  hi2c4.Init.OwnAddress1 = 104;
//  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//  hi2c4.Init.OwnAddress2 = 0;
//  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
//  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Analogue filter
//  */
//  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Digital filter
//  */
//  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN I2C4_Init 2 */
//
//  /* USER CODE END I2C4_Init 2 */
//
//}
//
///**
//  * @brief RAMECC Initialization Function
//  * @param None
//  * @retval None
//  */
static void MX_RAMECC_Init(void)
{

  /* USER CODE BEGIN RAMECC_Init 0 */

  /* USER CODE END RAMECC_Init 0 */

  /* USER CODE BEGIN RAMECC_Init 1 */

  /* USER CODE END RAMECC_Init 1 */
  /** Initialize RAMECC2 M1 : SRAM1_0
  */
  hramecc2_m1.Instance = RAMECC2_Monitor1;
  if (HAL_RAMECC_Init(&hramecc2_m1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initialize RAMECC2 M2 : SRAM2_0
  */
  hramecc2_m2.Instance = RAMECC2_Monitor2;
  if (HAL_RAMECC_Init(&hramecc2_m2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initialize RAMECC3 M1 : SRAM4
  */
  hramecc3_m1.Instance = RAMECC3_Monitor1;
  if (HAL_RAMECC_Init(&hramecc3_m1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RAMECC_Init 2 */

  /* USER CODE END RAMECC_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
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
//	If the data size is 16 bits, then data must be mapped on SAI_xDR[23:8].
//	The transfer is performed always with LSB first.

//	The SAI first sends the adequate preamble for each sub-frame in a block.
//	The SAI_xDR is then sent on the SD line (manchester coded).
  /* USER CODE END SAI4_Init 0 */

  /* USER CODE BEGIN SAI4_Init 1 */

  /* USER CODE END SAI4_Init 1 */
  hsai_BlockA4.Instance = SAI4_Block_A;
  hsai_BlockA4.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA4.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockA4.Init.DataSize = SAI_DATASIZE_16;
  hsai_BlockA4.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA4.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA4.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA4.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA4.Init.NoDivider = SAI_MASTERDIVIDER_DISABLE;
  hsai_BlockA4.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA4.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA4.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA4.Init.PdmInit.Activation = ENABLE;
  hsai_BlockA4.Init.PdmInit.MicPairsNbr = 2;
  hsai_BlockA4.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK2_ENABLE;
  hsai_BlockA4.FrameInit.FrameLength = 64;
  hsai_BlockA4.FrameInit.ActiveFrameLength = 32;
  hsai_BlockA4.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockA4.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA4.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA4.SlotInit.FirstBitOffset = 0;
  hsai_BlockA4.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA4.SlotInit.SlotNumber = 4;
  hsai_BlockA4.SlotInit.SlotActive = 0x0000FFFF;
  if (HAL_SAI_Init(&hsai_BlockA4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI4_Init 2 */
  hsai_BlockA4.Ack = 0;

  /* USER CODE END SAI4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_BDMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_BDMA_CLK_ENABLE();

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
  hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
  hdma_memtomem_dma2_stream0.Init.Request = DMA_REQUEST_MEM2MEM;
  hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)
  {
    Error_Handler( );
  }

  // Register some callbacks for the DMA
//    HAL_DMA_RegisterCallback(&hdma_memtomem_dma2_stream0, HAL_DMA_XFER_HALFCPLT_CB_ID, &FYDP_SAI4_RxHalfCallback);
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Register Bus IOs if component ID is OK
  * @retval error status
  */
static int32_t WM8994_Probe(void)
{
  int32_t                   ret = BSP_ERROR_NONE;
  WM8994_IO_t               IOCtx;
  static WM8994_Object_t    WM8994Obj;
  uint32_t                  wm8994_id;

  /* Configure the audio driver */
  IOCtx.Address     = AUDIO_I2C_ADDRESS;
  IOCtx.Init        = BSP_I2C4_Init;
  IOCtx.DeInit      = BSP_I2C4_DeInit;
  IOCtx.ReadReg     = BSP_I2C4_ReadReg16;
  IOCtx.WriteReg    = BSP_I2C4_WriteReg16;
  IOCtx.GetTick     = BSP_GetTick;

  if(WM8994_RegisterBusIO (&WM8994Obj, &IOCtx) != WM8994_OK)
  {
    ret = BSP_ERROR_BUS_FAILURE;
  }
  else if(WM8994_Reset(&WM8994Obj) != WM8994_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else if(WM8994_ReadID(&WM8994Obj, &wm8994_id) != WM8994_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else if(wm8994_id != WM8994_ID)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    AudioDrv = (AUDIO_Drv_t *) &WM8994_Driver;
    AudioCompObj = &WM8994Obj;
  }

  return ret;
}

/**
  * @brief  Playback initialization
  * @param  None
  * @retval None
  */
static void Playback_Init(void)
{
  WM8994_Init_t codec_init;
  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct;

  /* Configure PLLSAI prescalers */
  /* PLL2SAI_VCO: VCO_271M
     SAI_CLK(first level) = PLLSAI_VCO/PLL2P = 271/24 = 11.291 Mhz */
  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  RCC_PeriphCLKInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
  RCC_PeriphCLKInitStruct.PLL2.PLL2P = 24;
  RCC_PeriphCLKInitStruct.PLL2.PLL2Q = 24;
  RCC_PeriphCLKInitStruct.PLL2.PLL2R = 1;
  RCC_PeriphCLKInitStruct.PLL2.PLL2N = 271;
  RCC_PeriphCLKInitStruct.PLL2.PLL2FRACN = 0;
  RCC_PeriphCLKInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  RCC_PeriphCLKInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  RCC_PeriphCLKInitStruct.PLL2.PLL2M = 25;

  if(HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize SAI */
  __HAL_SAI_RESET_HANDLE_STATE(&hsai_BlockB1);

  hsai_BlockB1.Instance = AUDIO_OUT_SAIx;

  __HAL_SAI_DISABLE(&hsai_BlockB1);

  hsai_BlockB1.Init.AudioMode      = SAI_MODEMASTER_TX;
  hsai_BlockB1.Init.Synchro        = SAI_ASYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive    = SAI_OUTPUTDRIVE_ENABLE;
  hsai_BlockB1.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockB1.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_1QF;
  hsai_BlockB1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_22K;
  hsai_BlockB1.Init.Protocol       = SAI_FREE_PROTOCOL;
  hsai_BlockB1.Init.DataSize       = SAI_DATASIZE_16;
  hsai_BlockB1.Init.FirstBit       = SAI_FIRSTBIT_MSB;
  hsai_BlockB1.Init.ClockStrobing  = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockB1.Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.Mckdiv         = 0; /* N.U */
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState       = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockB1.Init.MckOverSampling      = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockB1.Init.PdmInit.Activation   = DISABLE;

  hsai_BlockB1.FrameInit.FrameLength       = 32;
  hsai_BlockB1.FrameInit.ActiveFrameLength = 16;
  hsai_BlockB1.FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockB1.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
  hsai_BlockB1.FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;

  hsai_BlockB1.SlotInit.FirstBitOffset = 0;
  hsai_BlockB1.SlotInit.SlotSize       = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockB1.SlotInit.SlotNumber     = 2;
  hsai_BlockB1.SlotInit.SlotActive     = (SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1);

  if(HAL_OK != HAL_SAI_Init(&hsai_BlockB1))
  {
    Error_Handler();
  }

  /* Enable SAI to generate clock used by audio driver */
  __HAL_SAI_ENABLE(&hsai_BlockB1);

  WM8994_Probe();

  /* Fill codec_init structure */
  codec_init.InputDevice  = WM8994_IN_NONE;
  codec_init.OutputDevice = WM8994_OUT_SPEAKER;
  codec_init.Frequency    = AUDIO_FREQUENCY_22K;
  codec_init.Resolution   = WM8994_RESOLUTION_16b; /* Not used */
  codec_init.Volume       = 80;

  /* Initialize the codec internal registers */
  if(AudioDrv->Init(AudioCompObj, &codec_init) < 0)
  {
     Error_Handler();
  }
}


/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_HFNMI_PRIVDEF);

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












// AUDIO PLAY CODE ________

/* Private define ------------------------------------------------------------*/
#define USE_SAI_INSTANCE

/* Audio file size */
#define AUDIO_FILE_SIZE               NUM_BYTES
#define AUDIO_BUFFER_SIZE			  NUM_BYTES

/* Private typedef -----------------------------------------------------------*/
typedef enum {
  AUDIO_STATE_IDLE = 0,
  AUDIO_STATE_INIT,
  AUDIO_STATE_PLAYING,
}AUDIO_PLAYBACK_StateTypeDef;

typedef enum {
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF,
  BUFFER_OFFSET_FULL,
}BUFFER_StateTypeDef;

typedef struct {
  uint8_t buff[NUM_BYTES];
  uint32_t fptr;
  BUFFER_StateTypeDef state;
  uint32_t AudioFileSize;
  uint32_t *SrcAddress;
}AUDIO_BufferTypeDef;

//#if defined ( __ICCARM__ )
//#pragma location=0x38000000
//uint16_t  PlayBuffer[AUDIO_BUFFER_SIZE];
//#elif defined ( __CC_ARM )
//__attribute__((at(0x38000000))) uint16_t PlayBuffer[AUDIO_BUFFER_SIZE];
//#elif defined ( __GNUC__ )
//uint16_t __attribute__((section(".RAM_D3")))  PlayBuffer[AUDIO_BUFFER_SIZE];
//#endif
uint16_t  PlayBuffer[NUM_BYTES];

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static AUDIO_BufferTypeDef  buffer_ctl;
static AUDIO_PLAYBACK_StateTypeDef  audio_state;
__IO uint32_t uwVolume = 20;

uint32_t PauseEnabledStatus = 0;
uint32_t updown = 1;

uint32_t AudioFreq[8] = {96000, 48000, 44100, 32000, 22050, 16000, 11025, 8000};

//TS_ActionTypeDef ts_action = TS_ACT_NONE;
BSP_AUDIO_Init_t* AudioPlayInit;
uint32_t AudioInstance = 0;
uint32_t OutputDevice = 0;
uint8_t FreqStr[256] = {0};

/* Private function prototypes -----------------------------------------------*/
//static void Audio_SetHint(uint32_t Index);
static uint32_t GetData(void *pdata, uint32_t offset, uint8_t *pbuf, uint32_t NbrOfData);
AUDIO_ErrorTypeDef AUDIO_Start(uint32_t *psrc_address, uint32_t file_size);
AUDIO_ErrorTypeDef AUDIO_Stop(void);


/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Audio Play demo
  * @param  None
  * @retval None
  */
void AudioPlay_demo (void)
{
  uint32_t *AudioFreq_ptr;
  uwVolume = 70;

#ifdef USE_SAI_INSTANCE
  AudioInstance = 0;
  AudioFreq_ptr = &AudioFreq[5]; /* 16K*/
#else
  AudioInstance = 1;
  AudioFreq_ptr = &AudioFreq[0]; /* 96K*/
#endif

//  Audio_SetHint(0);
  AudioPlayInit->Device = AUDIO_OUT_DEVICE_HEADPHONE;
  AudioPlayInit->ChannelsNbr = 2;
  AudioPlayInit->SampleRate = AUDIO_FREQUENCY_16K;
  AudioPlayInit->BitsPerSample = AUDIO_RESOLUTION_16B;
  AudioPlayInit->Volume = uwVolume;


  if(BSP_AUDIO_OUT_Init(AudioInstance, AudioPlayInit) != 0)
  {
//    ButtonState = 0;
//    BSP_TS_DeInit(0);
  }
  else
  {
    /*
    Start playing the file from a circular buffer, once the DMA is enabled, it is
    always in running state. Application has to fill the buffer with the audio data
    using Transfer complete and/or half transfer complete interrupts callbacks
    (BSP_AUDIO_OUT_TransferComplete_CallBack() or BSP_AUDIO_OUT_HalfTransfer_CallBack()...
    */
    AUDIO_Start((uint32_t *)audio_wav + 11, (uint32_t)AUDIO_FILE_SIZE);

    /* Infinite loop */
    while (1)
    {
      /* IMPORTANT: AUDIO_Process() should be called within a periodic process */
        HAL_SAI_Receive_DMA(&hsai_BlockA4, input_buffer, NUM_BYTES);
        PDM_Filter(pdm_buffer, audio_wav, &PDM1_filter_handler);
        AUDIO_Process();

    }
    }
  }
//}


/**
  * @brief  Display Audio demo hint
  * @param  None
  * @retval None
  */
//static void Audio_SetHint(uint32_t Index)
//{
//  uint32_t x_size, y_size;
//
//  BSP_LCD_GetXSize(0, &x_size);
//  BSP_LCD_GetYSize(0, &y_size);
//
//  /* Clear the LCD */
////  UTIL_LCD_Clear(UTIL_LCD_COLOR_WHITE);
//
//  /* Set Audio Demo description */
//  UTIL_LCD_FillRect(0, 0, x_size, 80, UTIL_LCD_COLOR_BLUE);
//  UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
//  UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_BLUE);
//  UTIL_LCD_SetFont(&Font24);
//  if(Index == 0)
//  {
//    UTIL_LCD_DisplayStringAt(0, 0, (uint8_t *)"SET MUTE / SET VOLUME / SET SAMPLE RATE", CENTER_MODE);
//    UTIL_LCD_SetFont(&Font12);
//    UTIL_LCD_DisplayStringAt(0, 30, (uint8_t *)"Press TAMPER button for next menu          ", CENTER_MODE);
//    UTIL_LCD_DisplayStringAt(0, 45, (uint8_t *)"Use touch screen +/- to change Volume/Frequency    ", CENTER_MODE);
//    UTIL_LCD_DisplayStringAt(0, 60, (uint8_t *)"Touch upper part of the screen to Pause/Resume    ", CENTER_MODE);
//  }
//
//  UTIL_LCD_DrawRect(10, 81, x_size - 20, y_size - 81, UTIL_LCD_COLOR_BLUE);
//  UTIL_LCD_DrawRect(11, 82, x_size - 22, y_size - 82, UTIL_LCD_COLOR_BLUE);
//}


/**
  * @brief  Starts Audio streaming.
  * @param  None
  * @retval Audio error
  */
AUDIO_ErrorTypeDef AUDIO_Start(uint32_t *psrc_address, uint32_t file_size)
{
  uint32_t bytesread;

  buffer_ctl.state = BUFFER_OFFSET_NONE;
  buffer_ctl.AudioFileSize = file_size;
  buffer_ctl.SrcAddress = psrc_address;

  bytesread = GetData( (void *)psrc_address,
                       0,
                       (uint8_t*)&PlayBuffer[0],
                       2*AUDIO_BUFFER_SIZE);
  if(bytesread > 0)
  {
    BSP_AUDIO_OUT_Play(AudioInstance, (uint8_t *)PlayBuffer, 2*AUDIO_BUFFER_SIZE);
    audio_state = AUDIO_STATE_PLAYING;
    buffer_ctl.fptr = bytesread;
    return AUDIO_ERROR_NONE;
  }
  return AUDIO_ERROR_IO;
}

/**
  * @brief  Manages Audio process.
  * @param  None
  * @retval Audio error
  */
uint8_t AUDIO_Process(void)
{
  uint32_t bytesread;
  AUDIO_ErrorTypeDef error_state = AUDIO_ERROR_NONE;

  switch(audio_state)
  {
  case AUDIO_STATE_PLAYING:

    if(buffer_ctl.fptr >= buffer_ctl.AudioFileSize)
    {
      /* Play audio sample again ... */
      buffer_ctl.fptr = 0;
      error_state = AUDIO_ERROR_EOF;
    }

    /* 1st half buffer played; so fill it and continue playing from bottom*/
    if(buffer_ctl.state == BUFFER_OFFSET_HALF)
    {
      SCB_InvalidateDCache_by_Addr((uint32_t *)&PlayBuffer[0], AUDIO_BUFFER_SIZE);
      bytesread = GetData((void *)buffer_ctl.SrcAddress,
                          buffer_ctl.fptr,
                          (uint8_t*)&PlayBuffer[0],
                          AUDIO_BUFFER_SIZE );

      if( bytesread >0)
      {
        buffer_ctl.state = BUFFER_OFFSET_NONE;
        buffer_ctl.fptr += bytesread;
        /* Clean Data Cache to update the content of the SRAM */
        SCB_CleanDCache_by_Addr((uint32_t*)&PlayBuffer[0], AUDIO_BUFFER_SIZE);
      }
    }

    /* 2nd half buffer played; so fill it and continue playing from top */
    if(buffer_ctl.state == BUFFER_OFFSET_FULL)
    {
      SCB_InvalidateDCache_by_Addr((uint32_t *)&PlayBuffer[AUDIO_BUFFER_SIZE/2], AUDIO_BUFFER_SIZE);
      bytesread = GetData((void *)buffer_ctl.SrcAddress,
                          buffer_ctl.fptr,
                          (uint8_t*)&PlayBuffer[AUDIO_BUFFER_SIZE /2],
                          AUDIO_BUFFER_SIZE );
      if( bytesread > 0)
      {
        buffer_ctl.state = BUFFER_OFFSET_NONE;
        buffer_ctl.fptr += bytesread;

        /* Clean Data Cache to update the content of the SRAM */
        SCB_CleanDCache_by_Addr((uint32_t*)&PlayBuffer[AUDIO_BUFFER_SIZE/2], AUDIO_BUFFER_SIZE);
      }
    }
    break;

  default:
    error_state = AUDIO_ERROR_NOTREADY;
    break;
  }
  return (uint8_t) error_state;
}

/**
  * @brief  Gets Data from storage unit.
  * @param  None
  * @retval None
  */
static uint32_t GetData(void *pdata, uint32_t offset, uint8_t *pbuf, uint32_t NbrOfData)
{
  uint8_t *lptr = pdata;
  uint32_t ReadDataNbr;

  ReadDataNbr = 0;
  while(((offset + ReadDataNbr) < buffer_ctl.AudioFileSize) && (ReadDataNbr < NbrOfData))
  {
    pbuf[ReadDataNbr]= lptr [offset + ReadDataNbr];
    ReadDataNbr++;
  }
  return ReadDataNbr;
}

/*------------------------------------------------------------------------------
       Callbacks implementation:
           the callbacks API are defined __weak in the stm32h735g_discovery_audio.c file
           and their implementation should be done the user code if they are needed.
           Below some examples of callback implementations.
  ----------------------------------------------------------------------------*/
/**
  * @brief  Manages the full Transfer complete event.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_TransferComplete_CallBack(uint32_t Instance)
{
  if(audio_state == AUDIO_STATE_PLAYING)
  {
    /* allows AUDIO_Process() to refill 2nd part of the buffer  */
    buffer_ctl.state = BUFFER_OFFSET_FULL;
  }
}

/**
  * @brief  Manages the DMA Half Transfer complete event.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(uint32_t Instance)
{
  if(audio_state == AUDIO_STATE_PLAYING)
  {
    /* allows AUDIO_Process() to refill 1st part of the buffer  */
    buffer_ctl.state = BUFFER_OFFSET_HALF;
  }
}

/**
  * @brief  Manages the DMA FIFO error event.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_Error_CallBack(uint32_t Instance)
{
//  /* Display message on the LCD screen */
//  UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_RED);
//  UTIL_LCD_DisplayStringAt(0, LINE(14), (uint8_t *)"       DMA  ERROR     ", CENTER_MODE);
//  UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_WHITE);
//
//  /* Stop the program with an infinite loop */
//  while (BSP_PB_GetState(BUTTON_USER) != RESET)
//  {
//    return;
//  }

  /* could also generate a system reset to recover from the error */
  /* .... */
}

/**
  * @}
  */

/**
  * @}
  */
