/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32h7xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief CRC MSP Initialization
* This function configures the hardware resources used in this example
* @param hcrc: CRC handle pointer
* @retval None
*/
void HAL_CRC_MspInit(CRC_HandleTypeDef* hcrc)
{
  if(hcrc->Instance==CRC)
  {
  /* USER CODE BEGIN CRC_MspInit 0 */

  /* USER CODE END CRC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CRC_CLK_ENABLE();
  /* USER CODE BEGIN CRC_MspInit 1 */

  /* USER CODE END CRC_MspInit 1 */
  }

}

/**
* @brief CRC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hcrc: CRC handle pointer
* @retval None
*/
void HAL_CRC_MspDeInit(CRC_HandleTypeDef* hcrc)
{
  if(hcrc->Instance==CRC)
  {
  /* USER CODE BEGIN CRC_MspDeInit 0 */

  /* USER CODE END CRC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CRC_CLK_DISABLE();
  /* USER CODE BEGIN CRC_MspDeInit 1 */

  /* USER CODE END CRC_MspDeInit 1 */
  }

}

extern DMA_HandleTypeDef hdma_sai4_a;

static uint32_t SAI1_client =0;
static uint32_t SAI4_client =0;

void HAL_SAI_MspInit(SAI_HandleTypeDef* hsai)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  HAL_DMA_MuxSyncConfigTypeDef pSyncConfig;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
/* SAI1 */
    if(hsai->Instance==SAI1_Block_B)
    {
      /* Peripheral clock enable */
  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
    PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

      if (SAI1_client == 0)
      {
       __HAL_RCC_SAI1_CLK_ENABLE();
      }
    SAI1_client ++;

    /**SAI1_B_Block_B GPIO Configuration
    PE3     ------> SAI1_SD_B
    PF8     ------> SAI1_SCK_B
    PF7     ------> SAI1_MCLK_B
    PF9     ------> SAI1_FS_B
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_7|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    }
    /* SAI4 */
        if(hsai->Instance==SAI4_Block_A)
        {
        /* Peripheral clock enable */
      /** Initializes the peripherals clock
      */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI4A;
        PeriphClkInitStruct.Sai4AClockSelection = RCC_SAI4ACLKSOURCE_PLL;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        {
          Error_Handler();
        }

        if (SAI4_client == 0)
        {
           __HAL_RCC_SAI4_CLK_ENABLE();
        }
        SAI4_client ++;

        /**SAI4_A_Block_A GPIO Configuration
        PE4     ------> SAI4_D2
        PE5     ------> SAI4_CK2
        PD6     ------> SAI4_D1
        */
        GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    //    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;

        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF10_SAI4;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_6;
    //    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;

        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_SAI4;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

          /* Peripheral DMA init*/

        hdma_sai4_a.Instance = BDMA_Channel1;
        hdma_sai4_a.Init.Request = BDMA_REQUEST_SAI4_A;
        hdma_sai4_a.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_sai4_a.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_sai4_a.Init.MemInc = DMA_MINC_ENABLE;
        hdma_sai4_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;

        // changed from word to byte so each word in memory
        // so that each spot in memofry fills
        hdma_sai4_a.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_sai4_a.Init.Mode = DMA_CIRCULAR;
        hdma_sai4_a.Init.Priority = DMA_PRIORITY_VERY_HIGH;
        if (HAL_DMA_Init(&hdma_sai4_a) != HAL_OK)
        {
          Error_Handler();
        }

        pSyncConfig.SyncSignalID = HAL_DMAMUX2_SYNC_EXTI0;
        pSyncConfig.SyncPolarity = HAL_DMAMUX_SYNC_NO_EVENT;
        pSyncConfig.SyncEnable = DISABLE;
        pSyncConfig.EventEnable = ENABLE;
        pSyncConfig.RequestNumber = 1;
        if (HAL_DMAEx_ConfigMuxSync(&hdma_sai4_a, &pSyncConfig) != HAL_OK)
        {
          Error_Handler();
        }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(hsai,hdmarx,hdma_sai4_a);

    __HAL_LINKDMA(hsai,hdmatx,hdma_sai4_a);

    }
}

void HAL_SAI_MspDeInit(SAI_HandleTypeDef* hsai)
{
/* SAI1 */
    if(hsai->Instance==SAI1_Block_B)
    {
    SAI1_client --;
      if (SAI1_client == 0)
      {
      /* Peripheral clock disable */
      __HAL_RCC_SAI1_CLK_DISABLE();
      }

    /**SAI1_B_Block_B GPIO Configuration
    PE3     ------> SAI1_SD_B
    PF8     ------> SAI1_SCK_B
    PF7     ------> SAI1_MCLK_B
    PF9     ------> SAI1_FS_B
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_8|GPIO_PIN_7|GPIO_PIN_9);

    }
/* SAI4 */
    if(hsai->Instance==SAI4_Block_A)
    {
    SAI4_client --;
    if (SAI4_client == 0)
      {
      /* Peripheral clock disable */
       __HAL_RCC_SAI4_CLK_DISABLE();
      }

    /**SAI4_A_Block_A GPIO Configuration
    PE4     ------> SAI4_D2
    PE5     ------> SAI4_CK2
    PD6     ------> SAI4_D1
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_4|GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_6);

    /* SAI4 DMA Deinit */
    HAL_DMA_DeInit(hsai->hdmarx);
    HAL_DMA_DeInit(hsai->hdmatx);
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

