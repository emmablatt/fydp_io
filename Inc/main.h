/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* SAI peripheral configuration defines */
//#define AUDIO_SAIx                           SAI1_Block_B
//#define AUDIO_SAIx_CLK_ENABLE()              __HAL_RCC_SAI1_CLK_ENABLE()
//
//#define AUDIO_SAIx_FS_GPIO_PORT              GPIOF
//#define AUDIO_SAIx_FS_AF                     GPIO_AF6_SAI1
//#define AUDIO_SAIx_FS_PIN                    GPIO_PIN_9
//#define AUDIO_SAIx_SCK_GPIO_PORT             GPIOF
//#define AUDIO_SAIx_SCK_AF                    GPIO_AF6_SAI1
//#define AUDIO_SAIx_SCK_PIN                   GPIO_PIN_8
//#define AUDIO_SAIx_SD_GPIO_PORT              GPIOF
//#define AUDIO_SAIx_SD_AF                     GPIO_AF6_SAI1
//#define AUDIO_SAIx_SD_PIN                    GPIO_PIN_6
//#define AUDIO_SAIx_MCLK_GPIO_PORT            GPIOF
//#define AUDIO_SAIx_MCLK_AF                   GPIO_AF6_SAI1
//#define AUDIO_SAIx_MCLK_PIN                  GPIO_PIN_7
//
//#define AUDIO_SAIx_MCLK_ENABLE()             __HAL_RCC_GPIOF_CLK_ENABLE()
//#define AUDIO_SAIx_SCK_ENABLE()              __HAL_RCC_GPIOF_CLK_ENABLE()
//#define AUDIO_SAIx_FS_ENABLE()               __HAL_RCC_GPIOF_CLK_ENABLE()
//#define AUDIO_SAIx_SD_ENABLE()               __HAL_RCC_GPIOF_CLK_ENABLE()



/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/** @defgroup STM32H735G_DISCO_AUDIO_Exported_Constants AUDIO Exported Constants
  * @{
  */
#define	NUM_BYTES  16384
//#define PLAY_BUFF_SIZE 1024
#define SRAM4_BASE 0x38000000
#define SRAM2_BASE 0x30004000
#define CODEC_I2C  0x34U
#define AUDIO_ZERO 32767
static __IO int16_t                 UpdatePointer = -1;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void RxAck(SAI_HandleTypeDef *hsai);
void TxHalfSpeaker(SAI_HandleTypeDef *hsai);
void TxFullSpeaker(SAI_HandleTypeDef *hsai);
uint8_t AUDIO_Process(void);
void AudioPlay_Demo(void);

/* USER CODE BEGIN EFP */


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
typedef enum {
  AUDIO_ERROR_NONE = 0,
  AUDIO_ERROR_NOTREADY,
  AUDIO_ERROR_IO,
  AUDIO_ERROR_EOF,
}AUDIO_ErrorTypeDef;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
