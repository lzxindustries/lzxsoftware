/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#define LINECOUNT_DELAY 0
#define NUM_BANKS 8
#define HPHASE_OFFSET (487-117)
#define VPHASE_OFFSET 0
#define MAX_SLIDER_VALUE 1220
#define DAC_MAX_VALUE 1023
#define NUM_SAMPLES 525
#define NUM_BUFFERS 4
#define MAX_BUFFER_SIZE (625*2)
#define HRES 525
#define HBLANK 0
#define VRES 525
#define VBLANK 0
#define IS32FL3738_ADDRESS_A 0x50 << 1
#define POTENTIOMETER_ADC_MAX 2898
#define ADC_SAMPLE_MAX 4022
#define ADC_SAMPLE_MIN 2850

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define DAC_D0_Pin GPIO_PIN_0
#define DAC_D0_GPIO_Port GPIOC
#define DAC_D1_Pin GPIO_PIN_1
#define DAC_D1_GPIO_Port GPIOC
#define DAC_D2_Pin GPIO_PIN_2
#define DAC_D2_GPIO_Port GPIOC
#define DAC_D3_Pin GPIO_PIN_3
#define DAC_D3_GPIO_Port GPIOC
#define SW0_Pin GPIO_PIN_6
#define SW0_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_7
#define SW1_GPIO_Port GPIOA
#define DAC_D4_Pin GPIO_PIN_4
#define DAC_D4_GPIO_Port GPIOC
#define DAC_D5_Pin GPIO_PIN_5
#define DAC_D5_GPIO_Port GPIOC
#define SW2_Pin GPIO_PIN_0
#define SW2_GPIO_Port GPIOB
#define DAC_RW_Pin GPIO_PIN_2
#define DAC_RW_GPIO_Port GPIOB
#define GEN_VSYNC_OUT_ALT_Pin GPIO_PIN_10
#define GEN_VSYNC_OUT_ALT_GPIO_Port GPIOB
#define SW6_Pin GPIO_PIN_12
#define SW6_GPIO_Port GPIOB
#define SW7_Pin GPIO_PIN_13
#define SW7_GPIO_Port GPIOB
#define SW8_Pin GPIO_PIN_14
#define SW8_GPIO_Port GPIOB
#define SW9_Pin GPIO_PIN_15
#define SW9_GPIO_Port GPIOB
#define DAC_D6_Pin GPIO_PIN_6
#define DAC_D6_GPIO_Port GPIOC
#define DAC_D7_Pin GPIO_PIN_7
#define DAC_D7_GPIO_Port GPIOC
#define DAC_D8_Pin GPIO_PIN_8
#define DAC_D8_GPIO_Port GPIOC
#define DAC_D9_Pin GPIO_PIN_9
#define DAC_D9_GPIO_Port GPIOC
#define GEN_BLANK_OUT_Pin GPIO_PIN_9
#define GEN_BLANK_OUT_GPIO_Port GPIOA
#define SW10_Pin GPIO_PIN_15
#define SW10_GPIO_Port GPIOA
#define DAC_AB_Pin GPIO_PIN_10
#define DAC_AB_GPIO_Port GPIOC
#define GEN_HSYNC_OUT_Pin GPIO_PIN_12
#define GEN_HSYNC_OUT_GPIO_Port GPIOC
#define GEN_HSYNC_OUT_EXTI_IRQn EXTI15_10_IRQn
#define GEN_FID_GLCO_OUT_Pin GPIO_PIN_2
#define GEN_FID_GLCO_OUT_GPIO_Port GPIOD
#define SW12_Pin GPIO_PIN_4
#define SW12_GPIO_Port GPIOB
#define TRIG_IN_Pin GPIO_PIN_5
#define TRIG_IN_GPIO_Port GPIOB
#define SW11_Pin GPIO_PIN_6
#define SW11_GPIO_Port GPIOB
#define GEN_VSYNC_OUT_Pin GPIO_PIN_7
#define GEN_VSYNC_OUT_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

void hsync_handler(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
