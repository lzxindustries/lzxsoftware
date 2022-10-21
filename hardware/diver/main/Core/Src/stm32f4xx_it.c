/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "main.h"

uint8_t fieldflag;
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
ADC_ChannelConfTypeDef sConfig;
//Video timing
uint8_t hsync_event;
uint8_t trigger_rising;
uint8_t trigger_falling;
uint8_t trigger_state;

uint8_t evenfield_event;
uint8_t oddfield_event;
uint8_t field;
uint8_t vsync;
uint16_t linecnt;
uint16_t lines_per_oddfield;
uint16_t lines_per_evenfield;
uint16_t lines_per_frame;
uint32_t dropped_frames;
uint32_t prelinecnt;

//Memory buffers
uint16_t samples_wave[NUM_BUFFERS][MAX_BUFFER_SIZE];
uint16_t samples_hphase_cv[NUM_BUFFERS][MAX_BUFFER_SIZE];
uint16_t hwave[NUM_BUFFERS][MAX_BUFFER_SIZE];
uint16_t vwave[NUM_BUFFERS][MAX_BUFFER_SIZE];
uint16_t hphase_cv[NUM_BUFFERS][MAX_BUFFER_SIZE];

//Memory pointers and enables
uint8_t waveReadPtr;
uint8_t waveWritePtr;
uint8_t sampleReadPtr;
uint8_t sampleWritePtr;
uint8_t waveRenderComplete;
uint8_t captureEnable;

//Application variables
uint16_t hres;
uint16_t vres;
uint16_t hphase_slider;
uint16_t vphase_slider;
uint16_t vphase_cv;
uint8_t interlace_mode;   // 0 = video sampling, 1 = audio sampling
uint8_t frozen;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_tim1_uev;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
	
	__HAL_TIM_ENABLE(&htim1);
	//if (linecnt > 6){__HAL_TIM_ENABLE(&htim1); }
	hsync_event = 1;
	linecnt += 1;	
	prelinecnt += 1;
	field = !HAL_GPIO_ReadPin(GEN_FID_GLCO_OUT_GPIO_Port, GEN_FID_GLCO_OUT_Pin);
	vsync = !HAL_GPIO_ReadPin(GEN_VSYNC_OUT_GPIO_Port, GEN_VSYNC_OUT_Pin);
	
	if (!vsync && field && !fieldflag && prelinecnt >= 256)
	{
		lines_per_oddfield = prelinecnt;
		evenfield_event = 1;	
		fieldflag = 1;
	}
	else if (!vsync && !field && prelinecnt >= 512)
	{
		fieldflag = 0;
		lines_per_frame = prelinecnt;
		lines_per_evenfield = lines_per_frame - lines_per_oddfield;
		prelinecnt = 0;
		oddfield_event = 1;
	}	

	if (prelinecnt == LINECOUNT_DELAY)
	{
		linecnt = 0;
		if (waveRenderComplete)
		{
			if (captureEnable)
			{
				sampleReadPtr = sampleWritePtr;	
				sampleWritePtr = (sampleWritePtr + 1) % 4;						
			}
				
			waveReadPtr = waveWritePtr;	
			waveWritePtr = (waveWritePtr + 1) % 4;	
			waveRenderComplete = 0;
		}
		else
		{
			dropped_frames++;
		}

	}
	if (linecnt >= 0)
	{
		
		//if (linecnt > 20) { __HAL_TIM_ENABLE(&htim1); }
		uint16_t sample;

		
		//HAL_ADC_PollForConversion(&hadc1, 1);
		sample = 4095 - HAL_ADC_GetValue(&hadc1);
		if (sample >= 4095 + 256 - 1024)
		{
			sample = 4095 + 256 - 1024;
		}
		else if (sample <= 2047 + 256 - 1024)
		{
			sample = 2047 + 256 - 1024;	
		}
		sample -= 2047 + 256 - 1024;
		sample = sample >> 1;
		//sample = (samples[sampleWritePtr][linecnt - VBLANK] + sample) >> 1;
		//samples[sampleWritePtr][linecnt - VBLANK] = (sample + samples[sampleWritePtr][linecnt - VBLANK - 1]) >> 1;
		//samples[sampleWritePtr][linecnt - VBLANK] = (sample + samples[sampleWritePtr][linecnt - VBLANK - 1]) >> 1;
		if(sample > 1023)
		{
			sample = 1023;
		}
		if (linecnt == 0)
		{
			samples_wave[sampleWritePtr][linecnt - VBLANK] = sample;
		}
		else if (linecnt < 3)
		{
			samples_wave[sampleWritePtr][linecnt - VBLANK] = (sample + samples_wave[sampleWritePtr][linecnt - VBLANK - 1]) >> 1;
		}
		else
		{
			samples_wave[sampleWritePtr][linecnt - VBLANK] = (sample + samples_wave[sampleWritePtr][linecnt - VBLANK - 1] + samples_wave[sampleWritePtr][linecnt - VBLANK - 2] + samples_wave[sampleWritePtr][linecnt - VBLANK - 3]) >> 2;
		}
		
		//samples[sampleWritePtr][linecnt - VBLANK] = (4095 - HAL_ADC_GetValue(&hadc1)) >> 2;
		//sConfig.Channel = ADC_CHANNEL_0;
		//sConfig.Rank = 2;
		//HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		//sConfig.Channel = ADC_CHANNEL_1;
		//sConfig.Rank = 3;
		//HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		sConfig.Channel = ADC_CHANNEL_2;
		sConfig.Rank = 1;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		//sConfig.Channel = ADC_CHANNEL_3;
		//sConfig.Rank = 4;
		//HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		sConfig.Channel = ADC_CHANNEL_4;
		sConfig.Rank = 2;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1);
		sample = 4095 - HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1);
		sample = (sample + 4095 - HAL_ADC_GetValue(&hadc1))>>1;
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1);
		sample = (sample + 4095 - HAL_ADC_GetValue(&hadc1)) >> 1;
		/*if (sample >= 4095 + 0 - 2048)
		{
			sample = 4095 + 0 - 2048;
		}
		else if (sample <= 0)
		{
			sample = 0;	
		}
		sample -= 0;*/
		if (sample <= 32)
		{
			sample = 32;
		}
		//sample -= 32;
		sample = (sample >> 1);

		if (sample >= 709)
		{
			sample = 709;
		}
		//sample = (hres + sample - 185) % hres;


		//sample = (samples_hphase[sampleWritePtr][(vres + linecnt - VBLANK - 3) % vres] + samples_hphase[sampleWritePtr][(vres + linecnt - VBLANK - 2) % vres] + samples_hphase[sampleWritePtr][(vres + linecnt - VBLANK - 1) % vres] + sample) >> 2;

		//if (sample > vres)
		//{
		//	sample = vres;	
		//}
		//if(sample > 1023)
		//{
		//	sample = 1023;
		//}
		

		if (linecnt == 0)
		{
			//samples_hphase[sampleWritePtr][linecnt - VBLANK] = (sample + samples_hphase[(4 + (sampleWritePtr - 1)) % 4][vres - 1]) >> 1;
			//samples_hphase[sampleWritePtr][linecnt - VBLANK] = (sample + samples_hphase[(4 + sampleWritePtr - 1) % 4][vres - 1] + samples_hphase[(4 + sampleWritePtr - 1) % 4][vres - 2] + samples_hphase[(4 + sampleWritePtr - 1) % 4][vres - 3]) >> 2;
			samples_hphase_cv[sampleWritePtr][linecnt - VBLANK] = sample;
		}
		else if (linecnt < 3)
		{
			samples_hphase_cv[sampleWritePtr][linecnt - VBLANK] = (sample + samples_hphase_cv[sampleWritePtr][linecnt - VBLANK - 1]) >> 1;
		}
		else
		{
			samples_hphase_cv[sampleWritePtr][linecnt - VBLANK] = (sample + samples_hphase_cv[sampleWritePtr][linecnt - VBLANK - 1] + samples_hphase_cv[sampleWritePtr][linecnt - VBLANK - 2] + samples_hphase_cv[sampleWritePtr][linecnt - VBLANK - 3]) >> 2;
		}

		//samples_hphase[sampleWritePtr][linecnt - VBLANK] = (4095 - HAL_ADC_GetValue(&hadc1)) >> 2;
		if(HAL_GPIO_ReadPin(TRIG_IN_GPIO_Port, TRIG_IN_Pin) && trigger_state == 0)  {  trigger_rising++; trigger_state = 1;} 
		else if(!HAL_GPIO_ReadPin(TRIG_IN_GPIO_Port, TRIG_IN_Pin) && trigger_state == 1)  {  trigger_falling++; trigger_state = 0; } 
		
			
			

		
		

		if (linecnt == 0)
		{
			
			sConfig.Channel = ADC_CHANNEL_0;
			sConfig.Rank = 1;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			sConfig.Channel = ADC_CHANNEL_1;
			sConfig.Rank = 2;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			sConfig.Channel = ADC_CHANNEL_2;
			sConfig.Rank = 3;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			sConfig.Channel = ADC_CHANNEL_3;
			sConfig.Rank = 4;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			sConfig.Channel = ADC_CHANNEL_4;
			sConfig.Rank = 5;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);

			HAL_ADC_Start(&hadc1);	
			HAL_ADC_PollForConversion(&hadc1, 1);
			sample = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Start(&hadc1);	
			HAL_ADC_PollForConversion(&hadc1, 1);
			sample = (sample+ HAL_ADC_GetValue(&hadc1))>>1;
			HAL_ADC_Start(&hadc1);	
			HAL_ADC_PollForConversion(&hadc1, 1);
			sample = (sample + HAL_ADC_GetValue(&hadc1)) >> 1;
			HAL_ADC_Start(&hadc1);	
			HAL_ADC_PollForConversion(&hadc1, 1);
			sample = (sample + HAL_ADC_GetValue(&hadc1)) >> 1;
			HAL_ADC_Start(&hadc1);	
			HAL_ADC_PollForConversion(&hadc1, 1);
			sample = (sample + HAL_ADC_GetValue(&hadc1)) >> 1;
			if (sample >= MAX_SLIDER_VALUE)
			{
				sample = MAX_SLIDER_VALUE;
			}
			vphase_slider = (((((sample) * (vres+7)) / MAX_SLIDER_VALUE) + vphase_slider + vphase_slider + vphase_slider + vphase_slider + vphase_slider + vphase_slider + vphase_slider) >> 3); 
		}
		else if (linecnt == 1)
			{
		
			//HAL_ADC_PollForConversion(&hadc1, 1);
			//sample = HAL_ADC_GetValue(&hadc1);
			//HAL_ADC_Start(&hadc1);	
			//Reset to sample hphase slider
			sConfig.Channel = ADC_CHANNEL_0;
			sConfig.Rank = 2;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			sConfig.Channel = ADC_CHANNEL_1;
			sConfig.Rank = 1;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			sConfig.Channel = ADC_CHANNEL_2;
			sConfig.Rank = 3;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			sConfig.Channel = ADC_CHANNEL_3;
			sConfig.Rank = 4;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			sConfig.Channel = ADC_CHANNEL_4;
			sConfig.Rank = 5;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);

			HAL_ADC_Start(&hadc1);	
			HAL_ADC_PollForConversion(&hadc1, 1);
			sample = HAL_ADC_GetValue(&hadc1) >> 1;
			HAL_ADC_Start(&hadc1);	
			HAL_ADC_PollForConversion(&hadc1, 1);
			sample = (sample + HAL_ADC_GetValue(&hadc1)) >> 1;
			HAL_ADC_Start(&hadc1);	
			HAL_ADC_PollForConversion(&hadc1, 1);
			sample = (sample + HAL_ADC_GetValue(&hadc1)) >> 1;
				HAL_ADC_Start(&hadc1);	
				HAL_ADC_PollForConversion(&hadc1, 1);
				sample = (sample + HAL_ADC_GetValue(&hadc1)) >> 1;
				HAL_ADC_Start(&hadc1);	
				HAL_ADC_PollForConversion(&hadc1, 1);
				sample = (sample + HAL_ADC_GetValue(&hadc1)) >> 1;
				HAL_ADC_Start(&hadc1);	
				HAL_ADC_PollForConversion(&hadc1, 1);
				sample = (sample + HAL_ADC_GetValue(&hadc1)) >> 1;
				HAL_ADC_Start(&hadc1);	
				HAL_ADC_PollForConversion(&hadc1, 1);
				sample = (sample + HAL_ADC_GetValue(&hadc1)) >> 1;
				HAL_ADC_Start(&hadc1);	
				HAL_ADC_PollForConversion(&hadc1, 1);
				sample = (sample + HAL_ADC_GetValue(&hadc1)) >> 1;
				//sample = (sample + (HAL_ADC_GetValue(&hadc1))) >> 1;
			if(sample >= MAX_SLIDER_VALUE)
				{
					sample = MAX_SLIDER_VALUE;
				}
				hphase_slider = (((((sample) * (hres+7)) / MAX_SLIDER_VALUE) + hphase_slider + hphase_slider + hphase_slider + hphase_slider + hphase_slider + hphase_slider + hphase_slider) >> 3); 
			//hphase_slider = (((sample) * (hres*3)) / 1239); 
		}
		else if (linecnt == 2)
		{
			//Reset to sample vphase CV
			sConfig.Channel = ADC_CHANNEL_0;
			sConfig.Rank = 2;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			sConfig.Channel = ADC_CHANNEL_1;
			sConfig.Rank = 3;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			sConfig.Channel = ADC_CHANNEL_2;
			sConfig.Rank = 4;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			sConfig.Channel = ADC_CHANNEL_3;
			sConfig.Rank = 1;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			sConfig.Channel = ADC_CHANNEL_4;
			sConfig.Rank = 5;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);

			HAL_ADC_Start(&hadc1);	

			HAL_ADC_PollForConversion(&hadc1, 1);
			//sample = (sample + (4095 - HAL_ADC_GetValue(&hadc1))) >> 1;
			sample = 4095 - HAL_ADC_GetValue(&hadc1);

			HAL_ADC_Start(&hadc1);	
			HAL_ADC_PollForConversion(&hadc1, 1);
			sample = (sample + 4095 - HAL_ADC_GetValue(&hadc1)) >> 1;
			HAL_ADC_Start(&hadc1);	
			HAL_ADC_PollForConversion(&hadc1, 1);
			sample = (sample + 4095 - HAL_ADC_GetValue(&hadc1)) >> 1;
			HAL_ADC_Start(&hadc1);	
			HAL_ADC_PollForConversion(&hadc1, 1);
			sample = (sample + 4095 - HAL_ADC_GetValue(&hadc1)) >> 1;
			HAL_ADC_Start(&hadc1);	
			HAL_ADC_PollForConversion(&hadc1, 1);
			sample = (sample + 4095 - HAL_ADC_GetValue(&hadc1)) >> 1;
			if (sample >= 4095 + 361 - 2048)
			{
				sample = 4095 + 361 - 2048;
			}
			else if (sample <= 361)
			{
				sample = 361;	
			}
			sample -= 361;
			sample = ((sample)*vres)/2035;
			vphase_cv = ((vphase_cv + vphase_cv + vphase_cv + sample) >> 2); 	
		} 

		//sConfig.Channel = ADC_CHANNEL_0;
		//sConfig.Rank = 2;
		//HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		//sConfig.Channel = ADC_CHANNEL_1;
		//sConfig.Rank = 3;
		sConfig.Channel = ADC_CHANNEL_2;
		sConfig.Rank = 2;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		//sConfig.Channel = ADC_CHANNEL_3;
		//sConfig.Rank = 4;
		//HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		sConfig.Channel = ADC_CHANNEL_4;
		sConfig.Rank = 1;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);

		HAL_ADC_Start(&hadc1);	
	}







  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
* @brief This function handles USB On The Go FS global interrupt.
*/
//void OTG_FS_IRQHandler(void)
//{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
//  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
//}

/**
* @brief This function handles DMA2 stream5 global interrupt.
*/
void DMA2_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */

  /* USER CODE END DMA2_Stream5_IRQn 0 */
  //TIM1->CCMR1 = 0x00000000;
  //TIM1->CR1 = 0; 
  HAL_DMA_IRQHandler(&hdma_tim1_uev);
  /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

  /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
