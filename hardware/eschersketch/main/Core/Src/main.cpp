
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
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
/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include <math.h>
#include <string>
#include "eeprom.h"
#include <stdio.h>
#include "calibrate.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define DEBUG 0
#define TOUCHPANEL_X_RESISTANCE_MIN 290
#define TOUCHPANEL_X_RESISTANCE_MAX 530
#define TOUCHPANEL_Y_RESISTANCE_MIN 500
#define TOUCHPANEL_Y_RESISTANCE_MAX 930
#define MAIN_DEBUG 1
#define POTENTIOMETER_ADC_MAX 2898
#define IS32FL3738_ADDRESS_A 0x50 << 1
#define IS32FL3738_ADDRESS_B 0x5F << 1
#define IS32FL3738_ADDRESS_C 0x55 << 1
#define IS32FL3738_ADDRESS_D 0x5A << 1
#define IS32FL3738_NUM_LEDS	48
#define DISPLAY_NUM_LEDS (IS32FL3738_NUM_LEDS*4)
#define DISPLAY_WIDTH	96
#define DISPLAY_HEIGHT	72
#define SAMPLE_BUFFER_SIZE 4096*4

#define TOUCHPANEL_THRESHOLD 750
#define TOUCHPANEL_BOUNDS_X_MIN 450
#define TOUCHPANEL_BOUNDS_X_MAX 3945
#define TOUCHPANEL_BOUNDS_Y_MIN 450
#define TOUCHPANEL_BOUNDS_Y_MAX 3714
#define	MAX_CAL_SAMPLES 3

enum
{
	kSensitivity       = 0,
	kPlaybackSpeed,
	kGateThreshold,
	kSmoothing,
	NUM_POTENTIOMETERS
}
;

enum
{
	kRecord     = 0,
	kGate1,
	kConstrain,
	kGate2,
	kPlay,
	kLoop,
	kTriggerIn,
	NUM_BUTTONS
};

enum
{
	kGate2Out = 0,
	kGate1Out,
	NUM_GATES
};

enum
{
	kXOut        = 0,
	kYOut,
	kVelocityOut,
	kPressureOut,
	NUM_DACS
};

enum
{
	kVar1    = 0,
	kVar2,
	kVar3,
	kVar4,
	NUM_VARS
};

struct PotentiometerConfig
{
	std::string name;
	uint16_t value;
};

struct ButtonConfig
{
	std::string name;
	uint8_t value;
	uint8_t last;
	uint8_t rising;
	uint8_t falling;
};

struct GateConfig
{
	std::string name;
	uint8_t value;
	uint8_t last;
};

struct DACConfig 
{
	std::string name;
	uint16_t value;
};

struct VarConfig 
{
	std::string name;
	uint32_t address;
	uint32_t value;
};

struct TouchpanelData
{
	uint16_t x;
	uint16_t y;
	uint16_t z1;
	uint16_t z2;
	uint16_t pressure;
};

struct LEDConfig
{
	uint8_t brightness;
	uint8_t lastBrightness;
};

struct XYZPoint
{
	uint16_t x;
	uint16_t y;
	uint16_t z;
};

PotentiometerConfig pots[NUM_POTENTIOMETERS] = { 
	{ "Sensitivity", 0 },
	{ "Gate Threshold", 0 },
	{ "Smoothing", 0 },
	{ "Playback Speed", 0 }
};

ButtonConfig buttons[NUM_BUTTONS] = { 
	{ "Record", 0 },
	{ "Gate 1", 0 },
	{ "Constrain", 0 },
	{ "Gate2", 0 },
	{ "Play", 0 },
	{ "Loop", 0 },
	{ "Trigger In", 0 }
};	

GateConfig gates[NUM_GATES] = { 
	{ "Gate 2 Out", 0 },
	{ "Gate 1 Out", 0 }
};

DACConfig dacs[NUM_DACS] = { 
	{ "X Out", 0 },
	{ "Y Out", 0 },
	{ "Velocity Out", 0 },
	{ "Pressure Out", 0 }
};

VarConfig vars[NUM_VARS] = { 
	{ "Var 1", 100 },
	{ "Var 2", 200 },
	{ "Var 3", 300 },
	{ "Var 4", 400 }
};

TouchpanelData touchpanel;
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart2;
XYZPoint point;
XYZPoint lastpoint;
XYZPoint samples[SAMPLE_BUFFER_SIZE];

LEDConfig leds[192];
uint8_t i2cData[2];
uint32_t sample_end;
uint32_t sample_counter;
uint32_t sample_counter_hires;
uint8_t playback=0;
uint8_t loop = 0;
uint8_t cutin = 0;
uint8_t recording=0;
uint8_t pentouch = 0;
double lastdistance;
double newdistance;
uint32_t tempx_filtered;
uint32_t tempy_filtered;
uint32_t tempx[6];
uint32_t tempy[6];
uint32_t tempz1[6];
uint32_t tempz2[6];

uint8_t constrainmode;

uint32_t sample_x;
uint32_t sample_y;
uint32_t sample_z1;
uint32_t sample_z2;

uint16_t CalX1;
uint16_t CalY1;
uint16_t CalZ1;
uint16_t CalX2;
uint16_t CalY2;
uint16_t CalZ2;
uint16_t CalX3;
uint16_t CalY3;
uint16_t CalZ3;

int32_t rtouch;

uint32_t lastmidipointx;
uint32_t lastmidipointy;
uint32_t lastmidipointz;

uint32_t velocity;
uint32_t lastvelocity;

//uint32_t VirtAddVarTab[NB_OF_VAR] = { 0x111, 0x222, 0x333, 0x555, 0x666, 0x777, 0x888, 0x999, 0xAAA };
uint32_t VirtAddVarTab[NB_OF_VAR] = { 0x1111, 0x2222, 0x3333, 0x5555, 0x6666, 0x7777 };
uint8_t fsync_event = 0;
uint8_t vsync_event = 0;
uint8_t hsync_event = 0;
uint32_t idlecounter = 0;
uint32_t dactestcnt = 0;

POINT perfectDisplaySample[MAX_CAL_SAMPLES] = {
	{ 0, 1023 },
	{ 1023, 0 },
	{ 0, 0 }
	//{ 63, 1023-64 },
	//{ 1023-64, 63 },
	//{ 1023 - 64, 1023 - 64 }
};

POINT perfectScreenSample[MAX_CAL_SAMPLES] = {
	{ perfectDisplaySample[0].x, perfectDisplaySample[0].y },
	{ perfectDisplaySample[1].x, perfectDisplaySample[1].y },
	{ perfectDisplaySample[2].x, perfectDisplaySample[2].y },
};

POINT calDisplaySample[MAX_CAL_SAMPLES] = {
	{ perfectDisplaySample[0].x, perfectDisplaySample[0].y },
	{ perfectDisplaySample[1].x, perfectDisplaySample[1].y },
	{ perfectDisplaySample[2].x, perfectDisplaySample[2].y },
};

POINT calScreenSample[MAX_CAL_SAMPLES] = {
	{ perfectDisplaySample[0].x, perfectDisplaySample[0].y },
	{ perfectDisplaySample[1].x, perfectDisplaySample[1].y },
	{ perfectDisplaySample[2].x, perfectDisplaySample[2].y },
};


MATRIX  calmatrix;


static const uint8_t display_address[4] = { IS32FL3738_ADDRESS_A, IS32FL3738_ADDRESS_B, IS32FL3738_ADDRESS_C, IS32FL3738_ADDRESS_D };

static const uint8_t display_pwm_startreg[48] = { 
	0x00,
	0x20,
	0x40,
	0x60,
	0x80,
	0xA0,
	0x02,
	0x22,
	0x42,
	0x62,
	0x82,
	0xA2,
	0x04,
	0x24,
	0x44,
	0x64,
	0x84,
	0xA4,
	0x06,
	0x26,
	0x46,
	0x66,
	0x86,
	0xA6,
	0x08,
	0x28,
	0x48,
	0x68,
	0x88,
	0xA8,
	0x0A,
	0x2A,
	0x4A,
	0x6A,
	0x8A,
	0xAA,
	0x0C,
	0x2C,
	0x4C,
	0x6C,
	0x8C,
	0xAC,
	0x0E,
	0x2E,
	0x4E,
	0x6E,
	0x8E,
	0xAE
};

static const uint8_t displayledonreg[24] = { 
	0x00,
	0x04,
	0x08,
	0x0C,
	0x10,
	0x14,
	0x01,
	0x05,
	0x09,
	0x0D,
	0x11,
	0x15
};

static const uint8_t gamma8[256] = {
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	2,
	2,
	2,
	2,
	2,
	2,
	2,
	2,
	3,
	3,
	3,
	3,
	3,
	3,
	3,
	4,
	4,
	4,
	4,
	4,
	5,
	5,
	5,
	5,
	6,
	6,
	6,
	6,
	7,
	7,
	7,
	7,
	8,
	8,
	8,
	9,
	9,
	9,
	10,
	10,
	10,
	11,
	11,
	11,
	12,
	12,
	13,
	13,
	13,
	14,
	14,
	15,
	15,
	16,
	16,
	17,
	17,
	18,
	18,
	19,
	19,
	20,
	20,
	21,
	21,
	22,
	22,
	23,
	24,
	24,
	25,
	25,
	26,
	27,
	27,
	28,
	29,
	29,
	30,
	31,
	32,
	32,
	33,
	34,
	35,
	35,
	36,
	37,
	38,
	39,
	39,
	40,
	41,
	42,
	43,
	44,
	45,
	46,
	47,
	48,
	49,
	50,
	50,
	51,
	52,
	54,
	55,
	56,
	57,
	58,
	59,
	60,
	61,
	62,
	63,
	64,
	66,
	67,
	68,
	69,
	70,
	72,
	73,
	74,
	75,
	77,
	78,
	79,
	81,
	82,
	83,
	85,
	86,
	87,
	89,
	90,
	92,
	93,
	95,
	96,
	98,
	99,
	101, 
	102,
	104,
	105,
	107,
	109, 
	110, 
	112,
	114,
	115,
	117, 
	119,
	120, 
	122,
	124,
	126,
	127, 
	129, 
	131, 
	133, 
	135, 
	137,
	138, 
	140, 
	142,
	144, 
	146,
	148, 
	150,
	152,
	154, 
	156,
	158,
	160,
	162,
	164,
	167, 
	169, 
	171, 
	173,
	175,
	177, 
	180, 
	182, 
	184,
	186, 
	189,
	191,
	193,
	196, 
	198, 
	200,
	203,
	205,
	208, 
	210, 
	213,
	215, 
	218,
	220,
	223,
	225,
	228, 
	231,
	233, 
	236,
	239,
	241, 
	244, 
	247, 
	249, 
	252,
	255
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void Display_Init(void);
void Pots_Poll(void);
void Buttons_Poll(void);
void Touchpanel_Poll(void);
void Touchpanel_Calibration_Init(void);
void StartupAnimation(void);
void Display_Refresh(void);
void Gates_Refresh(void);
void DAC_Refresh(void);
void Display_BlinkOn(void);
void Display_BlinkOff(void);
void Display_Crosshair(uint32_t TargetX, uint32_t TargetY, uint32_t Span);
void I2C_WriteRegister(uint32_t address, uint8_t byte1, uint8_t byte2);
void Calculate_Smoothing(void);
void Calculate_Sensitivity(void);
void TVP5150AM1_Setup(void);
void MIDI_CC_HiRes_Transmit(uint8_t channel, uint8_t ctrlchange, uint32_t value);
void MIDI_CC_Transmit(uint8_t channel, uint8_t ctrlchange, uint32_t value);
void MIDI_NoteOn_Transmit(uint8_t channel, uint8_t note, uint8_t velocity);
void MIDI_NoteOff_Transmit(uint8_t channel, uint8_t note, uint8_t velocity);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
	/* USER CODE BEGIN 1 */
	
	



	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	HAL_Delay(200);
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_SPI2_Init();
	MX_USART2_UART_Init();
	MX_USB_DEVICE_Init();


	 /* Initialize interrupts */
            	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */

	//Initialize infinity mirror display
	Display_Init();

	//Unlock flash and initialize EEPROM emulation
	//HAL_FLASH_Unlock(); 
	//uint32_t status;
	//status = EE_Init();

	//Initialize touchpanel calibration
	Touchpanel_Calibration_Init();	

	StartupAnimation();

	touchpanel.x = 0;
	touchpanel.y = 0;
	TVP5150AM1_Setup();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if (fsync_event>=1)
		{
			fsync_event = 0;
			idlecounter = 0;
			if (DEBUG)
			{
				gates[kGate1Out].value = 1;	
			}
			Gates_Refresh();
			
			DAC_Refresh();
			

			//Polling
			Pots_Poll();
			Buttons_Poll();
			Touchpanel_Poll();
			//Calculate_Sensitivity();
			if (buttons[kConstrain].value)
			{
				if (buttons[kConstrain].rising)
				{
					int32_t xdist = (lastpoint.x - touchpanel.x);			
					int32_t ydist = (lastpoint.y - touchpanel.y);		
					if (xdist < 0)
					{
						xdist = -xdist;
					}	
					if (ydist < 0)
					{
						ydist = -ydist;
					}	

					if (xdist >= ydist)
					{
						constrainmode = 1;
					}
					else
					{
						constrainmode = 0;
					}
					buttons[kConstrain].rising = 0;
				}
				if (constrainmode)
				{
					touchpanel.y = lastpoint.y;
				}
				else
				{
					touchpanel.x = lastpoint.x;
				}
			}
		
			//Process button events
			if(buttons[kPlay].rising)
			{
				if (playback)
				{
					playback = 0;
					touchpanel.x = samples[sample_counter].x;
					touchpanel.y = samples[sample_counter].y;
					touchpanel.pressure = samples[sample_counter].z;
				}
				else
				{
					//sample_counter = 0;			
					sample_counter_hires = 0;
					playback = 1;
					recording = 0;	
				
				}
				buttons[kPlay].rising = 0;
			}

			if (buttons[kTriggerIn].rising)
			{
				sample_counter_hires = 0;
				playback = 1;
				recording = 0;	
				buttons[kTriggerIn].rising = 0;
			}

			if (buttons[kLoop].rising)
			{
				if (loop)
				{
					loop = 0;
				}
				else
				{	
					loop = 1;
				}
				buttons[kLoop].rising = 0;
			}

			if (buttons[kRecord].rising)
			{
				if (!playback)
				{
					cutin = 0;
					//sample_counter = 0;	
					sample_counter_hires = 0;
				}
				else
				{
					cutin = 1;
				}
				recording = 1;
				playback = 0;
				buttons[kRecord].rising = 0;
			}
			else if (buttons[kRecord].falling)
			{
				recording = 0;
				if (!cutin)
				{
						
					sample_end = sample_counter - 1;	
					//sample_counter = 0;
					sample_counter_hires = 0;
				}
				else
				{
					playback = 1;
				}
		
				buttons[kRecord].falling = 0;
			}

			//Assign points
			sample_counter = sample_counter_hires >> 11;
			if (playback)
			{
				uint32_t interpolation_coeff = sample_counter_hires % 2048;
				point.x = ((samples[sample_counter].x * (interpolation_coeff)) + (samples[(sample_end + sample_counter - 1) % sample_end].x * (2047 - interpolation_coeff))) >> 11;
				point.y = ((samples[sample_counter].y * (interpolation_coeff)) + (samples[(sample_end + sample_counter - 1) % sample_end].y * (2047 - interpolation_coeff))) >> 11;
				point.z = ((samples[sample_counter].z * (interpolation_coeff)) + (samples[(sample_end + sample_counter - 1) % sample_end].z * (2047 - interpolation_coeff))) >> 11;
				sample_counter_hires += (256 + pots[kPlaybackSpeed].value);
				if (sample_counter_hires > (sample_end << 11))
				{
					if (loop)
					{
						sample_counter_hires = sample_counter_hires % (sample_end << 11);	
					}
					else
					{
						playback = 0;
					}			
				}
			}
			else if (recording && sample_counter < SAMPLE_BUFFER_SIZE)
			{		
				point.x = touchpanel.x;
				point.y = touchpanel.y;
				point.z = touchpanel.pressure;
				samples[sample_counter].x = touchpanel.x;
				samples[sample_counter].y = touchpanel.y;
				samples[sample_counter].z = touchpanel.pressure;
				if (cutin)
				{
					sample_counter_hires += 2048;
					if (sample_counter_hires > (sample_end << 11))
					{
						sample_counter_hires = sample_counter_hires % (sample_end << 11);	
					}
				}
				else
				{
					sample_counter_hires += 2048;
				}
			}
			else
			{
				point.x = touchpanel.x;
				point.y = touchpanel.y;
				point.z = touchpanel.pressure;			
			}
		
			Calculate_Smoothing();

			uint8_t pressuregate = 0;
			if (point.z > pots[kGateThreshold].value)
			{
				pressuregate = 1;
			}

			//Assign outputs

			dacs[kXOut].value = (uint16_t)4095&point.x;
			dacs[kYOut].value = (uint16_t)4095&point.y;

			dacs[kPressureOut].value = (uint16_t)4095&point.z;
			//dacs[kVelocityOut].value =  4095&(uint32_t)lastdistance;
			if(velocity > 4095)
			{
				velocity = 4095;
			}
			dacs[kVelocityOut].value =  (uint32_t)velocity;
			gates[kGate1Out].value = buttons[kGate1].value;
			gates[kGate2Out].value = pressuregate ^ buttons[kGate2].value;
		
			//Refresh
			Display_Crosshair(point.x, point.y, ((point.z*4)>>12)+1);
			Display_Refresh();

			if (point.x != lastmidipointx)
			{
				MIDI_CC_HiRes_Transmit(0, 10, point.x);	
				//MIDI_CC_Transmit(0, 10, point.x);	
				lastmidipointx = point.x;
			}
			if (point.y != lastmidipointy)
			{
				MIDI_CC_HiRes_Transmit(0, 11, point.y);	
				//MIDI_CC_Transmit(0, 11, point.y);	
				lastmidipointy = point.y;
			}
			if (point.z != lastmidipointz)
			{
				MIDI_CC_HiRes_Transmit(0, 12, point.z);	
				//MIDI_CC_Transmit(0, 12, point.z);	
				lastmidipointz = point.z;
			}
		
			if (gates[kGate1Out].value && !gates[kGate1Out].last)
			{
				MIDI_NoteOn_Transmit(0, 63, 0xFF);
			} 
			else if (!gates[kGate1Out].value && gates[kGate1Out].last)
			{
				MIDI_NoteOn_Transmit(0, 63, 0x00);
			}
			if (gates[kGate2Out].value && !gates[kGate2Out].last)
			{
				MIDI_NoteOn_Transmit(0, 64, 0xFF);
			} 
			else if (!gates[kGate2Out].value && gates[kGate2Out].last)
			{
				MIDI_NoteOn_Transmit(0, 64, 0x00);
			}

			gates[kGate1Out].last = gates[kGate1Out].value;
			gates[kGate2Out].last = gates[kGate2Out].value;
			if (DEBUG)
			{
				gates[kGate1Out].value = 0;	
				Gates_Refresh();
			}
			
			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */
		}
	}
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage 
	*/
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 27;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time 
	*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick 
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
	/* EXTI15_10_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	/* OTG_FS_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

/* USER CODE BEGIN 4 */


void TVP5150AM1_Setup(void)
{
	uint8_t decoder_address = 0xBA; //I2CSEL = HIGH
	I2C_WriteRegister(decoder_address, 0x03, 0b00000101); // Enable sync outputs
	I2C_WriteRegister(decoder_address, 0x0F, 0b00000000); // Enable FID output/Disable GLCO output
}

void MIDI_CC_HiRes_Transmit(uint8_t channel, uint8_t ctrlchange, uint32_t value)
{

	uint8_t buffer[] = { 0xB0 | (channel & 0xF), ctrlchange & 127, (value >> 5) & 127, 0xB0 | (channel & 0xF), (ctrlchange+32) & 127, (value << 2) & 127 };
	HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), HAL_MAX_DELAY);
}

void MIDI_CC_Transmit(uint8_t channel, uint8_t ctrlchange, uint32_t value)
{

	uint8_t buffer[] = { 0xB0 | (channel & 0xF), ctrlchange & 127, (value >> 5) & 127 };
	HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), HAL_MAX_DELAY);
}

void MIDI_NoteOn_Transmit(uint8_t channel, uint8_t note, uint8_t velocity)
{
	uint8_t buffer[3] = { (0x90) | (channel & 0xF), note & 127, velocity & 127 };
	HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), HAL_MAX_DELAY);
}
void MIDI_NoteOff_Transmit(uint8_t channel, uint8_t note, uint8_t velocity)
{
	uint8_t buffer[3] = { (0x80) | (channel & 0xF), note & 127, velocity & 127 };
	HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), HAL_MAX_DELAY);
}

void I2C_WriteRegister(uint32_t address, uint8_t byte1, uint8_t byte2)
{
	i2cData[0] = byte1;
	i2cData[1] = byte2;
	HAL_I2C_Master_Transmit(&hi2c1, address, i2cData, 2, 100);	
}

void Touchpanel_Calibration_Init(void)
{

	/*Buttons_Poll();
	if (buttons[kGate1].value && buttons[kGate2].value)
	{
		setCalibrationMatrix(&perfectDisplaySample[0], &perfectScreenSample[0], &calmatrix);
		
		uint8_t lastpentouch = 0;
		uint8_t cal_stage = 0;
		uint32_t increment = 4095 >> 3;

		Display_BlinkOn();
	
		while (1)
		{
			switch (cal_stage)
			{
			
			case 0: Display_Crosshair(perfectDisplaySample[0].x << 2, perfectDisplaySample[0].y << 2, 1); break;
			case 1: Display_Crosshair(perfectDisplaySample[1].x << 2, perfectDisplaySample[1].y << 2, 1); break;
			case 2: Display_Crosshair(perfectDisplaySample[2].x << 2, perfectDisplaySample[2].y << 2, 1); break;
			}
		
			Display_Refresh();
			lastpentouch = pentouch;
			Touchpanel_Poll();
			
			if (pentouch == 0 && lastpentouch == 1)
			{
				cal_stage++;
			} else
			{
				switch (cal_stage)
				{
				case 0:	EE_WriteVariable(VirtAddVarTab[kEEPROM_X1], touchpanel.x);
					EE_WriteVariable(VirtAddVarTab[kEEPROM_Y1], touchpanel.y);
					//EE_WriteVariable(VirtAddVarTab[kEEPROM_Z1], (uint16_t)rtouch);
					break;
				case 1: EE_WriteVariable(VirtAddVarTab[kEEPROM_X2], touchpanel.x);
					EE_WriteVariable(VirtAddVarTab[kEEPROM_Y2], touchpanel.y);
					//EE_WriteVariable(VirtAddVarTab[kEEPROM_Z2], (uint16_t)rtouch);
					
					break;
				case 2: EE_WriteVariable(VirtAddVarTab[kEEPROM_X3], touchpanel.x);
					EE_WriteVariable(VirtAddVarTab[kEEPROM_Y3], touchpanel.y);
					//EE_WriteVariable(VirtAddVarTab[kEEPROM_Z3], (uint16_t)rtouch);
					
					break;
				}
			}

			if (cal_stage > 2)
			{
				Display_BlinkOff();
				break;
			}
		
		}
	}	*/

	//EE_ReadVariable(VirtAddVarTab[kEEPROM_X1], &CalX1);
	//EE_ReadVariable(VirtAddVarTab[kEEPROM_Y1], &CalY1);
	
	//EE_ReadVariable(VirtAddVarTab[kEEPROM_X2], &CalX2);
	//EE_ReadVariable(VirtAddVarTab[kEEPROM_Y2], &CalY2);
	
	//EE_ReadVariable(VirtAddVarTab[kEEPROM_X3], &CalX3);
	//EE_ReadVariable(VirtAddVarTab[kEEPROM_Y3], &CalY3);
	//EE_ReadVariable(VirtAddVarTab[kEEPROM_Z1], &CalZ1);
	//EE_ReadVariable(VirtAddVarTab[kEEPROM_Z2], &CalZ2);
	//EE_ReadVariable(VirtAddVarTab[kEEPROM_Z3], &CalZ3);

	CalX1 = 3800;
	CalY1 = 330;
	CalZ1 = 500;

	CalX2 = 260;
	CalY2 = 3650;
	CalZ2 = 2200;
	
	CalX3 = 3720;
	CalY3 = 3670;
	CalZ3 = 1280;

	calScreenSample[0].x = CalX1 >> 2;
	calScreenSample[0].y = CalY1 >> 2;
	calScreenSample[1].x = CalX2 >> 2;
	calScreenSample[1].y = CalY2 >> 2;
	calScreenSample[2].x = CalX3 >> 2;
	calScreenSample[2].y = CalY3 >> 2;

	setCalibrationMatrix(&perfectDisplaySample[0], &calScreenSample[0], &calmatrix);
}

void Display_Init()
{
	HAL_GPIO_WritePin(GPIOC, LED_SDB_Pin, GPIO_PIN_RESET);	
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOC, LED_SDB_Pin, GPIO_PIN_SET);

	for (int i = 0; i < 4; i++)
	{
		I2C_WriteRegister(display_address[i], 0xFE, 0XC5);   		//Unlock
		I2C_WriteRegister(display_address[i], 0xFD, 0X03);          //PG03 Configuration Registers
		if(i == 3) //Disable Software Shutdown and set to master mode
		{
			I2C_WriteRegister(display_address[i], 0x00, 0b01000011); 
		} else	//Disable Software Shutdown and set to slave mode
		{
			I2C_WriteRegister(display_address[i], 0x00, 0b10000011); 
		}		
		
		I2C_WriteRegister(display_address[i], 0x01, 0x80);          //Global Current Register Set to 42mA.
		I2C_WriteRegister(display_address[i], 0x0F, 0x07);          //Enable De-ghosting 32k SWy Pullup
		I2C_WriteRegister(display_address[i], 0x10, 0x07);          //Enable De-ghosting 32k CSx Pulldown
		I2C_WriteRegister(display_address[i], 0xFE, 0XC5);          //Unlock
		I2C_WriteRegister(display_address[i], 0xFD, 0X00);          //PG00 LED on/off
		for(int j = 0 ; j < 24 ; j++)
		{
			I2C_WriteRegister(display_address[i], j, 0x00);             	//Turn all LEDs on
		}	

		for (int j = 0; j < 192; j++)
		{
			leds[j].brightness = 0;	
			leds[j].lastBrightness = 0;			
		}
	}
	uint8_t curLED = 0;
	for (int i = 0; i < 4; i++)
	{
		I2C_WriteRegister(display_address[i], 0xFE, 0XC5);           		//Unlock
		I2C_WriteRegister(display_address[i], 0xFD, 0X01);              	//PG01 LED PWM	
		for(int k = 0 ; k < 48 ; k++)
		{
			curLED = (48 * i) + k;
			I2C_WriteRegister(display_address[i], display_pwm_startreg[k], 255);
			I2C_WriteRegister(display_address[i], display_pwm_startreg[k] + 1, 255);
			I2C_WriteRegister(display_address[i], display_pwm_startreg[k] + 16, 255);
			I2C_WriteRegister(display_address[i], display_pwm_startreg[k] + 17, 255);		
		}		
		//if(i == 3) //Disable Software Shutdown and set to master mode
		//{
		//	I2C_WriteRegister(display_address[i], 0x00, 0b01000001); 
		//	I2C_WriteRegister(display_address[i], 0x00, 0b01000011); 
		//} else	//Disable Software Shutdown and set to slave mode
		//{
		//	I2C_WriteRegister(display_address[i], 0x00, 0b10000001); 
		//	I2C_WriteRegister(display_address[i], 0x00, 0b10000011); 
		//}		
		//I2C_WriteRegister(display_address[i], 0x0E, 0x00);       //Update timing registers
	}
}

void Display_BlinkOn(void)
{
	for (uint32_t i = 0; i < 4; i++)
	{
		I2C_WriteRegister(display_address[i], 0xFE, 0xC5);             		//Unlock
		I2C_WriteRegister(display_address[i], 0xFD, 0x02);                	//PG02 ABM
		for(int k = 0 ; k < 192 ; k++)
		{
			I2C_WriteRegister(display_address[i], k, 0x01); 	//ABM-1
		}
		I2C_WriteRegister(display_address[i], 0xFE, 0xC5);     		//Unlock
		I2C_WriteRegister(display_address[i], 0xFD, 0x03);            //PG03 Configuration Registers
		I2C_WriteRegister(display_address[i], 0x02, 0b00100100);      //T1/T2 ABM-1 times    
		I2C_WriteRegister(display_address[i], 0x03, 0b00100100);       //T3/T4 ABM-1 times 
		I2C_WriteRegister(display_address[i], 0x04, 0b01110001);        //Timing modes
		I2C_WriteRegister(display_address[i], 0x05, 0b00000001);        //Loop time
	}
}

void Display_BlinkOff(void)
{
	for (uint32_t i = 0; i < 4; i++)
	{
		I2C_WriteRegister(display_address[i], 0xFE, 0xC5);              		//Unlock
		I2C_WriteRegister(display_address[i], 0xFD, 0x02);                 	//PG02 ABM
		for(int k = 0 ; k < 192 ; k++)
		{
			I2C_WriteRegister(display_address[i], k, 0x00);  	//ABM-1
		}
	}
}

void Display_Crosshair(uint32_t TargetX, uint32_t TargetY, uint32_t Span)
{
	for (int i = 0; i < 192; i++)
	{
		leds[i].brightness = 0;
	}
	/*uint32_t XStart = ((TargetX * DISPLAY_WIDTH) >> 12) - (Span / 2);
	uint32_t YStart = ((TargetY * DISPLAY_HEIGHT) >> 12) - (Span / 2);
	uint32_t XStep = (4095 / DISPLAY_WIDTH);
	uint32_t YStep = (4095 / DISPLAY_HEIGHT);
	uint32_t XScale = XStep * (Span / 2);
	uint32_t YScale = YStep * (Span / 2);
	for (int i = 0; i < Span; i++)
	{
		uint32_t CurX = (XStart + i) * XStep;
		uint32_t difference;
		if (CurX > TargetX)
		{
			difference = (CurX - TargetX);
		}
		else
		{
			difference = (TargetX - CurX);
		}
		if (difference >= 127)
		{
			leds[XStart + i].brightness = 0;
		}
		else
		{
			leds[XStart + i].brightness = 127 - difference;
		}
		uint32_t CurY = (YStart + i) * YStep;
		if (CurY > TargetY)
		{
			difference = (CurY - TargetY);
		}
		else
		{
			difference = (TargetY - CurY);
		}
		if (difference >= 127)
		{
			leds[191 - (YStart + i)].brightness = 0;
		}
		else
		{
			leds[191 - (YStart + i)].brightness = 127 - difference;
		}		
	}

	for (int i = 0; i < (192 - DISPLAY_WIDTH - DISPLAY_HEIGHT); i++)
	{
		leds[i + DISPLAY_WIDTH].brightness = 0;
	}
	*/
	const static uint8_t MarginX = 10;
	const static uint8_t MarginY = 10;
	uint32_t CenterX = ((TargetX*(DISPLAY_WIDTH - MarginX)) >> 12)+(MarginX/2);
	uint32_t CenterY = (191 - ((TargetY*(DISPLAY_HEIGHT - MarginY)) >> 12))-(MarginY/2);

	switch (Span)
	{
	case 5:	leds[CenterX + 4].brightness = 255;
			leds[CenterX - 4].brightness = 255;
			leds[CenterY + 4].brightness = 255;
			leds[CenterY - 4].brightness = 255;
	case 4: leds[CenterX + 3].brightness = 255;
			leds[CenterX - 3].brightness = 255;
			leds[CenterY + 3].brightness = 255;
			leds[CenterY - 3].brightness = 255;
	case 3: leds[CenterX + 2].brightness = 255;
			leds[CenterX - 2].brightness = 255;
			leds[CenterY + 2].brightness = 255;
			leds[CenterY - 2].brightness = 255;
	case 2:	leds[CenterX+1].brightness = 255;
			leds[CenterX-1].brightness = 255;
			leds[CenterY+1].brightness = 255;
			leds[CenterY-1].brightness = 255;
	case 1:	leds[CenterX].brightness = 255;
			leds[CenterY].brightness = 255;
		break;
	}

	leds[CenterX].brightness = 255;
	leds[CenterY].brightness = 255;

}

void Display_Refresh()
{
	if (playback)
	{
		leds[96].brightness = 255;
	}
	else
	{
		leds[96].brightness = 0;
	}

	if (loop)
	{
		leds[97].brightness = 255;
	}
	else
	{
		leds[97].brightness = 0;
	}
	uint8_t curLED = 0;
	for (int i = 0; i < 4; i++)
	{

		uint8_t reg[24] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

		for (int k = 0; k < 48; k++)
		{
			curLED = (48 * i) + k;

			if (leds[curLED].brightness > 0)
			{
				switch (k)
				{
				case 0: reg[0x00] = reg[0x00] | 0b00000011;
						reg[0x02] = reg[0x02] | 0b00000011;					
						break;
				case 1: reg[0x04] = reg[0x04] | 0b00000011;
						reg[0x06] = reg[0x06] | 0b00000011;					
						break;
				case 2: reg[0x08] = reg[0x08] | 0b00000011;
						reg[0x0A] = reg[0x0A] | 0b00000011;					
						break;
				case 3: reg[0x0C] = reg[0x0C] | 0b00000011;
						reg[0x0E] = reg[0x0E] | 0b00000011;					
						break;
				case 4: reg[0x10] = reg[0x10] | 0b00000011;
						reg[0x12] = reg[0x12] | 0b00000011;					
						break;
				case 5: reg[0x14] = reg[0x14] | 0b00000011;
						reg[0x16] = reg[0x16] | 0b00000011;					
						break;

				case 6: reg[0x00] = reg[0x00] | 0b00001100;
					reg[0x02] = reg[0x02] | 0b00001100;					
					break;
				case 7: reg[0x04] = reg[0x04] | 0b00001100;
					reg[0x06] = reg[0x06] | 0b00001100;					
					break;
				case 8: reg[0x08] = reg[0x08] | 0b00001100;
					reg[0x0A] = reg[0x0A] | 0b00001100;					
					break;
				case 9: reg[0x0C] = reg[0x0C] | 0b00001100;
					reg[0x0E] = reg[0x0E] | 0b00001100;					
					break;
				case 10: reg[0x10] = reg[0x10] | 0b00001100;
					reg[0x12] = reg[0x12] | 0b00001100;					
					break;
				case 11: reg[0x14] = reg[0x14] | 0b00001100;
					reg[0x16] = reg[0x16] | 0b00001100;					
					break;

				case 12: reg[0x00] = reg[0x00] | 0b00110000;
					reg[0x02] = reg[0x02] | 0b00110000;					
					break;
				case 13: reg[0x04] = reg[0x04] | 0b00110000;
					reg[0x06] = reg[0x06] | 0b00110000;					
					break;
				case 14: reg[0x08] = reg[0x08] | 0b00110000;
					reg[0x0A] = reg[0x0A] | 0b00110000;					
					break;
				case 15: reg[0x0C] = reg[0x0C] | 0b00110000;
					reg[0x0E] = reg[0x0E] | 0b00110000;					
					break;
				case 16: reg[0x10] = reg[0x10] | 0b00110000;
					reg[0x12] = reg[0x12] | 0b00110000;					
					break;
				case 17: reg[0x14] = reg[0x14] | 0b00110000;
					reg[0x16] = reg[0x16] | 0b00110000;					
					break;

				case 18: reg[0x00] = reg[0x00] | 0b11000000;
					reg[0x02] = reg[0x02] | 0b11000000;					
					break;
				case 19: reg[0x04] = reg[0x04] | 0b11000000;
					reg[0x06] = reg[0x06] | 0b11000000;					
					break;
				case 20: reg[0x08] = reg[0x08] | 0b11000000;
					reg[0x0A] = reg[0x0A] | 0b11000000;					
					break;
				case 21: reg[0x0C] = reg[0x0C] | 0b11000000;
					reg[0x0E] = reg[0x0E] | 0b11000000;					
					break;
				case 22: reg[0x10] = reg[0x10] | 0b11000000;
					reg[0x12] = reg[0x12] | 0b11000000;					
					break;
				case 23: reg[0x14] = reg[0x14] | 0b11000000;
					reg[0x16] = reg[0x16] | 0b11000000;					
					break;

				case 24: reg[0x01] = reg[0x01] | 0b00000011;
					reg[0x03] = reg[0x03] | 0b00000011;					
					break;
				case 25: reg[0x05] = reg[0x05] | 0b00000011;
					reg[0x07] = reg[0x07] | 0b00000011;					
					break;
				case 26: reg[0x09] = reg[0x09] | 0b00000011;
					reg[0x0B] = reg[0x0B] | 0b00000011;					
					break;
				case 27: reg[0x0D] = reg[0x0D] | 0b00000011;
					reg[0x0F] = reg[0x0F] | 0b00000011;					
					break;
				case 28: reg[0x11] = reg[0x11] | 0b00000011;
					reg[0x13] = reg[0x13] | 0b00000011;					
					break;
				case 29: reg[0x15] = reg[0x15] | 0b00000011;
					reg[0x17] = reg[0x17] | 0b00000011;					
					break;

				case 30: reg[0x01] = reg[0x01] | 0b00001100;
					reg[0x03] = reg[0x03] | 0b00001100;					
					break;
				case 31: reg[0x05] = reg[0x05] | 0b00001100;
					reg[0x07] = reg[0x07] | 0b00001100;					
					break;
				case 32: reg[0x09] = reg[0x09] | 0b00001100;
					reg[0x0B] = reg[0x0B] | 0b00001100;					
					break;
				case 33: reg[0x0D] = reg[0x0D] | 0b00001100;
					reg[0x0F] = reg[0x0F] | 0b00001100;					
					break;
				case 34: reg[0x11] = reg[0x11] | 0b00001100;
					reg[0x13] = reg[0x13] | 0b00001100;					
					break;
				case 35: reg[0x15] = reg[0x15] | 0b00001100;
					reg[0x17] = reg[0x17] | 0b00001100;					
					break;

				case 36: reg[0x01] = reg[0x01] | 0b00110000;
					reg[0x03] = reg[0x03] | 0b00110000;					
					break;
				case 37: reg[0x05] = reg[0x05] | 0b00110000;
					reg[0x07] = reg[0x07] | 0b00110000;					
					break;
				case 38: reg[0x09] = reg[0x09] | 0b00110000;
					reg[0x0B] = reg[0x0B] | 0b00110000;					
					break;
				case 39: reg[0x0D] = reg[0x0D] | 0b00110000;
					reg[0x0F] = reg[0x0F] | 0b00110000;					
					break;
				case 40: reg[0x11] = reg[0x11] | 0b00110000;
					reg[0x13] = reg[0x13] | 0b00110000;					
					break;
				case 41: reg[0x15] = reg[0x15] | 0b00110000;
					reg[0x17] = reg[0x17] | 0b00110000;					
					break;

				case 42: reg[0x01] = reg[0x01] | 0b11000000;
					reg[0x03] = reg[0x03] | 0b11000000;					
					break;
				case 43: reg[0x05] = reg[0x05] | 0b11000000;
					reg[0x07] = reg[0x07] | 0b11000000;					
					break;
				case 44: reg[0x09] = reg[0x09] | 0b11000000;
					reg[0x0B] = reg[0x0B] | 0b11000000;					
					break;
				case 45: reg[0x0D] = reg[0x0D] | 0b11000000;
					reg[0x0F] = reg[0x0F] | 0b11000000;					
					break;
				case 46: reg[0x11] = reg[0x11] | 0b11000000;
					reg[0x13] = reg[0x13] | 0b11000000;					
					break;
				case 47: reg[0x15] = reg[0x15] | 0b11000000;
					reg[0x17] = reg[0x17] | 0b11000000;					
					break;
				}
			}

			//if (leds[curLED].brightness != leds[curLED].lastBrightness)
			//{
			//
			//	I2C_WriteRegister(display_address[i], display_pwm_startreg[k], gamma8[leds[curLED].brightness]);
			//	I2C_WriteRegister(display_address[i], display_pwm_startreg[k] + 1, gamma8[leds[curLED].brightness]);
			//	I2C_WriteRegister(display_address[i], display_pwm_startreg[k] + 16, gamma8[leds[curLED].brightness]);
			//	I2C_WriteRegister(display_address[i], display_pwm_startreg[k] + 17, gamma8[leds[curLED].brightness]);		
		
			//	leds[curLED].lastBrightness = leds[curLED].brightness;		
			//}		
		}	

		I2C_WriteRegister(display_address[i], 0xFE, 0XC5);           		//Unlock
		I2C_WriteRegister(display_address[i], 0xFD, 0X00);              	//PG00 LED ON/OFF
		for (int k = 0; k < 24; k++)
		{
			I2C_WriteRegister(display_address[i], k, reg[k]);
		}
	}
}

void Calculate_Smoothing()
{
	//Calculate distance and angle to new point
	double distance = sqrt(pow((double)lastpoint.x - (double)point.x, 2) + pow((double)lastpoint.y - (double)point.y, 2));         //Distance
	double ang = atan2((double)lastpoint.y - (double)point.y, (double)lastpoint.x - (double)point.x);       	// Angle in radians

	//Filter distance travelled with smoothing pot position
	//double coeff = (((double)pots[kSmoothing].value * 2) - (((double)pots[kSmoothing].value*(double)pots[kSmoothing].value) / (double)4095)) / (double)4095;
	if(pots[kSmoothing].value > (4095 - 128))
	{
		pots[kSmoothing].value = (4095 - 128);
	}
	double coeff = (double)pots[kSmoothing].value / (double)4095;
	//double newdist = ((double)lastdistance*(coeff)) + ((double)distance*(1 - (double)coeff));
	double newdist = ((double)distance*(1 - (double)coeff));
	
	//Move point along angle and distance
	uint32_t newx = (uint32_t)((double)lastpoint.x - (cos(ang) * (double)newdist));
	uint32_t newy = (uint32_t)((double)lastpoint.y - (sin(ang) * (double)newdist));
	uint32_t newz = ((double)point.z*(1 - (double)coeff)) + ((double)lastpoint.z*((double)coeff));

	velocity = distance*2;

	uint32_t newv = ((double)velocity*(1 - (double)coeff)) + ((double)lastvelocity*((double)coeff));

	point.x = newx;
	point.y = newy;
	point.z = newz;
	velocity = newv;

	//Store for next loop
	lastdistance = newdist;
	lastvelocity = velocity;
	lastpoint.x = point.x;
	lastpoint.y = point.y;
	lastpoint.z = point.z;
}

void Calculate_Sensitivity()
{
	float coeff = (float)pots[kSensitivity].value / 4096;
	float z1 = ((float)touchpanel.pressure*(float)touchpanel.pressure) / 4096;
	float z2 = ((float)touchpanel.pressure * 2) - z1;

	float z3 = ((float)z2*(float)z2) / 4096;
	float z4 = ((float)z2 * 2) - z3;

	float z5 = (z1*coeff) + (z2*(1 - coeff));
	touchpanel.pressure = (uint32_t)z5;
}

void Gates_Refresh()
{
	//Write current states to output pins
	if(gates[kGate1Out].value)	{HAL_GPIO_WritePin(GATE2_OUT_GPIO_Port, GATE2_OUT_Pin, GPIO_PIN_SET); }	
	else { HAL_GPIO_WritePin(GATE2_OUT_GPIO_Port, GATE2_OUT_Pin, GPIO_PIN_RESET); }
	if (gates[kGate2Out].value)	{HAL_GPIO_WritePin(GATE1_OUT_GPIO_Port, GATE1_OUT_Pin, GPIO_PIN_SET); }	
	else { HAL_GPIO_WritePin(GATE1_OUT_GPIO_Port, GATE1_OUT_Pin, GPIO_PIN_RESET); }
}

void DAC_Refresh()
{
	HAL_GPIO_WritePin(DAC_SHDN_GPIO_Port, DAC_SHDN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DAC_LDAC_GPIO_Port, DAC_LDAC_Pin, GPIO_PIN_SET);
	uint8_t dacout[2];
	uint8_t mask;
	for (uint32_t i = 0; i < NUM_DACS; i++)
	{
		switch (i)
		{
		case 0 : mask = 0b00110000; HAL_GPIO_WritePin(SPI_CS1_GPIO_Port, SPI_CS1_Pin, GPIO_PIN_RESET); break;
		case 1 : mask = 0b10110000; HAL_GPIO_WritePin(SPI_CS1_GPIO_Port, SPI_CS1_Pin, GPIO_PIN_RESET); break;
		case 2 : mask = 0b00110000; HAL_GPIO_WritePin(SPI_CS2_GPIO_Port, SPI_CS2_Pin, GPIO_PIN_RESET); break;
		case 3 : mask = 0b10110000; HAL_GPIO_WritePin(SPI_CS2_GPIO_Port, SPI_CS2_Pin, GPIO_PIN_RESET); break;	
		}
		

		dacout[0] = (uint8_t)(mask | ((dacs[i].value >> 8)));
		dacout[1] = (uint8_t)(dacs[i].value);

		HAL_SPI_Transmit(&hspi2, dacout, 2, 100);


		switch (i)
		{
		case 0 : HAL_GPIO_WritePin(SPI_CS1_GPIO_Port, SPI_CS1_Pin, GPIO_PIN_SET); break;
		case 1 : HAL_GPIO_WritePin(SPI_CS1_GPIO_Port, SPI_CS1_Pin, GPIO_PIN_SET); break;
		case 2 : HAL_GPIO_WritePin(SPI_CS2_GPIO_Port, SPI_CS2_Pin, GPIO_PIN_SET); break;
		case 3 : HAL_GPIO_WritePin(SPI_CS2_GPIO_Port, SPI_CS2_Pin, GPIO_PIN_SET); break;
		}

	}

	HAL_GPIO_WritePin(DAC_LDAC_GPIO_Port, DAC_LDAC_Pin, GPIO_PIN_RESET);
}

void Pots_Poll(void)
{
	HAL_ADC_Start(&hadc1);
	for (uint32_t i = 0; i < NUM_POTENTIOMETERS; i++)
	{
		HAL_ADC_PollForConversion(&hadc1, 10);
		pots[i].value = ((4095 - ((HAL_ADC_GetValue(&hadc1)*POTENTIOMETER_ADC_MAX) >> 11)) + pots[i].value) >> 1;	
	}	
	HAL_ADC_Stop(&hadc1);
}

void Buttons_Poll(void)
{
	//Store previous values
	for(uint32_t i = 0 ; i < NUM_BUTTONS ; i++)
	{
		buttons[i].last = buttons[i].value;
	}

	//Read new values
	if(HAL_GPIO_ReadPin(SW0_GPIO_Port, SW0_Pin))  { buttons[kRecord].value = 0; }
	else {	buttons[kRecord].value = 1; }
	if (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin))  { buttons[kGate1].value = 0; }
	else {	buttons[kGate1].value = 1; }
	if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin))  { buttons[kConstrain].value = 0; }
	else {	buttons[kConstrain].value = 1; }
	if (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin))  { buttons[kGate2].value = 0; }
	else {	buttons[kGate2].value = 1; }
	if (HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin))  { buttons[kPlay].value = 0; }
	else {	buttons[kPlay].value = 1; }
	if (HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin))  { buttons[kLoop].value = 0; }
	else {	buttons[kLoop].value = 1; }
	if (HAL_GPIO_ReadPin(TRIG_IN_GPIO_Port, TRIG_IN_Pin))  { buttons[kTriggerIn].value = 1; }
	else {	buttons[kTriggerIn].value = 0; }

	//Rising and falling edge flag detection
	for(uint32_t i = 0 ; i < NUM_BUTTONS ; i++)
	{
		if (!buttons[i].last && buttons[i].value)  { buttons[i].rising = 1; }	
		else if (buttons[i].last && !buttons[i].value)  { buttons[i].falling = 1; }	
	}

}

void StartupAnimation(void)
{

	for (uint32_t i = 0; i < ((DISPLAY_WIDTH+DISPLAY_HEIGHT)/2); i++)
	{

		if (i < (DISPLAY_WIDTH/2))
		{
			leds[i].brightness = 255;	
			leds[DISPLAY_WIDTH-i].brightness = 255;	
		}
		else
		{
			leds[191-((i-(DISPLAY_WIDTH/2)))].brightness = 255;	
			leds[(191 - DISPLAY_HEIGHT) + (i - (DISPLAY_WIDTH/2))].brightness = 255;	
		}
		Display_Refresh();		
	}
	for (uint32_t i = 0; i < ((DISPLAY_WIDTH + DISPLAY_HEIGHT) / 2); i++)
	{

		if (i < (DISPLAY_WIDTH / 2))
		{
			leds[i].brightness = 0;	
			leds[DISPLAY_WIDTH - i].brightness = 0;	
		}
		else
		{
			leds[191 - ((i - (DISPLAY_WIDTH / 2)))].brightness = 0;	
			leds[(191 - DISPLAY_HEIGHT) + (i - (DISPLAY_WIDTH / 2))].brightness = 0;	
		}

		Display_Refresh();		
	}
	Display_Crosshair(0, 0, 1);
	Display_Refresh();	
}

void Touchpanel_Poll(void)
{
	
	
	uint8_t cmd[1] = { 0 };
	uint8_t data[2] = { 0, 0 };
	

	for (uint32_t j = 0; j < 4; j++)
	{
		for (uint32_t i = 0; i < 1; i++)
		{
			switch (j)
			{
			case 0: cmd[0] = { 0b11010001 }; break;
			case 1: cmd[0] = { 0b10010001 }; break;
			case 2: cmd[0] = { 0b10110001 }; break;
			case 3: cmd[0] = { 0b11000001 }; break;
			}

			HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2, cmd, 1, 100);
			while (HAL_GPIO_ReadPin(TP_BUSY_GPIO_Port, TP_BUSY_Pin)) {}
			HAL_Delay(5);
			HAL_SPI_Receive(&hspi2, data, 2, 100);
			HAL_GPIO_WritePin(SPI_CS0_GPIO_Port, SPI_CS0_Pin, GPIO_PIN_SET);
			HAL_Delay(1);	

			switch (j)
			{
			case 0: tempx[i] = (data[0] << 5) | (data[1] >> 3); break;
			case 1: tempy[i] = (data[0] << 5) | (data[1] >> 3); break;
			case 2 : tempz1[i] = (data[0] << 5) | (data[1] >> 3); break;
			case 3 : tempz2[i] = (data[0] << 5) | (data[1] >> 3); break;
			}
		}
	}

	sample_x = tempy[0];
	sample_y = tempx[0];
	sample_z1 = tempz1[0];
	sample_z2 = tempz2[0];

	//rtouch = (4095 - (sample_z2 - sample_z1) - (pots[kSensitivity].value>>1));
	rtouch = (4095 - (sample_z2 - sample_z1) - 256);





	POINT display;
	POINT screenSample;
	screenSample.x = (sample_x >> 2) & 1023;
	screenSample.y = (sample_y >> 2) & 1023;

	getDisplayPoint(&display, &screenSample, &calmatrix);

	if (display.x > 1023)
	{
		display.x = 1023;
	}
	if (display.y > 1023)
	{
		display.y = 1023;
	}	
	if (display.x < 0)
	{
		display.x = 0;
	}
	if (display.y < 0)
	{
		display.y = 0;
	}			

	double PressureScaleY = 4096.0 / (double)CalZ1;
	double PressureScaleX = 4096.0 / (double)CalZ2;
	double PressureBaseScale = 4096.0 / (double)CalZ3;

	double ScaleX = (double)display.x / 1024.0;
	double ScaleY = (double)display.y / 1024.0;

	double pressureX = (double)rtouch*(double)PressureScaleX;
	double pressureY = (double)rtouch*(double)PressureScaleY;
	double pressureBase = (double)rtouch*(double)PressureBaseScale;

	double YCorrected = ((1 - ScaleY)*pressureBase) + ((ScaleY)*pressureY);
	double XCorrected = ((1 - ScaleX)*YCorrected) + ((ScaleX)*pressureX);
	XCorrected = XCorrected - (4095-pots[kSensitivity].value);

	if (XCorrected > 4095)
	{
		XCorrected = 4095;
	}
	
	if (XCorrected > 0)
	{
		pentouch = 1;
	}
	else
	{
		pentouch = 0;
	}
	
	

	if(pentouch && sample_x<CalX3 && sample_x > CalX2 && sample_y<CalY3 && sample_y>CalY1)
	{
		pentouch = 1;
		touchpanel.x = (uint32_t)(display.x << 2);
		touchpanel.y = (uint32_t)(display.y << 2);
		touchpanel.pressure = (uint32_t)XCorrected;
	}  else {
		pentouch = 0;
		touchpanel.pressure = 0;
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	 /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
