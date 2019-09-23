/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fatfs.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

	#include <stdio.h>
	#include <string.h>
	#include "lcd.h"
	#include "ringbuffer_dma.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

	//***********************************************************

	typedef enum
	{
		R_FAILED,
		R_OK
	} RESULT_ENUM;

	//***********************************************************
	typedef enum
	{
		SM_START					,
		SM_READ_FROM_RINGBUFFER		,
		SM_CHECK_FLAGS				,
		SM_FIND_GGA					,
		SM_FIND_ASTERISK			,
		SM_CALC_SHECKSUM			,
		SM_GET_TIME_FROM_GGA		,
		SM_FORCE_COPY_GGA			,
		SM_CORRECT_COPY_GGA			,
		SM_TIME_INCREMENT			,
		SM_PREPARE_FILENAME			,
		SM_WRITE_SDCARD				,
		SM_PRINT					,
		SM_FINISH					,
		SM_ERROR_HANDLER			,
		SM_SHUTDOWN					,
		SM_READ_FROM_SDCARD
	} GPS_state_machine;

	//***********************************************************

	#define NEO6_LENGTH_MIN			160
	#define	NEO6_MAX_LENGTH			650
	#define MAX_CHAR_IN_NEO6		630

	#define GGA_STRING_MAX_SIZE 	99
	#define GGA_FORCE_START			103
	#define GGA_FORCE_LENGTH		75
	#define GGA_LENGTH_MIN			33

	#define FILE_NAME_SIZE 			25
	#define RX_BUFFER_SIZE 			550

	#define	TIMEZONE				3
	#define	DEBUG_STRING_SIZE		650

	//***********************************************************

	typedef struct
	{
		int 		hour_int	;
		int 		minutes_int	;
		int 		seconds_int	;
		uint8_t		updated_flag;
	} 	Time_struct;

	//***********************************************************

	typedef struct
	{
		char		string[GGA_STRING_MAX_SIZE];
		int			Neo6_start;
		int			Neo6_end;
		int			length;
		uint8_t 	beginning_chars;
		uint8_t 	ending_char;
	}	GGA_struct;

	//***********************************************************

	typedef struct
	{
		char 		string[NEO6_MAX_LENGTH];
		int 		length_int;
	}	NEO6_struct;

	//***********************************************************

	typedef struct
	{
		uint8_t 	calc_u8				;
		uint8_t 	glue_u8				;
		uint8_t 	status_flag			;
	}	CheckSum_struct;

	//***********************************************************

	typedef struct
	{
		TCHAR 		filename[FILE_NAME_SIZE];
		FRESULT 	write_status;
	}	SD_Card_struct;

	//***********************************************************

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

	volatile uint8_t shudown_button_pressed_flag	= 0 ;
	volatile uint8_t time_write_to_SD_flag			= 0 ;
	volatile uint8_t time_read_from_SD_u8			= 0 ;

	extern DMA_HandleTypeDef hdma_usart3_rx;
	RingBuffer_DMA rx_buffer;
	uint8_t rx_circular_buffer[RX_BUFFER_SIZE];

	char DebugString[DEBUG_STRING_SIZE];

	FRESULT fres;
	RESULT_ENUM result;
	GPS_state_machine sm_stage = SM_START;

	Time_struct Time;
	GGA_struct GGA;
	NEO6_struct NEO6;
	CheckSum_struct CS;
	SD_Card_struct SD;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

	void Increment_time(Time_struct *);
	void Get_time_from_GGA_string(GGA_struct* , Time_struct *);
	uint8_t Find_Begin_of_GGA_string(NEO6_struct*, GGA_struct* );
	uint8_t Find_End_of_GGA_string(NEO6_struct*, GGA_struct*);
	uint8_t Calc_SheckSum_GGA(GGA_struct * _gga, CheckSum_struct * _cs);
	void Write_SD_card(GGA_struct * _gga, SD_Card_struct * _sd);
	void ShutDown(void);
	void Beep(void);
	void Force_copy_GGA(NEO6_struct * _neo6, GGA_struct * _gga);
	void Correct_copy_GGA(NEO6_struct * _neo6, GGA_struct * _gga);
	void Print(NEO6_struct * _neo6, GGA_struct * _gga, CheckSum_struct * _cs, Time_struct * _time, SD_Card_struct * _sd);
	void Prepere_filename(CheckSum_struct * _cs, Time_struct * _time, SD_Card_struct * _sd);
	void Clear_variables(NEO6_struct * _neo6, GGA_struct * _gga, CheckSum_struct * _cs);
	void Read_SD_card(void);
	void TIM_Start(void);
	void TIM_Stop(void);
	void Read_from_RingBuffer(NEO6_struct * _neo6, RingBuffer_DMA * buffer);
	void NAUR_Init (void);
	void NAUR_Main (void);

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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  MX_UART5_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  	  NAUR_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  NAUR_Main();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }	// end main while
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  /** Initializes the CPU, AHB and APB busses clocks 
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

/* USER CODE BEGIN 4 */

//***********************************************************

//***********************************************************

void Increment_time(Time_struct * _time)
{
	if (_time->seconds_int < 59)
	{
		_time->seconds_int++;
		return;
	}

	_time->seconds_int = 0;
	if (_time->minutes_int < 59)
	{
		_time->minutes_int++;
		return;
	}

	_time->minutes_int = 0;
	_time->hour_int++;
}

//***********************************************************

void Get_time_from_GGA_string(GGA_struct * _gga, Time_struct * _time)
{
	if (_time->updated_flag == 1)
	{
		return;
	}

	_time->updated_flag = 1;

	#define TIME_SIZE	5
	char time_string[TIME_SIZE];

//	for (int i=0; i<TIME_SIZE; i++)
//	{
//		time_string[i] = 0x00;
//	}

	memset(time_string,0,TIME_SIZE);

	memcpy(time_string, &_gga->string[7], 2);
	_time->hour_int = atoi(time_string) + TIMEZONE;

	memcpy(time_string, &_gga->string[9], 2);
	_time->minutes_int = atoi(time_string);

	memcpy(time_string, &_gga->string[11], 2);
	_time->seconds_int = atoi(time_string);
}

//***********************************************************

uint8_t Find_Begin_of_GGA_string(NEO6_struct* _neo6, GGA_struct* _gga )
{
	for (int i=6; i<= _neo6->length_int; i++)
	{
		if (memcmp(&_neo6->string[i-6], "$GPGGA," ,7) == 0)
		{
			_gga->Neo6_start = i - 6;
			_gga->beginning_chars = 1;
			return 1;
		}
	}
	return 0;
}

//***********************************************************

uint8_t Find_End_of_GGA_string(NEO6_struct* _neo6, GGA_struct* _gga)
{
	if (_gga->beginning_chars == 0)
	{
		return 0;
	}

	for (int i=_gga->Neo6_start; i<=_neo6->length_int; i++)
	{
		if (_neo6->string[i] == '*')
		{
			_gga->Neo6_end = i + 5;
			_gga->length = _gga->Neo6_end - _gga->Neo6_start;
			if ((_gga->length < GGA_LENGTH_MIN) || (_gga->length > GGA_STRING_MAX_SIZE))
			{
				return 0;
			}
			_gga->ending_char = 1;
			return 1;
		}
	}
	return 0;
}

//***********************************************************

uint8_t Calc_SheckSum_GGA(GGA_struct * _gga, CheckSum_struct * _cs)
{
	#define CS_SIZE	4
	char cs_glue_string[CS_SIZE];

	//	glue:
//	for (int i=0; i<CS_SIZE; i++)
//	{
//		cs_glue_string[i] = 0x00;
//	}
	memset(cs_glue_string,0,CS_SIZE);
	memcpy(cs_glue_string, &_gga->string[GGA.length - 4], 2);
	_cs->glue_u8 = strtol(cs_glue_string, NULL, 16);

	//	calc:
	_cs->calc_u8 = _gga->string[1];
	for (int i=2; i<(_gga->length - 5); i++)
	{
		_cs->calc_u8 ^= _gga->string[i];
	}

	//	compare:
	if (_cs->calc_u8 == _cs->glue_u8)
	{
		_cs->status_flag = 1;
		return 1;
	}
	return 0;
}

//***********************************************************

void ShutDown(void)
{
	LCD_FillScreen(0x0000);
	LCD_SetCursor(0, 0);
	for (int i=5; i>=0; i--)
	{
		sprintf(DebugString,"SHUTDOWN %d\r\n", i);
		HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
		LCD_Printf("%s",DebugString);
		HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_RESET);
		HAL_Delay(800);
		HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET);
		HAL_Delay(300);
		HAL_IWDG_Refresh(&hiwdg);
	}
	LCD_FillScreen(0x0000);
	LCD_SetCursor(0, 0);
}

//***********************************************************

void Write_SD_card(GGA_struct * _gga, SD_Card_struct * _sd)
{
	fres = f_open(&USERFile, _sd->filename, FA_OPEN_APPEND | FA_WRITE );			/* Try to open file */
	_sd->write_status = fres;
	if (fres == FR_OK)
	{
		f_printf(&USERFile, "%s", _gga->string);	/* Write to file */
		f_close(&USERFile);	/* Close file */
	}
}

//***********************************************************

void Beep (void)
{
	HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET);
}

//***********************************************************

void Force_copy_GGA(NEO6_struct * _neo6, GGA_struct * _gga)
{
	memcpy(_gga->string, &_neo6->string[103], 75);
}

//***********************************************************

void Correct_copy_GGA(NEO6_struct * _neo6, GGA_struct * _gga)
{
	memcpy(_gga->string, &_neo6->string[_gga->Neo6_start], _gga->length );
}

//***********************************************************

void Clear_variables(NEO6_struct * _neo6, GGA_struct * _gga, CheckSum_struct * _cs)
{
	_gga->Neo6_start 		= 0;
	_gga->Neo6_end   		= 0;
	_gga->beginning_chars	= 0;
	_gga->ending_char		= 0;
	_gga->length = 0;

	_cs->calc_u8 		= 0;
	_cs->glue_u8 		= 0;
	_cs->status_flag 	= 0;

	_neo6->length_int = 0;
}

//***********************************************************
void TIM_Start(void)
{
	TIM3->CNT = 0;
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start(&htim3);
}

//***********************************************************

void TIM_Stop(void)
{
	HAL_TIM_Base_Stop_IT(&htim3);
	HAL_TIM_Base_Stop(&htim3);
}

//***********************************************************

void Print(NEO6_struct * _neo6, GGA_struct * _gga, CheckSum_struct * _cs, Time_struct * _time, SD_Card_struct * _sd)
{
	LCD_SetCursor(0, 95*(_time->seconds_int%2));
	sprintf(DebugString,"%s", _gga->string);
	HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
	LCD_Printf("%s", DebugString);

	sprintf(DebugString,"%d/%d/%d/cs%d; %s; SD_write: %d\r\n",
								_gga->Neo6_start,
								_gga->Neo6_end,
								_gga->length,
								_cs->status_flag,
								_sd->filename,
								_sd->write_status);
	HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);

	sprintf(DebugString,"%s SD_wr %d\r\n", _sd->filename, _sd->write_status);
	LCD_SetCursor(0, 195);
	LCD_Printf("%s", DebugString);
}

//***********************************************************

void Read_SD_card(void)
{
	sprintf(DebugString,"Start read SD\r\n");
	HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);

	fres = f_open(&USERFile, "tmm.txt", FA_OPEN_EXISTING | FA_READ);
	if (fres == FR_OK)
	{
		sprintf(DebugString,"st: \r\n");
		HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);

			char buff[200];
			LCD_SetCursor(0, 0);
			LCD_FillScreen(0x0000);
			/* Read from file */
			while (f_gets(buff, 200, &USERFile))
			{
				LCD_Printf(buff);
				sprintf(DebugString,"%s\r\n", buff);
				HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
			}
		f_close(&USERFile);	/* Close file */
	}
	else
	{
		sprintf(DebugString,"6) read from SD FAILED\r\n");
		HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
	}

	sprintf(DebugString,"7) END read from SD.\r\n");
	HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
}

//***********************************************************

void Prepere_filename(CheckSum_struct * _cs, Time_struct * _time, SD_Card_struct * _sd)
{
	char file_name_char[FILE_NAME_SIZE];
	int file_name_int = _time->hour_int*10000 + _time->minutes_int*100 + 12*(_time->seconds_int/12);
	sprintf(file_name_char,"%06d_%d.txt", file_name_int, (int)_cs->status_flag);
	int len = strlen(file_name_char) + 1;
	char PathString[len];
	snprintf(PathString, len,"%s", file_name_char);

	TCHAR *f_tmp = _sd->filename;
	char *s_tmp = PathString;
	while(*s_tmp)
	 *f_tmp++ = (TCHAR)*s_tmp++;
	*f_tmp = 0;
}

//***********************************************************
void Read_from_RingBuffer(NEO6_struct * _neo6, RingBuffer_DMA * _rx_buffer)
{
  	uint32_t rx_count = RingBuffer_DMA_Count(_rx_buffer);
	while ((rx_count--) && (time_write_to_SD_flag == 0))
	{
		_neo6->string[_neo6->length_int] = RingBuffer_DMA_GetByte(_rx_buffer);
		_neo6->length_int++;
		if (_neo6->length_int > MAX_CHAR_IN_NEO6)
		{
			time_write_to_SD_flag = 1;
			return;
		}
	} // end while rx_count
}

//***********************************************************

void NAUR_Init (void)
{
	LCD_Init();
	LCD_SetRotation(1);
	LCD_FillScreen(ILI92_BLACK);
	LCD_SetTextColor(ILI92_GREEN, ILI92_BLACK);
	LCD_SetCursor(0, 0);

	sprintf(DebugString,"\r\n\r\n");
	HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
	sprintf(DebugString,"NAU_Rocket Find_It\r\n2019 v2.0.0\r\nfor_debug UART5 115200/8-N-1\r\n");
	HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
	LCD_Printf("%s",DebugString);

	RingBuffer_DMA_Init(&rx_buffer, &hdma_usart3_rx, rx_circular_buffer, RX_BUFFER_SIZE);  	// Start UART receive
  	HAL_UART_Receive_DMA(&huart3, rx_circular_buffer, RX_BUFFER_SIZE);  	// how many bytes in buffer

  	FATFS_SPI_Init(&hspi1);	/* Initialize SD Card low level SPI driver */
	if (f_mount(&USERFatFS, "0:", 1) != FR_OK)	/* try to mount SDCARD */
	{
		f_mount(NULL, "0:", 0);			/* Unmount SDCARD */
		Error_Handler();
		sprintf(DebugString,"\r\nSD-card_mount - Failed \r\n");
		HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
		LCD_Printf("%s",DebugString);
		HAL_Delay(1000);
	}
	else
	{
		sprintf(DebugString,"\r\nSD-card_mount - Ok \r\n");
		HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
		LCD_Printf("%s",DebugString);
	}

	LCD_FillScreen(ILI92_BLACK);

	HAL_IWDG_Refresh(&hiwdg);
}

//***********************************************************

void NAUR_Main (void)
{
	switch (sm_stage)
	{
		case SM_START:
		{
			Clear_variables(&NEO6, &GGA, &CS);
			TIM_Start();
			sm_stage =SM_READ_FROM_RINGBUFFER;
		} break;

		//***********************************************************

		case SM_READ_FROM_RINGBUFFER:
		{
			sm_stage = SM_CHECK_FLAGS;
			Read_from_RingBuffer(&NEO6, &rx_buffer);

			if (time_write_to_SD_flag == 1)
			{
				TIM_Stop();
				sm_stage = SM_FIND_GGA;
			}
			if ((time_write_to_SD_flag == 1) && (NEO6.length_int < NEO6_LENGTH_MIN))
			{
				sm_stage = SM_START;
				time_write_to_SD_flag = 0;
			}

		} break;

		//***********************************************************

		case SM_CHECK_FLAGS:
		{
			if (shudown_button_pressed_flag == 1)
			{
				sm_stage = SM_SHUTDOWN;
				break;
			}

			if (time_read_from_SD_u8 == 1)
			{
				sm_stage =SM_READ_FROM_SDCARD;
				break;
			}
			sm_stage = SM_READ_FROM_RINGBUFFER;
		} break;

		//***********************************************************

		case SM_FIND_GGA:
		{
			result = Find_Begin_of_GGA_string(&NEO6, &GGA);
			if ( result == R_OK)
			{
				sm_stage = SM_FIND_ASTERISK;
			}
			else
			{
				sm_stage = SM_FORCE_COPY_GGA;
			}
		} break;

	//***********************************************************

		case SM_FIND_ASTERISK:
		{
			result = Find_End_of_GGA_string(&NEO6, &GGA);
			if ( result == R_OK )
			{
				sm_stage = SM_CORRECT_COPY_GGA;
			}
			else
			{
				sm_stage = SM_FORCE_COPY_GGA;
			}
		} break;

	//***********************************************************

		case SM_CORRECT_COPY_GGA:
		{
			Correct_copy_GGA(&NEO6, &GGA);
			sm_stage = SM_CALC_SHECKSUM;
		} break;

	//***********************************************************

		case SM_CALC_SHECKSUM:
		{
			result = Calc_SheckSum_GGA(&GGA, &CS);
			if ( result == R_OK )
			{
				sm_stage = SM_GET_TIME_FROM_GGA;
			}
			else
			{
				sm_stage = SM_TIME_INCREMENT;
			}
		} break;

	//***********************************************************

		case SM_GET_TIME_FROM_GGA:
		{
			Get_time_from_GGA_string(&GGA, &Time);
			sm_stage = SM_TIME_INCREMENT;
		} break;

	//***********************************************************

		case SM_FORCE_COPY_GGA:
		{
			Force_copy_GGA(&NEO6, &GGA);
			sm_stage = SM_TIME_INCREMENT;
		} break;

	//***********************************************************

		case SM_TIME_INCREMENT:
		{
			Increment_time(&Time);
			sm_stage = SM_PREPARE_FILENAME;
		} break;

	//***********************************************************

		case SM_PREPARE_FILENAME:
		{
			Prepere_filename(&CS, &Time, &SD);
			sm_stage = SM_WRITE_SDCARD;
		} break;
		//***********************************************************

		case SM_WRITE_SDCARD:
		{
			HAL_IWDG_Refresh(&hiwdg);
			Write_SD_card(&GGA, &SD);
			sm_stage = SM_PRINT;
		} break;
		//***********************************************************

		case SM_PRINT:
		{
			Print(&NEO6, &GGA, &CS, &Time, &SD);
			sm_stage = SM_FINISH;
		} break;
		//***********************************************************

		case SM_FINISH:
		{
			//HAL_GPIO_WritePin(TEST_PIN_GPIO_Port, TEST_PIN_Pin, GPIO_PIN_RESET);
			time_write_to_SD_flag  = 0 ;
			sm_stage = SM_START;
		} break;
		//***********************************************************

		case SM_ERROR_HANDLER:
		{
			LCD_SetCursor(0, 0);
			sprintf(DebugString,"Buf empty. L= %d\r\n", NEO6.length_int);
			HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
			LCD_Printf("%s", DebugString);
			NEO6.length_int = 0;

			sm_stage = SM_FINISH;
		} break;
		//***********************************************************

		case SM_SHUTDOWN:
		{
			TIM_Stop();
			ShutDown();
			shudown_button_pressed_flag = 0;
			sm_stage = SM_START;
		} break;
		//***********************************************************

		case SM_READ_FROM_SDCARD:
		{
			Read_SD_card();
			time_read_from_SD_u8 = 0;
			sm_stage = SM_START;
		} break;
		//***********************************************************

		default:
		{
			sm_stage = SM_START;
		} break;
	}	//			switch (sm_stage)
}
//***********************************************************
//***********************************************************
//***********************************************************

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
