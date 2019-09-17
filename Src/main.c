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
#include "spi.h"
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

	uint8_t _char_to_uint8 (uint8_t char_1, uint8_t char_0);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

	volatile uint8_t shudown_button_pressed_flag = 0;

	extern DMA_HandleTypeDef hdma_usart3_rx;
	RingBuffer_DMA rx_buffer;

	#define RX_BUFFER_SIZE 100
	uint8_t rx_circular_buffer[RX_BUFFER_SIZE];

	#define	GPS_DATA_SIZE	250
	char GPSdata_string[GPS_DATA_SIZE];
	int GPSdata_length_int = 0;

	#define MAX_CHAR_IN_GPS_DATA	80
	#define	TIMEZONE				3
	#define	DEBUG_STRING_SIZE		200

	 uint8_t time_write_to_SD_flag	= 0;
	 uint8_t first_circle_flag 		= 1;
	uint32_t circle_u32 			= 0;

	int file_name_hour_int			= 1 ;
	int file_name_minutes_int		= 1 ;
	int file_name_seconds_int		= 1 ;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	FRESULT fres;

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
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

	LCD_Init();
	LCD_SetRotation(1);
	//	LCD_FillScreen(BLACK);
	LCD_FillScreen(0x0000);
	//	LCD_SetTextColor(GREEN, BLACK);
	LCD_SetTextColor(0x07E0, 0x0000);

	char DebugString[DEBUG_STRING_SIZE];
	sprintf(DebugString,"\r\n\r\nNAU_Rocket Find_It\r\n2019 v1.2.0\r\nfor_debug UART5 115200/8-N-1\r\n");
	HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
	LCD_Printf("%s",DebugString);

	RingBuffer_DMA_Init(&rx_buffer, &hdma_usart3_rx, rx_circular_buffer, RX_BUFFER_SIZE);  	// Start UART receive
  	HAL_UART_Receive_DMA(&huart3, rx_circular_buffer, RX_BUFFER_SIZE);  	// how many bytes in buffer
  	uint32_t rx_count;

  	FATFS_SPI_Init(&hspi1);	/* Initialize SD Card low level SPI driver */
	/* Mount SDCARD */
	if (f_mount(&USERFatFS, "", 1) != FR_OK) {
		/* Unmount SDCARD */
		f_mount(NULL, "", 0);
		Error_Handler();

		sprintf(DebugString,"\r\nSD-card_mount  Failed \r\n");
		HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
		LCD_Printf("%s",DebugString);

		HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_RESET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_RESET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_RESET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET);
	}
	else
	{
		sprintf(DebugString,"\r\nSD-card_mount - OK \r\n");
		HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
		LCD_Printf("%s",DebugString);
	}

	LCD_FillScreen(0x0000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	rx_count = RingBuffer_DMA_Count(&rx_buffer);

	while (rx_count--)
	{
		uint8_t current_GPS_char = RingBuffer_DMA_GetByte(&rx_buffer);
		if (current_GPS_char == '\n')
		{
			time_write_to_SD_flag = 1;
		}
		else if (GPSdata_length_int > MAX_CHAR_IN_GPS_DATA)
		{
			GPSdata_string[GPSdata_length_int++] = '\r' ;
			GPSdata_string[GPSdata_length_int++] = '\n' ;
			time_write_to_SD_flag = 1;
		}
		else if (current_GPS_char == '\r')
		{
			//  skip \r  }
		}
		else {GPSdata_string[GPSdata_length_int++ % GPS_DATA_SIZE] = current_GPS_char;}
	} // end while rx_count
	//***********************************************************

	if (time_write_to_SD_flag == 1)
	{
		int current_GPSstring_size_int = strlen(GPSdata_string);

		GPSdata_string[GPSdata_length_int++] = 0; // we received whole command, setting end of string
		GPSdata_length_int = 0;
		sprintf(DebugString,"%s", GPSdata_string);
		HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);

		circle_u32++;
		LCD_SetCursor(0, 70*(circle_u32%2));
		LCD_Printf("%s", GPSdata_string);

		// CountCheckSum
		uint8_t check_sum_calc_u8 = GPSdata_string[1];
		for (uint32_t i=2; i<(current_GPSstring_size_int-3); i++)
		{
			check_sum_calc_u8 ^= GPSdata_string[i];
		}

		uint8_t check_sum_glue_u8 = _char_to_uint8 (GPSdata_string[current_GPSstring_size_int-2], GPSdata_string[current_GPSstring_size_int-1]);

		sprintf(DebugString," %X %d\r\n", check_sum_calc_u8, current_GPSstring_size_int);
		HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
		//LCD_Printf(DebugString);

		uint8_t check_sum_status_flag = 0;
		if (check_sum_calc_u8 == check_sum_glue_u8)
		{
			check_sum_status_flag = 1;
		}

		if ((check_sum_status_flag == 1) && (first_circle_flag == 1))
		{
			sprintf(DebugString,"\r\nCheckSum - Ok\r\n");
			HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
			first_circle_flag = 0;

			#define TMP_TIME_FILENAME_SIZE	5
			char tmp_time_filename_string[TMP_TIME_FILENAME_SIZE];

			for (int i=0; i<TMP_TIME_FILENAME_SIZE; i++)
			{
				tmp_time_filename_string[i] = 0x00;
			}
			memcpy(tmp_time_filename_string, &GPSdata_string[7], 2);
			file_name_hour_int = atoi(tmp_time_filename_string) + TIMEZONE;

			memcpy(tmp_time_filename_string, &GPSdata_string[9], 2);
			file_name_minutes_int = atoi(tmp_time_filename_string);

			memcpy(tmp_time_filename_string, &GPSdata_string[11], 2);
			file_name_seconds_int = atoi(tmp_time_filename_string);
		}
		else
		{
			if (file_name_seconds_int < 59)
			{
				//	file_name_seconds_int++;
			}
			else
			{
				file_name_seconds_int = 0;
				if (file_name_minutes_int < 59)
				{
					file_name_minutes_int++;
				}
				else
				{
					file_name_minutes_int = 0;
					file_name_hour_int++;
				}
			}

		}

		#define FILE_NAME_SIZE 15
		char current_file_name_char[FILE_NAME_SIZE];

		int file_name_int = file_name_hour_int*10000 + file_name_minutes_int*100 + file_name_seconds_int;
		sprintf(current_file_name_char,"%06d_%d.txt", file_name_int, (int)check_sum_status_flag);
		//LCD_Printf("%s",current_file_name_char);
		sprintf(DebugString,"\r\n fn:%s\r\n", current_file_name_char);
		HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);

		//	write to first file
		HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_RESET);

		fres = f_open(&USERFile, current_file_name_char, FA_OPEN_APPEND | FA_WRITE);			/* Try to open file */
		if (fres == FR_OK)
		{
			f_printf(&USERFile, "%s\r\n", GPSdata_string);	/* Write to file */
			f_close(&USERFile);	/* Close file */
			sprintf(DebugString," write_to_SD - OK \r\n");
			HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
			//	LCD_Printf("%s",DebugString);
		}
		else
		{
			sprintf(DebugString," write_to_SD - FAILED \r\n");
			HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
			//	LCD_Printf("%s",DebugString);
		}
		 HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET);
		//	end write to file

		time_write_to_SD_flag  = 0 ;
	}	//		if (time_write_to_SD_flag == 1)
	//*****************************************************************************************

	if (shudown_button_pressed_flag == 1)
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
		}

		shudown_button_pressed_flag = 0;
	}
	//*****************************************************************************************


				//	if (time_read_from_SD_u8 == 1)
				//		{
				//			sprintf(DebugString,"5) Start read from SD-card\r\n");
				//			HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
				//
				//			fres = f_open(&USERFile, "tmm.txt", FA_OPEN_EXISTING | FA_READ);
				//			if (fres == FR_OK)
				//			{
				//				sprintf(DebugString,"6) read from SD OK\r\n");
				//				HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
				//
				//					char buff[200];
				//					LCD_SetCursor(0, 0);
				//					LCD_FillScreen(0x0000);
				//					/* Read from file */
				//					while (f_gets(buff, 200, &USERFile))
				//					{
				//						LCD_Printf(buff);
				//						sprintf(DebugString,"%s\r\n", buff);
				//						HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
				//					}
				//						/* Close file */
				//						f_close(&USERFile);
				//			}
				//			else
				//			{
				//				sprintf(DebugString,"6) read from SD FAILED\r\n");
				//				HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
				//			}
				//
				//			sprintf(DebugString,"7) END read from SD.\r\n");
				//			HAL_UART_Transmit(&huart5, (uint8_t *)DebugString, strlen(DebugString), 100);
				//
				//			time_read_from_SD_u8 = 0;
				//		}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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

uint8_t _char_to_uint8 (uint8_t char_1, uint8_t char_0)
{
	uint8_t result_u8 = 0;

	if (char_1 <= 0x39)
	{
		result_u8 = ((char_1 - 0x30        )<<4);
	}
	else
	{
		result_u8 = ((char_1 - 0x41 + 0x0A )<<4) ;
	}

	if (char_0 <= 0x39)
	{
		result_u8 = result_u8 + (char_0 - 0x30 ) ;
	}
	else
	{
		result_u8 = result_u8 + (char_0 - 0x41 +0x0A) ;
	}
return result_u8;
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
