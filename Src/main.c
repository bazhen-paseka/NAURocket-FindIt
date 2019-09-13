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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

	extern DMA_HandleTypeDef hdma_usart3_rx;
	RingBuffer_DMA rx_buffer;
	#define RX_BUFFER_SIZE 100
	uint8_t rx_circular_buffer[RX_BUFFER_SIZE];
	char cmd[500];
	int iCmd = 0;

	volatile uint8_t time_read_from_SD_u8 = 0;

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
  /* USER CODE BEGIN 2 */

	LCD_Init();
	LCD_SetRotation(0);
	//	LCD_FillScreen(BLACK);
	LCD_FillScreen(0x0000);
	//	LCD_SetTextColor(GREEN, BLACK);
	LCD_SetTextColor(0x07E0, 0x0000);

	char DataChar[100];
	sprintf(DataChar,"\r\nNAU_Rocket Find_It 2019 v1.0.0\r\nfor_debug UART3 9600/8-N-1\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);
	LCD_Printf("%s",DataChar);

	RingBuffer_DMA_Init(&rx_buffer, &hdma_usart3_rx, rx_circular_buffer, RX_BUFFER_SIZE);  	// Start UART receive
  	HAL_UART_Receive_DMA(&huart3, rx_circular_buffer, RX_BUFFER_SIZE);  	// how many bytes in buffer
  	uint32_t rx_count;

	/* Initialize SD Card low level SPI driver */
  	FATFS_SPI_Init(&hspi1);
	/* Mount SDCARD */
	if (f_mount(&USERFatFS, "", 1) != FR_OK) {
		/* Unmount SDCARD */
		f_mount(NULL, "", 0);
		Error_Handler();

		sprintf(DataChar,"\r\nSD-card_mount  Failed \r\n");
		HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);
		LCD_Printf("%s",DataChar);
	}
	else
	{
		sprintf(DataChar,"\r\nSD-card_mount - OK \r\n");
		HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);
		LCD_Printf("%s",DataChar);
	}
	HAL_Delay(3000);

	LCD_FillScreen(0x0000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	rx_count = RingBuffer_DMA_Count(&rx_buffer);

	while (rx_count--)
	{
		uint8_t c = RingBuffer_DMA_GetByte(&rx_buffer);
		if (c == '\n')
		{
			cmd[iCmd++] = 0; // we received whole command, setting end of string
			iCmd = 0;

			sprintf(DataChar,"%s", cmd);
			HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);

			//LCD_FillScreen(BLACK);
			LCD_SetCursor(0, 0);
			uint32_t cmd_size_u32 = strlen(cmd);
			for (uint32_t i=0; i<cmd_size_u32; i++)
			{
				if (cmd[i] == ',') LCD_Printf("\n");
				else LCD_Printf("%c",cmd[i]);
			}
			LCD_Printf("\n");

				// CountCheckSum
			uint8_t check_sum_u8 = cmd[1];
			for (uint32_t i=2; i<(cmd_size_u32-3); i++)
			{
				check_sum_u8 ^= cmd[i];
			}
			sprintf(DataChar," (%X)\r\n", check_sum_u8);
			HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);
			LCD_Printf(" %X - CheckSum\n", check_sum_u8);

			uint8_t check_sum_2_u8 = ((cmd[cmd_size_u32-2]-0x30)<<4) + (cmd[cmd_size_u32-1]-0x30);
			LCD_Printf(" %X - CheckSum 2\n", check_sum_2_u8);
			LCD_Printf(" %d - packet size\n\n", cmd_size_u32);

			char my_file_name[100];
			char my_file_name_1[100];
			char my_file_name_2[100];
			my_file_name[0] = 'C';
			my_file_name[1] = '3';
			my_file_name[2] = '4';
			my_file_name[3] = '.';
			my_file_name[4] = 't';
			my_file_name[5] = 'x';
			my_file_name[6] = 't';
			my_file_name[7] = '\n';


			for (int i =0; i<100; i++)
			{
				my_file_name_1[i] = 0x00;
				my_file_name_2[i] = 0x00;
			}
			memcpy(my_file_name_1, &cmd[7], 6);
			LCD_Printf("%s\n",my_file_name_1);
			sprintf(my_file_name_2,"T_%s.txt", my_file_name_1);
			LCD_Printf("%s\n",my_file_name_2);

			//snprintf(my_file_name_rez, 11, "%s%s", my_file_name, second);

				//	write to first file
			//fres = f_open(&USERFile, "A0.txt", FA_OPEN_APPEND | FA_WRITE);			/* Try to open file */
			fres = f_open(&USERFile, my_file_name_2, FA_OPEN_APPEND | FA_WRITE);			/* Try to open file */
			if (fres == FR_OK)
			{
				f_printf(&USERFile, "%s\r\n", cmd);	/* Write to file */
				f_close(&USERFile);	/* Close file */
				sprintf(DataChar,"Write_to_A0 - OK \r\n");
				HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);
				LCD_Printf("%s",DataChar);
			}
			else
			{
				sprintf(DataChar,"Write_to_A0 FAILED \r\n");
				HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);
				LCD_Printf("%s",DataChar);
			}

			HAL_Delay(10);

				//	write to second file
			fres = f_open(&USERFile, "B1.txt", FA_OPEN_APPEND | FA_WRITE);	/* Try to open file */
			if (fres == FR_OK)
			{
				f_printf(&USERFile, "%s\r\n", cmd);	/* Write to file */
				f_close(&USERFile);	/* Close file */

				sprintf(DataChar,"Write_to_B1 - OK\r\n");
				HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);
				LCD_Printf("%s",DataChar);
			}
			else
			{
				sprintf(DataChar,"Write_to_B1 FAILED \r\n");
				HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);
				LCD_Printf("%s",DataChar);
			}
				//	end write to file
		}
		else if (c == '\r')
		{
			//  skip \r  }
		}
		else {cmd[iCmd++ % 500] = c;}
	} // end while rx_count

	if (time_read_from_SD_u8 == 1)
		{
			sprintf(DataChar,"5) Start read from SD-card\r\n");
			HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);

			fres = f_open(&USERFile, "tmm.txt", FA_OPEN_EXISTING | FA_READ);
			if (fres == FR_OK)
			{
				sprintf(DataChar,"6) read from SD OK\r\n");
				HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);

					char buff[200];
					LCD_SetCursor(0, 0);
					LCD_FillScreen(0x0000);
					/* Read from file */
					while (f_gets(buff, 200, &USERFile))
					{
						LCD_Printf(buff);
						sprintf(DataChar,"%s\r\n", buff);
						HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);
					}
						/* Close file */
						f_close(&USERFile);
			}
			else
			{
				sprintf(DataChar,"6) read from SD FAILED\r\n");
				HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);
			}

			sprintf(DataChar,"7) END read from SD.\r\n");
			HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);

			time_read_from_SD_u8 = 0;
		}

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
