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

	extern DMA_HandleTypeDef hdma_usart3_rx;
	RingBuffer_DMA rx_buffer;
	#define RX_BUFFER_SIZE 100

	uint8_t rx_circular_buffer[RX_BUFFER_SIZE];
	char cmd[500];
	int iCmd = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
	LCD_SetRotation(1);
	LCD_FillScreen(BLACK);
	LCD_SetTextColor(GREEN, BLACK);
	LCD_Printf("\n NAU_Rocket Find_It 2019 v0.1.0\n ");

	char DataChar[100];
	sprintf(DataChar,"\r\n NAU_Rocket Find_It 2019 v0.1.0\r\nUART1 for debug started on speed 115200\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);

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

		sprintf(DataChar,"1) f_mount = Failed \r\n");
		HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);
	}
	else
	{
		sprintf(DataChar,"1) f_mount = FR_OK \r\n");
		HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t line = 0;
	uint32_t circle_u32;
  while (1)
  {
	//HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	rx_count = RingBuffer_DMA_Count(&rx_buffer);

	while (rx_count--)
	{
		uint8_t c = RingBuffer_DMA_GetByte(&rx_buffer);
		if (c == '\n')
		{
			cmd[iCmd++] = 0; // we received whole command, setting end of string
			iCmd = 0;
			LCD_Printf("%s\n", cmd);
			sprintf(DataChar,"%s\r\n", cmd);
			HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);

			/* Try to open file */
			fres = f_open(&USERFile, "tm.txt", FA_OPEN_APPEND | FA_WRITE);
			if (fres == FR_OK) {
				/* Write to file */
				f_printf(&USERFile, "%ul: Hello, sd card from TechMaker!\r\n", line++);
				/* Close file */
				f_close(&USERFile);
				circle_u32++;
				sprintf(DataChar,"2)fres = FR_OK \r\n");
				HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);
			}
			else
			{
				sprintf(DataChar,"2)fres FAILED \r\n");
				HAL_UART_Transmit(&huart3, (uint8_t *)DataChar, strlen(DataChar), 100);
			}

			//if ((cmd[0]=='l')&& (cmd[1]=='e')&&(cmd[2]=='d')) LCD_Printf(text);
			// react to command here
			//if (strcmp(cmd,"adc")==0) LCD_Printf("U=3.3v\n");
		}
		else if (c == '\r')
		{
			//  skip \r  }
		}
		else {cmd[iCmd++ % 500] = c;}

		if (circle_u32/10 == 1)
		{
			fres = f_open(&USERFile, "tm.txt", FA_OPEN_EXISTING | FA_READ);
			if (fres == FR_OK)
			{
					char buff[200];
					LCD_SetCursor(0, 0);
					LCD_FillScreen(BLACK);
					/* Read from file */
					while (f_gets(buff, 200, &USERFile))
					{
						LCD_Printf(buff);
					}
						/* Close file */
						f_close(&USERFile);
			}
			circle_u32 = 0;
		}
	} // end while

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
