/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "ds18b20.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
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
char str1[20]; // массив для вывода строк
uint8_t Dev_ID[3][8] = { 0 }; // массив серийных номеров
uint8_t Dev_Cnt = 0; // число датчиков на шине
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
	uint8_t status; // присутствие датчиков на шине
  uint8_t dt[8]; // массив серийный номер датчика
  uint16_t raw_temper; // получаемая температура в бинарном виде
  float temper; // преобразованная температура
  char ch; // знак температуры +/-
  float temper_arr[3]; // массив считанных температур
  char c_arr[3]= {0}; // массив знаков температур
  uint8_t i; // счетчик
	int ceil; // целая часть температуры
	int fraction; // дробная часть температуры
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
  /* USER CODE BEGIN 2 */
	lcd_ini(); // инициализация дисплея
	port_init(); // инициализация PB11
	status = ds18b20_init(NO_SKIP_ROM);
	if(status == 1)
		{
			sprintf(str1, "Initialization error");
			LCD_STRING(str1);
			HAL_Delay(1);
			LCD(COM, 0xC0);
			HAL_Delay(1);
			sprintf(str1, "Check the sensors");
			LCD_STRING(str1);
			HAL_Delay(1);
			LCD(COM, 0x94);
			sprintf(str1, "and reboot!");
			LCD_STRING(str1);
			while(1);
		}
	sprintf(str1, "Success");
	LCD_STRING(str1);
	HAL_Delay(1);
	LCD(COM, 0xC0);
	HAL_Delay(1);
	sprintf(str1, "Sensors found: %d", Dev_Cnt);
	LCD_STRING(str1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// подаем датчикам команду на запись температуры 
        for (i = 0; i < Dev_Cnt; i++)
        {
            ds18b20_MeasureTemperCmd(NO_SKIP_ROM, i);
        }
       HAL_Delay(800); // ждём готовности
				// переходим к считываению температур из памяти
        for (i = 0; i < Dev_Cnt; i++)
        {
            ds18b20_ReadStratcpad(NO_SKIP_ROM, dt, i); // читаем
            raw_temper = ((uint16_t)dt[1] << 8) | dt[0]; // переводим в одну переменную
            if (ds18b20_GetSign(raw_temper)) // устанавливаем знак
						{
							ch = '-';
						} 
            else
							ch = '+';
            temper = ds18b20_Convert(raw_temper); // избавляемся от знака          
						// сохраняем значения в массив
            temper_arr[i] = temper;
            c_arr[i] = ch;
        }
        LCD(COM, 0x01); // очищаем дисплей
        for (i = 0; i < Dev_Cnt; i++) // выводим на дисплей температуры 
        {
					ceil = (int)temper_arr[i];
					fraction = (int)(temper_arr[i]*10000);
					fraction = fraction % 10000;
                sprintf(str1, "T%d=%c%d.%04d%cC", i+1, c_arr[i], ceil, fraction, 0xDF);
            if (i == 0)
                LCD(COM, 0x80);
            else if (i == 1)
                LCD(COM, 0xC0);
            else
                LCD(COM, 0x94);
            LCD_STRING(str1);          
        }
        HAL_Delay(1000);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
