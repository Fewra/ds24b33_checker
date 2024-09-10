/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "onewire.h"
#include "ds24b33.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void get_rand_data(uint8_t *data);
static bool check_memory();
static bool compare_data(uint8_t *write_data, uint8_t *read_data);
static void write_data_toMem(uint8_t *data, size_t size, uint16_t addr);
static bool write_Userdata(const uint8_t *data, size_t size, uint16_t addr);
static void read_mem_data(uint8_t *data, size_t size, uint16_t addr);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  ONEWIRE_Status res = OneWire_Reset(&huart1);

  if (res == ONEWIRE_OK)
  {
	  /*
	  if (check_memory())
		  HAL_UART_Transmit(&huart2, (uint8_t*)"OK\n\r", 4, ONEWIRE_UART_TIMEOUT);
	  else
		  HAL_UART_Transmit(&huart2, (uint8_t*)"MEMORY ERROR\n\r", 14, ONEWIRE_UART_TIMEOUT);
		*/
	  const uint8_t userData[] = {
			  0x0B, 0x47, 0x4B, 0x47, 0x48, 0x00, 0xFF, 0xFF,
			  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
			  0x80, 0x3D, 0x36, 0xC7, 0x00, 0x3F, 0x83, 0x40,
			  0x84, 0x81, 0x6F, 0xAC, 0x7F, 0xED, 0x11, 0x7E,

			  0x0B, 0x47, 0x4B, 0x47, 0x48, 0x00, 0xFF, 0xFF,
			  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
			  0x80, 0x3D, 0x36, 0xC7, 0x00, 0x3F, 0x83, 0x40,
			  0x84, 0x81, 0x6F, 0xAC, 0x7F, 0xED, 0x11, 0x7E
	  };
	  uint16_t addr = 0x001E;

	  size_t userData_size = sizeof(userData) / sizeof(userData[0]);
	  bool test = write_Userdata(userData, userData_size, addr);

	  uint8_t getUserData[64] = {0};
	  read_mem_data(getUserData, userData_size, addr);

	  test++;

//	  const uint8_t userData[] = {
//			  0x0B, 0x47, 0x4B, 0x47, 0x48, 0x0, 0xFF, 0xFF,
//			  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
//			  0x80, 0x3D, 0x4C, 0x26, 0x00, 0x3F, 0x82, 0x70, 0x50
//	  };
//
//	  uint16_t userData_size = sizeof(userData) / sizeof(userData[0]);
//
//	  write_Userdata(userData, userData_size, 0x0008);
  }
  else
	  HAL_UART_Transmit(&huart2, (uint8_t*)"ERROR\n\r", 7, ONEWIRE_UART_TIMEOUT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void write_data_toMem(uint8_t *data, size_t size, uint16_t addr)
{
	uint8_t TA1 = (addr & 0x00FF);
	uint8_t TA2 = ((addr & 0xFF00)>>8);

	uint8_t dataRom[8];

	uint8_t writeScratch_data[size+2];
	uint8_t readScratch_data[35];
	uint8_t copyScratch_data[3];

	writeScratch_data[0] = TA1;
	writeScratch_data[1] = TA2;
	for (size_t i = 2; i < size+2; i++)
		writeScratch_data[i] = data[i-2];

	// сначала делаем ReadRom, а потом другие команды
	ReadRom(&huart1, dataRom);
	WriteScratchpad(&huart1, writeScratch_data, size+2);

	ReadRom(&huart1, dataRom);
	ReadScratchpad(&huart1, readScratch_data);

	// запись целевого адреса в начало массивов
	for (int i = 0; i < 3; i++)
		copyScratch_data[i] = readScratch_data[i];

	ReadRom(&huart1, dataRom);
	CopyScratchpad(&huart1, copyScratch_data);
}

static bool write_Userdata(const uint8_t *data, size_t size, uint16_t addr)
{
	if(addr + size - 1 > 0x1FF)
		return false;

	if (addr < 0x0 || addr > 0x1FF)
		return false;

	for (size_t i = 0, size_arr = 32 - (addr % 32); i < size;)
		{
			if (size_arr > (size-i))
				size_arr = (size-i) % 32;

			uint8_t part_data[size_arr];

			for (uint8_t j = 0; j < size_arr; j++)
				part_data[j] = data[i+j];

			// запись в память
			write_data_toMem(part_data, size_arr, addr);

			addr += size_arr;
			i += size_arr;

			if (size-i >= 32)
				size_arr = 32;
			else
			{
				size_arr = (size-i) % 32;
				if (size_arr == 0)
					break;
			}
		}

	return true;
}


static void get_rand_data(uint8_t *data)
{
	for (uint8_t i=0; i<32; i++)
	{
		data[i] = (rand() % 256);
	}
}

static bool compare_data(uint8_t *write_data, uint8_t *read_data)
{
	for (uint8_t i = 0; i < 32; i++)
	{
		if (write_data[i] != read_data[i])
			return memory_false;
	}
	return memory_true;
}

static void read_mem_data(uint8_t *data, size_t size, uint16_t addr)
{
	uint8_t TA1 = (addr & 0x00FF);
	uint8_t TA2 = ((addr & 0xFF00)>>8);
	uint8_t buf[size+2];

	buf[0] = TA1;
	buf[1] = TA2;

	uint8_t dataRom[8];

	ReadRom(&huart1, dataRom);
	ReadMemory(&huart1, buf, size);

	for(size_t i = 0; i < size; i++)
		data[i] = buf[i];
}

static bool check_memory()
{
	uint8_t data[32] = {0};
	uint8_t readMem_data[32] = {0};

	for (uint32_t addr = 0; addr <= 0x01E0; addr+=32)
	{

		get_rand_data(data);
		write_data_toMem(data, 32, addr);
		read_mem_data(readMem_data, 32, addr);

		if (!compare_data(data, readMem_data))
			return memory_false;
	}

	return memory_true;
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
  __disable_irq();
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
