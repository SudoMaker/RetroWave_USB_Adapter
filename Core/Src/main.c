/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

int packet_tx_done = 1;

void uprintf(const char *fmt, ...)
{
	int size = 0;
	char p[128];
	va_list ap;

	va_start(ap, fmt);
	size = vsnprintf(p, 127, fmt, ap);
	va_end(ap);

	HAL_UART_Transmit(&huart1, (uint8_t *)p, size, HAL_MAX_DELAY);

}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
//	if (packet_tx_done)
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 1);
	packet_tx_done = 1;
//	uprintf("SPI TX Done\n");

}

typedef enum {
	RetroWave_Board_Unknown = 0,
	RetroWave_Board_OPL3 = 0x21 << 1,
	RetroWave_Board_MiniBlaster = 0x20 << 1,
	RetroWave_Board_MasterGear = 0x24 << 1
} RetroWaveBoardType;

void retrowave_mini_io_callback(void *userp, uint32_t data_rate, const void *tx_buf, void *rx_buf, uint32_t len) {
	LED_GPIO_Port->BSRR = (uint32_t)LED_Pin; // 1
	SPI1_CS_GPIO_Port->BSRR = (uint32_t)SPI1_CS_Pin << 16; // 0

	HAL_SPI_Transmit(&hspi1, (uint8_t *)tx_buf, len, HAL_MAX_DELAY);

	SPI1_CS_GPIO_Port->BSRR = (uint32_t)SPI1_CS_Pin; // 1
	LED_GPIO_Port->BSRR = (uint32_t)LED_Pin << 16; // 0
}

void retrowave_mini_io_init() {
	// Sync CS state
	uint8_t empty_byte = 0;
	retrowave_mini_io_callback(NULL, 1e6, &empty_byte, NULL, 1);

	uint8_t init_sequence_1[] = {
		0x00,
		0x0a,	// IOCON register
		0x28	// Enable: HAEN, SEQOP
	};

	uint8_t init_sequence_2[] = {
		0x00,
		0x00,	// IODIRA register
		0x00,	// Set output
		0x00	// Set output
	};

	uint8_t init_sequence_3[] = {
		0x00,
		0x12,	// GPIOA register
		0xff,	// Set all HIGH
		0xff	// Set all HIGH
	};

	for (uint8_t i=0x20; i<0x28; i++) {
		uint8_t addr = i << 1;

		init_sequence_1[0] = init_sequence_2[0] = init_sequence_3[0] = addr;
		retrowave_mini_io_callback(NULL, 1e6, init_sequence_1, NULL, sizeof(init_sequence_1));
		retrowave_mini_io_callback(NULL, 1e6, init_sequence_2, NULL, sizeof(init_sequence_2));
		retrowave_mini_io_callback(NULL, 1e6, init_sequence_3, NULL, sizeof(init_sequence_3));
	}
}

void retrowave_mini_reset_opl3() {
	uint8_t buf[] = {RetroWave_Board_OPL3, 0x12, 0xfe};
	retrowave_mini_io_callback(NULL, 1e6, buf, NULL, sizeof(buf));
	buf[2] = 0xff;
	retrowave_mini_io_callback(NULL, 1e6, buf, NULL, sizeof(buf));
}

void retrowave_mini_reset_ym2413() {
	uint8_t buf[] = {RetroWave_Board_MasterGear, 0x12, 0xfe};
	retrowave_mini_io_callback(NULL, 1e6, buf, NULL, sizeof(buf));
	buf[2] = 0xff;
	retrowave_mini_io_callback(NULL, 1e6, buf, NULL, sizeof(buf));
}

void retrowave_mini_mute_sn76489() {
	uint8_t mute_tone1 = 0x9f;
	uint8_t mute_tone2 = 0xdf;
	uint8_t mute_tone3 = 0xbf;
	uint8_t mute_noise = 0xff;

	uint8_t buf[] = {RetroWave_Board_MasterGear, 0x12,
			 0xff, mute_tone1, 0x5f, mute_tone1, 0x0f, mute_tone1, 0xaf, mute_tone1, 0xff, 0x00,
			 0xff, mute_tone2, 0x5f, mute_tone2, 0x0f, mute_tone2, 0xaf, mute_tone2, 0xff, 0x00,
			 0xff, mute_tone3, 0x5f, mute_tone3, 0x0f, mute_tone3, 0xaf, mute_tone3, 0xff, 0x00,
			 0xff, mute_noise, 0x5f, mute_noise, 0x0f, mute_noise, 0xaf, mute_noise, 0xff, 0x00,
	};

	retrowave_mini_io_callback(NULL, 1e6, buf, NULL, sizeof(buf));
}

void retrowave_mini_reset() {
	retrowave_mini_reset_opl3();
	retrowave_mini_reset_ym2413();
	retrowave_mini_mute_sn76489();
}

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
	MX_SPI1_Init();
	MX_USART1_UART_Init();

	/* USER CODE BEGIN 2 */


	uprintf("Hello RetroWave!\n");
	uprintf("Initializing boards, plese wait...\n");

	HAL_Delay(10);

	retrowave_mini_io_init();
	retrowave_mini_reset();

	MX_USB_DEVICE_Init();

	uprintf("Initialization done. Enjoy!\n");

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

	/** Configure the main internal regulator output voltage
	*/
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
