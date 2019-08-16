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
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OFF 0
#define ON 1

#define NUM_BUTTONS 4
#define UART_PACKET_SIZE 8
#define DEBOUNCE_DELAY_MS 100

#define NUM_RADIO_PACKETS 6
#define RADIO_PACKET_SIZE 19
#define COUNTER_INC_US 25
#define RADIO_HIGH_US 500
#define RADIO_LOW_US 200
#define RADIO_PACKET_DELAY_US 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef struct
{
	GPIO_TypeDef* GPIO_port;
	uint16_t GPIO_pin;
	uint64_t counter; // Used for debouncing
	uint8_t status;   // Indicates whether or not the button is pressed
}
button;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
uint32_t Ms_Tick(void);
void Button_Init(uint8_t i, GPIO_TypeDef* GPIO_bank, uint16_t GPIO_pin);
void Buttons_Init(void);
void Read_Button(uint8_t i);
void Encode_UART_Packets(uint8_t UART_Packets[]);
void LED_Debug(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t counter_25us;
button buttons[NUM_BUTTONS];
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  //MX_USB_HOST_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);
	Buttons_Init();
	uint32_t ms_counter = Ms_Tick();
	uint32_t sec_counter = Ms_Tick();
	uint8_t i;
	uint8_t UART_packets[NUM_BUTTONS];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    //MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    /* Executes every 1 ms */
		if (Ms_Tick() - ms_counter > 0)
		{
			for (i = 0; i < NUM_BUTTONS; i++)
			{
				Read_Button(i);
			}
			ms_counter++;
		}
		/* Executes every 1 sec */
		if (Ms_Tick() - sec_counter > 1000)
		{
			Encode_UART_Packets(UART_packets);
			HAL_UART_Transmit(&huart2, UART_packets, NUM_BUTTONS, 10);
			sec_counter = Ms_Tick();
		}
		
		LED_Debug();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  This function is used for interrupt service routines.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
	{
		counter_25us++;
	}
}

/**
  * @brief  This function is used as a millisecond counter.
  * @retval A tick value in milliseconds
  */
uint32_t Ms_Tick(void)
{
	return HAL_GetTick();
}

/**
  * @brief  This funciton is used to initialize a struct button.
  * @retval None
  */
void Button_Init(uint8_t i, GPIO_TypeDef* GPIO_port, uint16_t GPIO_pin)
{
	buttons[i].GPIO_port = GPIO_port;
	buttons[i].GPIO_pin = GPIO_pin;
	buttons[i].counter = 0;
	buttons[i].status = OFF;
}

/**
  * @brief  This function is used to inialize the global variable
  *         buttons, an array of struct buttons.
  * @retval None
  */
void Buttons_Init(void)
{
	Button_Init(0, GPIO_Input_GPIO_Port, GPIO_Input_Pin);   // PE2
	Button_Init(1, GPIO_Input_GPIO_Port, GPIO_InputE4_Pin); // PE4
	Button_Init(2, GPIO_Input_GPIO_Port, GPIO_InputE5_Pin); // PE5
	Button_Init(3, GPIO_Input_GPIO_Port, GPIO_InputE6_Pin); // PE6
}

/**
  * @brief  This function is used to read the appropriate GPIO pins and
  *         update the struct button accordingly.
  *         When the GPIO pin has been pressed for DEBOUNCE_DELAY_MS ms,
  *         the final if statement is entered once to update the status
  *         of the button.
  * @retval None
  */
void Read_Button(uint8_t i)
{
	if (HAL_GPIO_ReadPin(buttons[i].GPIO_port, buttons[i].GPIO_pin) == GPIO_PIN_SET)
	{
		buttons[i].counter++;
	}
	else
	{
		buttons[i].counter = 0;
	}
	if (buttons[i].counter == DEBOUNCE_DELAY_MS)
	{
		buttons[i].status ^= 1;
	}
}

void Encode_UART_Packets(uint8_t UART_Packets[])
{
	int i;
	for (i = 0; i < NUM_BUTTONS; i++)
	{
		UART_Packets[i] = i;
		UART_Packets[i] <<= 1;
		if (buttons[i].status == ON)
		{
			UART_Packets[i] |= 1;
		}
	}
}

/**
  * @brief  This function is used for debugging GPIO input pins.
  *         When the status field of a struct button is ON,
  *         an LED lights up, with priority given to the lowest 
  *         struct button in the array. 
  * @retval None
  */
void LED_Debug(void)
{
	if (buttons[0].status == ON) // Connect PE2 to 5V (since it is pull-down) to light up LD3
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		}
	else
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		}
	if (buttons[1].status == ON)
		{
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
		}
	else
		{
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
		}
	if (buttons[2].status == ON)
		{
			HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		}
	else
		{
			HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
		}
	if (buttons[3].status == ON)
		{
			HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
		}
	else
		{
			HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
		}
}

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
