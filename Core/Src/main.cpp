/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PWM.hpp"
#include "Serial.hpp"
#include "IOPin.hpp"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DATA 2
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ANGLE_LR 8
#define ANGLE_UD 7
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
enum BUTTON1
{
	 SQR = 1,
	 L1 = 2,
	 L2 = 4,
	 R1 = 8,
	 R2 = 16,
};

enum BUTTON2
{
	 UP = 1,
	 DOWN = 2,
	 RIGHT = 4,
	 LEFT = 8,
	 TRI = 16,
	 CRO = 32,
	 CIR = 64,
};

struct Button
{
	uint8_t up;
	uint8_t down;
	uint8_t right;
	uint8_t left;
	uint8_t tri;
	uint8_t cro;
	uint8_t cir;
	uint8_t sqr;
	uint8_t l1;
	uint8_t l2;
	uint8_t r1;
	uint8_t r2;
};
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
//  Servo motor1(&htim1, TIM_CHANNEL_1);

  PWM pwm[9] = {
		  {&htim1,TIM_CHANNEL_4,10000,84,timMode::APB2},
		  {&htim1,TIM_CHANNEL_3,10000,84,timMode::APB2},
		  {&htim1,TIM_CHANNEL_2,10000,84,timMode::APB2},
		  {&htim2,TIM_CHANNEL_2,10000,84,timMode::APB2},
		  {&htim2,TIM_CHANNEL_1,10000,84,timMode::APB2},
		  {&htim2,TIM_CHANNEL_3,10000,84,timMode::APB2},
		  {&htim3,TIM_CHANNEL_1,10000,84,timMode::APB2},
		  {&htim3,TIM_CHANNEL_2,10000,84,timMode::APB2},
		  {&htim3,TIM_CHANNEL_3,10000,84,timMode::APB2},
  };

  servo motor[9] = {
		  pwm[0],
		  pwm[1],
		  pwm[2],
		  pwm[3],
		  pwm[4],
		  pwm[5],
		  pwm[6],
		  pwm[7],
		  pwm[8],
  };

  Serial stm32(huart4, 115200);
  Serial serial(huart2, 115200);

  uint8_t buf[DATA]={0};
  Button button;
  uint8_t move = 0;
  uint16_t injec = 150;
  uint8_t angle_LR = 50;
  uint8_t angle_UD = 50;


  for (int i = 0; i < 9; i++)
  {
	  motor[i].setTime(1500);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  if (stm32.available() > DATA)
	  {
		  for (int i = 0; i < DATA; i++)
		  {
			  buf[i]=stm32.read();
		  }


		  button.up 	= (buf[1] & UP) 	>> 0;
		  button.down 	= (buf[1] & DOWN) 	>> 1;
		  button.right 	= (buf[1] & RIGHT)	>> 2;
		  button.left 	= (buf[1] & LEFT) 	>> 3;
		  button.tri 	= (buf[1] & TRI) 	>> 4;
		  button.cro	= (buf[1] & CRO) 	>> 5;
		  button.cir	= (buf[1] & CIR) 	>> 6;
		  button.sqr	= (buf[0] & SQR) 	>> 0;
		  button.l1		= (buf[0] & L1) 	>> 1;
		  button.l2		= (buf[0] & L2) 	>> 2;
		  button.r1		= (buf[0] & R1) 	>> 3;
		  button.r2		= (buf[0] & R2) 	>> 4;

		  if(button.cir && !move)
		  {
			  move = 1;

			  for (int i = 0; i < 1; i++)
			  {
				  for(int pulse = 1500 + injec; pulse <= 1900; pulse += 3)
				  {
					  HAL_Delay(6);
					  motor[i].setTime(pulse);
				  }
			  }
		  }

		  else if (button.cro && move)
		  {
			  move = 0;
			  for(int i = 0; i < 6; i++)
			  {
				  for(int pulse = 1900; pulse >= 1500 + injec; pulse -= 3)
				  {
						  HAL_Delay(6);
						  motor[i].setTime(pulse);
				  }
				  motor[i].setTime(1500);
			  }
		  }

		  if(button.left)
		  {
//			  for(int pulse = 1500 + angle_LR; pulse <= 1700; pulse += 3)
//			  {
//				  motor[ANGLE_LR].setTime(pulse);
//			  }
			  motor[ANGLE_LR].setTime(1600);
		  }

		  else if(button.right)
		  {
//			  for(int pulse = 1900; pulse >= 1500 + injec; pulse -= 3)
//			  {
//					  HAL_Delay(6);
//					  motor[ANGLE_LR].setTime(pulse);
//			  }
			  motor[ANGLE_LR].setTime(1300);
		  }

		  else
			  motor[ANGLE_LR].setTime(1500);

		  if(button.up)
		  {
//			  for(int pulse = 1500 + angle_LR; pulse <= 1700; pulse += 3)
//			  {
//				  motor[ANGLE_LR].setTime(pulse);
//			  }
			  motor[ANGLE_UD].setTime(1600);
		  }

		  else if(button.down)
		  {
//			  for(int pulse = 1900; pulse >= 1500 + injec; pulse -= 3)
//			  {
//					  HAL_Delay(6);
//					  motor[ANGLE_LR].setTime(pulse);
//			  }
			  motor[ANGLE_UD].setTime(1300);
		  }
		  else

			  motor[ANGLE_UD].setTime(1500);
	  }
//		  for (uint16_t power = 1500; power > 1000; power-=250)
//		  {
//			  for (int j = 0; j < 6; j++){
//			  motor[j].setTime(power);
//			  HAL_Delay(2000);
//			  }
//		  }
//
//
//		  for (uint16_t power = 1000; power < 1500 ; power+=250)
//		  {
//			  for (int j = 0; j < 6; j++){
//			  motor[j].setTime(power);
//			  HAL_Delay(2000);
//			  }
//		  }
//
//
//	  for (uint16_t power = 1500; power < 2000; power+=250)
//	  {
//		  for (int j = 0; j < 6; j++){
//		  motor[j].setTime(power);
//		  HAL_Delay(2000);
//		  }
//	  }
//
//	  for (uint16_t power = 2000; power > 1500; power-=250)
//	  {
//		  for (int j = 0; j < 6; j++){
//		  motor[j].setTime(power);
//		  HAL_Delay(2000);
//		  }
//	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
