/* USER CODE BEGIN Header */
/**
 *
 * This code shows how to change frequency of the PWM.
 * The PWM  frequency depends on the value of the ADC where 4095 is 100% or the period = 16000  and
 * the value 0 in the ADC is 0% o 0
 *
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
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
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  //start the ADC conversion
  HAL_ADC_Start(&hadc1);

  //start PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  //ADC_PERCENT=VARIABLE TO SAVE THE CALCULATED PERCENT FOR PWM
  uint32_t ADC_PERCENT;
  //ARR_VALUE = save the calculated value (32 bits) for ARR register of TIM2
  uint32_t ARR_VALUE;
  //ADC_PERCENT_BEFORE= save the value before a change in the voltage input or save the
  // before voltage input
  uint32_t ADC_PERCENT_BEFORE=50;
  //diference = save the diference between ADC_PERCENT_BEFORE and ADC_PERCENT like that we know there was
  // a change
  uint8_t diference;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Wait until 5 milliseconds
	 HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  //calculate the percent which will be the value of the frequency
	  ADC_PERCENT= (uint32_t)((HAL_ADC_GetValue(&hadc1) * 100)/4095);

	  // we compute if there was a change in the ADC conversion value
	  diference=(ADC_PERCENT_BEFORE - ADC_PERCENT);
	  /* if the change in the actual ADC conversion value was more 1 respect to the before ADC conversion value
	  (ADC_PERCENT_BEFORE)
	  NOTE:if the difference is negative the diference variable is uint8_t so when this happen the variable
	  	  	  takes the maximum value that diference variable can take, this way we can detect
	  	  	  changes in both directions.
	  */
	  if(diference >1)
	  {
		  //we save the present value of the ADC in the ADC_PERCENT_BEFORE
		  // like that we will be able to know about the future changes in the voltage input on the ADC
		  // peripheral
		  ADC_PERCENT_BEFORE = ADC_PERCENT;
		  ARR_VALUE= (uint32_t)(ADC_PERCENT * 16000U/100U);
		  /*This if statement avoids the PWM stop due the change in the
		   * ARR register and CCR  of the channel 1
		   */
		  if(__HAL_TIM_GET_COUNTER(&htim2)>ARR_VALUE)
		  {
			  __HAL_TIM_SET_COUNTER(&htim2,0);
		  }
		  // set period
		  __HAL_TIM_SET_AUTORELOAD(&htim2,ARR_VALUE);


		  //set the value corresponding to ADC_PERCENT in the compare register of the channel1
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(ARR_VALUE/2));

	  }




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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
