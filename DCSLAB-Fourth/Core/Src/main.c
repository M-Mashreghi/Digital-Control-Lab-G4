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
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
uint16_t counter;
int pos_counter;
float pos;
uint16_t speed;
uint16_t speed_previous;
int dir;
//uint16_t CCR_DB;
//uint16_t CCR_K1;
//uint16_t Speed_K1;
//uint16_t CCR_K2;
//uint16_t Speed_K2;
//uint16_t k_value;
//uint16_t Tau_value;

float Setpoint = 800;
float ek_1 = 0;
float ek;
float uk_1 = 0;
float uk;
uint16_t duty;
float Kp = 0.02;
float Ki = 0.2;
float Ts = 10;

typedef enum{
	INIT = 0, INIT2
} Engine_State_t;

Engine_State_t state = INIT;

typedef enum{
	OK = 0, ERR
} result_t;

//result_t init(void);
//result_t idle(void);
//result_t db(void);
//result_t k1(void);
//result_t k2(void);
//result_t tau(void);

void setDir(uint8_t dir);
void setDuty(uint16_t duty);
void setTs(uint8_t Ts);
void Vout2PWM(float u);


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM7_Init(void);
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
  MX_TIM6_Init();
  MX_TIM11_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

  TIM11->CCR1 = 0;
  pos = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  	  switch(state){

	  	  	  case INIT:
	  	  		  HAL_Delay(1000);
	  	  		  TIM11->CCR1 = 350;
	  	  		  HAL_Delay(2000);
//	  	  		  state = set_Ts;
	  	  		  break;
	  	  	  case INIT2:

	  	  		  break;

	  	  	  default:
	  	  		  state = INIT;
	  	  		  break;
	  	  }


//	  switch(state){
//
//	  	  case INIT:
//	  		  state = IDLE;
//	  		  break;
//	  	  case IDLE:
//	  		  state = DB;
//	  		  break;
//	  	  case DB:
//	  		  if(db() == OK){
//	  			  state = K1;
//	  		  }
//	  		  break;
//	  	  case K1:
//	  		  TIM11->CCR1 = CCR_DB + 100;
//	  		  HAL_Delay(6000);
//	  		  CCR_K1 = TIM11->CCR1;
//	  		  Speed_K1 = speed;
//
//	  		  TIM11->CCR1 = TIM11->CCR1 * 2;
//	  		  state = K2;
//	  		  break;
//
//	  	  case K2:
//	  		  break;
//	  	  case Tau0:
//	  		  TIM11->CCR1 = CCR_K1;
//	  		  HAL_Delay(5000);
//	  		  TIM11->CCR1 = CCR_K2;
//	  		  state = Tau;
//	  		  break;
//	  	  case Tau:
//	  		  break;
//	  	  default:
//	  		  state = IDLE;
//	  		  break;
//
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 41;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 159;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 9;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1119;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : Z_Pin A_Pin */
  GPIO_InitStruct.Pin = Z_Pin|A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B_Pin */
  GPIO_InitStruct.Pin = B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : K_Pin */
  GPIO_InitStruct.Pin = K_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(K_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PF6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PG6 PG7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == A_Pin)
	{
		counter = counter + 1;
		dir = HAL_GPIO_ReadPin(B_GPIO_Port, B_Pin);
		if (dir == 1){
			pos_counter = pos_counter - (1);
			if(pos_counter == -1){
				pos_counter = 1023;
			}
			//pos = pos + (360/1024);
		}
		else if (dir == 0){
			pos_counter = pos_counter + (1);
		}
	}

	if (GPIO_Pin == Z_Pin)
	{
		pos_counter = 0;
	}
	pos = ((pos_counter * 360) / 1024);


	if(GPIO_Pin == K_Pin){
		state = INIT2;
	}


}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6)
	{
		speed = ((counter * 12000) / 1024);
		counter = 0;

//		if(state == Tau0){
//			Tau_value = 0;
//		}
//		if(state == Tau){
//			if(speed < Speed_K1 + 0.63 * (Speed_K2 - Speed_K1)){
//				Tau_value = Tau_value + 5;
//			}
//		}
	}

	if (htim->Instance == TIM7)
	{
		if(state == INIT2){
			ek = Setpoint - speed;
			uk = uk_1 + ek * (1000*Kp + Ki * Ts)/1000 - ek_1 * Kp;
			ek_1 = ek;
			uk_1 = uk;

			Vout2PWM(uk);

			setDir(dir);
			setDuty(duty);
		}

	}

//	if (htim->Instance == TIM7)
//	{
//		if(state == DB){
//			TIM11->CCR1 = TIM11->CCR1 + 10;
//		}
////		if(state == K1){
////			if((speed - speed_previous) < 20){
////				if((speed - speed_previous) > -20){
////			  		CCR_K1 = TIM->CCR1;
////			  		Speed_K1 = speed;
////
////			  		TIM->CCR1 =
////			  		state = K2;
////				}
////			}
////		}

//		if(state == K2){
//			if((speed - speed_previous) < 10){
//				if((speed - speed_previous) > -10){
//			  		CCR_K2 = TIM11->CCR1;
//			  		Speed_K2 = speed;
//
//			  		k_value = ((Speed_K2 - Speed_K1) / (CCR_K2 - CCR_K1)) * (1119 / 12);
//			  		state = Tau0;
//				}
//			}
//			speed_previous = speed;
//		}
//	}

}


//result_t init(void){
//	return OK;
//}
//result_t idle(void){
//	CCR_DB = 0;
//	TIM11->CCR1 = 0;
//	return OK;
//}
//result_t db(void){
//	if(speed > 15){
//		CCR_DB = TIM11->CCR1;
//		return OK;
//	}
//	else{
//		return ERR;
//	}
//}
//result_t k1(void){
//	return OK;
//}
//result_t k2(void){
//	return OK;
//}
//result_t tau(void){
//	return OK;
//}



void setDir(uint8_t dir){
	if(dir == 0){
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, 0);
	}
	if(dir == 1){
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, 1);
	}
}

void setDuty(uint16_t duty){
	TIM11->CCR1 = (duty * 1119) / 1000;
}

void setTs(uint8_t Ts){
	TIM7->ARR = ((Ts * (84000000 / (839 + 1))) / 1000) - 1;
} // 839 : PSC

void Vout2PWM(float u){
	if(u > 12){
		u = 12;
	}
	else if(u < -12){
		u = -12;
	}

	duty = (u * 1000) / 12;

	if(u > 0){
		dir = 0;
	}
	else if(u <= 0){
		dir = 1;
	}
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
