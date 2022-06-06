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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "oled.h"
#include "adc.h"
#include "soc.h"
#include "sochelper.h"
#include "power.h"
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
DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */

typedef enum {
	CHG_ENABLE = 1, CHG_DISABLE = 0
} CHG_EN;

typedef enum {
	IDLE = 0, CHG = 1, DCHG = 2
} STATE;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

STATE currentState = IDLE;
uint8_t currentDchgPct = 0;
static float lastReadBattV = 0;
static float lastReadCurr_mA = 0;
static float currentCellSOC = 0;
static float currChargeRemaining = 0; //Ah how much charge is remaining inside the cell
static const float fullChargeCapacity = 3.0 ; //Ah //TODO: Change this as per your cell
static float lastComputedPower = 0;

void Charging_Enable(CHG_EN chg_en);
void Change_State(STATE new_state);
void Discharging_Set(uint8_t pct);
void Safety_Loop();
void calcSOC(float ocv_V, float chargeRemain_Ah);
void getLatestADCValues();
void updateOLED();
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
  MX_LPUART1_UART_Init();
  MX_DAC1_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  //Initialize OLED display
  	ssd1306_Init();

  	ssd1306_Fill(Black);
  	ssd1306_WriteString("MakerMax", Font_7x10, White);

  	//Initial ADC
  	LTC2990_ConfigureControlReg(&hi2c1);

  	//Initialize TIM7 to run safety loop
  	HAL_TIM_Base_Start_IT(&htim7);

  	//Init TIM16 to run SOC loop
  	HAL_TIM_Base_Start_IT(&htim16);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	while (1) {
  		getLatestADCValues();
  		calcSOC(lastReadBattV, currChargeRemaining);
  		lastComputedPower = computePower(lastReadBattV);
  		updateOLED();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization 
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_7B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  htim6.Init.Prescaler = 36000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2000;
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
  htim7.Init.Prescaler = 36000;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 2000;
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
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 36000;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 4000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 36000;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 10000;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_USR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CHG_EN_Pin|LED_USR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD2_Pin LED_USR2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LED_USR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CHG_EN_Pin LED_USR1_Pin */
  GPIO_InitStruct.Pin = CHG_EN_Pin|LED_USR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : S2_INTERRUPT_Pin S1_INTERRUPT_Pin */
  GPIO_InitStruct.Pin = S2_INTERRUPT_Pin|S1_INTERRUPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim6) {
		HAL_GPIO_TogglePin(LED_USR1_GPIO_Port, LED_USR1_Pin);
		//If both buttons S1 and S1 pressed
		if (HAL_GPIO_ReadPin(GPIOA, S1_INTERRUPT_Pin) == GPIO_PIN_RESET
				&& HAL_GPIO_ReadPin(GPIOA, S2_INTERRUPT_Pin)
						== GPIO_PIN_RESET) {
			if (currentState == IDLE) {
				Change_State(CHG);
			} else if (currentState == CHG) {
				Change_State(DCHG);
			} else if (currentState == DCHG) {
				Change_State(IDLE);
			} else { //This should not trigger
				Change_State(IDLE);
			}
		}
		//If either S2 or S1 is pressed while in DCHG state, then change discharge current
		else if (currentState == DCHG && HAL_GPIO_ReadPin(GPIOA, S1_INTERRUPT_Pin) == GPIO_PIN_RESET) {
			uint8_t newDchgPct = currentDchgPct + 10;
			//Increase discharge current
			Discharging_Set(newDchgPct);
		}
		else if (currentState == DCHG
				&& HAL_GPIO_ReadPin(GPIOA, S2_INTERRUPT_Pin) == GPIO_PIN_RESET) {
			uint8_t newDchgPct = currentDchgPct - 10;
			//Decrease discharge current
			Discharging_Set(newDchgPct);
		}
		HAL_TIM_Base_Stop_IT(&htim6);
	} else if (htim == &htim7) {
		Safety_Loop(); //Safety loop run every 0.5 seconds
	} else if (htim == &htim16) { //This timer ticks every 1 second and is used for charge remaining calculations
		if (currentState == DCHG || currentState == CHG) {
			//Supply Charge Remaining with updated current
			currChargeRemaining += calcdeltaAh(1, lastReadCurr_mA / 1000.0);
		}
	} else if (htim == &htim17) {
		//This timer ticks every 10 seconds and is used for polarization calculations
		//If state is idle //TODO: HOMEWORK
		//Start counting rest time
	}
}

/*
 * Gets the latest current and cell voltages, stores it in global variables
 */
void getLatestADCValues() {
	float battV_2 = 0;
	LTC2990_Trigger(&hi2c1);
	LTC2990_WaitForConversion(&hi2c1, 100);

	//Current reading
	LTC2990_ReadVoltage(&hi2c1, BATTV, &lastReadBattV);
	LTC2990_ReadVoltage(&hi2c1, BATTV_2, &battV_2);
	LTC2990_ReadCurrent(&hi2c1, lastReadBattV, battV_2, &lastReadCurr_mA);
}

/*
 * Displaying latest cell values on OLED display
 */
void updateOLED()
{
	//Show current on OLED
	char currentString[10];
	sprintf(currentString, "%.3f mA", lastReadCurr_mA);
	ssd1306_SetCursor(15, 37);
	ssd1306_WriteString("Cur  ", Font_7x10, White);
	ssd1306_SetCursor(52, 37);
	ssd1306_WriteString(currentString, Font_7x10, White);

	//Battery voltage
	char battVString[10];
	sprintf(battVString, "%.3f V", lastReadBattV);
	ssd1306_SetCursor(15, 49);
	ssd1306_WriteString("BattV   ", Font_7x10, White);
	ssd1306_SetCursor(52, 49);
	ssd1306_WriteString(battVString, Font_7x10, White);

	//SOC
	char socString[10];
	sprintf(socString, "%.2f  %", currentCellSOC);
	ssd1306_SetCursor(15, 25);
	ssd1306_WriteString("SOC   ", Font_7x10, White);
	ssd1306_SetCursor(52, 25);
	ssd1306_WriteString(socString, Font_7x10, White);

	//Power
	char powerString[10];
	sprintf(powerString, "%.2f  W", lastComputedPower);
	ssd1306_SetCursor(15, 14);
	ssd1306_WriteString("Pwr   ", Font_7x10, White);
	ssd1306_SetCursor(52, 14);
	ssd1306_WriteString(powerString, Font_7x10, White);

	ssd1306_UpdateScreen();
}

//TODO: HOMEWORK THIS FUNCTION NEEDS TO BE REFINED
void calcSOC(float ocv_V, float chargeRemain_Ah) {
	//Simple switch to start with
	if (currentState == IDLE) {
		//If cell is not polarized - This is an incomplete implementation TODO: HOMEWORK
		currentCellSOC = socByOCV(ocv_V);
		currChargeRemaining = (currentCellSOC / 100) * fullChargeCapacity;
		//IMMEDIATE , DIRECT CLAMP USING OCV
		//ROOM FOR IMPROVEMENT
		//SLOWLY CONVERGE SOCBYAH TO SOCBYOCV
	} else {
		//If cell is polarized use columb counting
		currentCellSOC = (currChargeRemaining/fullChargeCapacity)*100;
	}
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	HAL_GPIO_TogglePin(LED_USR1_GPIO_Port, LED_USR1_Pin);

	if (GPIO_Pin == S2_INTERRUPT_Pin || GPIO_Pin == S1_INTERRUPT_Pin) {

		//S2 has been pressed, start software debouncing
		HAL_TIM_Base_Start_IT(&htim6);
	}
}

//Where PCT should be 0 - 100
void Discharging_Set(uint8_t pct) {
	if (pct < 0) {
		pct = 0;
	} else if (pct > 100) {
		pct = 100;
	}
	//DAC is 12 bit resolution - 0 - 4095 data codes which translates to 0 - 3.3V analog

	uint32_t dacCode = (uint32_t) ((pct / 100.0) * 4095.0);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacCode);

	currentDchgPct = pct;

	//Start DAC
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
}

void Charging_Enable(CHG_EN chg_en) {
	if (chg_en == CHG_ENABLE) {
		HAL_GPIO_WritePin(GPIOB, CHG_EN_Pin, GPIO_PIN_SET);
	} else //Disabling charging
	{
		HAL_GPIO_WritePin(GPIOB, CHG_EN_Pin, GPIO_PIN_RESET);
	}
}

void Change_State(STATE new_state) {
	currentState = new_state;

	//IDLE
	if (currentState == IDLE) {
		Charging_Enable(CHG_DISABLE);
		Discharging_Set(0); //Set discharge current to 0A
	}
	//CHARGING
	else if (currentState == CHG) {
		Charging_Enable(CHG_ENABLE);
		Discharging_Set(0); //Set discharge current to 0A
	}
	//DISCHARGING
	else if (currentState == DCHG) {
		Charging_Enable(CHG_DISABLE);
		Discharging_Set(10); //Set discharge current to 10%
	} else {
		//HANDLE DEFAULT CASE - MISRA C
	}
}

void Safety_Loop() {
	//Undervoltage - 2.8V
	if (lastReadBattV < 2.8) {
		//Stop charging, stop discharging
		Change_State(IDLE);
}}

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
