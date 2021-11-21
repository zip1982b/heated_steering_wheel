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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "ntc.h"
#include "ntcConfig.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t adcResult = 0;
uint32_t Ust = 37;
uint32_t hist = 1;
uint32_t state = 0;
GPIO_PinState currentButtonState;
GPIO_PinState ill_State;

float Vsupply = 3.2; //power supply voltage (3.3 V rail) -STM32 ADC pin is NOT 5 V tolerant
float Vout; //Voltage divider output
float R_NTC; //NTC thermistor resistance in Ohms
float R_10k = 10000; //10k resistor measured resistance in Ohms (other element in the voltage divider)
float B_param = 7000; //B-coefficient of the thermistor
float T0 = 298.15; //25°C in Kelvin
float Temp_K; //Temperature measured by the thermistor (Kelvin)
float Temp_C; //Temperature measured by the thermistor (Celsius)
int TempC = 0;


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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    adcResult = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    
    Vout = adcResult * (3.2/4095);
    R_NTC = (Vout * R_10k) /(Vsupply - Vout); //calculating the resistance of the thermistor
    
    Temp_K = (T0*B_param)/(T0*log(R_NTC/R_10k)+B_param); //Temperature in Kelvin
    Temp_C = Temp_K - 273.15; //converting into Celsius
    TempC = (int)Temp_C;
    
    currentButtonState = HAL_GPIO_ReadPin(GPIOA, switch_Pin);
    ill_State = HAL_GPIO_ReadPin(GPIOB, ILL_Pin);
    
    switch(state){
    case 0:
      HAL_GPIO_WritePin(GPIOA, heater_Pin, GPIO_PIN_RESET); // heater off
      HAL_GPIO_WritePin(GPIOB, ILL_Pin, GPIO_PIN_RESET); // illumination off
      if (currentButtonState == GPIO_PIN_RESET){
        state = 10;
      }
      break;
    case 10:
       if (currentButtonState == GPIO_PIN_SET){
        state = 1;
      }
      break;
    case 1:
      HAL_GPIO_WritePin(GPIOB, ILL_Pin, GPIO_PIN_SET); // illumination on
      if(TempC <= (Ust - hist)){
        HAL_GPIO_WritePin(GPIOA, heater_Pin, GPIO_PIN_SET); // heater on
      }
      else if(TempC >= (Ust + hist)){
        HAL_GPIO_WritePin(GPIOA, heater_Pin, GPIO_PIN_RESET); // heater off
      }
      
      if(TempC >= 45){
        HAL_GPIO_WritePin(GPIOA, heater_Pin, GPIO_PIN_RESET);
        state = 0;
        HAL_GPIO_WritePin(GPIOB, ILL_Pin, GPIO_PIN_RESET); 
      }
      if (currentButtonState == GPIO_PIN_RESET){
        state = 11;
      }
      break;
    case 11:
      if (currentButtonState == GPIO_PIN_SET){
        state = 0;
      }
      break;
    default:
      HAL_GPIO_WritePin(GPIOA, heater_Pin, GPIO_PIN_RESET); // heater off
      HAL_GPIO_WritePin(GPIOB, ILL_Pin, GPIO_PIN_RESET); // illumination off
      state = 0;
      break;
    }
    //TempC = 0;
    
    
    /*
    if (state == 0 && currentButtonState == GPIO_PIN_RESET){
      state = 10;
    }
    else if (state == 10 && currentButtonState == GPIO_PIN_SET){
      state = 1;
    }
    
    if (state == 1){
      HAL_GPIO_WritePin(GPIOB, ILL_Pin, GPIO_PIN_SET); 
      if(TempC <= (Ust - hist)){
        HAL_GPIO_WritePin(GPIOA, heater_Pin, GPIO_PIN_SET); 
      }
      else if(TempC >= (Ust + hist)){
        HAL_GPIO_WritePin(GPIOA, heater_Pin, GPIO_PIN_RESET); 
      }
    }

    if (state == 1 && currentButtonState == GPIO_PIN_RESET){
      state = 11;
    }
    else if (state == 11 && currentButtonState == GPIO_PIN_SET){
      state = 0;
      HAL_GPIO_WritePin(GPIOA, heater_Pin, GPIO_PIN_RESET); 
      HAL_GPIO_WritePin(GPIOB, ILL_Pin, GPIO_PIN_RESET); 
    }
    else if (state == 0){
      HAL_GPIO_WritePin(GPIOA, heater_Pin, GPIO_PIN_RESET); 
      HAL_GPIO_WritePin(GPIOB, ILL_Pin, GPIO_PIN_RESET);
    }
    
    if(TempC >= 45){
      HAL_GPIO_WritePin(GPIOA, heater_Pin, GPIO_PIN_RESET);
      state = 0;
      HAL_GPIO_WritePin(GPIOB, ILL_Pin, GPIO_PIN_RESET); 
    }
    */
    
    
    HAL_Delay(100);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
