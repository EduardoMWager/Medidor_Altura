/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Eduardo Wagner
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define NUM_VALORES 10  // Número de valores para calcular a média
float V_buffer[5][NUM_VALORES];  // Buffer para armazenar os últimos 10 valores de cada canal
int indice = 0;  // �?ndice para inserir os novos valores
uint16_t V[5];  // Array para armazenar os valores ADC
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
float altura = 0.0;  // Variável para armazenar a altura calculada
float medidas[5];
float coef_const = 3.3165;       // 4.1751;
float coef_x1 = 0.0006;          //0.0003;
float coef_x2 = -0.0005;         //-0.0003;
float coef_x3 = 0.0003;          // 0.0003;
float coef_x4 = 0.00008897;      //0.00007219;
float coef_x5 = -0.0003;         //-0.0005;
int altura_int;
char buffer[50];  // Buffer para transmissão de dados via UART

float medidas_suavizadas[5] = {0};  // Array para armazenar valores suavizados
int contagem = 0;  // Contador para calcular a média móvel
float medidas_filtradas[5];  // Array para armazenar valores filtrados
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void sendHeighToCubeMonitor(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Função para calcular a altura com base nos coeficientes do modelo de regressão */

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
  MX_ADC1_Init();
  MX_TIM10_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  hadc1.Init.ContinuousConvMode = ENABLE;
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim10);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)V, 5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if (__HAL_TIM_GET_FLAG(&htim10,TIM_FLAG_UPDATE)) {
	  		// Limpa a flag de atualização do Timer 10
	  		__HAL_TIM_CLEAR_FLAG(&htim10, TIM_FLAG_UPDATE);

	  		int altura_int = (int)(altura);  // Multiplica por 100 para preservar 2 casas decimais
	  		  snprintf(buffer, sizeof(buffer), "%d.%02d\r\n", altura_int / 100, altura_int % 100);
	  		  HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buffer, strlen(buffer));
	  		}
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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1)
{
    // Copiar os valores de V para medidas
    for (int i = 0; i < 5; i++) {
        medidas[i] = (float)V[i];

        // Armazenar o novo valor no buffer
        V_buffer[i][indice] = medidas[i];

        // Calcular a média dos 10 valores
        float soma = 0.0;
        for (int j = 0; j < NUM_VALORES; j++) {
            soma += V_buffer[i][j];
        }
        medidas_filtradas[i] = soma / NUM_VALORES;
    }

    // Cálculo da altura usando os valores filtrados de cada canal
    altura = coef_const
           + coef_x1 * medidas_filtradas[0]
           + coef_x2 * medidas_filtradas[1]
           + coef_x3 * medidas_filtradas[2]
           + coef_x4 * medidas_filtradas[3]
           + coef_x5 * medidas_filtradas[4];

    // Atualizar o índice para o próximo valor
    indice = (indice + 1) % NUM_VALORES;  // Circular
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
