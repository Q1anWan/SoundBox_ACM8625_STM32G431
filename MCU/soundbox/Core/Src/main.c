/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "sai.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <math.h>
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
// Global test arrays - use volatile to prevent optimization
volatile float a[4];
volatile float b[4];
volatile float c[4];

// Global variables for CMSIS-DSP test results - prevent optimization
volatile float32_t mean_value, variance, std_dev;
volatile float32_t dot_product;

// Matrix test data - prevent optimization
volatile float32_t mat_a_data[4] = {1.0f, 2.0f, 3.0f, 4.0f};
volatile float32_t mat_b_data[4] = {5.0f, 6.0f, 7.0f, 8.0f};
volatile float32_t mat_result_data[4];

// Matrix instances
arm_matrix_instance_f32 mat_a, mat_b, mat_result;


extern SAI_HandleTypeDef hsai_BlockA1;
extern DMA_HandleTypeDef hdma_sai1_a;
Audio_t audio = {0};

extern void TransferComplete_CallBack_FS(void);
extern void HalfTransfer_CallBack_FS(void);

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai){
  TransferComplete_CallBack_FS();
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai){
  HalfTransfer_CallBack_FS();
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
  MX_I2C1_Init();
  MX_SAI1_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */
  // __HAL_DMA_ENABLE_IT(&hdma_sai1_a, DMA_IT_HT);
  // __HAL_DMA_ENABLE_IT(&hdma_sai1_a, DMA_IT_TC);


  // Initialize test data
  for(int i = 0; i < 4; i++) {
    a[i] = (float)(i + 1);  // 1.0, 2.0, 3.0, 4.0
    b[i] = (float)(i * 2);  // 0.0, 2.0, 4.0, 6.0
  }
  
  // Initialize matrix instances (do this once)
  arm_mat_init_f32(&mat_a, 2, 2, (float32_t*)mat_a_data);
  arm_mat_init_f32(&mat_b, 2, 2, (float32_t*)mat_b_data);
  arm_mat_init_f32(&mat_result, 2, 2, (float32_t*)mat_result_data);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Test CMSIS-DSP vector addition - cast volatile to non-volatile
    arm_add_f32((float32_t*)a, (float32_t*)b, (float32_t*)c, 4);
    
    // Test CMSIS-DSP statistics
    arm_mean_f32((float32_t*)a, 4, (float32_t*)&mean_value);
    arm_var_f32((float32_t*)a, 4, (float32_t*)&variance);
    arm_std_f32((float32_t*)a, 4, (float32_t*)&std_dev);
    
    // Test CMSIS-DSP basic math
    arm_dot_prod_f32((float32_t*)a, (float32_t*)b, 4, (float32_t*)&dot_product);
    
    // Test CMSIS-DSP matrix operations
    arm_mat_mult_f32(&mat_a, &mat_b, &mat_result);
    
    // Force compiler to keep variables by using them
    // This prevents dead code elimination
    if (mean_value > 1000.0f || dot_product < -1000.0f || mat_result_data[0] > 1000.0f) {
      // This condition should never be true with our test data
      // but prevents the compiler from optimizing away the calculations
      Error_Handler();
    }
    
    // Add a delay to prevent overwhelming the system
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 83;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
#ifdef USE_FULL_ASSERT
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
