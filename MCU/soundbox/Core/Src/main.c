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
#include <stdint.h>
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
/* 音频缓冲区约10ms (48000 * 0.01 * 2 * 4 = 3840字节) - 适合32KB RAM */
uint8_t audio_data_temp[480*2*4];  /* 10ms @ 48kHz, stereo, 32-bit */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void fill_test_tone_32bit_pcm(uint8_t *buf, size_t buf_bytes, uint32_t sample_rate, float freq_hz);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

Audio_t audio;

extern void TransferComplete_CallBack_FS(void);
extern void HalfTransfer_CallBack_FS(void);

/* DMA传输完成回调 - 循环模式不需要重启 */
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai){
  /* 循环DMA模式下，硬件自动重启，无需手动操作 */
  
  /* 调用USB音频回调 */
  // TransferComplete_CallBack_FS();
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai){
  // HalfTransfer_CallBack_FS();
}

#define ACM_I2C_ADDR       0x58        /* 7-bit device addr (ADR=15k => 0x5A) */
#define ACM_I2C_TIMEOUT    2000

static HAL_StatusTypeDef acm_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return HAL_I2C_Master_Transmit(&hi2c1, ACM_I2C_ADDR, buf, 2, ACM_I2C_TIMEOUT);
}

static HAL_StatusTypeDef acm_read_reg(uint8_t reg, uint8_t *pval)
{
    HAL_StatusTypeDef st;
    st = HAL_I2C_Master_Transmit(&hi2c1, ACM_I2C_ADDR, &reg, 1, ACM_I2C_TIMEOUT);
    if (st != HAL_OK) return st;
    return HAL_I2C_Master_Receive(&hi2c1, ACM_I2C_ADDR, pval, 1, ACM_I2C_TIMEOUT);
}

HAL_StatusTypeDef xxx;
/* --- ACM8625P init sequence (minimal) --- */
void ACM_PowerUp_And_Init(void)
{
    HAL_Delay(10);

    /* 1) Ensure PDN is high (PDN low = shutdown) */
    HAL_GPIO_WritePin(PDN_GPIO_Port, PDN_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
  
    /* 2) Optionally clear faults (write FAULT_CLR bit in AMP_CTRL1 reg 0x01) */
    xxx = acm_write_reg(0x01, 0x80); /* FAULT_CLR = 1 (auto-clear) */

    /* 3) Configure I2S format: write register 0x07
       Bits [5:4] = I2S format (00 = I2S), Bits [1:0] = word length (00 = 16-bit)
       So 0x00 -> I2S + 16-bit + default 44k flags off.
       0x03 -> I2S + 32-bit + 48kHz (for 48kHz input)
    */
    xxx = acm_write_reg(0x07, 0x03); 

    /* 4) (Optional) Set volumes (0x0F, 0x10). Default is 0xD0; keep as default or change. */
    xxx = acm_write_reg(0x0F, 0xC0); /* left volume example */
    xxx = acm_write_reg(0x10, 0xC0); /* right volume example */

    HAL_Delay(5);

    /* 5) Set STATE_CTRL (reg 0x04) CTRL_STATE = 0b11 (Play) -> set lower 2 bits = 3 */
    /* Keep other bits default (muting bits = 0) */
    xxx = acm_write_reg(0x04, 0x03);

    HAL_Delay(10);
}

/* 对齐模式常量 */
#define ALIGN_LEFT     0   /* 16-bit/24-bit left-aligned (常见) */
#define ALIGN_RIGHT    1   /* 16-bit right-aligned in low bits */


/* 填充 32-bit PCM (signed) 的函数 - 左声道sin，右声道cos */
void fill_test_tone_32bit_pcm(uint8_t *buf, size_t buf_bytes,
                             uint32_t sample_rate, float freq_hz)
{
    const uint32_t channels = 2;
    const uint32_t bytes_per_slot = 4; /* 32-bit slot */
    size_t frames = buf_bytes / (channels * bytes_per_slot);
    const float two_pi = 2.0f * 3.14159265358979323846f;

    /* 振幅：int32 安全值，避免接近饱和（不要用 0x7FFFFFFF） */
    const float max32 = (float)0x5FFFF; /* 30-bit safe amplitude */

    /* 保持原始的相位增量，确保频率准确 */
    float phase_inc = two_pi * freq_hz / (float)sample_rate;
    
    /* 对于循环DMA，我们生成标准的正弦波，让硬件处理循环 */
    /* 虽然可能有轻微的相位跳跃，但保证波形的正确性 */

    for (size_t n = 0; n < frames; ++n) {
        float phase = n * phase_inc;
        
        /* 左声道：sin信号，右声道：cos信号 */
        float sin_val = sinf(phase);     /* -1..1 */
        float cos_val = cosf(phase);     /* -1..1 */

        int32_t left_sample = (int32_t)(sin_val * max32);
        int32_t right_sample = (int32_t)(cos_val * max32);

        /* 小端写入：每个 slot 一个 32-bit signed sample */
        size_t idx = n * channels * bytes_per_slot;

        /* left channel (sin) */
        buf[idx + 0] = (uint8_t)(left_sample & 0xFF);
        buf[idx + 1] = (uint8_t)((left_sample >> 8) & 0xFF);
        buf[idx + 2] = (uint8_t)((left_sample >> 16) & 0xFF);
        buf[idx + 3] = (uint8_t)((left_sample >> 24) & 0xFF);

        /* right channel (cos) */
        buf[idx + 4] = (uint8_t)(right_sample & 0xFF);
        buf[idx + 5] = (uint8_t)((right_sample >> 8) & 0xFF);
        buf[idx + 6] = (uint8_t)((right_sample >> 16) & 0xFF);
        buf[idx + 7] = (uint8_t)((right_sample >> 24) & 0xFF);
    }
}

/* 使用示例：在 SAI/hal 初始化并且 ACM 初始化完后调用一次 */
void prepare_and_start_32bit_playback(void)
{
    /* 使用500Hz测试信号：480帧缓冲区正好包含5个完整周期(480*500/48000=5) */
    /* 这样循环DMA播放时首尾能完美连接，左声道sin，右声道cos */
    fill_test_tone_32bit_pcm(audio_data_temp, sizeof(audio_data_temp), 48000, 500.0f);

    /* HAL_SAI_Transmit_DMA 的 Size 参数应为 32-bit 单元数（不是字节数） */
    uint32_t words32 = sizeof(audio_data_temp) / 4; /* 3840 / 4 = 960 */

    /* 启动循环DMA传输（一次启动，硬件自动循环播放无缝的波形） */
    HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*)audio_data_temp, words32);
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

  ACM_PowerUp_And_Init();
  prepare_and_start_32bit_playback();
  

  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* DMA传输由回调函数自动重启，这里只需要处理其他任务 */
    HAL_Delay(100);  /* 可以用来处理其他任务或降低CPU占用 */

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
