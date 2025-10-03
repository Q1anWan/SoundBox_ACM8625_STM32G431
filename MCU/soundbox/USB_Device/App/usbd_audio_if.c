/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_audio_if.c
  * @version        : v3.0_Cube
  * @brief          : Generic media access layer.
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
#include "usbd_audio_if.h"

/* USER CODE BEGIN INCLUDE */
#include "main.h"
#include "stm32g4xx_hal_sai.h"
#include <stdint.h>
#include <string.h>

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_AUDIO_IF
  * @{
  */

/** @defgroup USBD_AUDIO_IF_Private_TypesDefinitions USBD_AUDIO_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Defines USBD_AUDIO_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Macros USBD_AUDIO_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

uint32_t call_times[2];
uint32_t call_timelast[2];
uint32_t time_d[2];

int32_t audio_data[128];

/**
  * @brief  Controls AUDIO Volume with high resolution (16-bit raw value).
  * @param  vol_raw: Raw volume value from USB host (-32768 to 32767, maps to 0.0f-1.0f)
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */

#include "arm_math.h"
/* 配置 */
#define RAW_MUTE  ((int32_t)-32768)  /* 0x8000 -> mute / -inf dB */
#define RAW_MIN   ((int32_t)-32767)  /* 最小有效负值 */
#define RAW_MAX   ((int32_t) 32768)  /* 最大有效正值 */
#define DB_MAX    (127.9961f)        /* 你给的最大 dB */

/* 预计算一次性的最大 gain（对应 +DB_MAX） */
static float precomputed_gain_max(void) {
    /* powf(10, DB_MAX/20) */
    return powf(10.0f, DB_MAX * 0.05f);
}

/* raw -> 线性归一化振幅 [0.0 .. 1.0]（0 表示静音）
   说明：
   - RAW_MUTE 表示静音，直接返回 0
   - raw == 0 -> 0 dB -> 返回一个非常小但非零值（除非你希望 0 对应 1.0）
   - 负值与正值分别归一化（负侧除以 32767，正侧除以 32768）以保证边界精确 */
float raw_to_linear_amplitude(int32_t raw) {
    if (raw == RAW_MUTE) return 0.0f; /* mute */

    const float gain_max = precomputed_gain_max();

    /* 计算 dB */
    float dB;
    if (raw > 0) {
        /* 正侧：+1 .. +32768 -> 0 .. +DB_MAX */
        dB = ((float)raw / (float)RAW_MAX) * DB_MAX;
    } else if (raw < 0) {
        /* 负侧： -1 .. -32767 -> 0 .. -DB_MAX （注意分母用 32767） */
        dB = ((float)raw / (float)(-RAW_MIN - 1)) * DB_MAX; /* -RAW_MIN-1 == 32767 */
    } else { /* raw == 0 */
        dB = 0.0f;
    }

    /* dB -> 振幅 gain */
    float gain = powf(10.0f, dB / 20.0f);

    /* 归一化，使得 +DB_MAX 对应 1.0 */
    return gain / gain_max;
}
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Variables USBD_AUDIO_IF_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN PRIVATE_VARIABLES */
/* Debug counters for monitoring USB Audio control calls */
static uint32_t volume_call_count = 0;
static uint32_t mute_call_count = 0;

/* Volume control stability variables */
static uint32_t vol_zero_count = 0;     /* 连续收到0的次数 */
#define VOL_ZERO_THRESHOLD 3             /* 连续收到多少次0才认为是真正的静音 */
// #define VOL_CHANGE_MIN_TIME 50           /* 音量变化最小时间间隔(ms) */
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Exported_Variables USBD_AUDIO_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */
extern Audio_t audio;
extern SAI_HandleTypeDef hsai_BlockA1;
extern DMA_HandleTypeDef hdma_sai1_a;
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_FunctionPrototypes USBD_AUDIO_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t AUDIO_Init_FS(uint32_t AudioFreq, uint32_t Volume, uint32_t options);
static int8_t AUDIO_DeInit_FS(uint32_t options);
static int8_t AUDIO_AudioCmd_FS(uint8_t* pbuf, uint32_t size, uint8_t cmd);
// static int8_t AUDIO_VolumeCtl_FS(uint8_t vol);
static int8_t AUDIO_MuteCtl_FS(uint8_t cmd);
static int8_t AUDIO_PeriodicTC_FS(uint8_t *pbuf, uint32_t size, uint8_t cmd);
static int8_t AUDIO_GetState_FS(void);
static int8_t AUDIO_VolumeCtlHighRes_FS(int16_t vol_raw);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
/* Debug function to check USB Audio control status */
void usb_audio_debug_info(uint32_t *vol_calls, uint32_t *mute_calls, 
                         uint8_t *last_vol, uint8_t *last_mute);

/* High precision volume interface functions */
float usb_audio_get_volume_float(void);
int16_t usb_audio_get_volume_raw(void);
// uint8_t usb_audio_get_volume_percent(void);
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops_FS =
{
  AUDIO_Init_FS,
  AUDIO_DeInit_FS,
  AUDIO_AudioCmd_FS,
  // AUDIO_VolumeCtl_FS,
  AUDIO_MuteCtl_FS,
  AUDIO_PeriodicTC_FS,
  AUDIO_GetState_FS,
  AUDIO_VolumeCtlHighRes_FS,  /* 高分辨率音量控制回调 */
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the AUDIO media low layer over USB FS IP
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param  options: Reserved for future use
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_Init_FS(uint32_t AudioFreq, uint32_t Volume, uint32_t options)
{
  /* USER CODE BEGIN 0 */
  // UNUSED(AudioFreq);
  // UNUSED(Volume);
  // UNUSED(options);
  audio.audio_freq = AudioFreq;

  /* 初始化高精度音量值 - 默认使用float存储 */
  audio.volume = raw_to_linear_amplitude(Volume);
  audio.vol_raw = Volume;  /* 默认音量0dB */    

  return (USBD_OK);
  /* USER CODE END 0 */
}

/**
  * @brief  De-Initializes the AUDIO media low layer
  * @param  options: Reserved for future use
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_DeInit_FS(uint32_t options)
{
  /* USER CODE BEGIN 1 */
  UNUSED(options);
  return (USBD_OK);
  /* USER CODE END 1 */
}

/**
  * @brief  Handles AUDIO command.
  * @param  pbuf: Pointer to buffer of data to be sent
  * @param  size: Number of data to be sent (in bytes)
  * @param  cmd: Command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_AudioCmd_FS(uint8_t* pbuf, uint32_t size, uint8_t cmd)
{
  /* USER CODE BEGIN 2 */
  // switch(cmd)
  // {
  //   case AUDIO_CMD_START:
  //   break;

  //   case AUDIO_CMD_PLAY:
  //   break;
  // }
  // audio.play_state = cmd;
  // audio.data_size = size;

  // audio.play_state = cmd;
  // audio.data_size = size;
  call_times[0]++;
  // audio.data0 = pbuf[0];
  // HAL_SAI_Transmit_DMA(&hsai_BlockA1, pbuf, size>>2);
  

  time_d[0] = HAL_GetTick() - call_timelast[0];
  call_timelast[0] = HAL_GetTick();

  return (USBD_OK);
  /* USER CODE END 2 */
}

// /**
//   * @brief  Controls AUDIO Volume.
//   * @param  vol: volume level (0..100)
//   * @retval USBD_OK if all operations are OK else USBD_FAIL
//   */
// static int8_t AUDIO_VolumeCtl_FS(uint8_t vol)
// {
//   /* USER CODE BEGIN 3 */
  
//   audio.volume = (float)vol / 100.0f;  /* 更新主音量float值 */
  
//   /* 更新调试计数器 */
//   volume_call_count++;
//   last_volume_value = vol;
  
//   /* 这里可以添加实际的音量控制硬件操作 */
//   /* 例如：控制DAC的音量寄存器，或者调整DMA传输的增益 */
  
//   return (USBD_OK);
//   /* USER CODE END 3 */
// }

/**
  * @brief  Controls AUDIO Mute.
  * @param  cmd: command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_MuteCtl_FS(uint8_t cmd)
{
  /* USER CODE BEGIN 4 */
  audio.mute = cmd;
  
  /* 更新调试计数器 */
  mute_call_count++;

  if(audio.vol_raw == -32768 && cmd == 1) {
    audio.volume = 0.0f;  /* 主音量float值 */
  }
  
  return (USBD_OK);
  /* USER CODE END 4 */
}



static int8_t AUDIO_VolumeCtlHighRes_FS(int16_t vol_raw)
{
  /* USER CODE BEGIN 5 */
  audio.vol_raw = vol_raw;  /* 存储原始音量值 */
  /* 防止音量跳跃的逻辑 */
  if (vol_raw == -32768)  /* 静音值 */
  {
    /* 只有连续收到多次-32768，或者距离上次音量更新时间较长，才认为是真正的静音 */
    if (++vol_zero_count > VOL_ZERO_THRESHOLD)
    {
      /* 真正的静音操作 */
      audio.volume = 0.0f;  /* 主音量float值 */
    }
    else
    {
      /* 疑似跳跃，保持上次的有效值 */
      return (USBD_OK);
    }
  }
  else
  {
    vol_zero_count = 0;  /* 重置连续0计数 */
    /* 存储有效的音量值 */

    /* 计算主音量float值 (0.0-1.0) 使用-32768到32767的范围 */
    /* 将-32768映射到0.0f，32767映射到1.0f */
    audio.volume = raw_to_linear_amplitude(vol_raw);
    // audio.volume = (float)(vol_raw) / 32767.0f;
    // if (audio.volume > 1.0f) audio.volume = 1.0f;
    // if (audio.volume < 0.0f) audio.volume = 0.0f;
  }
  
  /* 更新调试计数器 */
  volume_call_count++;
  /* 这里可以添加实际的高精度音量控制硬件操作 */
  /* 例如：使用audio.volume直接控制DAC，获得0.0-1.0精度 */
  /* 示例：dac_set_volume_float(audio.volume); */
  
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  AUDIO_PeriodicT_FS
  * @param  cmd: Command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_PeriodicTC_FS(uint8_t *pbuf, uint32_t size, uint8_t cmd)
{
  /* USER CODE BEGIN 5 */
  // UNUSED(pbuf);
  // UNUSED(size);
  // UNUSED(cmd);

  // audio.play_state = cmd;
  audio.data_size = size;
  memcpy(&audio.data0, pbuf, 4);
  call_times[1]++;
  HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*)audio_data, sizeof(audio_data));
  

  time_d[1] = HAL_GetTick() - call_timelast[1];
  call_timelast[1] = HAL_GetTick();

  // audio.data_size = size;
  // audio.play_state = cmd;
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Gets AUDIO State.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_GetState_FS(void)
{
  /* USER CODE BEGIN 6 */
  return audio.play_state;
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  Manages the DMA full transfer complete event.
  * @retval None
  */
void TransferComplete_CallBack_FS(void)
{
  /* USER CODE BEGIN 7 */
  USBD_AUDIO_Sync(&hUsbDeviceFS, AUDIO_OFFSET_FULL);
  /* USER CODE END 7 */
}

/**
  * @brief  Manages the DMA Half transfer complete event.
  * @retval None
  */
void HalfTransfer_CallBack_FS(void)
{
  /* USER CODE BEGIN 8 */
  USBD_AUDIO_Sync(&hUsbDeviceFS, AUDIO_OFFSET_HALF);
  /* USER CODE END 8 */
}

/**
 * @brief Get current volume as float value (0.0-1.0) - 主音量接口
 * @retval float volume value in range 0.0-1.0
 */
float usb_audio_get_volume_float(void)
{
  return audio.volume;  /* 直接返回主音量float值 */
}

/**
 * @brief Get current volume as raw 16-bit value (-32768 to 32767, maps to 0.0f-1.0f)
 * @retval int16_t raw volume value in range -32768 to 32767
 */
int16_t usb_audio_get_volume_raw(void)
{
  return audio.vol_raw;
}

/**
 * @brief Get current volume as percentage (0-100)
 * @retval uint8_t volume percentage in range 0-100
 */
// uint8_t usb_audio_get_volume_percent(void)
// {
//   return audio.vol_percent;
// }
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
