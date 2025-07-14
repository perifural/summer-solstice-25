/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
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
#include "rtc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */
    HAL_PWR_EnableBkUpAccess();
    /* Enable BKP CLK enable for backup registers */
    __HAL_RCC_BKP_CLK_ENABLE();
    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint32_t RTC_GetUnixTime(void)
{
    uint32_t high1 = RTC->CNTH;
    uint32_t low = RTC->CNTL;
    uint32_t high2 = RTC->CNTH;

    if (high1 != high2) {
        // reread if overflow happened between reads
        low = RTC->CNTL;
        high1 = high2;
    }

    return (high1 << 16) | low;
}

void RTC_SetUnixTime(uint32_t seconds)
{
    RTC->CRL |= RTC_CRL_CNF;
    RTC->CNTH = seconds >> 16;
    RTC->CNTL = seconds & 0xFFFF;
    RTC->CRL &= ~RTC_CRL_CNF;
    while (!(RTC->CRL & RTC_CRL_RTOFF));  // Wait for write complete
}

void RTC_ClearBackupDomain(void)
{
    // Enable access to backup domain
    HAL_PWR_EnableBkUpAccess();

    // Reset backup domain (clears RTC, prescaler, backup regs)
    __HAL_RCC_BACKUPRESET_FORCE();
    __HAL_RCC_BACKUPRESET_RELEASE();
}

struct tm *gmtime_embedded(const time_t *t)
{
    static struct tm tm;
    time_t seconds = *t;
    const int seconds_per_day = 86400;
    const int seconds_per_hour = 3600;
    const int seconds_per_min = 60;

    int days = seconds / seconds_per_day;
    int rem = seconds % seconds_per_day;

    tm.tm_hour = rem / seconds_per_hour;
    tm.tm_min  = (rem % seconds_per_hour) / seconds_per_min;
    tm.tm_sec  = rem % seconds_per_min;

    tm.tm_wday = (4 + days) % 7;  // 1970-01-01 was a Thursday

    int year = 1970;
    while (1) {
        int leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
        int ydays = leap ? 366 : 365;
        if (days >= ydays) {
            days -= ydays;
            year++;
        } else {
            break;
        }
    }

    tm.tm_year = year - 1900;
    tm.tm_yday = days;
    tm.tm_mon = 0;

    static const int days_in_month[2][12] = {
        { 31,28,31,30,31,30,31,31,30,31,30,31 },
        { 31,29,31,30,31,30,31,31,30,31,30,31 }
    };

    int leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
    while (days >= days_in_month[leap][tm.tm_mon]) {
        days -= days_in_month[leap][tm.tm_mon];
        tm.tm_mon++;
    }

    tm.tm_mday = days + 1;
    return &tm;
}
/* USER CODE END 1 */
