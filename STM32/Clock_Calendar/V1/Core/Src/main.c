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
#include "i2c.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pcf8574.h"
#include "stdio.h"

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
RTC_TimeTypeDef nowTime;
RTC_DateTypeDef nowDate;
uint8_t pastTimeSec = 0;

const char* weekDays[7] = {
    "Sun",    // 0
    "Mon",    // 1
    "Tue",    // 2
    "Wed",    // 3
    "Thu",    // 4
    "Fri",    // 5
    "Sat",    // 6
};

const char* months[12] = {
    "Jan",    // 0
    "Feb",    // 1
    "Mar",    // 2
    "Apr",    // 3
    "May",    // 4
    "Jun",    // 5
    "Jul",    // 6
    "Aug",    // 7
    "Sep",    // 8
    "Oct",    // 9
    "Nov",    // 10
    "Dec",    // 11
};

const char* modes[7] = {
    "   ",       // 0
    " Hr",    // 1
    "Min",    // 2
    "Sec",    // 3
    " Mo",    // 4
    "Day",    // 5
    " Yr",    // 6
};

extern Key_t key[4];

struct tm time_set = 
    {
    .tm_year = 2025 - 1900,
    .tm_mon = 7 - 1,      // July
    .tm_mday = 10,
    .tm_hour = 12,
    .tm_min = 0,
    .tm_sec = 0
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
//  RTC_ClearBackupDomain();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  pcf8574_init();
//  RTC_SetUnixTime(mktime(&time_set));
  int8_t mode = 0;
  
  int16_t year = 0;
  int8_t month = 0;
  int8_t day = 0;
  int8_t weekday = 0;
  int8_t hour = 0;
  int8_t minute = 0;
  int8_t second = 0;
  
  time_t time_unix;
  struct tm *time_gmt;
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      
      if(mode == 0)
      {
          time_unix = RTC_GetUnixTime();
      }
      else
      {
          time_set.tm_year = year - 1900;
          time_set.tm_mon = month;
          time_set.tm_mday = day;
          time_set.tm_hour = hour;
          time_set.tm_min = minute;
          time_set.tm_sec = second;
          time_unix = mktime(&time_set);
          RTC_SetUnixTime(time_unix);
      }
      
      time_gmt = gmtime_embedded(&time_unix);
      year = time_gmt->tm_year + 1900;
      month = time_gmt->tm_mon;
      day = time_gmt->tm_mday;
      weekday = time_gmt->tm_wday;
      hour = time_gmt->tm_hour;
      minute = time_gmt->tm_min;
      second = time_gmt->tm_sec;
      
      char lcd_buffer[17];
      
      pcf8574_cursor(0,0);
      sprintf(lcd_buffer, "%02d:%02d:%02d    %s", hour, minute, second, modes[mode]);
      pcf8574_send_string(lcd_buffer);
      
      pcf8574_cursor(1,0);
      sprintf(lcd_buffer, "%s %02d %s %d", months[month], day, weekDays[weekday], year);
      pcf8574_send_string(lcd_buffer);
      
      keyScan();
      
      if(key[2].pressed_event == 1)
      {
          key[2].pressed_event = 0;
          mode--;
          if(mode < 0)
          {
              mode = 6;
          }
      }
      
      if(key[0].pressed_event == 1)
      {
          key[0].pressed_event = 0;
          mode++;
          if(mode > 6)
          {
              mode = 0;
          }
      }
      
      switch (mode) 
      {
          case 1:
          {
              if(key[1].pressed_event == 1)
              {
                  key[1].pressed_event = 0;
                  hour--;
              }
              
              if(key[3].pressed_event == 1)
              {
                  key[3].pressed_event = 0;
                  hour++;
              }
              
              break;
          }
          
          case 2:
          {
              if(key[1].pressed_event == 1)
              {
                  key[1].pressed_event = 0;
                  minute--;
              }
              
              if(key[3].pressed_event == 1)
              {
                  key[3].pressed_event = 0;
                  minute++;
              }
              
              break;
          }
          
          case 3:
          {
              if(key[1].pressed_event == 1)
              {
                  key[1].pressed_event = 0;
                  second--;
              }
              
              if(key[3].pressed_event == 1)
              {
                  key[3].pressed_event = 0;
                  second++;
              }
              
              break;
          }
          
          case 4:
          {
              if(key[1].pressed_event == 1)
              {
                  key[1].pressed_event = 0;
                  month--;
              }
              
              if(key[3].pressed_event == 1)
              {
                  key[3].pressed_event = 0;
                  month++;
              }
              
              break;
          }
          
          case 5:
          {
              if(key[1].pressed_event == 1)
              {
                  key[1].pressed_event = 0;
                  day--;
              }
              
              if(key[3].pressed_event == 1)
              {
                  key[3].pressed_event = 0;
                  day++;
              }
              
              break;
          }
          
          case 6:
          {
              if(key[1].pressed_event == 1)
              {
                  key[1].pressed_event = 0;
                  year--;
                  if(year < 1970)
                  {
                      year = 2099;
                  }
              }
              
              if(key[3].pressed_event == 1)
              {
                  key[3].pressed_event = 0;
                  year++;
                  if(year > 2099)
                  {
                      year = 1970;
                  }
              }
              
              break;
          }
          
          default:
          {
              // Do nothing
          }
      }
      
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
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
