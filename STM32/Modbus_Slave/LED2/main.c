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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MODBUS_SLAVE_ADDR 0x01

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_byte;
uint8_t rx_buffer[256];
uint16_t rx_index = 0;

uint16_t value = 100;
bool led_status[2] = {false, false};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void parse_modbus_frame(uint8_t *frame, uint16_t length);
void pack_modbus_coils(const bool *coils, uint16_t start_addr, uint16_t coil_count, uint8_t *packed_bytes);
uint16_t ModRTU_CRC(uint8_t *buf, int len);

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
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);  // Start receiving
  __HAL_TIM_SET_COUNTER(&htim6, 0);
  HAL_TIM_Base_Stop_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    value += 100;
    HAL_Delay(200);
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
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
    if (huart->Instance == USART1) 
    {
        rx_buffer[rx_index++] = rx_byte;

        // Restart TIM6 to count silence interval
        __HAL_TIM_SET_COUNTER(&htim6, 0);
        HAL_TIM_Base_Start_IT(&htim6);

        // Restart receiving next byte
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
    if (htim->Instance == TIM6) 
    {
        // Stop timer until next UART reception
        HAL_TIM_Base_Stop_IT(&htim6); 

        // Frame complete, process Modbus frame
        parse_modbus_frame(rx_buffer, rx_index);

        // Reset index
        rx_index = 0;
    }
}

void parse_modbus_frame(uint8_t *frame, uint16_t length) 
{
    // Minimum Modbus RTU length
    if (length < 4) return; 

    uint8_t slave_addr = frame[0];
    uint8_t function = frame[1];
    uint16_t crc_received = (frame[length - 2] | (frame[length - 1] << 8));
    uint16_t crc_calculated = ModRTU_CRC(frame, length - 2);
    if (crc_received != crc_calculated) return;

    if (slave_addr != MODBUS_SLAVE_ADDR) return;

    switch (function) 
    {
        case (0x01):
        {
            // Ensure enough bytes for address and content
            if (length < 8) goto flg_modbus_illegal;

            uint16_t start_addr = (frame[2] << 8) | frame[3];
            uint16_t coil_count = (frame[4] << 8) | frame[5];
            
            if (start_addr + coil_count - 1 > 0x0001) goto flg_modbus_illegal;
            uint8_t packed_byte_count = (coil_count + 7) / 8;
            
            uint8_t packed_byte[packed_byte_count]; 
            pack_modbus_coils(led_status, start_addr, coil_count, packed_byte);
            
            uint8_t response[6] = {
                MODBUS_SLAVE_ADDR,      // Slave addr
                0x01,                   // Func code
                packed_byte_count,      // byte count
                packed_byte[0]          // Coil status
            };
            
            uint16_t crc = ModRTU_CRC(response, 4);
            response[4] = crc & 0xFF;
            response[5] = (crc >> 8) & 0xFF;
            HAL_UART_Transmit(&huart1, response, 6, HAL_MAX_DELAY);
            
//            if (start_addr == 0x0000) 
//            {
//                uint8_t response[6] = {
//                    MODBUS_SLAVE_ADDR, 0x01, 0x01, led_status[0]
//                };
//                uint16_t crc = ModRTU_CRC(response, 4);
//                response[4] = crc & 0xFF;
//                response[5] = (crc >> 8) & 0xFF;
//                HAL_UART_Transmit(&huart1, response, 6, HAL_MAX_DELAY);
//            }
//            else if (start_addr == 0x0001)
//            {
//                uint8_t response[6] = {
//                    MODBUS_SLAVE_ADDR, 0x01, 0x01, led_status[1]
//                };
//                uint16_t crc = ModRTU_CRC(response, 4);
//                response[4] = crc & 0xFF;
//                response[5] = (crc >> 8) & 0xFF;
//                HAL_UART_Transmit(&huart1, response, 6, HAL_MAX_DELAY);
//            }
//            else goto flg_modbus_illegal;
            break;
        }
        
        case (0x03):
        {
            uint8_t response[7] = {
                MODBUS_SLAVE_ADDR, 0x03, 0x02, 
                (value >> 8) & 0xFF,  // High byte
                value & 0xFF          // Low byte
            };
            uint16_t crc = ModRTU_CRC(response, 5);
            response[5] = crc & 0xFF;
            response[6] = (crc >> 8) & 0xFF;
            HAL_UART_Transmit(&huart1, response, 7, HAL_MAX_DELAY);
            break;
        }
        
        case (0x05):
        {
            // Ensure enough bytes for address and content
            if (length < 8) break;

            uint16_t start_addr = (frame[2] << 8) | frame[3];
            uint16_t write_cont = (frame[4] << 8) | frame[5];
//            uint8_t start_addr_hi = (start_addr >> 8) & 0xFF;
//            uint8_t start_addr_lo = start_addr & 0xFF;
//            uint8_t write_cont_hi = (write_cont >> 8) & 0xFF;
//            uint8_t write_cont_lo = write_cont & 0xFF;
            
            if (start_addr == 0x0000) 
            {
                if (write_cont == 0xFF00)
                {
                    led_status[0] = 1;
                    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
                    HAL_UART_Transmit(&huart1, frame, 8, HAL_MAX_DELAY);
                }
                else if (write_cont == 0x0000)
                {
                    led_status[0] = 0;
                    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
                    HAL_UART_Transmit(&huart1, frame, 8, HAL_MAX_DELAY);
                }
                else goto flg_modbus_illegal;
            }
            else if (start_addr == 0x0001)
            {
                if (write_cont == 0xFF00)
                {
                    led_status[1] = 1;
                    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
                    HAL_UART_Transmit(&huart1, frame, 8, HAL_MAX_DELAY);
                }
                else if (write_cont == 0x0000)
                {
                    led_status[1] = 0;
                    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
                    HAL_UART_Transmit(&huart1, frame, 8, HAL_MAX_DELAY);
                }
                else goto flg_modbus_illegal;
            }
            else goto flg_modbus_illegal;
            break;
        }
        
        default:
        flg_modbus_illegal: 
        {
            // Unsupported function: send exception response (Illegal Function = 0x01)
            uint8_t exception[5];
            exception[0] = MODBUS_SLAVE_ADDR;
            exception[1] = function | 0x80;  // Set MSB for exception
            exception[2] = 0x01;             // Exception Code: Illegal Function
            uint16_t crc = ModRTU_CRC(exception, 3);
            exception[3] = crc & 0xFF;
            exception[4] = (crc >> 8) & 0xFF;
            HAL_UART_Transmit(&huart1, exception, 5, HAL_MAX_DELAY);
        }
    }
}

void pack_modbus_coils(const bool *coils, uint16_t start_addr, uint16_t coil_count, uint8_t *output_bytes) 
{
    memset(output_bytes, 0, (coil_count + 7) / 8);  // Clear output

    for (uint16_t i = start_addr, j = 0; i < start_addr + coil_count; i++, j++) {
        if (coils[i]) {
            output_bytes[j / 8] |= (1 << (j % 8));  // LSB-first bit order
        }
//        printf("%d, %d\n", i, coils[i]);
    }
}

uint16_t ModRTU_CRC(uint8_t *buf, int len) 
{
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) 
    {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) 
        {
            if ((crc & 0x0001) != 0) 
            {
                crc >>= 1;
                crc ^= 0xA001;
            } 
            else 
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
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
