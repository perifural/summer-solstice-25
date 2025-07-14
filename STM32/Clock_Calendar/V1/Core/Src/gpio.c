/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
#define KEY_PERIOD 20

Key_t key[4] = {
    {KEY0_GPIO_Port, KEY0_Pin, 1, 1, 1, 0, 0},
    {KEY1_GPIO_Port, KEY1_Pin, 1, 1, 1, 0, 0},
    {KEY2_GPIO_Port, KEY2_Pin, 1, 1, 1, 0, 0},
    {KEY3_GPIO_Port, KEY3_Pin, 1, 1, 1, 0, 0}
};

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : KEY2_Pin KEY1_Pin KEY0_Pin */
  GPIO_InitStruct.Pin = KEY2_Pin|KEY1_Pin|KEY0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY3_Pin */
  GPIO_InitStruct.Pin = KEY3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
void keyScan(void)
{
    for(uint8_t i = 0; i <= 3; i++)
    {
        if(i == 3)
        {
            key[i].current_state = !HAL_GPIO_ReadPin(key[i].port, key[i].pin);
        }
        else
        {
            key[i].current_state = HAL_GPIO_ReadPin(key[i].port, key[i].pin);
        }
        
        if(key[i].current_state != key[i].last_state)
        {
            key[i].debounce_counter = HAL_GetTick();
        }
        else
        {
            if(HAL_GetTick() - key[i].debounce_counter >= KEY_PERIOD)
            {
                if(key[i].current_state != key[i].stable_state)
                {
                    key[i].stable_state = key[i].current_state;
                    if(key[i].stable_state == 0)
                    {
                        key[i].pressed_event = 1;
                    }
                }
            }
        }
        
        key[i].last_state = key[i].current_state;
    }
}

/* USER CODE END 2 */
