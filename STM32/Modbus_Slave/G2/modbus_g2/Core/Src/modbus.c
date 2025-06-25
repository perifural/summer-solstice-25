#include "main.h"
#include "tim.h"
#include "usart.h"

extern modbus_t modbus;
extern value_t value;

void modbus_handler()
{
    if (modbus.rx_flag)
    {
        modbus.rx_flag = 0;
        
        modbus.rx_buffer[modbus.rx_index++] = modbus.rx_byte;

        // Restart TIM6 to count silence interval
        __HAL_TIM_SET_COUNTER(&htim6, 0);
        HAL_TIM_Base_Start_IT(&htim6);

        // Restart receiving next byte
        HAL_UART_Receive_IT(&huart1, &modbus.rx_byte, 1);
    }
    
    if (modbus.tim_flag)
    {
        modbus.tim_flag = 0;
        
        // Stop timer until next UART reception
        HAL_TIM_Base_Stop_IT(&htim6); 
        
        // Frame complete, process Modbus frame
        parse_modbus_frame(modbus.rx_buffer, modbus.rx_index);

        // Reset index
        modbus.rx_index = 0;
    }
}

void value_init()
{
    value.reg[0] = 0;
    value.reg[1] = 1;
    value.reg[2] = 2;
    value.reg[3] = 3;
    value.led[0] = false;
    value.led[1] = false;
    value.key[0] = false;
    value.key[1] = false;
    value.key[2] = false;
    value.key[3] = false;
}

void value_handler()
{
    if (value.tim_flag)
    {
        value.tim_flag = 0;
        
        value.key[0] = !HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin);
        value.key[1] = !HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
        value.key[2] = !HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin);
        value.key[3] = HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin);
    }
}
