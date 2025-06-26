#include "modbus.h"

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
        modbus_parse();

        // Reset index
        modbus.rx_index = 0;
    }
}

void value_init()
{
    for (uint16_t i = 0; i < 1000; i++)
    {
        value.bit[i] = i % 2;
    }
    
    for (uint16_t i = 0; i < 100; i++)
    {
        value.reg[i] = i * 100;
    }
    
    value.led[0] = 0;
    value.led[1] = 0;
    value.key[0] = 0;
    value.key[1] = 0;
    value.key[2] = 0;
    value.key[3] = 0;
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

void pack_modbus_bits(const uint8_t *bit, uint16_t start_addr, uint16_t bit_count, uint8_t *packed_byte) 
{
    memset(packed_byte, 0, (bit_count + 7) / 8);   // Clear output

    for (uint16_t i = start_addr, j = 0; i < start_addr + bit_count; i++, j++) {
        if (bit[i]) {
            packed_byte[j / 8] |= (1 << (j % 8));  // LSB-first bit order
        }
//        printf("%d, %d\n", i, bit[i]);
    }
}

void modbus_parse() 
{
    if (modbus.rx_index < 4) return;
    uint8_t slave_addr = modbus.rx_buffer[0];
    uint8_t func = modbus.rx_buffer[1];
    uint16_t crc_rx = (modbus.rx_buffer[modbus.rx_index - 2] | (modbus.rx_buffer[modbus.rx_index - 1] << 8));
    uint16_t crc_calc = ModRTU_CRC(modbus.rx_buffer, modbus.rx_index - 2);
    if (crc_rx != crc_calc) return;
    if (slave_addr != MODBUS_SLAVE_ADDR) return;
    
    switch (func) 
    {
        case (0x01):
        {
            modbus_fc_01_02(0x01);
            break;
        }
        
        case (0x02):
        {
            modbus_fc_01_02(0x02);
            break;
        }
        
        case (0x03):
        {
            modbus_fc_03_04(0x03);
            break;
        }
        
        case (0x04):
        {
            modbus_fc_03_04(0x04);
            break;
        }
        
        case (0x05):
        {
            modbus_fc_05();
            break;
        }
        
        case (0x06):
        {
            modbus_fc_06();
            break;
        }
        
        case (0x10):
        {
            modbus_fc_10();
            break;
        }
        
        default:
        {
            modbus_exception(func, MODBUS_ILLEGAL_FUNCTION);
        }
    }
}

void modbus_fc_01_02(uint8_t func)
{
    if (modbus.rx_index < 8) modbus_illegal();

    uint16_t start_addr = (modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
    uint16_t bit_count = (modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
    
    if (start_addr > 0x03E7)
    {
        modbus_exception(func, MODBUS_ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    if (start_addr + bit_count - 1 > 0x03E7) 
    {
        modbus_exception(func, MODBUS_ILLEGAL_DATA_VALUE);
        return;
    }
    
    uint8_t packed_byte_count = (bit_count + 7) / 8;

    uint8_t packed_byte[packed_byte_count]; 
    pack_modbus_bits(value.bit, start_addr, bit_count, packed_byte);

    uint16_t response_length = 5 + packed_byte_count;
    uint8_t response[response_length];

    response[0] = MODBUS_SLAVE_ADDR;
    response[1] = func;
    response[2] = packed_byte_count;

    for (int i = 3, j = 0; i < 3 + packed_byte_count; i++, j++)
    {
        response[i] = packed_byte[j];
    }

    uint16_t crc = ModRTU_CRC(response, response_length - 2);
    response[response_length - 2] = crc & 0xFF;
    response[response_length - 1] = (crc >> 8) & 0xFF;
    HAL_UART_Transmit(&huart1, response, response_length, HAL_MAX_DELAY);
}

void modbus_fc_03_04(uint8_t func)
{
    if (modbus.rx_index < 6) modbus_illegal();

    uint16_t start_addr = (modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
    uint16_t reg_count = (modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];

    if (start_addr > 0x0063)
    {
        modbus_exception(func, MODBUS_ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    if (start_addr + reg_count - 1 > 0x0063)
    {
        modbus_exception(func, MODBUS_ILLEGAL_DATA_VALUE);
        return;
    }
    
    uint16_t byte_count = reg_count * 2;
    uint16_t response_length = 5 + byte_count;

    uint8_t response[response_length];
    response[0] = MODBUS_SLAVE_ADDR;
    response[1] = func;
    response[2] = byte_count;

    for (int i = 3, j = start_addr; i < 3 + byte_count; i += 2, j++)
    {
        response[i] = (value.reg[j] >> 8) & 0xFF;    // High byte
        response[i+1] = value.reg[j] & 0xFF;         // Low byte
    }

    uint16_t crc = ModRTU_CRC(response, response_length - 2);
    response[response_length - 2] = crc & 0xFF;
    response[response_length - 1] = (crc >> 8) & 0xFF;
    HAL_UART_Transmit(&huart1, response, response_length, HAL_MAX_DELAY);
}

void modbus_fc_05()
{
    if (modbus.rx_index < 8) modbus_illegal();

    uint16_t start_addr = (modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
    uint16_t write_cont = (modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];

    switch (start_addr)
    {
        case (0x0000):
        {
            switch (write_cont)
            {
                case (0xFF00):
                {
                    value.led[0] = 1;
                    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
                    HAL_UART_Transmit(&huart1, modbus.rx_buffer, 8, HAL_MAX_DELAY);
                    break;
                }
                
                case (0x0000):
                {
                    value.led[0] = 0;
                    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
                    HAL_UART_Transmit(&huart1, modbus.rx_buffer, 8, HAL_MAX_DELAY);
                    break;
                }
                
                default:
                    modbus_illegal();
            }
            break;
        }
        
        case (0x0001):
        {
            switch (write_cont)
            {
                case (0xFF00):
                {
                    value.led[1] = 1;
                    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
                    HAL_UART_Transmit(&huart1, modbus.rx_buffer, 8, HAL_MAX_DELAY);
                    break;
                }
                
                case (0x0000):
                {
                    value.led[1] = 0;
                    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
                    HAL_UART_Transmit(&huart1, modbus.rx_buffer, 8, HAL_MAX_DELAY);
                    break;
                }
                
                default:
                    modbus_illegal();
            }
            break;
        }
        
        default:
            modbus_illegal();
    }
}

void modbus_fc_06()
{
    if (modbus.rx_index < 8) modbus_illegal(); 
    uint16_t start_addr = (modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
    if (start_addr > 0x0003) modbus_illegal();
    value.reg[start_addr] = (modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
    HAL_UART_Transmit(&huart1, modbus.rx_buffer, modbus.rx_index, HAL_MAX_DELAY);
}

void modbus_fc_10()
{
    if (modbus.rx_index < 11) modbus_illegal();
    uint16_t start_addr = (modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
    uint16_t reg_count = (modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
    uint8_t byte_count = modbus.rx_buffer[6];

    if (start_addr + reg_count - 1 > 0x0003) modbus_illegal();
    if (reg_count * 2 != byte_count) modbus_illegal();

    for (uint16_t i = start_addr, j = 7; i < start_addr + reg_count; i++, j += 2)
    {
        value.reg[i] = (modbus.rx_buffer[j] << 8) | modbus.rx_buffer[j+1];
    }

    uint8_t response[8];
    response[0] = MODBUS_SLAVE_ADDR;
    response[1] = 0x10;
    response[2] = start_addr & 0xFF;
    response[3] = (start_addr >> 8) & 0xFF;
    response[4] = (reg_count >> 8) & 0xFF;
    response[5] = reg_count & 0xFF;

    uint16_t crc = ModRTU_CRC(response, 8 - 2);
    response[8 - 2] = crc & 0xFF;
    response[8 - 1] = (crc >> 8) & 0xFF;
    HAL_UART_Transmit(&huart1, response, 8, HAL_MAX_DELAY);
}

void modbus_exception(uint8_t func, uint8_t exception)
{
    uint8_t response[5];
    response[0] = MODBUS_SLAVE_ADDR;
    response[1] = func | 0x80;  // Set MSB for exception
    response[2] = exception;
    uint16_t crc = ModRTU_CRC(response, 3);
    response[3] = crc & 0xFF;
    response[4] = (crc >> 8) & 0xFF;
    HAL_UART_Transmit(&huart1, response, 5, HAL_MAX_DELAY);
}

void modbus_illegal()
{
}

