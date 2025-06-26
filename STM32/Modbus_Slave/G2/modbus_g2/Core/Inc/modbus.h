#include "main.h"
#include "tim.h"
#include "usart.h"
#include "string.h"

#define MODBUS_SLAVE_ADDR 0x01

#define MODBUS_ILLEGAL_FUNCTION 0x01
#define MODBUS_ILLEGAL_DATA_ADDRESS 0x02
#define MODBUS_ILLEGAL_DATA_VALUE 0x03
#define MODBUS_SLAVE_DEVICE_FAILURE 0x04
#define MODBUS_SLAVE_DEVICE_BUSY 0x06


void modbus_handler(void);
void value_handler(void);
void value_init(void);
uint16_t ModRTU_CRC(uint8_t *buf, int len);
void pack_modbus_bits(const uint8_t *bit, uint16_t start_addr, uint16_t bit_count, uint8_t *output_bytes);
void modbus_parse();
void modbus_fc_01_02(uint8_t func);
void modbus_fc_03();
void modbus_fc_05();
void modbus_fc_06();
void modbus_fc_10();
void modbus_exception(uint8_t func, uint8_t exception_code);
void modbus_illegal();