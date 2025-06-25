#include "main.h"
#include "tim.h"
#include "usart.h"
#include "string.h"

#define MODBUS_SLAVE_ADDR 0x01

void modbus_handler(void);
void value_handler(void);
void value_init(void);
uint16_t ModRTU_CRC(uint8_t *buf, int len);
void pack_modbus_coils(const uint8_t *coils, uint16_t start_addr, uint16_t coil_count, uint8_t *output_bytes);
void parse_modbus_frame(uint8_t *frame, uint16_t length);