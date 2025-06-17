# Modbus slave demo

Address: `0x01`, set in `#define`. 

Function code: `0x03`

Respond: `uint16_t += 100` for `0x03` or else illegal. 

This example uses `TIM2`. 