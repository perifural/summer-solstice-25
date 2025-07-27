#include "usart.h"	  

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  Add the following code to support the printf function without selecting use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数       Support functions required by the standard library          
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    Define _sys_exit() to avoid using semihosting mode
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 Redefine fputc function
int fputc(int ch, FILE *f)
{      
	  while((USART1->SR&0X40)==0);
		USART1->DR = (u8) ch;      
	return ch;
}
#endif 
/**************************************************************************
Function: Serial port 1 initialization
Input   : bound：Baud rate
Output  : none
函数功能：串口1初始化
入口参数：bound：波特率
返回  值：无
**************************************************************************/
void uart_init(u32 bound)
{
  //GPIO端口设置 GPIO port settings
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟  Enable USART1, GPIOA clock
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出  Multiplexed push-pull output
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9  Initialize GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化  GPIOA.10 Initialization
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入 Floating Input
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10   Initialize GPIOA.10
   //USART 初始化设置 Initial Setup

	USART_InitStructure.USART_BaudRate = bound;//串口波特率 Serial port baud rate
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式 The word length is 8-bit data format
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位 One stop bit
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位 No parity bit
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制 No hardware flow control
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式 Transceiver mode

  USART_Init(USART1, &USART_InitStructure); //初始化串口1 Initialize serial port 1
  USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//关闭串口接受中断 Disable serial port receive interrupt
  USART_Cmd(USART1, ENABLE);                    //使能串口1  Enable serial port 1
	
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

/**
 * @Brief: UART1发送数据 UART1 sends data
 * @Note: 
 * @Parm: ch:待发送的数据  Data to be sent
 * @Retval: 
 */
void USART1_Send_U8(uint8_t ch)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(USART1, ch);
}

/**
 * @Brief: UART1发送数据 UART1 sends data
 * @Note: 
 * @Parm: BufferPtr:待发送的数据(Data to be sent)  Length:数据长度(Data length)
 * @Retval: 
 */
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
	while (Length--)
	{
		USART1_Send_U8(*BufferPtr);
		BufferPtr++;
	}
}

//串口中断服务函数 Serial port interrupt service function
void USART1_IRQHandler(void)
{
	uint8_t Rx1_Temp = 0;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		Rx1_Temp = USART_ReceiveData(USART1);
		USART1_Send_U8(Rx1_Temp);
	}
}


