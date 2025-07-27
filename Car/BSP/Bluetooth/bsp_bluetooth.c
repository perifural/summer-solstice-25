#include "bsp_bluetooth.h"

//�������ڳ�ʼ�� ֻ����9600������
// Initialize the Bluetooth serial port, the baud rate can only be 9600
void bluetooth_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD , ENABLE);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);    

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	  
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART5, &USART_InitStructure); 
//	USART_ITConfig(UART5, USART_IT_TXE, DISABLE);  

	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);   
	USART_Cmd(UART5, ENABLE);
	
}


//����һ���ַ�  Send a character
void bluetooth_send_char(uint8_t ch)
{
	while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(UART5, ch);
}

//����һ���ַ���  Send a string
/**
 * @Brief: UART5�������� UART5 sends data
 * @Note: 
 * @Parm: BufferPtr:�����͵�����(Data to be sent)  Length:���ݳ���(Data length)
 * @Retval: 
 */
void UART5_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
	while (Length--)
	{
		bluetooth_send_char(*BufferPtr);
		BufferPtr++;
	}
}

/*����5���ͺ��� Serial port 5 sending function*/
void USART5_Send_Byte(unsigned char byte)   //���ڷ���һ���ֽ� The serial port sends a byte
{
//	while( USART_GetFlagStatus(UART5,USART_FLAG_TC)!= SET);  
	while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
	USART_SendData(UART5, byte);        //ͨ���⺯��  ��������  Send data through library functions
        
}
void bluetooth_send_string(char *s)
{
	char *p;
	p=s;
	while(*p != '\0')
	{
		USART5_Send_Byte(*p);
		p++;
	}	
}


/*\
//�����жϷ�����  Serial port interrupt service function
void UART5_IRQHandler(void)
{
	uint8_t Rx5_Temp;
	if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{
		Rx5_Temp = USART_ReceiveData(UART5);
		
	}
}


 */

