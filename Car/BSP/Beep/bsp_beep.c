#include "bsp_beep.h"

u32 beep_time = 0;

void init_beep(void)
{
	//��ʼ�������������� Initialize the buzzer pin
  GPIO_InitTypeDef GPIO_InitStructure;
	/*��������ʱ��  Enable peripheral clock */
	RCC_APB2PeriphClockCmd(BEEP_RCC, ENABLE); 
	/*ѡ��Ҫ���Ƶ�����  Select the pin to control */															   
	GPIO_InitStructure.GPIO_Pin = BEEP_PIN;	
	/*��������ģʽΪͨ��������� Set the pin mode to general push-pull output */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
	/*������������Ϊ50MHz Set the pin rate to 50MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
	/*���ÿ⺯������ʼ��BEEP_PORT Call library function to initialize BEEP_PORT */
	GPIO_Init(BEEP_PORT, &GPIO_InitStructure);
	
	BEEP_BEEP = 0;

}

//��������ʱ�� Buzzer on time
//beep_time ��1msΪ��λ  1ms as unit
void open_beep(u32 beep_time) //10ms��һ��  Subtract once every 10ms
{
	beep_time = beep_time/10;
}

