#include "bsp_LED.h"


void init_led_gpio(void)
{
		
		GPIO_InitTypeDef GPIO_InitStructure;
	
//		PWR_BackupAccessCmd(ENABLE); //�����޸�RTC �ͺ󱸼Ĵ���  //Allow modification of RTC and backup registers
//		RCC_LSEConfig(RCC_LSE_OFF);  //�ر��ⲿ�����ⲿʱ���źŹ��� ��PC13 PC14 PC15 �ſ��Ե���ͨIO�á� //After turning off the external low-speed external clock signal function, PC13 PC14 PC15 can be used as ordinary IO.
//		BKP_TamperPinCmd(DISABLE);   //�ر����ּ�⹦�ܣ�Ҳ���� PC13��Ҳ���Ե���ͨIO ʹ��  //Disable the intrusion detection function, that is, PC13, which can also be used as ordinary IO
//		PWR_BackupAccessCmd(DISABLE);//��ֹ�޸ĺ󱸼Ĵ��� //Prohibit modification of backup registers
	
		//��ʼ��LED������ Initialize the LED pins
		RCC_APB2PeriphClockCmd(LED_RCC, ENABLE); 									   
  	GPIO_InitStructure.GPIO_Pin = LED_PIN;	
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
  	GPIO_Init(LED_PORT, &GPIO_InitStructure);
	
		LED = 0;

}


