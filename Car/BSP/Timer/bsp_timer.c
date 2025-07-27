#include "bsp_timer.h"

static float battery_All;
static uint8_t battery_count=0,battery_flag=0;

static u16 stop_time = 0;//�ӳ�ʱ��  delay time

u16 led_flag = 0; //1:������˸״̬ 0:�ȴ���˸ //1: Entering flashing state 0: waiting for flashing
u16 led_twinkle_count = 0;// ��˸����  //Flashing Count

u16 led_count = 0; //��ʼ����  //Start counting

u8 lower_power_flag = 0; //�͵�ѹ��־  0:��ѹ���� 1����ѹ  //Low Voltage Flag 0: Normal Voltage 1: Low Voltage


//��ʱ��6���ӳ� 10ms���ӳ� �˷�����delay׼ȷ
//Timer 6 has a delay of 10ms. This method is more accurate than delay
void delay_time(u16 time)
{
	stop_time = time;
	while(stop_time);//���� Wait
}

//�ӳ�1s  Unit second
void my_delay(u16 s)//s
{
	for(int i = 0;i<s;i++)
	{
		delay_time(100);
	}
}


/**************************************************************************
Function function: TIM6 initialization, timed for 10 milliseconds
Entrance parameters: None
Return value: None
�������ܣ�TIM6��ʼ������ʱ10����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM6_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //ʹ�ܶ�ʱ����ʱ��  Enable the clock of the timer
	TIM_TimeBaseStructure.TIM_Prescaler = 7199;			 // Ԥ��Ƶ��  Prescaler
	TIM_TimeBaseStructure.TIM_Period = 99;				 //�趨�������Զ���װֵ  Set the automatic reset value of the counter
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);                //���TIM�ĸ��±�־λ Clear the update flag of TIM
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

	//�ж����ȼ�NVIC����  Interrupt priority NVIC setting
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;			  //TIM6�ж� TIM6 interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4; //��ռ���ȼ�4�� Preemption priority level 4
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  //�����ȼ�2�� From priority level 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQͨ����ʹ�� IRQ channel is enabled
	NVIC_Init(&NVIC_InitStructure);							  //��ʼ��NVIC�Ĵ��� Initialize NVIC registers

	TIM_Cmd(TIM6, ENABLE);
}


u8 bulettohflag = 0;

// TIM6�ж� //TIM6 Interrupt service
void TIM6_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) //���TIM�����жϷ������  Check whether TIM update interruption occurs
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);    //���TIMx�����жϱ�־  Clear TIMx update interrupt flag
		led_count++;  //led������ʾ��־ LED service display logo
		battery_flag ++;		//������ʾ��־	 Electricity display sign

		
		if(stop_time>0)
		{
			stop_time --;
		}
		
		if(mode == Normal  ||mode == U_Follow|| mode == U_Avoid || mode == Weight_M) //����ģʽ����Ҫ������Normal  Normal mode does not need to follow Normal
			Get_Distane();//��ȡ����  Get distance
		
		if(mode == Normal || mode == Weight_M)
		{ 
				bulettohflag = 1;

//				//�����ж������Ӱ��ƽ��  Putting it here will affect the balance.
//				SendAutoUp();//�����Զ��ϱ����� Bluetooth automatically reports data 
		}
		
		if(mode == U_Follow)
		{
			App_Change_Car();//���������� Ultrasonic Follow
		}
		
			
		
		
		
		
////////��ѹ�������		 Voltage detection process
		if(battery_flag > 2)//20ms
		{		
			battery_flag = 0;
			battery_All += Get_Battery_Volotage();//��ȡ��Դ���� Obtain the power level of the power supply
			battery_count++;
			if(battery_count == 50)//1000ms
			{
				battery = battery_All/50; //ƽ��ֵ average value
				battery_All = 0;  
				battery_count = 0;
				power_decect();//��ѹ����  Voltage processing
			}
			
		}
///////////
		
		cotrol_led();//�Ʒ���  led service
		
				
		
	}
}


void power_decect(void)
{
	static u8 normal_power_flag = 1; //��ѹ�ָ���־ 0��û�ָ� 1:�ָ� //Voltage recovery flag 0: not restored 1: restored
	if(battery < 9.6) //С��9.6V���� //Alarm below 9.6V
	{
		lower_power_flag = 1;
		normal_power_flag = 0;
	}
	else
	{
		if(normal_power_flag == 0)
		{
			lower_power_flag = 0;
			normal_power_flag = 1;
			BEEP_BEEP = 0;
		}
		
	}
}

void cotrol_led(void)
{
	//�Ƶ�Ч���ͷ�������Ч�� ��ѹ���� //The effect of the lamp and buzzer is low voltage alarm
		if(!led_flag)
		{
			if(led_count>300)//3S
			{
				led_count = 0;
				led_flag = 1;
			}
		}
		else
		{
			if(led_count>20)//200ms
			{
				led_count = 0;
				
				if(lower_power_flag == 0)
				{
					LED = !LED;//״̬��ת //State reversal
				}
				else
				{
					BEEP_BEEP = !BEEP_BEEP;
					LED = 1;//��ѹ���Ƴ��� //Low voltage blue light is always on
				}
				
				led_twinkle_count++;
				if(led_twinkle_count == 6)
				{
					if(lower_power_flag == 0)
					{
						LED = 0;
					}
					else
					{
						BEEP_BEEP = 0;
					}
					
					led_twinkle_count = 0;
					led_flag = 0;
				}
				
			}
		}

}

