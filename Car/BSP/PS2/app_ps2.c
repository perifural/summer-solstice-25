#include "app_ps2.h"


int g_PS2_LX, g_PS2_LY, g_PS2_RX, g_PS2_RY, g_PS2_KEY;
int g_flag = 1; // ģʽ��ӡ������־  Mode print interlock flag

uint8_t speed_flag=3;
 
//ps2����С��  ps2 control car
void PS2_Contorl_Car(void)
{
	
	//ģ��ֵ���� Analog value control
	g_PS2_LX = PS2_AnologData(PSS_LX);
	g_PS2_LY = PS2_AnologData(PSS_LY);
	g_PS2_RX = PS2_AnologData(PSS_RX);
	g_PS2_RY = PS2_AnologData(PSS_RY);
	g_PS2_KEY = PS2_DataKey();

	
	// �ֱ�ûͨ���� The handle is not communicating
	if ((g_PS2_LX == 255) && (g_PS2_LY == 255) && (g_PS2_RX == 255) && (g_PS2_RY == 255))
	{
		if (g_flag == 1)
		{
			g_flag = 0;
		}
	}

	else
	{
		if (g_flag == 0)
		{
			g_flag = 1;
		}

	
		if((g_PS2_LX<50 && g_PS2_LY<50) || (g_PS2_RX<50 && g_PS2_RY<50))
		{
			g_newcarstate=enps2Fleft; //ǰ���� Front left
		}
		else if((g_PS2_LX>150 && g_PS2_LY<50) || (g_PS2_RX>150 && g_PS2_RY<50))
		{
			g_newcarstate=enps2Fright; //ǰ���� Front right turn
		}
		else if((g_PS2_LX<50 && g_PS2_LY>150) || (g_PS2_RX<50 && g_PS2_RY>150))
		{
			g_newcarstate=enps2Bleft; //������ Back left
		}
		else if((g_PS2_LX>150 && g_PS2_LY>150) || (g_PS2_RX>150 && g_PS2_RY>150))
		{
			g_newcarstate=enps2Bright; //������ Back right
		}
		else if((g_PS2_LY<90) || (g_PS2_RY<90))
		{
			g_newcarstate=enRUN; //ǰ�� go ahead
			return;
		}
			else if((g_PS2_LY>150) || (g_PS2_RY>150))
		{
			g_newcarstate=enBACK; //���� Back
			return;
		}
			else if((g_PS2_LX<90) || (g_PS2_RX<90))
		{
			g_newcarstate=enLEFT; //��ת Turn left
			return;
		}
			else if((g_PS2_LX>150) || (g_PS2_RX>150))
		{
			g_newcarstate=enRIGHT; //��ת Turn right
			return;
		}
		else
		{
			if (g_PS2_KEY == 0) // û�������� No buttons pressed
				g_newcarstate=enSTOP;
		}
	}
	

		
		
	//�����Ǽ�ֵ����  The following are key value controls
	switch (g_PS2_KEY)
	{
		case PSB_L1:
		case PSB_L2:
			speed_flag ++;
			if(speed_flag>5)
			{
				speed_flag = 5;
			}
			
			//��ʵ���� real control
			Car_Target_Velocity +=10 ; 
			Car_Turn_Amplitude_speed +=12 ;
			if(Car_Target_Velocity > 50)
			{
				Car_Target_Velocity = 50;
			}
			if(Car_Turn_Amplitude_speed>60)
			{
				Car_Turn_Amplitude_speed =60;
			}
			break; // ����  accelerate
			

		case PSB_R1:
		case PSB_R2:
			speed_flag --;
			if(speed_flag<1)
			{
				speed_flag = 1;
			}
			
			//��ʵ���� real control
			Car_Target_Velocity -=10 ; 
			Car_Turn_Amplitude_speed -=12 ;
			if(Car_Target_Velocity < 10)
			{
				Car_Target_Velocity = 10;
			}
			if(Car_Turn_Amplitude_speed<12)
			{
				Car_Turn_Amplitude_speed =12;
			}
			break; // ����  slow down
		
 
		//ǰ�� forward
		case PSB_PAD_UP:
		case PSB_GREEN:
				g_newcarstate=enRUN;
				break;
		
		//���� back
		case PSB_PAD_RIGHT:
		case PSB_RED:
			g_newcarstate=enRIGHT;
			break;
		
		//��ת left
		case PSB_PAD_DOWN:
		case PSB_BLUE:
			g_newcarstate=enBACK;
			
			break;
		
		//��ת right
		case PSB_PAD_LEFT:
		case PSB_PINK:
			g_newcarstate=enLEFT;
			break;

		//ֹͣ stop
		default:
			if(g_flag == 1) //����ģʽ���ж� Red and green mode to judge
			{
				if(((g_PS2_LY >90 && g_PS2_LY < 140) && (g_PS2_LX >90 && g_PS2_LX < 140)) && ((g_PS2_RY >90 && g_PS2_RY < 140) && (g_PS2_RX >90 && g_PS2_RX < 140)))
				{
					g_newcarstate=enSTOP;
					break;
				}
			}
			else
			{
				g_newcarstate=enSTOP;
			}
			
			
	}

//	delay_ms(20); // �������� Stay connected
}









//����ֵ���Ժ��� //Key value testing function
void PS2_Data_Show(void)
{
	g_PS2_LX = PS2_AnologData(PSS_LX);
	g_PS2_LY = PS2_AnologData(PSS_LY);
	g_PS2_RX = PS2_AnologData(PSS_RX);
	g_PS2_RY = PS2_AnologData(PSS_RY);
	g_PS2_KEY = PS2_DataKey();

	
	if ((g_PS2_LX == 255) && (g_PS2_LY == 255) && (g_PS2_RX == 255) && (g_PS2_RY == 255))
	{
		if (g_flag == 1)
		{
			printf("PS2 mode is RED or GREEN mode \r\n");
			printf("Or PS2 disconnect! \r\n");
			g_flag = 0;
		}
	}

	else
	{
		if (g_flag == 0)
		{
			printf("PS2 mode is RED^GREEN mode \r\n");
			g_flag = 1;
		}
		// Only the red green mode has the correct joystick value ֻ�к���ģʽ������ȷ��ҡ��ֵ 
		if (g_PS2_LX > 130 || g_PS2_LX < 100)
			printf("PS2_LX = %d \r\n", g_PS2_LX);
		if (g_PS2_LY > 130 || g_PS2_LY < 100)
			printf("PS2_LY = %d \r\n", g_PS2_LY);
		if (g_PS2_RX > 130 || g_PS2_RX < 100)
			printf("PS2_RX = %d \r\n", g_PS2_RX);
		if (g_PS2_RY > 130 || g_PS2_RY < 100)
			printf("PS2_RY = %d \r\n", g_PS2_RY);
	}

	switch (g_PS2_KEY)
	{
	case PSB_SELECT:
		printf("key = PSB_SELECT\r\n");
		break;
	case PSB_L3:
		printf("key = PSB_L3\r\n");
		break;
	case PSB_R3:
		printf("key = PSB_R3\r\n");
		break;
	case PSB_START:
		printf("key = PSB_START\r\n");
		break;
	case PSB_PAD_UP:
		printf("key = PSB_PAD_UP\r\n");
		break;
	case PSB_PAD_RIGHT:
		printf("key = PSB_PAD_RIGHT\r\n");
		break;
	case PSB_PAD_DOWN:
		printf("key = PSB_PAD_DOWN\r\n");
		break;
	case PSB_PAD_LEFT:
		printf("key = PSB_PAD_LEFT\r\n");
		break;
	case PSB_L2:
		printf("key = PSB_L2\r\n"); 
		break;
	case PSB_R2:
		printf("key = PSB_R2\r\n");
		break;
	case PSB_L1:
		printf("key = PSB_L1\r\n");
		break;
	case PSB_R1:
		printf("key = PSB_R1\r\n");
		break;
	case PSB_GREEN:
		printf("key = PSB_GREEN\r\n");
		break;
	case PSB_RED:
		printf("key = PSB_RED\r\n");
		break;
	case PSB_BLUE:
		printf("key = PSB_BLUE\r\n");
		break;
	case PSB_PINK:
		printf("key = PSB_PINK\r\n");
		break;
	}

//	delay_ms(100); // ��ʱ100ms�� Delay 100ms
}

