#include "app_bluetooth.h"

#define 	run_car     '1'//����ǰ Before pressing the button
#define 	back_car    '2'//������ After pressing the button
#define 	left_car    '3'//������ Left button
#define 	right_car   '4'//������ Right button
#define 	stop_car    '0'//����ͣ Press the button to stop

//�ϱ����� Reporting data
int g_autoup = 0;
char manydisplay[80] ={0};
char updata[80] ={0};
char lspeed[10],rspeed[10],daccel[10],dgyro[10],csb[10],vi[10];

u8 newLineReceived = 0;
int num = 0;
u8 startBit = 0;
int int9num =0;
u8 inputString[80] = {0};
u8 ProtocolString[80] = {0};

enCarState g_newcarstate = enSTOP; //  1ǰ2��3��4��0ֹͣ 1 forward 2 backward 3 left 4 right 0 stop


//PID����  PID section
extern float Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki,Turn_Kp,Turn_Kd; //������ֱ�����ٶȻ�,ת�� //Introducing vertical rings, speed rings, and steering rings
char piddisplay[50] ="$AP";
char charkp[10],charkd[10],charksp[10],charksi[10] ,charktp[10],charktd[10];
float PID_Original[6] = {0};

int StringFind(const char *pSrc, const char *pDst)  
{  
    int i, j;  
    for (i=0; pSrc[i]!='\0'; i++)  
    {  
        if(pSrc[i]!=pDst[0])  
            continue;         
        j = 0;  
        while(pDst[j]!='\0' && pSrc[i+j]!='\0')  
        {  
            j++;  
            if(pDst[j]!=pSrc[i+j])  
            break;  
        }  
        if(pDst[j]=='\0')  
            return i;  
    }  
    return -1;  
}  


//�������ܣ�����6��PID�ĳ�ʼֵ  Function Function: Retain the initial values of 6 PIDs
void Init_PID(void)
{
	PID_Original [0] = Balance_Kp;
	PID_Original [1] = Balance_Kd;
	PID_Original [2] = Velocity_Kp;
	PID_Original [3] = Velocity_Ki;
	PID_Original [4] = Turn_Kp;
	PID_Original [5] = Turn_Kd;
}

//��������:�ָ�������PIDֵ  Function function: Restore the PID value when turned on
void ResetPID(void)
{	
	if(Balance_Kp != PID_Original[0])
	{
		Balance_Kp = PID_Original[0];
	}
	if(Balance_Kd != PID_Original[1])
	{
		Balance_Kd = PID_Original[1];
	}
	if(Velocity_Kp != PID_Original[2])
	{
		Velocity_Kp = PID_Original[2];
	}
	if(Velocity_Ki != PID_Original[3])
	{
		Velocity_Ki = PID_Original[3];
	}
	if(Turn_Kp != PID_Original[4])
	{
		Turn_Kp = PID_Original[4];
	}
	if(Turn_Kd != PID_Original[5])
	{
		Turn_Kd = PID_Original[5];
	}
}	


void deal_bluetooth(uint8_t rxbuf)
{
		u8 uartvalue = rxbuf;
	
	 if(uartvalue == '$')
	    {
	      startBit = 1;
		    num = 0;
	    }
	    if(startBit == 1)
	    {
	       	inputString[num] = uartvalue;     
	    }  
	    if (startBit == 1 && uartvalue == '#') 
	    {
	    	
			newLineReceived = 1; 
			startBit = 0;
			int9num = num;	
		
	    }
		num++;
		if(num >= 80)
		{
			num = 0;
			startBit = 0;
			newLineReceived	= 0;
		}	 

}


void ProtocolCpyData(void)
{
	memcpy(ProtocolString, inputString, num+1);
	memset(inputString, 0x00, sizeof(inputString));
}

//��ͣ���ϱ����ݻ�Ӱ��ƽ��   //Continuously reporting data can affect balance
void Protocol(void)
{	
	switch (ProtocolString[1])
	{
		case run_car:	 g_newcarstate = enRUN; break;
		case back_car:  g_newcarstate = enBACK; break;
		case left_car:  g_newcarstate = enLEFT; break;
		case right_car: g_newcarstate = enRIGHT; break;
		case stop_car:  g_newcarstate = enSTOP; break;
		default: g_newcarstate = enSTOP; break;
		
	}
	if (ProtocolString[3] == '1') //���� Left-handed
	{
		g_newcarstate = enTLEFT;	
	}
	
	if (ProtocolString[3] == '2') //���� Right
	{
		g_newcarstate = enTRIGHT;
	}

//	/*��ֹ���ݶ��� Preventing Data Loss */
	if(strlen((const char *)ProtocolString)<21)
	{
		newLineReceived = 0;  
		memset(ProtocolString, 0x00, sizeof(ProtocolString));  
		bluetooth_send_string("$ReceivePackError#"); //����Э�����ݰ�  Return protocol data packet
		return;
	}

	
	//��ѯPID   Query PID
	if(ProtocolString[5]=='1')
	{
		ProtocolGetPID(); //app bug ����2��  App bug sent 2 times
		delay_ms(5);
		ProtocolGetPID();
	}
	else if(ProtocolString[5]=='2')  //�ָ�Ĭ��PID  Restore the default PID
	{
		ResetPID();
		ProtocolGetPID(); //�ڷ���һ��pid  Sending a pid
		bluetooth_send_string("$OK#");//����Э�����ݰ�  Return protocol data packet
	}

	//�Զ��ϱ�  Automatic reporting
	if(ProtocolString[7]=='1')
	{
			g_autoup = 1; 
			bluetooth_send_string("$OK#"); //����Э�����ݰ�  	 Return protocol data packet
	}
	else if(ProtocolString[7]=='2')
	{		
			g_autoup = 0;		
			bluetooth_send_string("$OK#"); //����Э�����ݰ�  	 	Return protocol data packet
	}

	//����PID�Ĳ���  Update PID parameters
	if (ProtocolString[9] == '1') //�ǶȻ�����  Angular Ring Update  $0,0,0,0,1,1,AP23.54,AD85.45,VP10.78,VI0.26,TP0.12,TD0.00#
	{
		//$0,0,0,0,1,1,AP23.54,AD85.45,VP10.78,VI0.26,TP0.12,TD0.00#

		int pos,z; 
		char apad[25] = {0},apvalue[8] = {0},advalue[8] = {0};
			
		pos = StringFind((const char *)ProtocolString, (const char *)"AP");
		if(pos == -1) return;
		
		memcpy(apad,ProtocolString+pos,int9num-pos);

		//AP23.54,AD85.45,VP10.78,VI0.26,TP0.12,TD0.00#
		z = StringFind(apad, ",");
		if(z == -1) return;
		memcpy(apvalue, apad+2, z-2);
		
		Balance_Kp = atof(apvalue)*100; //*100�Ƿŵ�100��  *100 means magnify 100 times
		
		
		memset(apad, 0x00, sizeof(apad));
		memcpy(apad, ProtocolString + pos + z + 1, int9num - (pos + z)); //�洢AD��������� Store data after AD
		z = StringFind(apad, ",");
		if(z == -1) return;
		memcpy(advalue,apad+2, z-2);
		
		Balance_Kd=atof(advalue); //Ĭ�ϵ�ֵ���ǷŴ�100��  The default value is to magnify 100 times.

		bluetooth_send_string("$OK#"); //����Э�����ݰ�   Return protocol packet				
	}
		
  	if(ProtocolString[11] == '1')  //�˽���Ҫ΢��  This analysis needs to be slightly modified
	{
		int pos,z; 
		char vpvi[25] = {0},vpvalue[8] = {0},vivalue[8] = {0};
			
		pos = StringFind((const char *)ProtocolString, (const char *)"VP");
		if(pos == -1) return;
		
		memcpy(vpvi, ProtocolString+pos, int9num-pos);
		//y=strchr(apad,'AP');
		//AP23.54,AD85.45,VP10.78,VI0.26,TP0.12,TD0.00#
		z = StringFind(vpvi, ",");
		if(z == -1) return;
		memcpy(vpvalue, vpvi+2, z-2);
		
		Velocity_Kp = atof(vpvalue) *100;//*100�Ƿŵ�100��  *100 means magnify 100 times
		
		
		memset(vpvi, 0x00, sizeof(vpvi));
		memcpy(vpvi, ProtocolString + pos + z + 1, int9num - (pos + z)); //�洢AD���������  Store data after AD
		z = StringFind(vpvi, ","); //�����ת�򻷺� #��,    After adding the turning ring # changes,
		if(z == -1) return;
		memcpy(vivalue,vpvi+2, z-2);
		
		Velocity_Ki=atof(vivalue);//Ĭ�ϵ�ֵ���ǷŴ�100��  The default value is to magnify 100 times.

		bluetooth_send_string("$OK#"); //����Э�����ݰ�  	Return protocol data packet	
			
			
	}
	
	//����ת�򻷵����� Analyze the data of the steering ring
	 if(ProtocolString[13] == '1')
	{
		int pos,z; 
		char tptd[25] = {0},tpvalue[8] = {0},tdvalue[8] = {0};
			
		pos = StringFind((const char *)ProtocolString, (const char *)"TP");
		if(pos == -1) return;
		
		memcpy(tptd, ProtocolString+pos, int9num-pos);
		
		z = StringFind(tptd, ",");
		if(z == -1) return;
		memcpy(tpvalue, tptd+2, z-2);
		
		Turn_Kp = atof(tpvalue) *100;//*100�Ƿŵ�100��   *100 means magnify 100 times
		
		
		memset(tptd, 0x00, sizeof(tptd));
		memcpy(tptd, ProtocolString + pos + z + 1, int9num - (pos + z)); //�洢AD���������  Store data after AD
		z = StringFind(tptd, "#");
		if(z == -1) return;
		memcpy(tdvalue,tptd+2, z-2);
		
		Turn_Kd = atof(tdvalue);//��Ϊapp bug,Ĭ�ϵ�ֵ���ǷŴ�100��  Because of the app bug, the default value is to magnify 100 times

		bluetooth_send_string("$OK#"); //����Э�����ݰ�  Return protocol packet		
			
			
	}
		
	newLineReceived = 0;  
	memset(ProtocolString, 0x00, sizeof(ProtocolString));  


}


float s_Acc = 0, s_Gyro = 0;
void CalcUpData(void)
{
	float ls, rs,sLence;
	
	if(g_autoup == 1)
	{
		ls = Velocity_Left;//�����ٶ�  left speed
		rs = Velocity_Right;//�ҵ���ٶ�  right speed
		s_Acc = Acceleration_Z/100; //���ٶ�  acceleration //Ϊ������app��ʾ��ȫ����С100��  In order to fully display it in the app, it is reduced by 100 times
		s_Gyro = Gyro_Balance; //������  gyroscope
		sLence = g_distance/10.0; //������  ultrasonic
		
	
		memset(manydisplay, 0x00, 80);
		memcpy(manydisplay, "$LV", 4);
	
		//�˷������ݿ��ܻ����	 This method may cause data errors	
		//'$LV20.5,RV33.2,AC140,GY40,CSB50,VI12.3#
		sprintf(manydisplay,"$LV%3.2f,RV%3.2f,AC%3.2f,GY%3.2f,CSB%3.2f,VT%3.2f#",ls,rs,s_Acc,s_Gyro,(float)sLence,battery); 
		memset(updata, 0x00, 80);
		memcpy(updata, manydisplay, 80);
		
// ��Ϊ�˿��bug ,����Ͳ�Ҫ��	 Because of the bug of this library, the following is not needed
//		memset(lspeed, 0x00, sizeof(lspeed));
//		memset(rspeed, 0x00, sizeof(rspeed));
//		memset(daccel, 0x00, sizeof(daccel));
//		memset(dgyro, 0x00, sizeof(dgyro));
//		memset(csb, 0x00, sizeof(csb));
//		memset(vi, 0x00, sizeof(vi));
	
		//����ٶ�  left speed
//		if((ls <= 1000) && (ls >= -1000))
//			sprintf(lspeed,"%3.2f",ls);
//		else
//		{
//			return;
//		}
//			
//		//�ұ��ٶ� right speed
//		if((rs <= 1000) && (rs >= -1000))
//			sprintf(rspeed,"%3.2f",rs);
//		else
//		{	
//			return;
//		}
//		
//		//���ٶ�  acc
//		if((s_Acc > -2000) && (s_Acc < 2000))
//			sprintf(daccel,"%3.2f",s_Acc);
//		else
//		{
//			return;
//		}
//		
//		//������ gryo
//		if((s_Gyro > -10000) && (s_Gyro < 10000))
//			sprintf(dgyro,"%3.2f",s_Gyro);
//		else
//		{
//			return;
//		}
//	
//		//��ʱ������  ultrasonic
//		if((sLence >= 0) && (sLence < 10000))
//			sprintf(csb,"%3.2f",(float)sLence);
//		else
//		{
//			return;
//		}
//		
//		//����  quantity of electricity
//		if((battery >= 0) && (battery < 20))
//			sprintf(vi,"%3.2f",battery);
//		else
//		{
//			return;
//		}
//	
//		strcat(manydisplay,lspeed);
//		strcat(manydisplay,",RV");
//		strcat(manydisplay,rspeed);
//		strcat(manydisplay,",AC");
//		strcat(manydisplay,daccel);
//		strcat(manydisplay,",GY");
//		strcat(manydisplay,dgyro);
//		strcat(manydisplay,",CSB");
//		strcat(manydisplay,csb);
//		strcat(manydisplay,",VT");
//		strcat(manydisplay,vi);
//		strcat(manydisplay,"#");
//		memset(updata, 0x00, 80);
//		memcpy(updata, manydisplay, 80);
	}
	
}


//�Զ��ϱ�  Automatic reporting
int g_uptimes = 1; //�Զ��ϱ� 2�뱨һ�� Automatically report every 2 seconds
void SendAutoUp(void)
{
	g_uptimes --;
	if ((g_autoup == 1) && (g_uptimes == 0))
	{
		CalcUpData();
		bluetooth_send_string(updata); //����Э�����ݰ�	
	}
	if(g_uptimes == 0)
		 g_uptimes = 1;

}


//��ѶPID Query PID
void ProtocolGetPID(void)
{
	memset(piddisplay, 0x00, sizeof(piddisplay));
	memcpy(piddisplay, "$AP", 4);

	if(Balance_Kp >= 0 ) //&& Balance_Kp <= 28800  
	{
		sprintf(charkp,"%3.2f",Balance_Kp/100);
	}
	else
	{	
		bluetooth_send_string("$GetPIDError#"); //����Э�����ݰ�   Return protocol data packet
		return;
	}

	
	if(Balance_Kd >= 0 ) //&& Balance_Kd <= 100
	{
		sprintf(charkd,"%3.2f",Balance_Kd);//��ֵԭ������ȥ This value is passed natively
	}
	else
	{	
		bluetooth_send_string("$GetPIDError#"); //����Э�����ݰ�   Return protocol data packet
		return;
	}
	
	if(Velocity_Kp >= 0 ) //&& Velocity_Kp <= 20000
	{
		sprintf(charksp,"%3.2f",Velocity_Kp/100);
	}
	else
	{	
		bluetooth_send_string("$GetPIDError#"); //����Э�����ݰ� 
		return;
	}

	if(Velocity_Ki >= 0 ) //&& Velocity_Ki <= 100
	{
		sprintf(charksi,"%3.2f",Velocity_Ki); //��ֵԭ������ȥ  This value is passed natively
	}
	else
	{	
		bluetooth_send_string("$GetPIDError#"); //����Э�����ݰ�   Return protocol data packet
		return;
	}
	
	
	//ת�� TP  Steering ring TP
	if(Turn_Kp >= 0 ) 
	{
		sprintf(charktp,"%3.2f",Turn_Kp/100); 
	}
	else
	{	
		bluetooth_send_string("$GetPIDError#"); //����Э�����ݰ�   Return protocol data packet
		return;
	}
	
	//ת�� TD  Steering ring TD
	if(Turn_Kd >= 0 ) 
	{
		sprintf(charktd,"%3.2f",Turn_Kd); 
	}
	else
	{	
		bluetooth_send_string("$GetPIDError#"); //����Э�����ݰ�   Return protocol data packet
		return;
	}
	
	
	strcat(piddisplay,charkp);
	strcat(piddisplay,",AD");
	strcat(piddisplay,charkd);
	strcat(piddisplay,",VP");
	strcat(piddisplay,charksp);
	strcat(piddisplay,",VI");
	strcat(piddisplay,charksi);
	
	//���ת��  Add steering ring
	strcat(piddisplay,",TP");
	strcat(piddisplay,charktp);
	strcat(piddisplay,",TD");
	strcat(piddisplay,charktd);
	strcat(piddisplay,"#");
	
	bluetooth_send_string(piddisplay); //����Э�����ݰ�   Return protocol data packet

}



