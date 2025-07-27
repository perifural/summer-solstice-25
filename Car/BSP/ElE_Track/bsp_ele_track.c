#include "bsp_ele_track.h"

int Sensor_Left_1,Sensor_Left_2,Sensor_Left_3;
int Sensor_Right_1,Sensor_Right_2,Sensor_Right_3;
int Sensor_Middle;
int ele_seat = 0;

/**************************************************************************
Function function: Electromagnetic sensor sampling initialization
Entrance parameters: None
Return value: None
�������ܣ���Ŵ�����������ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/

//PC0-3  pC4-5 pB0-1 ����Ŵ������ɼ�  pC4-5 pB0-1�⼸�������ұ�,�ͺ���Ѳ�ߴ����������ض���
//PC0-3 pC4-5 pB0-1 for electromagnetic sensor collection pC4-5 pB0-1 these receive the right side and infrared line patrol sensor pin redefine
void  ele_Init(void)
{    
 	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | ELE_ADC_CLK	, ENABLE );	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M //Set the ADC division factor to 6 72M/6=12, and the maximum ADC time cannot exceed 14M
	//����ģ��ͨ����������    //Set analog channel input pins
	
	
	
	//��� ADC 10,11,12   Left ADC 10,11,12
	GPIO_InitStructure.GPIO_Pin = ELE_L1_Pin | ELE_L2_Pin |ELE_L3_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		//ģ����������  ���ղ�Ϊ0�����Ը�Ϊ���� //The analog input pin is not suspended at 0 and can be changed to pull-down
	GPIO_Init(ELE_L1_Port, &GPIO_InitStructure);			//����GPIOC��ѡһ���ͺ� //All are GPOC, just choose one
	
	
	//�м� ADC 13   Middle ADC 13
	GPIO_InitStructure.GPIO_Pin = ELE_MID_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		//ģ����������  ���ղ�Ϊ0�����Ը�Ϊ���� //The analog input pin is not suspended at 0 and can be changed to pull-down
	GPIO_Init(ELE_MID_Port, &GPIO_InitStructure);		
	
	//�ұ� ADC 14,15   Right ADC 14,15
	GPIO_InitStructure.GPIO_Pin = ELE_R1_Pin |ELE_R2_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		//ģ����������  ���ղ�Ϊ0�����Ը�Ϊ���� //The analog input pin is not suspended at 0 and can be changed to pull-down
	GPIO_Init(ELE_R1_Port, &GPIO_InitStructure);	 //����GPIOC��ѡһ���ͺ�  //All are GPOC, just choose one
	
	//�ұ� ADC 8  Right ADC 8
	GPIO_InitStructure.GPIO_Pin = ELE_R3_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		//ģ����������  ���ղ�Ϊ0�����Ը�Ϊ����  //The analog input pin is not suspended at 0 and can be changed to pull-down
	GPIO_Init(ELE_R3_Port, &GPIO_InitStructure);	 
	
	
	ADC_DeInit(ELE_ADC);  //��λADC,������ ADC ��ȫ���Ĵ�������Ϊȱʡֵ  //Reset ADC, reset all registers of the peripheral ADC to default values
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ  //ADC working mode: ADC1 and ADC2 work in independent mode
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//ģ��ת�������ڵ�ͨ��ģʽ  //Analog to digital conversion works in single channel mode
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//ģ��ת�������ڵ���ת��ģʽ  //Analog to digital conversion works in single conversion mode
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������  //The conversion is initiated by software rather than external triggers
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ��� //Right aligned ADC data
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//˳����й���ת����ADCͨ������Ŀ  //The number of ADC channels for sequential rule conversion
	ADC_Init(ELE_ADC, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���    //Initialize the registers of the peripheral ADCx based on the parameters specified in ADC_initStruct
	ADC_Cmd(ELE_ADC, ENABLE);	
	ADC_ResetCalibration(ELE_ADC);	//ʹ�ܸ�λУ׼   //Enable reset calibration	 
	while(ADC_GetResetCalibrationStatus(ELE_ADC));	//�ȴ���λУ׼����	 //Waiting for reset calibration to end
	ADC_StartCalibration(ELE_ADC);	 //����ADУ׼  //Enable AD calibration
	while(ADC_GetCalibrationStatus(ELE_ADC));	 //�ȴ�У׼���� //Waiting for calibration to end
}		

/**************************************************************************
Function Function: AD Sampling
Entry parameter: Channel of ADC1
Return value: AD conversion result
�������ܣ�AD����
��ڲ�����ADC1 ��ͨ��
����  ֵ��ADת�����
**************************************************************************/
u16 Get_Adc_ele(u8 ch)   
{
	  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ�� //Set the rule group channel for the specified ADC, a sequence, and sampling time
	ADC_RegularChannelConfig(ELE_ADC, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	 //ADC1, ADC channel, sampling time is 239.5 cycles  			     
	ADC_SoftwareStartConvCmd(ELE_ADC, ENABLE);		//ʹ��ָ����ADC1�����ת����������	 //Enable the software conversion startup function of the specified ADC1	 
	while(!ADC_GetFlagStatus(ELE_ADC, ADC_FLAG_EOC ));//�ȴ�ת������ //Waiting for the conversion to end
	return ADC_GetConversionValue(ELE_ADC);	//�������һ��ADC1�������ת����� //Return the latest conversion result of ADC1 rule group
}

//�õ�����������һ�㷨 //Use the obtained data as a normalization algorithm
int guiyi_way(void)
{
	int sum , Sensor;
	int Sensor_Left,Sensor_Right;
	
	//��һ������  Normalization
//	sum=(Sensor_Left_1*1+Sensor_Left_3*100) 
//			+ Sensor_Middle *200 
//			+(Sensor_Right_1*300+Sensor_Right_3*399);  
//	Sensor_Left =  Sensor_Left_1+Sensor_Left_3;
//	Sensor_Right = Sensor_Right_1+Sensor_Right_3;
	
	sum=(Sensor_Left_3*1) 
			+ Sensor_Middle *100 
			+(Sensor_Right_1*199);  
	
		Sensor_Left =  Sensor_Left_3; //+ Sensor_Left_1;
  	Sensor_Right = Sensor_Right_1;// + Sensor_Right_3;
	
	Sensor=sum/(Sensor_Left+Sensor_Middle+Sensor_Right);   //��ƫ�� //Request deviation
	return Sensor;//����Ŀǰ���ڴų���λ��  //Return to the current position in the magnetic field
}

//��ȡ����������  //Obtain sensor data
void getEleData(void)
{
	//С��ƨ�ɶ����Լ����������� //The car's butt is facing itself, counting from left to right
	
	
	Sensor_Left_1=Get_Adc_ele(ELE_L1_CH)>>4;                //�ɼ���ߵ�е�����  Collect data from the left inductor
	Sensor_Left_3=Get_Adc_ele(ELE_L3_CH)>>4;                //�ɼ���ߵ�е�����  Collect data from the left inductor
	
	Sensor_Middle=Get_Adc_ele(ELE_M1_CH)>>4;              //�ɼ��м��е�����  Collect data on the intermediate inductance
	
	Sensor_Right_1=Get_Adc_ele(ELE_R1_CH)>>4;               //�ɼ��ұߵ�е�����  Collect data from the right inductor
	Sensor_Right_3=Get_Adc_ele(ELE_R3_CH)>>4;               //�ɼ��ұߵ�е�����  Collect data from the right inductor

	//��Ϊ�Ŵ������ȣ��˲�һ�£������ǲ��ᣬѲ��ֻ�ô�3��
	//Because the amplifier is unstable, filtering it will not work normally. Only these 3 are used for line inspection
	Sensor_Left_3 = deal_getdata(Sensor_Left_3);
	Sensor_Right_1 = deal_getdata(Sensor_Right_1);
	Sensor_Middle = deal_getdata(Sensor_Middle);
	
	ele_seat = guiyi_way();
	
}


int deal_getdata(int a)
{
	if(a<=10)
	{
		return 0;
	}
	else
		return a;
}

//������ʾ����Ļ��
//Data is displayed on the screen
void EleDataDeal(void)
{
	
	char ele_data[30]={'\0'};
	//getEleData();�жϵ�����  The interrupt call

//	sprintf(ele_data,"ele_seat:%d     ",ele_seat);
//	OLED_Draw_Line(ele_data, 1 , false, true);
	//	sprintf(ele_data,"MID:%d        ",Sensor_Middle);
//	OLED_Draw_Line(ele_data, 2 , false, true);
	
	sprintf(ele_data,"seat:%d  MID:%d   ",ele_seat,Sensor_Middle);
	OLED_Draw_Line(ele_data, 1 , false, true);
		
	sprintf(ele_data,"L1:%d  L3:%d     ",Sensor_Left_1,Sensor_Left_3);
	OLED_Draw_Line(ele_data, 2 , false, true);
		
	sprintf(ele_data,"R1:%d  R3:%d     ",Sensor_Right_1,Sensor_Right_3);
	OLED_Draw_Line(ele_data, 3 , false, true);
	

}



