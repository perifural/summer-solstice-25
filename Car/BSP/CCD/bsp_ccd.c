#include "bsp_ccd.h"

char buf_CCD[20] = {'\0'};
u16 ADV[128]={0};
u8 CCD_Zhongzhi,CCD_Yuzhi;

/**************************************************************************
Function function: Linear CCD initialization
Entrance parameters: None
Return value: None
�������ܣ�����CCD��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
/*
*PF5 -> CLK
*PF4 -> CS
*PF6 ->AO
*/

void  ccd_Init(void)
{    
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd( CCD_AO_CLK | CCD_ADC_CLK	, ENABLE );	  //ʹ��ADC3ͨ��ʱ��  // Enable ADC3 channel clock
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M  //Set ADC division factor 6 72M/6=12, ADC maximum time cannot exceed 14M
	
	//����ģ��ͨ����������       Set analog channel input pins                   
	GPIO_InitStructure.GPIO_Pin = CCD_AO_PIN;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ���������� Analog input pins
	GPIO_Init(CCD_AO_PORT, &GPIO_InitStructure);	
	
	//��ʼ��SI�ӿ� Initialize SI interface
	RCC_APB2PeriphClockCmd( CCD_SI_CLK	, ENABLE );
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//������� Push pull output
	GPIO_InitStructure.GPIO_Pin = CCD_SI_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(CCD_SI_PORT,&GPIO_InitStructure);
	
	//��ʼ��CLK�ӿ� Initialize CLK interface
	RCC_APB2PeriphClockCmd( CCD_CLK_CLK	, ENABLE );
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//������� Push pull output
	GPIO_InitStructure.GPIO_Pin = CCD_CLK_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(CCD_CLK_PORT,&GPIO_InitStructure);
	
	ADC_DeInit(CCD_ADC);  //��λADC2,������ ADC2 ��ȫ���Ĵ�������Ϊȱʡֵ Reset ADC2 and reset all registers of the peripheral ADC2 to default values
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ  ADC working mode: ADC1 and ADC2 work in independent mode
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//ģ��ת�������ڵ�ͨ��ģʽ Analog to digital conversion works in single channel mode
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//ģ��ת�������ڵ���ת��ģʽ Analog to digital conversion works in single conversion mode
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ�������� The conversion is initiated by software rather than external triggers
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ��� Right aligned ADC data
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//˳����й���ת����ADCͨ������Ŀ The number of ADC channels for sequential rule conversion
	ADC_Init(CCD_ADC, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���    Initialize the registers of the peripheral ADCx based on the parameters specified in ADC_initStruct
	ADC_Cmd(CCD_ADC, ENABLE);	//ʹ��ָ����ADC1 Enable specified ADC1
	
	ADC_ResetCalibration(CCD_ADC);	//ʹ�ܸ�λУ׼   Enable reset calibration	 
	while(ADC_GetResetCalibrationStatus(CCD_ADC));	//�ȴ���λУ׼����	Waiting for reset calibration to end
	ADC_StartCalibration(CCD_ADC);	 //����ADУ׼  Enable AD calibration
	while(ADC_GetCalibrationStatus(CCD_ADC));	 //�ȴ�У׼���� Waiting for calibration to end

}	
/**************************************************************************
Function: The AD sampling
Input   : The ADC channels
Output  : AD conversion results
�������ܣ�AD����
��ڲ�����ADC��ͨ��
����  ֵ��ADת�����
**************************************************************************/
u16 Get_Adc_CCD(u8 ch)   
{
	//Sets the specified ADC rule group channel, one sequence, and sampling time
	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	
	//ADC1,ADCͨ��,����ʱ��Ϊ480���� ADC1, ADC channel, sampling time is 480 cycles
	ADC_RegularChannelConfig(CCD_ADC, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	   ADC1, ADC channel, sampling time is 239.5 cycles   
  //Enable the specified ADC1 software transformation startup function	
  //ʹ��ָ����ADC1�����ת����������	
	ADC_SoftwareStartConvCmd(CCD_ADC, ENABLE);			 
	//Wait for the conversion to finish
  //�ȴ�ת������	
	while(!ADC_GetFlagStatus(CCD_ADC, ADC_FLAG_EOC ));
	//Returns the result of the last ADC1 rule group conversion
	//�������һ��ADC1�������ת�����
	return ADC_GetConversionValue(CCD_ADC);	
}

/**************************************************************************
Function Function: Delay
Entrance parameters: None
Return value: None
�������ܣ���ʱ
��ڲ�������
����  ֵ����
**************************************************************************/
void Dly_us(void)
{
   int ii;    
   for(ii=0;ii<10;ii++); 
}

/**************************************************************************
Function function: CCD data acquisition
Entrance parameters: None
Return value: None
�������ܣ�CCD���ݲɼ�
��ڲ�������
����  ֵ����
**************************************************************************/
 void RD_TSL(void) 
{
  u8 i=0,tslp=0;
  TSL_CLK=1;
  TSL_SI=0; 
  Dly_us();
      
  TSL_SI=1; 
  TSL_CLK=0;
  Dly_us();
	
	
  TSL_CLK=1;
  TSL_SI=0;
  Dly_us(); 
  for(i=0;i<128;i++)					//��ȡ128�����ص��ѹֵ Read 128 pixel voltage values
  { 
    TSL_CLK=0; 
    Dly_us();  //�����ع�ʱ�� Adjust exposure time
		Dly_us();
		
    ADV[tslp]=(Get_Adc_CCD(CCD_ADC_CH))>>4; 
    ++tslp;
    TSL_CLK=1;
    Dly_us();	
		


  }  
}

//��ʼCCD�ɼ��������������
//Start CCD collection and processing of output data
void deal_data_ccd(void)
{
//		RD_TSL();  //��ȡͼ��ʱ�Ѿ���ȡ���� Already acquired when acquiring the image
		Find_CCD_Zhongzhi();	 
}

/**************************************************************************
Function function: Linear CCD takes the median value
Entrance parameters: None
Return value: None
�������ܣ�����CCDȡ��ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static u16 i,j,Left,Right;
	 static u16 value1_max,value1_min;
	
	   value1_max=ADV[0];  //��̬��ֵ�㷨����ȡ������Сֵ Dynamic threshold algorithm, reading maximum and minimum values
     for(i=5;i<123;i++)   //���߸�ȥ��5���� Remove 5 points on each side
     {
        if(value1_max<=ADV[i])
        value1_max=ADV[i];
     }
	   value1_min=ADV[0];  //��Сֵ minimum value
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ADV[i])
        value1_min=ADV[i];
     }
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //���������������ȡ����ֵ Calculate the threshold for extracting the centerline in this round
	 for(i = 5;i<118; i++)   //Ѱ����������� Find the left jump edge
	{
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
			Left=i;
			break;	
		}
	}
	 for(j = 118;j>5; j--)//Ѱ���ұ������� Find the right jump edge
  {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		{	
		  Right=j;
		  break;	
		}
  }
	CCD_Zhongzhi=(Right+Left)/2;//��������λ�� Calculate the centerline position
	
	
	
//	printf("zhong: %d\r\n",CCD_Yuzhi);
//	printf("middle : %d\r\n",CCD_Zhongzhi);
	
//	sprintf(buf_CCD,"Yuzhi:%d ",CCD_Yuzhi);
//	OLED_Draw_Line(buf_CCD, 2 , false, true);
//	sprintf(buf_CCD,"Zhongzhi:%d ",CCD_Zhongzhi);
//	OLED_Draw_Line(buf_CCD, 3 , false, true);
//	
//	memset(buf_CCD,0,sizeof(buf_CCD));
	
	//����ʵ�����  According to the actual situation
//	if(math_abs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //�������ߵ�ƫ����̫�� Calculate the deviation from the midline, if it is too large
//	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //��ȡ��һ�ε�ֵ Then take the last value
//	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //������һ�ε�ƫ�� Save the last deviation
	
}


uint8_t ADC_128X32[128] = {0};
// ����128�����ص��ADV�ɼ���ѹֵ��������ֵѹ����128*32��
// Return the ADV collected voltage value of 128 pixels and compress the amplitude into 128*32.
uint8_t* CCD_Get_ADC_128X32(void)
{
		RD_TSL();
    // ��8λADֵת����5λADֵ
		// Convert 8-bit AD value to 5-bit AD value
    for (int i = 0; i < 128; i++)
    {
        ADC_128X32[i] = ADV[i] >> 3;
    }
    return ADC_128X32;
}

void OLED_Show_CCD_Image(uint8_t* p_img)
{
    OLED_Clear();
    for (int i = 0; i < 128; i++)
    {
        if (p_img[i] < 32)
        {
            SSD1306_DrawPixel(i, p_img[i], SSD1306_COLOR_WHITE);
        }
    }
    OLED_Refresh();
}
