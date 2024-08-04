#include "CCD.h"

extern uint8_t CCD_Zhongzhi;
uint16_t ADV[128]={0};
unsigned int adc_value = 0;
unsigned int voltage_value = 0;
volatile bool gCheckADC;        //ADC�ɼ��ɹ���־λ
//��ȡADC������
unsigned int adc_getValue(void)
{
        unsigned int gAdcResult = 0;
        
        //�������ADC��ʼת��
        DL_ADC12_startConversion(ADC_VOLTAGE_INST);
        //�����ǰ״̬Ϊ����ת������ȴ�ת������
        while (false == gCheckADC) {
            __WFE();
        }
        //��ȡ����
        gAdcResult = DL_ADC12_getMemResult(ADC_VOLTAGE_INST, ADC_VOLTAGE_ADCMEM_0);
        
        //�����־λ
        gCheckADC = false;

        return gAdcResult;
}

/**************************************************************************
�������ܣ�CCD���ݲɼ�
��ڲ�������
����  ֵ����
**************************************************************************/
 void RD_TSL(void) 
{
  int i=0,tslp=0;
   DL_GPIO_setPins(GPIO_CLK_PORT,GPIO_CLK_PIN_23_PIN); delay_us(5);//TSL_CLK=1;
  DL_GPIO_clearPins(GPIO_SI_PORT,GPIO_SI_PIN_25_PIN); //TSL_SI=0;  
  delay_us(20);//Dly_us();
      
   DL_GPIO_setPins(GPIO_SI_PORT,GPIO_SI_PIN_25_PIN);delay_us(5); //TSL_SI=1; 
  DL_GPIO_clearPins(GPIO_CLK_PORT,GPIO_CLK_PIN_23_PIN); //TSL_CLK=0;  
  delay_us(30);//Dly_us();
	
	
  DL_GPIO_setPins(GPIO_CLK_PORT,GPIO_CLK_PIN_23_PIN);delay_us(5);//TSL_CLK=1; 
   DL_GPIO_clearPins(GPIO_SI_PORT,GPIO_SI_PIN_25_PIN);//TSL_SI=0; 
  delay_us(10);//Dly_us(); 		
	 DL_GPIO_clearPins(GPIO_CLK_PORT,GPIO_CLK_PIN_23_PIN); //TSL_CLK=0;  
	 delay_us(100);
	 
	 
		 for(i=0;i<128;i++)					//��ȡ128�����ص��ѹֵ
  { 
      DL_GPIO_clearPins(GPIO_CLK_PORT,GPIO_CLK_PIN_23_PIN); //TSL_CLK=0;
     delay_us(100);//Dly_us();  //�����ع�ʱ��
		

    //ADV[tslp]=(adc_getValue())>>4;
		adc_value = adc_getValue();
		//printf("%d", (int)ADV[tslp]); 
		//uart0_send_uint8(ADV[tslp]);
   
     DL_GPIO_setPins(GPIO_CLK_PORT,GPIO_CLK_PIN_23_PIN); //TSL_CLK=1;
    delay_us(50);//Dly_us();	
  }  
}


/**************************************************************************
�������ܣ�����CCDȡ��ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void  Find_CCD_Median (void)
{ 
	 static uint16_t i,j,Left,Right,Last_CCD_Median;
	 static uint16_t value1_max,value1_min;
	 static uint16_t CCD_Threshold;
	 //��ֵ˵����CCD�ɼ�������128�����ݣ�ÿ�����ݵ�������ֵ���бȽϣ�����ֵ��Ϊ��ɫ������ֵСΪ��ɫ
	 //��̬��ֵ�㷨����ȡÿ�βɼ����ݵ�������Сֵ��ƽ������Ϊ��ֵ 
	 value1_max=ADV[0];  
   for(i=5;i<123;i++)   //���߸�ȥ��5����
     {
       if(value1_max<=ADV[i])
       value1_max=ADV[i];
     }
	  value1_min=ADV[0];  //��Сֵ
    for(i=5;i<123;i++) 
     {
       if(value1_min>=ADV[i])
       value1_min=ADV[i];
     }
   CCD_Threshold =(value1_max+value1_min)/2;	  //���������������ȡ����ֵ
		 
	 for(i = 5;i<118; i++)   //Ѱ����������أ��������������غ����������������ж����������
	 {
		 if(ADV[i]>CCD_Threshold &&ADV[i+1]>CCD_Threshold &&ADV[i+2]>CCD_Threshold &&ADV[i+3]<CCD_Threshold &&ADV[i+4]<CCD_Threshold &&ADV[i+5]<CCD_Threshold )
		 {	
			 Left=i+2;
			 break;	
		 }
	 }
	 for(j = 118;j>5; j--)//Ѱ���ұ������أ��������������غ����������������ж��ұ�������
   {
		if(ADV[j]<CCD_Threshold &&ADV[j+1]<CCD_Threshold &&ADV[j+2]<CCD_Threshold &&ADV[j+3]>CCD_Threshold &&ADV[j+4]>CCD_Threshold &&ADV[j+5]>CCD_Threshold )
		 {	
		   Right=j+2;
		   break;	
		 }
   }
	CCD_Zhongzhi =(uint8_t)(Right+Left)/2;//��������λ��
//	if(myabs_uint_16_t(CCD_Median-Last_CCD_Median)>90)   //�������ߵ�ƫ����̫��
//	CCD_Median=Last_CCD_Median;    //��ȡ��һ�ε�ֵ
//	Last_CCD_Median=CCD_Median;  //������һ�ε�ƫ��
	
	//uart0_send_uint8(CCD_Median);
}



