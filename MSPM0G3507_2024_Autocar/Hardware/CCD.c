#include "CCD.h"

extern uint8_t CCD_Zhongzhi;
uint16_t ADV[128]={0};
unsigned int adc_value = 0;
unsigned int voltage_value = 0;
volatile bool gCheckADC;        //ADC采集成功标志位
//读取ADC的数据
unsigned int adc_getValue(void)
{
        unsigned int gAdcResult = 0;
        
        //软件触发ADC开始转换
        DL_ADC12_startConversion(ADC_VOLTAGE_INST);
        //如果当前状态为正在转换中则等待转换结束
        while (false == gCheckADC) {
            __WFE();
        }
        //获取数据
        gAdcResult = DL_ADC12_getMemResult(ADC_VOLTAGE_INST, ADC_VOLTAGE_ADCMEM_0);
        
        //清除标志位
        gCheckADC = false;

        return gAdcResult;
}

/**************************************************************************
函数功能：CCD数据采集
入口参数：无
返回  值：无
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
	 
	 
		 for(i=0;i<128;i++)					//读取128个像素点电压值
  { 
      DL_GPIO_clearPins(GPIO_CLK_PORT,GPIO_CLK_PIN_23_PIN); //TSL_CLK=0;
     delay_us(100);//Dly_us();  //调节曝光时间
		

    //ADV[tslp]=(adc_getValue())>>4;
		adc_value = adc_getValue();
		//printf("%d", (int)ADV[tslp]); 
		//uart0_send_uint8(ADV[tslp]);
   
     DL_GPIO_setPins(GPIO_CLK_PORT,GPIO_CLK_PIN_23_PIN); //TSL_CLK=1;
    delay_us(50);//Dly_us();	
  }  
}


/**************************************************************************
函数功能：线性CCD取中值
入口参数：无
返回  值：无
**************************************************************************/
void  Find_CCD_Median (void)
{ 
	 static uint16_t i,j,Left,Right,Last_CCD_Median;
	 static uint16_t value1_max,value1_min;
	 static uint16_t CCD_Threshold;
	 //阈值说明：CCD采集回来的128个数据，每个数据单独与阈值进行比较，比阈值大为白色，比阈值小为黑色
	 //动态阈值算法，读取每次采集数据的最大和最小值的平均数作为阈值 
	 value1_max=ADV[0];  
   for(i=5;i<123;i++)   //两边各去掉5个点
     {
       if(value1_max<=ADV[i])
       value1_max=ADV[i];
     }
	  value1_min=ADV[0];  //最小值
    for(i=5;i<123;i++) 
     {
       if(value1_min>=ADV[i])
       value1_min=ADV[i];
     }
   CCD_Threshold =(value1_max+value1_min)/2;	  //计算出本次中线提取的阈值
		 
	 for(i = 5;i<118; i++)   //寻找左边跳变沿，连续三个白像素后连续三个黑像素判断左边跳变沿
	 {
		 if(ADV[i]>CCD_Threshold &&ADV[i+1]>CCD_Threshold &&ADV[i+2]>CCD_Threshold &&ADV[i+3]<CCD_Threshold &&ADV[i+4]<CCD_Threshold &&ADV[i+5]<CCD_Threshold )
		 {	
			 Left=i+2;
			 break;	
		 }
	 }
	 for(j = 118;j>5; j--)//寻找右边跳变沿，连续三个黑像素后连续三个白像素判断右边跳变沿
   {
		if(ADV[j]<CCD_Threshold &&ADV[j+1]<CCD_Threshold &&ADV[j+2]<CCD_Threshold &&ADV[j+3]>CCD_Threshold &&ADV[j+4]>CCD_Threshold &&ADV[j+5]>CCD_Threshold )
		 {	
		   Right=j+2;
		   break;	
		 }
   }
	CCD_Zhongzhi =(uint8_t)(Right+Left)/2;//计算中线位置
//	if(myabs_uint_16_t(CCD_Median-Last_CCD_Median)>90)   //计算中线的偏差，如果太大
//	CCD_Median=Last_CCD_Median;    //则取上一次的值
//	Last_CCD_Median=CCD_Median;  //保存上一次的偏差
	
	//uart0_send_uint8(CCD_Median);
}



