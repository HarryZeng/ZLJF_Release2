
///**
//  ********************************  STM32F0x1  *********************************
//  * @文件名     ： SelftStudy.c
//  * @作者       ： HarryZeng
//  * @库版本     ： V1.5.0
//  * @文件版本   ： V1.0.0
//  * @日期       ： 2017年11月24日
//  * @摘要       ： 数据处理
//  ******************************************************************************/
///*----------------------------------------------------------------------------
//  更新日志:
//  2017-11-24 V1.0.0:初始版本
//  ----------------------------------------------------------------------------*/
///* 包含的头文件 --------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "project.h"
#include "bsp_init.h"
#include "key.h"
#include "SelfStudy.h"
#include "flash.h"
#include "display.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_dac.h"

/*第一次SET按键按下处理函数*/
uint8_t  ATTcalibration(void) ;

/*第二次SET按键按下处理函数*/
void GetMAXADCValue(void);

uint8_t SelftStudyflag=0;
uint8_t ATTcalibrationFlag=0;
uint8_t 	selfDisplayEndFlay=0;
uint32_t  SET1ADC_Value=0;
uint32_t CalibrateADCValue=0;
uint32_t CalibrateSAValue=0;
uint32_t CalibrateSBValue=0;

uint32_t CalibrateS1Value=0;
uint32_t CalibrateS2Value=0;


extern int16_t adc_dma_tab[DMA_BUFFER_SIZE];
extern  uint8_t DX_Flag;
extern uint8_t sample_finish;  
void ADCINcalibration(void);

void selfstudy(void)
{
	uint8_t OUT1_STATUS,OUT2_STATUS;
	

	if(SetButton.Status == Press && ModeButton.Status==Press)
	{
		
	}
	else
	{
			/*第一次进入SET模式*/
			while(SetButton.Status == Press )     //只要在显示模式下第一次按下SET按键
			{	
				DX_Flag = 0;
					/*保持OUT1的状态*/
					OUT1_STATUS = GPIO_ReadInputDataBit(OUT1_GPIO_Port,OUT1_Pin);/*获取当前的OUT1状态*/
				GPIO_WriteBit(OUT1_GPIO_Port,OUT1_Pin,(BitAction)OUT1_STATUS);/*保持着OUT1状态*/
					OUT2_STATUS = GPIO_ReadInputDataBit(OUT2_GPIO_Port,OUT2_Pin);/*获取当前的OUT2状态*/
				GPIO_WriteBit(OUT2_GPIO_Port,OUT2_Pin,(BitAction)OUT2_STATUS);/*保持着OUT1状态*/

				/*按着按键3秒内*/
				SMG_DisplaySET_Step_1_Mode(0,0);  //显示SET1
				
				/*DAC输出，ADC调零*/
				ADCINcalibration();
				
				while(SetButton.Effect == PressShort || SetButton.Effect == PressLong) /*按下按键已经超过时间，而且没有释放，闪烁提醒*/
				{		
					/*3秒到了,闪烁提醒*/
					//if(EventFlag&Blink500msFlag) 
					//{
					//	EventFlag = EventFlag &(~Blink500msFlag);  //清楚标志位
						//if(EventFlag&ADVtimeFlag) //按下的3秒内，一直定时采集ADC值
						//{
							//EventFlag = EventFlag &(~ADVtimeFlag);  //清楚标志位
							//GetADCValue(&SET1ADC_Value);		//定时ADC采样
							//SMG_DisplaySET_Step_1_Mode(2,CalibrateADCValue);
					//	}
					if(SetButton.Status == Press && SetButton.Effect == PressShort) 
					{
						SMG_DisplaySET_Step_2_Mode(0,S_Total_Final,0);
						GetMAXADCValue();/*按下的三秒钟内，不断查找最大值*/
	//					DX_Flag = 1;
							//break;
					}
					else if(SetButton.Effect == PressLong && SetButton.Status==Press ) /*按键达到3秒后，第一次进入自学习，等待第二次按下SET 3秒*/
					{	
						DX_Flag = 0;

						/*一直等待第二次SET的按下*
						**************************/
						SetButton.LastCounter = SetButton.PressCounter;
						SetButton.Effect = PressShort;
						UpButton.PressCounter=0;
						UpButton.Effect=PressNOEffect;
						DownButton.PressCounter=0;
						DownButton.Effect=PressNOEffect;
						ModeButton.PressCounter=0;
						ModeButton.Effect=PressNOEffect;
						
						selfDisplayEndFlay =0;
						//GetMAXADCValue();
//						while(SetButton.Status==Press)
//						{
//							DX_Flag = 1;
//							SMG_DisplaySET_Step_1_Mode(2,Threshold);
//							SetButton.Effect = PressShort;
//						}
					}
				}
				
			}
	}
}


/*在一系列的ADCvalue中寻找最大的ADV MAX*/
int32_t 			ADCMAX=0;
int32_t 		NewThreshold=0;
int32_t 		S_MaxValue=0;
int32_t    	SA_MaxValue=0;
int32_t    	SB_MaxValue=0;
void GetMAXADCValue(void)
{
		uint32_t 		TempADCValue=0;
//		static uint8_t lastCounter;
	  S_MaxValue =0;
		while(selfDisplayEndFlay==0)     //第二次按下SET按键
		{
			while(SetButton.Status == Press )
			{
					TempADCValue = 	S_Total_Final ;
				
					if(TempADCValue>=S_MaxValue)   //不断寻找最大值
					{
						S_MaxValue = TempADCValue;
						SA_MaxValue = SA_Final;    	//SA_Max
						//SB_MaxValue = SB_Final;			//SB_Max
					}
				//}
				/*这里要显示SET2*/
				if(EventFlag&Blink500msFlag) 
				{
					EventFlag = EventFlag &(~Blink500msFlag);  //清楚标志位
					SMG_DisplaySET_Step_2_Mode(0,S_Total_Final,0);
				}
				
				while(SetButton.Effect == PressLong) /*按下按键已经超过时间，而且没有释放，闪烁提醒*/
				{	/*3秒到了,闪烁提醒*/
					if(EventFlag&Blink500msFlag) 
					{
						EventFlag = EventFlag &(~Blink500msFlag);  //清楚标志位
						SMG_DisplaySET_Step_2_Mode(1,0,0);     
					}				
					while(SetButton.Effect == PressLong && SetButton.Status == Release) /*按键达到3秒后，结束第二次SET按键*/
					{		/*3秒到了，并释放了按键*/
						
						NewThreshold = (S_MaxValue*3)/4;  //SMAX的3/4作为阈值
						if(NewThreshold<=20) NewThreshold=20;
						if(NewThreshold>=4095) NewThreshold=4095;
						
						
						GPIO_WriteBit(OUT1_GPIO_Port,OUT1_Pin,(BitAction)GPIO_ReadInputDataBit(OUT1_GPIO_Port,OUT1_Pin));
						GPIO_WriteBit(OUT2_GPIO_Port,OUT2_Pin,(BitAction)GPIO_ReadInputDataBit(OUT2_GPIO_Port,OUT2_Pin));
						
						Threshold = NewThreshold;
						selfDisplayEndFlay = 1;
						SetButton.PressCounter = 0;					/*清楚按键次数*/
						SetButton.Status = Release;					/*释放按键*/
						SetButton.Effect = PressNOEffect;

						WriteFlash(Threshold_FLASH_DATA_ADDRESS,NewThreshold);
						WriteFlash(SA_MAX_FLASH_DATA_ADDRESS,SA_MaxValue);
						WriteFlash(SB_MAX_FLASH_DATA_ADDRESS,SB_MaxValue);
						WriteFlash(DACOUT1_FLASH_DATA_ADDRESS,DACOUT1);
						
							}
					}
				}
			}
}

/*ADCIN的数据调零*/

extern uint8_t S_Final_FinishFlag;
extern uint32_t 	S[4];
extern uint32_t 	S_Final;


/*获取四个ADC通道采样后，求平均的值*/
uint8_t  JudgeSvalue(uint32_t *S_Value)
{
	uint8_t flag=0x00;
		if(S_Value[0]>4000) flag |= 0x01;/*0000 0001*/
	else flag &= 0xFE;/*1111 1110*/
	
		if(S_Value[1]>4000) flag |= 0x02;/*0000 0010*/
	else flag &= 0xFD;/*1111 1101*/
	
		if(S_Value[2]>4000) flag |= 0x04;/*0000 0100*/
	else flag &= 0xFB;/*1111 1011*/
	
		if(S_Value[3]>4000) flag |= 0x08;/*0000 1000*/
	else flag &= 0xF7;/*1111 0111*/
	
	if(flag==0x0f)
		return 1;
	else 
		return 0;
}

/*ADCIN的数据调零*/

void ADCINcalibration(void) 
{
	CalibrateS1Value = 0;
	CalibrateS2Value = 0;

//	while(CalibrateS1Value>=1400 || CalibrateS1Value<=1000 || CalibrateS2Value>=1400 || CalibrateS2Value<=1000)
//	{
		if(sample_finish)
		{
			sample_finish = 0;

			Get_S1_Value(&CalibrateS1Value);					//定时ADC采样
//			if(CalibrateS1Value > 1400)
//			{
//				DACOUT1 = DACOUT1-1;
//			}
//			else if(CalibrateS1Value < 1000)
//			{
//				DACOUT1 = DACOUT1+1;
//			}
			DACOUT1 = CalibrateS1Value;
			
			if(DACOUT1>=4095)
				DACOUT1 = 4095;
			else if(DACOUT1<=0)
				DACOUT1 = 0;
			DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t)DACOUT1);
			DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);
			/*******************************************************/
			Get_S2_Value(&CalibrateS2Value);					//定时ADC采样
//			if(CalibrateS2Value > 1400)
//			{
//				DACOUT2 = DACOUT2-1;
//			}
//			else if(CalibrateS2Value < 1000)
//			{
//				DACOUT2 = DACOUT2+1;
//			}
			DACOUT2 = CalibrateS2Value;
			if(DACOUT2>=4095)
				DACOUT2 = 4095;
			else if(DACOUT2<=0)
				DACOUT2 = 0;
			DAC_SetChannel2Data(DAC_Align_12b_R,(uint16_t)DACOUT2);
			DAC_SoftwareTriggerCmd(DAC_Channel_2,ENABLE);
			
			/*限位*/
//			if((DACOUT1>=4095||DACOUT1<=0)&&(CalibrateS1Value>=1400||CalibrateS1Value<=1000))
//				break;
//			if((DACOUT2>=4095||DACOUT2<=0)&&(CalibrateS2Value>=1400||CalibrateS2Value<=1000))
//				break;
//		}
	}
}

