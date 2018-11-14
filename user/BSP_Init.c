#include "bsp_init.h"
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_dac.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_pwr.h"
int16_t adc_dma_tab[DMA_BUFFER_SIZE] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  

void RCC_Configuration(void)
{
	
	RCC_DeInit(); /*???RCC????????? */ 
	RCC_HSICmd(ENABLE); 
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY)== RESET);//??HSI?? 
	RCC_HCLKConfig(RCC_SYSCLK_Div1);   /*??AHB??(HCLK) RCC_SYSCLK_Div1��AHB?? = ???*/  
	RCC_PCLK2Config(RCC_HCLK_Div1);   /* ????AHB??(PCLK2)RCC_HCLK_Div1��APB2?? = HCLK*/     
	RCC_PCLK1Config(RCC_HCLK_Div1); /*????AHB??(PCLK1)RCC_HCLK_Div2��APB1?? = HCLK / 2*/      
	
	FLASH_SetLatency(FLASH_Latency_0);	//FLASH_Latency_2 2????
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);//???????

	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_6);/*??PLL????????,???8/2*16=64Mhz*/    
	RCC_PLLCmd(ENABLE); 	 /*??PLL */ 
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) ; /*?????RCC???(PLL?????)????*/    
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);  /*??????(SYSCLK) */  
	while(RCC_GetSYSCLKSource() != 0x08);     /*0x08:PLL?????? */	
	
	
}

/*******************************************
����GPIO��ʼ��
********************************************/
void SMG_GPIO_INIT(void)
{
	
	   GPIO_InitTypeDef GPIO_InitStructure;  
    //??GPIO??  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);  
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);  
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);  
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		
	
    GPIO_StructInit(&GPIO_InitStructure);  
	
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  //PB3,PB4,PA15����Ϊ��ͨIOʹ��
		GPIO_PinRemapConfig( GPIO_Remap_PD01 , ENABLE );

		//GPIOD2~4
    GPIO_InitStructure.GPIO_Pin = D4_Pin;  
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;             
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;                                 
    GPIO_Init(D4_GPIO_Port, &GPIO_InitStructure); 
		GPIO_InitStructure.GPIO_Pin = D3_Pin;
		GPIO_Init(D3_GPIO_Port, &GPIO_InitStructure);  
		GPIO_InitStructure.GPIO_Pin = D2_Pin;
		GPIO_Init(D2_GPIO_Port, &GPIO_InitStructure); 
		GPIO_InitStructure.GPIO_Pin = D1_Pin;
		GPIO_Init(D1_GPIO_Port, &GPIO_InitStructure); 
	  //GPIOD5~7                                                       
    GPIO_InitStructure.GPIO_Pin = D5_Pin;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;                             
    GPIO_Init(D5_GPIO_Port, &GPIO_InitStructure); 
		GPIO_InitStructure.GPIO_Pin = D6_Pin;  		
		GPIO_Init(D6_GPIO_Port, &GPIO_InitStructure);  
		GPIO_InitStructure.GPIO_Pin = D7_Pin;  
		GPIO_Init(D7_GPIO_Port, &GPIO_InitStructure);  
		//GPIOA                                                        
    GPIO_InitStructure.GPIO_Pin = D8_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 		
    GPIO_Init(D8_GPIO_Port, &GPIO_InitStructure);  
		GPIO_InitStructure.GPIO_Pin = D9_Pin;  
		GPIO_Init(D9_GPIO_Port, &GPIO_InitStructure);  

		//GPIOB
	  GPIO_InitStructure.GPIO_Pin = A_Pin;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;               
		GPIO_Init(A_GPIO_Port, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = B_Pin; 
		GPIO_Init(B_GPIO_Port, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = C_Pin; 
		GPIO_Init(C_GPIO_Port, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = D_Pin; 
		GPIO_Init(D_GPIO_Port, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = E_Pin; 
		GPIO_Init(E_GPIO_Port, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = F_Pin; 
		GPIO_Init(F_GPIO_Port, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = G_Pin; 
		GPIO_Init(G_GPIO_Port, &GPIO_InitStructure); 
		
		
		GPIO_WriteBit(D1_GPIO_Port, D1_Pin, Bit_SET);
		GPIO_WriteBit(D2_GPIO_Port, D2_Pin, Bit_SET);
		GPIO_WriteBit(D3_GPIO_Port, D3_Pin, Bit_SET);
		GPIO_WriteBit(D4_GPIO_Port, D4_Pin, Bit_SET);
		GPIO_WriteBit(D5_GPIO_Port, D5_Pin, Bit_SET);
		GPIO_WriteBit(D6_GPIO_Port, D6_Pin, Bit_SET);
		GPIO_WriteBit(D7_GPIO_Port, D7_Pin, Bit_SET);
		GPIO_WriteBit(D8_GPIO_Port, D8_Pin, Bit_SET);
		GPIO_WriteBit(D9_GPIO_Port, D9_Pin, Bit_SET);

		GPIO_WriteBit(A_GPIO_Port, A_Pin, Bit_SET);
		GPIO_WriteBit(B_GPIO_Port, B_Pin, Bit_SET);
		GPIO_WriteBit(C_GPIO_Port, C_Pin, Bit_SET);
		GPIO_WriteBit(D_GPIO_Port, D_Pin, Bit_SET);
		GPIO_WriteBit(E_GPIO_Port, E_Pin, Bit_SET);
		GPIO_WriteBit(F_GPIO_Port, F_Pin, Bit_SET);
		GPIO_WriteBit(G_GPIO_Port, G_Pin, Bit_SET);

}

void IO_GPIO_INIT(void)
{
		GPIO_InitTypeDef gpio_init_structure;  
    //??GPIO??  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);  
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);  
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);  
	
    GPIO_StructInit(&gpio_init_structure);  
	
		//OUT1_GPIO_Port,OUT2_GPIO_Port
    gpio_init_structure.GPIO_Pin = OUT1_Pin;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_Out_PP;             
    gpio_init_structure.GPIO_Speed = GPIO_Speed_50MHz;                               
		GPIO_Init(OUT1_GPIO_Port, &gpio_init_structure);

	  gpio_init_structure.GPIO_Pin = OUT2_Pin;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_Out_PP;             
    gpio_init_structure.GPIO_Speed = GPIO_Speed_50MHz;                               
		GPIO_Init(OUT2_GPIO_Port, &gpio_init_structure);

		gpio_init_structure.GPIO_Pin = GPIO_Pin_7;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_Out_PP;             
    gpio_init_structure.GPIO_Speed = GPIO_Speed_50MHz;                               
		GPIO_Init(GPIOA, &gpio_init_structure);

		//COMP_OUT1_GPIO_Port
    gpio_init_structure.GPIO_Pin = COMP_OUT1_Pin;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                                 
    gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;                                
		GPIO_Init(COMP_OUT1_GPIO_Port, &gpio_init_structure);
		
		
}


void TIM4_init(void)
{
	TIM_TimeBaseInitTypeDef 		timer_init_structure; 
	NVIC_InitTypeDef 						NVIC_InitStructure;
	TIM_OCInitTypeDef         	TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;                //ʹ��TIM3�ж�ͨ��  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 3;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 3;          
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;     
	NVIC_Init(&NVIC_InitStructure);
	
	/*TIM4*/
	TIM_DeInit(TIM4);                                               //��λTIM3
	TIM_TimeBaseStructInit(&timer_init_structure);                  //��ʼ��TIM�ṹ��  

	timer_init_structure.TIM_ClockDivision = TIM_CKD_DIV1;          //ϵͳʱ��,����Ƶ,24M  
	timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;      //���ϼ���ģʽ  
	timer_init_structure.TIM_Period = 5000;                          //ÿ300 uS����һ���ж�,??ADC  
	timer_init_structure.TIM_Prescaler = 0;                      //����ʱ�ӷ�Ƶ,f=1M,systick=1 uS  
	timer_init_structure.TIM_RepetitionCounter = 0x00;              //����0+1��update�¼������ж� 
	
			/*OCInit Channel 1 Configuration in PWM mode */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                                
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;         
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
		TIM_OCInitStructure.TIM_Pulse = 96;                                  	//PWM      96->1.5us                
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;                 
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;     
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset; 


		TIM_OC4Init(TIM4,&TIM_OCInitStructure);                                                 
		TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);	
	
	TIM_TimeBaseInit(TIM4, &timer_init_structure);  
	
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);                       //ʹ��TIM3�ж�
	TIM_ITConfig(TIM4, TIM_IT_CC4 , ENABLE);
	
	TIM_Cmd(TIM4, ENABLE);                                          //ʹ��TIM3
	
	

}


void TIM3_init(void)
{
	TIM_TimeBaseInitTypeDef timer_init_structure; 
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;                //ʹ��TIM3�ж�ͨ��  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 1;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 1;          
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;     
	NVIC_Init(&NVIC_InitStructure);
//	
	/*TIM3*/
	TIM_DeInit(TIM3);                                               //��λTIM3
	TIM_TimeBaseStructInit(&timer_init_structure);                  //��ʼ��TIM�ṹ��  

	timer_init_structure.TIM_ClockDivision = TIM_CKD_DIV1;          //ϵͳʱ��,����Ƶ,24M  
	timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;      //���ϼ���ģʽ  
	timer_init_structure.TIM_Period = 50;                          //ÿ300 uS����һ���ж�,??ADC  
	timer_init_structure.TIM_Prescaler = 23;                      //����ʱ�ӷ�Ƶ,f=1M,systick=1 uS  
	timer_init_structure.TIM_RepetitionCounter = 0x00;              //����0+1��update�¼������ж� 
	
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);							//ѡ��TIM1��timerΪ����Դ  
	TIM_TimeBaseInit(TIM3, &timer_init_structure);  
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);                       //ʹ��TIM3�ж�
	TIM_Cmd(TIM3, ENABLE);                                          //ʹ��TIM3

}


void TIM2_PWM_OUT_Init(void)
{
		TIM_OCInitTypeDef         TIM_OCInitStructure;
		
		GPIO_InitTypeDef GPIO_InitStructure;
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  
  
    GPIO_StructInit(&GPIO_InitStructure);  
    //GPIOA                                                         //PA-0~3 
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3; /*PWM ->PA0,ATT100->PA1,PWM1->PA2,PWM2->PA3*/
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
		GPIO_Init(GPIOA,&GPIO_InitStructure);
	
			/*OCInit Channel 1 Configuration in PWM mode */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                                
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;         
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
		TIM_OCInitStructure.TIM_Pulse = 96;                                  	//PWM      96->1.5us                
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;                 
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;     
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;  
		TIM_OC1Init(TIM2,&TIM_OCInitStructure);                                                 
		TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);	
		
		TIM_OCInitStructure.TIM_Pulse = 1600;                                   //ATT100	25us                
		TIM_OC2Init(TIM2,&TIM_OCInitStructure);                                                
		TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);	
	
		TIM_OCInitStructure.TIM_Pulse = 256;                                   //PWM1     256->4us      
		TIM_OC3Init(TIM2,&TIM_OCInitStructure);                                                 
		TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);	
	
		TIM_OCInitStructure.TIM_Pulse = 109;                                   //PWM2    109->1.7us
		TIM_OC4Init(TIM2,&TIM_OCInitStructure);                                                 
		TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);	
		
		TIM_CtrlPWMOutputs(TIM2,ENABLE);
		
}


void TIM2_init(void)  
{  
		TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
		NVIC_InitTypeDef nvic_init_structure;
	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

		/*MVIC*/
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
		nvic_init_structure.NVIC_IRQChannelPreemptionPriority = 3;
		nvic_init_structure.NVIC_IRQChannelPreemptionPriority= 3;
		nvic_init_structure.NVIC_IRQChannel = TIM2_IRQn;                //ʹ��TIM2�ж�ͨ��  
    nvic_init_structure.NVIC_IRQChannelCmd = ENABLE;                //ʹ��TIM2�ж�  
    NVIC_Init(&nvic_init_structure); 
		
		/*TIM2 Base Init*/
    TIM_DeInit(TIM2);                                               //��λTIM2  
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;          //ϵͳʱ��,����Ƶ,24M  
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;      //���ϼ���ģʽ  
		TIM_TimeBaseStructure.TIM_Period = 100;                          //ÿ100 uS����һ���ж�,??ADC  
		TIM_TimeBaseStructure.TIM_Prescaler = 23;                      //����ʱ�ӷ�Ƶ,f=1M,systick=1 uS  
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x00;              //����0+1��update�¼������ж� 
		
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
		TIM_ARRPreloadConfig(TIM2, ENABLE);
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);                      //ʹ��TIM2�ж�

		//TIM2_PWM_OUT_Init();

		TIM_Cmd(TIM2, ENABLE);
		
		
}  


void TIM1_Init(void)
{
	
    TIM_TimeBaseInitTypeDef timer_init_structure;  
		TIM_OCInitTypeDef timer_OCinit_structure; 
    //NVIC_InitTypeDef nvic_init_structure;  
  	//GPIO_InitTypeDef gpio_init_structure; 
	
	/* TIM1 ??? ---------------------------------------------------
   TIM1 ????(TIM1CLK) ??? APB2 ?? (PCLK2)    
    => TIM1CLK = PCLK2 = SystemCoreClock
   TIM1CLK = SystemCoreClock, Prescaler = 0, TIM1 counter clock = SystemCoreClock
   SystemCoreClock ?48 MHz 
   
   ??????? 4 ?PWM ???17.57 KHz:
     - TIM1_Period = (SystemCoreClock / 17570) - 1
   ??1??????? 50%
   ??2??????? 37.5%
   ??3??????? 25%
   ??4??????? 12.5%
   ????????????:
     - ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
	*/  
    /*???????,???????????????*/
  //TimerPeriod = (SystemCoreClock / 17570 ) - 1;
  //TimerPeriod = (SystemCoreClock / DEF_PWMFRE ) - 1;
  //TimerPeriod = (SystemCoreClock / DEF_PWMFRE);
  /* TIM1 ???? */
  
  
  /* Time ??????*/
  timer_init_structure.TIM_Prescaler = 47;
  timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;  /* Time ????????????*/
  timer_init_structure.TIM_Period = 25;
  timer_init_structure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &timer_init_structure);

  /* ??1,2,3,4?PWM ???? */
  timer_OCinit_structure.TIM_OCMode = TIM_OCMode_PWM1;
  timer_OCinit_structure.TIM_OutputState = TIM_OutputState_Enable ;//TIM_OutputState_Enable; //PWM?????
  timer_OCinit_structure.TIM_OutputNState = TIM_OutputNState_Disable ;//TIM_OutputNState_Enable; //??PWM?????
  timer_OCinit_structure.TIM_OCPolarity = TIM_OCPolarity_High;  //PWM 1?????
  timer_OCinit_structure.TIM_OCNPolarity = TIM_OCNPolarity_Low; //PWM?? 0?????
  timer_OCinit_structure.TIM_OCIdleState = TIM_OCIdleState_Set;
  timer_OCinit_structure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  timer_OCinit_structure.TIM_Pulse = 3; //?????
  TIM_OC1Init(TIM1, &timer_OCinit_structure);//????1??
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
	
	timer_OCinit_structure.TIM_Pulse = 3; //?????
  TIM_OC2Init(TIM1, &timer_OCinit_structure);//????1??
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	
	timer_OCinit_structure.TIM_Pulse = 3; //?????
  TIM_OC3Init(TIM1, &timer_OCinit_structure);//????1??
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);

  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);                      //ʹ��TIM1�ж�
	TIM_ARRPreloadConfig(TIM1,ENABLE);
	
  /* TIM1 ?????*/
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Enable);							//ѡ��TIM1��timerΪ����Դ  
	//TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC1Ref);							//ѡ��TIM1��timerΪ����Դ  
	//TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC2Ref);							//ѡ��TIM1��timerΪ����Դ  
	//TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC3Ref);							//ѡ��TIM1��timerΪ����Դ  
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);     //���update�¼��жϱ�־
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);//����ģʽMSM  
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	//TIM_SelectOnePulseMode(TIM1,TIM_OPMode_Single);
	
  TIM_Cmd(TIM1, ENABLE);
	
  /* TIM1 ????? */
}

void ADC1_GPIO_Config()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE );	  //ʹ��ADC1ͨ��ʱ��
		//PA1 ��Ϊģ��ͨ����������                         
	GPIO_InitStructure.GPIO_Pin = ADCIN_1_Pin;//CH->1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(ADCIN_1_GPIO_Port, &GPIO_InitStructure);	
	
//	GPIO_InitStructure.GPIO_Pin = ADCIN_2_Pin;//CH->2
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
//	GPIO_Init(ADCIN_2_GPIO_Port, &GPIO_InitStructure);	
//	
//	GPIO_InitStructure.GPIO_Pin = ADCIN_3_Pin;//CH->7
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
//	GPIO_Init(ADCIN_3_GPIO_Port, &GPIO_InitStructure);	
//	
//	GPIO_InitStructure.GPIO_Pin = ADCIN_4_Pin;//CH->6
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
//	GPIO_Init(ADCIN_4_GPIO_Port, &GPIO_InitStructure);
}

void ADC1_DMA1_Init()
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_ADDRESS;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&adc_dma_tab;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = DMA_BUFFER_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	DMA_ClearFlag(DMA1_FLAG_TC1);
  DMA_ClearITPendingBit(DMA1_IT_TC1);
  
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);  //DMA��������ж�
	
	  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
	
}

void ADC1_Init(void)
{
  ADC_InitTypeDef  ADC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1	, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
 
	RCC_ADCCLKConfig(RCC_PCLK2_Div2);   //����ADC��Ƶ����6 64M/8=8,ADC���ʱ�䲻�ܳ���14M
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular configuration */ 
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_7Cycles5);
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_7Cycles5);
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_7Cycles5);

   ADC_DMACmd(ADC1 , ENABLE);
   ADC_ExternalTrigConvCmd(ADC1, ENABLE);
  
   ADC_Cmd(ADC1 , ENABLE);   
   ADC_ResetCalibration(ADC1);
   while(ADC_GetResetCalibrationStatus(ADC1)){};
   ADC_StartCalibration(ADC1);
   while(ADC_GetCalibrationStatus(ADC1)){};
}

void ADC1_Configuration(void)
{
    ADC1_GPIO_Config();
    ADC1_DMA1_Init();
    ADC1_Init();
}


//void S_ADCChannel_Init(void)
//{
//  ADC_InitTypeDef  ADC_InitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1	, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
// 
//	RCC_ADCCLKConfig(RCC_PCLK2_Div2);   //����ADC��Ƶ����6 64M/8=8,ADC���ʱ�䲻�ܳ���14M
//  
//  /* ADC1 configuration ------------------------------------------------------*/
//  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
//  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
//  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
//  //ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
//  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//  ADC_InitStructure.ADC_NbrOfChannel = 2;
//  ADC_Init(ADC1, &ADC_InitStructure);

//  /* ADC1 regular configuration */ 
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_7Cycles5);
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_7Cycles5);
////  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 3, ADC_SampleTime_7Cycles5);
////	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 4, ADC_SampleTime_7Cycles5);

//   ADC_DMACmd(ADC1 , ENABLE);
//   ADC_ExternalTrigConvCmd(ADC1, ENABLE);
//  
//   ADC_Cmd(ADC1 , ENABLE);   
//   ADC_ResetCalibration(ADC1);
//   while(ADC_GetResetCalibrationStatus(ADC1)){};
//   ADC_StartCalibration(ADC1);
//   while(ADC_GetCalibrationStatus(ADC1)){};
//}

/*DAC ����*/
/*
DAC-1
DAC-2
*/
void DAC_GPIO_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;  
    //??GPIO??  
    RCC_AHBPeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
			
		GPIO_InitStructure.GPIO_Pin = DACOUT1_Pin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������                
		GPIO_Init(DACOUT1_GPIO_Port, &GPIO_InitStructure);
	
//		GPIO_InitStructure.GPIO_Pin = DACOUT2_Pin;
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������                
//		GPIO_Init(DACOUT2_GPIO_Port, &GPIO_InitStructure);
}

void DAC_OUT_Init(void)
{
		DAC_InitTypeDef DAC_InitStructure;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
		DAC_GPIO_Init();

		DAC_InitStructure.DAC_Trigger = DAC_Trigger_Software;  //�������DAת��
		DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;//����������
		DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
		DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable ;  //ʹ���������
		DAC_Init(DAC_Channel_1,&DAC_InitStructure);    //��ʼ�� DAC ͨ�� 1
		DAC_Init(DAC_Channel_2,&DAC_InitStructure);    //��ʼ�� DAC ͨ�� 1
	
		
		DAC_Cmd(DAC_Channel_1,ENABLE); 
		DAC_Cmd(DAC_Channel_2,ENABLE);
}

void DAC_Configuration(void)
{
	DAC_GPIO_Init();
	
	DAC_OUT_Init();
}

/**
  * @brief Configures EXTI Lines.
  * @param None
  * @retval None
  */
void EXTI_Configuration(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Configure EXTI Line16(PVD Output) to generate an interrupt on rising and
     falling edges */
  EXTI_ClearITPendingBit(EXTI_Line16); 
  EXTI_InitStructure.EXTI_Line = EXTI_Line16;// PVD���ӵ��ж�16���� 
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//ʹ���ж�ģʽ
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//��ѹ�½����趨��ֵʱ�����ж�
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;// ʹ���ж�
  EXTI_Init(&EXTI_InitStructure);// ��ʼ��
}

/**
  * @brief Configures NVIC and Vector Table base location.
  * @param None
  * @retval None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);//�ж����ȼ�����
  
  /* Enable the PVD Interrupt */ //����PVD�ж�
  NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void PVD_init(void)
{
	EXTI_Configuration();
	
	NVIC_Configuration();
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	
	PWR_PVDLevelConfig(PWR_PVDLevel_2V7); /*����PVD��ѹ���*/
	PWR_PVDCmd(ENABLE);
}

