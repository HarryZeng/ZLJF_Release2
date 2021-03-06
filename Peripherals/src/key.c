///**
//  ********************************  STM32F0x1  *********************************
//  * @文件名     ： key.c
//  * @作者       ： HarryZeng
//  * @库版本     ： V1.5.0
//  * @文件版本   ： V1.0.0
//  * @日期       ： 2017年05月11日
//  * @摘要       ： 数据处理
//  ******************************************************************************/
///*----------------------------------------------------------------------------
//  更新日志:
//  2017-05-11 V1.0.0:初始版本
//  ----------------------------------------------------------------------------*/
///* 包含的头文件 --------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "project.h"
#include "key.h"

#define timer_period 5 //ms
#define shortKEY 	100
#define middleKEY	1000
#define longKEY		3000

//uint32_t key_counter;
ButtonStruct SetButton;
ButtonStruct ModeButton;
ButtonStruct UpButton;
ButtonStruct DownButton;

Button_Status ReadButtonStatus(ButtonStruct *Button);
void PressCallback(ButtonStruct* Button);

void Button_Init(void)
{
	GPIO_InitTypeDef gpio_init_structure;  
//	SetButton.ButtonPort = BUTTON_SET_GPIO_Port;
//	SetButton.ButtonPin = BUTTON_SET_Pin;
//	SetButton.Mode = ShortAndLong;
//	SetButton.PressCounter = 0;
//	SetButton.PressTimer = 0;
//	SetButton.Status = Release;
//	SetButton.WorkIn = WorkHigh;
//	SetButton.Effect = PressNOEffect;
//	SetButton.ShortTime = 1*timer_period;
//	SetButton.LongTime = 20*timer_period;   
 
	ModeButton.ButtonPort = BUTTON_MODE_GPIO_Port;
	ModeButton.ButtonPin = BUTTON_MODE_Pin;
	ModeButton.Mode = ShortAndLong;
	ModeButton.PressCounter = 0;
	ModeButton.PressTimer = 0;
	ModeButton.Status = Release;
	ModeButton.WorkIn = WorkLow;
	ModeButton.Effect = PressNOEffect;
	ModeButton.ShortTime = 1*timer_period;  
	ModeButton.LongTime = 45*timer_period;		
	
	UpButton.ButtonPort = BUTTON_UP_GPIO_Port;
	UpButton.ButtonPin = BUTTON_UP_Pin;
	UpButton.Mode = ShortAndLong;
	UpButton.PressCounter = 0;
	UpButton.PressTimer = 0;
	UpButton.Status = Release;
	UpButton.WorkIn = WorkLow;
	UpButton.Effect = PressNOEffect;
	UpButton.ShortTime = 1*timer_period;
	UpButton.LongTime = 45*timer_period;
	
	DownButton.ButtonPort = BUTTON_DOWN_GPIO_Port;
	DownButton.ButtonPin = BUTTON_DOWN_Pin;
	DownButton.Mode = ShortAndLong;
	DownButton.PressCounter = 0;
	DownButton.PressTimer = 0;
	DownButton.Status = Release;
	DownButton.WorkIn = WorkLow;
	DownButton.Effect = PressNOEffect;
	DownButton.ShortTime = 1*timer_period;
	DownButton.LongTime = 45*timer_period;
	
  gpio_init_structure.GPIO_Mode = GPIO_Mode_IPU;                                 
  gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;      
	
	gpio_init_structure.GPIO_Pin = UpButton.ButtonPin; 
	GPIO_Init(UpButton.ButtonPort, &gpio_init_structure);
	
	gpio_init_structure.GPIO_Pin = DownButton.ButtonPin; 
	GPIO_Init(DownButton.ButtonPort, &gpio_init_structure);
	
	gpio_init_structure.GPIO_Pin = ModeButton.ButtonPin; 
	GPIO_Init(ModeButton.ButtonPort, &gpio_init_structure);
	
//	gpio_init_structure.GPIO_Pin = SetButton.ButtonPin; 
//	GPIO_Init(SetButton.ButtonPort, &gpio_init_structure);
//	
}

void PressCallback(ButtonStruct* Button)
{	
	if(Button->Mode == ShortAndLong)		//长短模式，根据时间判断长按短按
		{
			if(Button->PressTimer>=Button->LongTime) //按键按下时长对比
			{
				Button->Effect = PressLong;  
				if(Button->Status==Release)
				{
					Button->PressTimer = 0; 
					Button->PressCounter++;				//记录按键被按下次数
				}
			}
			else if((Button->PressTimer>=Button->ShortTime)&&(Button->PressTimer<Button->LongTime)) 
			{
				Button->Effect = PressShort;       
				Button->PressTimer = 0; 
				Button->PressCounter++;				//记录按键被按下次数
			}
		}
		else 
		{	
				if(Button->Mode == Shortpress)
				{
						Button->Effect = PressShort;       //短按模式
						Button->PressTimer = 0; 
						Button->PressCounter++;				//记录按键被按下次数
				}
				else if(Button->Mode == Longpress)		//长按模式
				{
					if(Button->PressTimer>=Button->LongTime) //判断长按时间
					{
						Button->Effect = PressLong;       
						Button->PressTimer = 0; 
						Button->PressCounter++;				//记录按键被按下次数
					} 
				}
		}			 
}


/*按键扫描*/  
void Key_Scan(void)  
{
			/*MODE BUTTON*/
		if(ReadButtonStatus(&ModeButton) == Press )     //UP键按下，一直计算按下时长,根据定时器周期性，计算时长									
    {
				ModeButton.PressTimer++;
				ModeButton.Status = Press;
    } 
		else if(ReadButtonStatus(&ModeButton) == Release)
		{
			ModeButton.Status = Release;
		}
		if(ModeButton.PressTimer > ModeButton.ShortTime && ModeButton.PressTimer < ModeButton.LongTime && ReadButtonStatus(&ModeButton) == Release)  //按键释放之后再判断按下的时间长度
		{
			PressCallback(&ModeButton);
		}
		else if(ModeButton.PressTimer>ModeButton.LongTime && ReadButtonStatus(&ModeButton) == Press) /*长安，还在按下状态*/
		{
			PressCallback(&ModeButton);
		}
		else if(ModeButton.PressTimer>ModeButton.LongTime && ReadButtonStatus(&ModeButton) == Release) /*长安，释放了*/
		{
			ModeButton.PressTimer = 0;
		}
		
		
		/*SET BUTTON*/
		if(ReadButtonStatus(&SetButton) == Press )     //UP键按下，一直计算按下时长,根据定时器周期性，计算时长									
    {
				SetButton.PressTimer++;
				SetButton.Status = Press;
    } 
		else if(ReadButtonStatus(&SetButton) == Release)
		{
			SetButton.Status = Release;
		}
		if(SetButton.PressTimer>SetButton.ShortTime && SetButton.PressTimer < SetButton.LongTime && ReadButtonStatus(&SetButton) == Release)  //按键释放之后再判断按下的时间长度
		{
			PressCallback(&SetButton);
		}
		else if(SetButton.PressTimer>SetButton.LongTime && ReadButtonStatus(&SetButton)== Press) /*长安，还在按下状态*/
		{
			PressCallback(&SetButton);
		}
		else if(SetButton.PressTimer>SetButton.LongTime && ReadButtonStatus(&SetButton) == Release) /*长安，释放了*/
		{
			SetButton.PressTimer = 0;
		}
		
		/*UP BUTTON*/
		if(ReadButtonStatus(&UpButton) == Press )     //UP键按下，一直计算按下时长,根据定时器周期性，计算时长									
    {
				UpButton.PressTimer++;
				UpButton.Status = Press;
    } 
		else if(ReadButtonStatus(&UpButton) == Release)
		{
			UpButton.Status = Release;
		}
		if(UpButton.PressTimer>UpButton.ShortTime && UpButton.PressTimer < UpButton.LongTime &&ReadButtonStatus(&UpButton) == Release)  //按键释放之后再判断按下的时间长度
		{
			PressCallback(&UpButton);
		}
		else if(UpButton.PressTimer>UpButton.LongTime &&  ReadButtonStatus(&UpButton) == Press) /*长安，还在按下状态*/
		{
			PressCallback(&UpButton);
		}
		else if(UpButton.PressTimer>UpButton.LongTime && ReadButtonStatus(&UpButton) == Release) /*长安，释放了*/
		{
			UpButton.PressTimer = 0;
		}
		
		/*DOWN BUTTON*/
		if(ReadButtonStatus(&DownButton) == Press )     //DOWN键按下，一直计算按下时长,根据定时器周期性，计算时长									
    {
				DownButton.PressTimer++;
				DownButton.Status = Press;
    } 
		else if(ReadButtonStatus(&DownButton) == Release)
		{
			DownButton.Status = Release;
		}
		if(DownButton.PressTimer>DownButton.ShortTime && DownButton.PressTimer<DownButton.LongTime &&  ReadButtonStatus(&DownButton) == Release)  //按键释放之后再判断按下的时间长度
		{
			PressCallback(&DownButton);
		}
		else if(DownButton.PressTimer>DownButton.LongTime && ReadButtonStatus(&DownButton)== Press) /*长安，还在按下状态*/
		{
			PressCallback(&DownButton);
		}
		else if(DownButton.PressTimer>DownButton.LongTime && ReadButtonStatus(&DownButton) == Release) /*长安，释放了*/
		{
			DownButton.PressTimer = 0;
		}
}  

Button_Status ReadButtonStatus(ButtonStruct *Button)
{
	uint8_t PinState=0;
	PinState = GPIO_ReadInputDataBit(Button->ButtonPort ,Button->ButtonPin);
	if(Button->WorkIn == WorkLow )
	{
		if(PinState==(uint8_t)Bit_RESET)
			return Press;
		else
			return Release;
	}
	else if(Button->WorkIn == WorkHigh )
	{
		if(PinState==(uint8_t)Bit_SET)
			return Press;
		else
			return Release;
	}
}
