#define _IN_LASER
#include "Marlin.h"
#include "HMIScreen.h"
#include "HMI_Define.h"
#include "stepper.h"
#include "Udisk.h"
#include "stopwatch.h"
#include "temperature.h"
#include "mesh_bed_leveling.h"
#include "configuration_store.h"
#include "SystemControl.h"
#include "Laser.h"
#include "LaserPowerTable.h"

#include "stm32f10x_conf.h"


//中断入口需要查看.S  文件



#if(HAVE_LASER_EEPROM == 1)
enum
{
	AT24C02 = 0,
	AT24C04,
	AT24C08,
	AT24C16
};

#define EEPROM_TYPE	AT24C02

#if (EEPROM_TYPE == AT24C02)
#define PAGE_SIZE	8
#else
#define PAGE_SIZE	16
#endif

#define AT24C_ADDR	0x50


#define SCL_H()	LASER_I2C_SCL_PORT->BSRR = LASER_I2C_SCL_PIN
#define SCL_L()	LASER_I2C_SCL_PORT->BRR = LASER_I2C_SCL_PIN

#define SDA_H()	LASER_I2C_SDA_PORT->BSRR = LASER_I2C_SDA_PIN
#define SDA_L()	LASER_I2C_SDA_PORT->BRR = LASER_I2C_SDA_PIN

#define RDSDA (LASER_I2C_SDA_PORT->IDR & LASER_I2C_SDA_PIN)

#define CMD_WRITE	(uint8_t)(AT24C_ADDR << 1)
#define CMD_READ	(uint8_t)((AT24C_ADDR << 1) | 1)


static void delay_us(unsigned short t)
{
	unsigned long i;
	while(t--)
	{
		for(i=0;i<220;i++);
	}
}


static void I2CStart()
{
	SCL_L();
	delay_us(5);
	SDA_H();
	delay_us(5);
	SCL_H();
	delay_us(5);
	SDA_L();
	delay_us(5);
	SCL_L();
	delay_us(5);
}

static void I2CStop()
{
	SDA_L();
	delay_us(5);

	SCL_H();
	delay_us(5);
	SDA_H();
	delay_us(2);
}

static void I2CRestart()
{
	SDA_H();
	delay_us(5);
	SCL_H();
	delay_us(5);
	SDA_L();
	delay_us(5);
	SCL_L();
	delay_us(5);
}

static U8 WaitAck(void)
{
	U8 Result = 0;

	SDA_H();
	delay_us(5);
	SCL_H();
	delay_us(5);
	if(RDSDA)
		Result = 1;
	else
		Result = 0;
	SCL_L();
	delay_us(5);

	SDA_L();
	delay_us(3);
	return Result;
}

static U8 I2CWriteByte(U8 Data)
{
	U8 i;
	for(i=0;i<8;i++)
	{
		if(Data & 0x80)
			SDA_H();
		else
			SDA_L();
		delay_us(5);
		SCL_H();
		delay_us(5);
		SCL_L();
		Data = Data << 1;
	}
	if(WaitAck() == 0)
		return 0;
	else
		return 1;
}

static void I2CReadByte(U8 *pBuff, U8 Len)
{
	U8 i;
	U8 Result;

	while(Len--)
	{
		SDA_H();
		
		for(i=0;i<8;i++)
		{
			SCL_H();
			delay_us(5);
				
			if(RDSDA)
				Result = (Result << 1) | 0x1;
			else
				Result = (Result << 1) & 0xfe;
			
			SCL_L();
			delay_us(5);
			
		}
		if(Len == 0)
		{
			SDA_H();
			delay_us(5);
			SCL_H();
			delay_us(5);
			SCL_L();
			delay_us(5);
		}
		else
		{
			SDA_L();
			delay_us(5);
			SCL_H();
			delay_us(5);
			SCL_L();
			delay_us(5);
		}
		*pBuff++ = Result;
	}
	
}


/********************************************
写入EEPROM	数据
pData  :  数据写入区指针
Len  :	写入长度
返回值:  true  成功，false	失败
********************************************/
bool WriteData(uint16_t Address, uint8_t *pData, uint8_t Len)
{
	I2CStart();
	if(I2CWriteByte(CMD_WRITE) != 0)
	{
		I2CStop();
		return false;
	}
	#if (EEPROM_TYPE >= AT24C04)
	if(I2CWriteByte(Address >> 8) != 0)
	{
		I2CStop();
		return false;
	}
	#endif
	if(I2CWriteByte(Address) != 0)
	{
		I2CStop();
		return false;
	}
	for(int i=0;i<Len;i++)
	{
		if(I2CWriteByte(*pData++) != 0)
		{
			I2CStop();
			return false;
		}
	}
	I2CStop();
	return true;
}

/********************************************
读取EEPROM  数据
pData  :  数据接收区指针
Len  :  读取长度
返回值:  true  成功，false  失败
********************************************/
bool ReadData(uint16_t Address, uint8_t *pData, uint8_t Len)
{
	I2CStart();
	if(I2CWriteByte(CMD_WRITE) != 0)
	{
		I2CStop();
		return false;
	}
	#if (EEPROM_TYPE >= AT24C04)
	if(I2CWriteByte(Address >> 8) != 0)
	{
		I2CStop();
		return false;
	}
	#endif
	if(I2CWriteByte(Address) != 0)
	{
		I2CStop();
		return false;
	}
	I2CRestart();
	if(I2CWriteByte(CMD_READ) != 0)
	{
		I2CStop();
		return false;
	}
	I2CReadByte(pData++, Len);
	I2CStop();

	return true;
}



/********************************************
读取激光模组焦点高度
ZValue  :  保存数据的变量指针
返回值:  true  读取成功，false  读取失败
********************************************/
char LoadLaserFocusZValue(float *ZValue)
{
	int i = 0;
	uint8_t *p;
	uint8_t WBuff[PAGE_SIZE];
	p = (uint8_t*)&ZValue;

	if(ReadData(0, WBuff, PAGE_SIZE) == false)
		return (char)-1;
	p = WBuff;
	for(i=0;i<4;i++)
	{
		if((p[i] + p[i+4]) != 0xff)
			return (char)-2;
	}
	*ZValue = *((float*)p);
	return 0;
}

/********************************************
保存激光模组焦点高度
ZValue  :  高度值
返回值:  true  保存成功，false  保存失败
********************************************/
bool SaveLaserFocusZValue(float ZValue)
{
	int i = 0;
	uint8_t *p;
	uint8_t WBuff[PAGE_SIZE];
	uint8_t RBuff[PAGE_SIZE];
	p = (uint8_t*)&ZValue;
	
	WBuff[i++] = p[0];
	WBuff[i++] = p[1];
	WBuff[i++] = p[2];
	WBuff[i++] = p[3];

	WBuff[i++] = ~p[0];
	WBuff[i++] = ~p[1];
	WBuff[i++] = ~p[2];
	WBuff[i++] = ~p[3];
	WriteData(0, WBuff, PAGE_SIZE);
	delay_us(300);
	ReadData(0, RBuff, PAGE_SIZE);
	for(i=0;i<PAGE_SIZE;i++)
	{
		if(WBuff[i] != RBuff[i])
			return false;
	}
	return true;
}

/********************************************
数据测试
Page0  :  写入单数
Page1  :  写入双数
********************************************/
bool EEPRomTest()
{
	uint8_t WBuff[PAGE_SIZE];
	uint8_t RBuff[PAGE_SIZE];
	uint8_t i;
	for(i=0;i<PAGE_SIZE;i++)
	{
		WBuff[i] = i * 2 + 1;
		RBuff[i] = 0;
	}
	WriteData(0, WBuff, PAGE_SIZE);
	ReadData(0, RBuff, PAGE_SIZE);
	for(i=0;i<PAGE_SIZE;i++)
	{
		if(WBuff[i] != RBuff[i])
			return false;
	}
	return true;
}


#endif

/********************************************
初始化激光
********************************************/
void LaserInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	#ifdef H_FAN2_PORT
	H_FAN2_PORT->BSRR = H_FAN2_PIN;
	#endif
	
	#if(HAVE_LASER_EEPROM == 1)
	{
		RCC_APB1PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
		
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
		
		GPIO_InitStruct.GPIO_Pin = LASER_I2C_SDA_PIN;
		GPIO_Init(LASER_I2C_SDA_PORT, &GPIO_InitStruct);

		GPIO_InitStruct.GPIO_Pin = LASER_I2C_SCL_PIN;
		GPIO_Init(LASER_I2C_SCL_PORT, &GPIO_InitStruct);

		float tmpLaserZValue;
		char Result;
		Result = LoadLaserFocusZValue(&tmpLaserZValue);
		//高度读取失败，存储器损坏
		if(Result == (char)-1)
		{
			
		}
		//存储器未初始化
		else if(Result == (char)-2)
		{
			LaserZValue = 35;
			SaveLaserFocusZValue(LaserZValue);
		}
		//高度读取成功
		else
		{
			//与主控存储高度不一致
			if(tmpLaserZValue != LaserZValue)
			{
				LaserZValue = tmpLaserZValue;
				Config_StoreSettings();
			}
		}
	}
	#endif

	//旧的板子需要把与非门控制脚打开
	#if(SNAP_OLD_BASE == 1)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIOB->BRR = GPIO_Pin_1;
	#endif

	#if((LASER_USE_DAC == 1) && (SNAP_OLD_BASE == 0))
	
	//DAC  方式调光
	DAC_InitTypeDef DAC_InitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);           
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	//关闭激光
	LASER_PWM_PORT->BSRR = LASER_PWM_PIN;
	DAC_SetChannel1Data( DAC_Align_12b_R, 0);

	//激光DAC  脚
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStruct.GPIO_Pin = LASER_DAC_PIN;
	GPIO_Init(LASER_DAC_PORT, &GPIO_InitStruct);

	//激光PWM  开关脚
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = LASER_PWM_PIN;
	GPIO_Init(LASER_PWM_PORT, &GPIO_InitStruct);
	

	DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits11_0;
	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
	DAC_InitStruct.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_Init(DAC_Channel_1, &DAC_InitStruct);
	DAC_Cmd(DAC_Channel_1, ENABLE);
	#else
	//PWM  方式
	TIM_DeInit(TIM1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_BDTRInitTypeDef TIM_BDTRInitStruct;

	//100HZ  ，255  Level
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = LASER_PIN;
	GPIO_Init(LASER_PORT, &GPIO_InitStruct);

	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 1862;//959;//1862;//1344;
	TIM_TimeBaseInitStruct.TIM_Period = 255;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);
	TIM_ARRPreloadConfig(TIM1, DISABLE);
	TIM_SelectOnePulseMode(TIM1, TIM_OPMode_Repetitive);

	TIM_BDTRStructInit(&TIM_BDTRInitStruct);
	TIM_BDTRInitStruct.TIM_OSSRState = ENABLE;
	TIM_BDTRInitStruct.TIM_OSSIState = ENABLE;
	TIM_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStruct.TIM_DeadTime = 0;
	TIM_BDTRInitStruct.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStruct);
	
	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OC2Init(TIM1, &TIM_OCInitStruct);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_Cmd(TIM1, ENABLE);
	#endif

	FileFilter = FIL_CNC;
	MachineSelected = MA_LASER;
	#if(HAVE_HMI == 1)
	HMI_ChangePage(PAGE_LASER);
	#endif
	LaserFocusPwm = 8;

	// 关闭门外罩 检测
	DisableDoorCheck();

	//DAC_SetChannel1Data( DAC_Align_12b_R, 0);
	//Add by Paladin:Update Acc according the machine
	gcode_M201();

	//自检
	SystemSelfCheck();
	//开启电源
	if((SysFaultFlag & FAULT_FLAG_LOAD & FAULT_FLAG_BED & FAULT_FLAG_HEATER0))
	{
		MachineMode = WORK_MODE_IDLE;
	}
}


/********************************************
逆始化激光
********************************************/
void LaserDeinit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_StructInit(&GPIO_InitStruct);

	#if(HAVE_LASER_EEPROM == 1)
	{
		RCC_APB1PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
		
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		
		GPIO_InitStruct.GPIO_Pin = LASER_I2C_SDA_PIN;
		GPIO_Init(LASER_I2C_SDA_PORT, &GPIO_InitStruct);

		GPIO_InitStruct.GPIO_Pin = LASER_I2C_SCL_PIN;
		GPIO_Init(LASER_I2C_SCL_PORT, &GPIO_InitStruct);

		
	}
	#endif

	#if(LASER_USE_DAC == 1)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, DISABLE);           
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	#else

	TIM_DeInit(TIM1);

	#ifdef LASER_PIN
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = LASER_PIN;
	GPIO_Init(LASER_PORT, &GPIO_InitStruct);
	#endif
	
	//100HZ  ，255  Level
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, DISABLE);
	#endif
}

/*****************************************************
激光变量初始化
*****************************************************/
void LaserVariableInit(void)
{
	CRITICAL_SECTION_START;
	ClearCommand();
	current_block = NULL;
	block_buffer_tail = block_buffer_head = 0;
	FlushCommand();
	CRITICAL_SECTION_END;
}

/*****************************************************
激光处理进程
*****************************************************/
void LaserProcess(void)
{
	LaserInit();

	while(MachineMode == WORK_MODE_RUNNING)
	{
		UDiskIdleProceess();
		if (commands_in_queue < (BUFSIZE - 4)) get_available_commands();

		//Modify by Paladin:系统不要求停止，并且队列中有指令待执行
		
		if ((commands_in_queue > 0) && (IsStoppingPrint() == false)) 
		{
			//Paladin
			process_next_command();
			DispatchComandIndex();
			ShieldLedOn();
			ShieldFanOn();
		}
		
		
		//Paladin
		FinishPrintProcess();
		PauseProcess();
		checkHitEndstops();
		#if(HAVE_ANDROID == 1)
		AndroidScreenPowerManager();
		#endif
		idle();
	}
	LaserDeinit();
}

/*****************************************************
//Linear interpolation 线性插值
*****************************************************/
uint16_t LaserPowerLERP(float v)
{
    uint16_t ret = 0 ;
    
    if( v <= 0 )
    {
        return LaserPowerTable[0];
    }
    else if( v + 1 >= POWER_TABLE_SIZE )
    {
        return LaserPowerTable[POWER_TABLE_SIZE - 1];//数据范围不对
    }
    int int_v = int(v);//整数部分
    float float_v = v - int_v;//x小数部分
    ret = (uint16_t)(LaserPowerTable[int_v] + (LaserPowerTable[int_v + 1] - LaserPowerTable[int_v]) * float_v);
    return ret;
}
