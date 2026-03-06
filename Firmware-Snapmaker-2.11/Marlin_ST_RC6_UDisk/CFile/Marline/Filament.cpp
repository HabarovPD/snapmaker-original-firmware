#define _IN_FILAMENT
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

#include "stm32f10x_conf.h"




#if(HAVE_FALIMENT_SENSOR == 1)
#if(SNAP_OLD_BASE == 0)
#define THIS_EXTI_LINE	EXTI_Line3
#define THIS_IRQN		EXTI3_IRQn
#define SOURCE_PORT		GPIO_PortSourceGPIOG
#define SOURCE_PIN		GPIO_PinSource3
#else


#endif

//差值缓存
int16_t delX, delY;
//错误计数
static volatile uint8_t FaultCounter;
//错误次数累计，测试
static uint16_t ErrorCount;

//中断入口需要查看.S  文件
const uint32_t PortTable[] = 
{
	(uint32_t)GPIOA,
	(uint32_t)GPIOB,
	(uint32_t)GPIOC,
	(uint32_t)GPIOD,
	(uint32_t)GPIOE,
	(uint32_t)GPIOF
};

const uint32_t PortRccTable[] = 
{
	RCC_APB2Periph_GPIOA,
	RCC_APB2Periph_GPIOB,
	RCC_APB2Periph_GPIOC,
	RCC_APB2Periph_GPIOD,
	RCC_APB2Periph_GPIOE,
	RCC_APB2Periph_GPIOF
};


#define THIS_I2C	I2C2
#define PAT9125_I2C_ADDR  0x75  //ID=LO
//#define PAT9125_I2C_ADDR  0x73  //ID=HI
//#define PAT9125_I2C_ADDR  0x79  //ID=NC

#define PAT9125_XRES      0
#define PAT9125_YRES      240

#define REG_PID0	0
#define REG_PID1	1
#define REG_MOTION_STATUS	2
#define REG_DELTA_X_LOW	3
#define REG_DELTA_Y_LOW	4
#define REG_OPERATION_MODE	5
#define REG_CONFIG	6
#define REG_WIRTE_PROTECT	9
#define REG_SLEEP1	0x0A
#define REG_SLEEP2	0x0B
#define REG_RES_X	0x0D
#define REG_RES_Y	0x0E
#define REG_DELTA_XY_HIGH	0x12
#define REG_SHUTTER	0x14
#define REG_FRAME_AVG	0x17
#define REG_ORIENTATION	0x19

#define SCL_H()	FILAMNET_I2C_SCL_PORT->BSRR = FILAMNET_I2C_SCL_PIN
#define SCL_L()	FILAMNET_I2C_SCL_PORT->BRR = FILAMNET_I2C_SCL_PIN

#define SDA_H()	FILAMNET_I2C_SDA_PORT->BSRR = FILAMNET_I2C_SDA_PIN
#define SDA_L()	FILAMNET_I2C_SDA_PORT->BRR = FILAMNET_I2C_SDA_PIN

#define RDSDA (FILAMNET_I2C_SDA_PORT->IDR & FILAMNET_I2C_SDA_PIN)

#define CMD_WRITE	(uint8_t)(PAT9125_I2C_ADDR << 1)
#define CMD_READ	(uint8_t)((PAT9125_I2C_ADDR << 1) | 1)

static const uint8_t PATInitRegs[][2] = {
	{ 0x09, 0x5A },     //Disables write-protect.
	{ 0x0D, 0xB5 },     //Sets X-axis resolution (necessary value here will depend on physical design and application).
	{ 0x0E, 0xFF },     //Sets Y-axis resolution (necessary value here will depend on physical design and application).
	{ 0x19, 0x04 },     //Sets 12-bit X/Y data format.
	//{ 0x4B, 0x04 },     //Needed for when using low-voltage rail (1.7-1.9V): Power saving configuration.
};

#pragma optimize = none
static void delay_us(unsigned short t)
{
	unsigned long i;
	while(t--)
	{
		for(i=0;i<20;i++);
	}
}

static void I2CStart()
{
	SCL_L();
	delay_us(2);
	SDA_H();
	delay_us(2);
	SCL_H();
	delay_us(2);
	SDA_L();
	delay_us(2);
	SCL_L();
	delay_us(2);
}

static void I2CStop()
{
	SDA_L();
	delay_us(2);

	SCL_H();
	delay_us(2);
	SDA_H();
	delay_us(2);
	SDA_L();
	delay_us(2);
	SCL_L();
}

static void I2CRestart()
{
	SDA_H();
	delay_us(2);
	SCL_H();
	delay_us(2);
	SDA_L();
	delay_us(2);
	SCL_L();
	delay_us(2);
}

static U8 WaitAck(void)
{
	U8 Result = 0;

	SDA_H();

	delay_us(2);
	SCL_H();
	delay_us(2);
	if(RDSDA)
		Result = 1;
	else
		Result = 0;
	SCL_L();
	delay_us(2);

	SDA_L();
	delay_us(2);

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
		delay_us(2);
		SCL_H();
		delay_us(2);
		SCL_L();
		delay_us(2);
		Data = Data << 1;
	}
	delay_us(2);
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
		delay_us(3);

		for(i=0;i<8;i++)
		{
			SCL_H();
			delay_us(2);
				
			if(RDSDA)
				Result = (Result << 1) | 0x1;
			else
				Result = (Result << 1) & 0xfe;
			SCL_L();
			delay_us(2);
		}

		if(Len == 0)
		{
			SDA_H();
			delay_us(2);
			SCL_H();
			delay_us(2);
			SCL_L();
			delay_us(2);
		}
		else
		{
			SDA_L();
			delay_us(2);
			SCL_H();
			delay_us(2);
			SCL_L();
			delay_us(2);
		}
		*pBuff++ = Result;
	}
	
}



/********************************************
PAT9125  写寄存器
RegAddr:寄存器地址
Value: 写入的值
********************************************/
void PAT9125WriteReg(uint8_t RegAddr, uint8_t Value)
{
	I2CStart();
	if(I2CWriteByte(CMD_WRITE) != 0)
	{
		I2CStop();
		return;
	}
	if(I2CWriteByte(RegAddr) != 0)
	{
		I2CStop();
		return;
	}
	if(I2CWriteByte(Value) != 0)
	{
		I2CStop();
		return;
	}
	I2CStop();
}

/********************************************
PAT9125  写寄存器
RegAddr:寄存器地址
返回值:  寄存器内容
********************************************/
uint8_t PAT9125ReadReg(uint8_t RegAddr)
{
	uint8_t pData = 0xff;

	I2CStart();
	if(I2CWriteByte(CMD_WRITE) != 0)
	{
		I2CStop();
		return 0xff;
	}
	if(I2CWriteByte(RegAddr) != 0)
	{
		I2CStop();
		return 0xff;
	}
	I2CRestart();
	if(I2CWriteByte(CMD_READ) != 0)
	{
		I2CStop();
		return 0xff;
	}
	I2CReadByte(&pData, 1);
	I2CStop();

	return pData;
}


/********************************************
PAT9125  检测是否存在
返回值:  true  表示存在，false  表示不存在
********************************************/
bool PAT9125CheckExisting(void)
{
	uint8_t PID0, PID1;
	PID0 = PAT9125ReadReg(REG_PID0);
	PID1 = PAT9125ReadReg(REG_PID1);
	if((PID0 == 0x31) && (PID1 == 0x91))
		return true;
	else
		return false;
}

/********************************************
PAT9125 读取移动量
X  :  16  位数据的指针
Y  :  16  位数据的指针
********************************************/
//static uint8_t tmpB;
//static uint8_t tmpS;
//static uint8_t tmpU;

bool PAT9125ReadDelta(int16_t *X, int16_t *Y)
{
	uint16_t tmpX, tmpY;
	uint8_t delxlow,delylow;
	uint8_t delxyhigh;
	uint8_t pat9125_b;
	uint8_t pat9125_s;
	//uint8_t Retry;
	uint8_t ucMotion;

	//Retry = 0;
	//do
	//{
		ucMotion = PAT9125ReadReg(REG_MOTION_STATUS);
		
		pat9125_b = PAT9125ReadReg(REG_FRAME_AVG);
		pat9125_s = PAT9125ReadReg(REG_SHUTTER);
		//tmpU = ucMotion;
		//if(ucMotion == 0xff)
		//	continue;

		if((ucMotion & 0x80) && (pat9125_b != 0xff) && (pat9125_s != 0xff))
		{
			delxlow = PAT9125ReadReg(REG_DELTA_X_LOW);
			delylow = PAT9125ReadReg(REG_DELTA_Y_LOW);
			delxyhigh = PAT9125ReadReg(REG_DELTA_XY_HIGH);

			//tmpX = (((delxyhigh >> 4) << 8) | delxlow) & 0xfff;
			tmpY = (((delxyhigh) << 8) | delylow) & 0xfff;
			/*
			if(tmpX & (1<<11))
				*X = -(~((tmpX & 0x7ff) - 1));
			else
				*X = tmpX & 0x7ff;
			*/

			if(tmpY & (1<<11))
			{
				tmpY = tmpY & 0x7ff;
				tmpY = tmpY - 1;
				tmpY = (~tmpY) & 0x7ff;
				*Y = -tmpY;
			}
			else
			{
				*Y = tmpY & 0x7ff;
			}
			return true;
		}
		else if(ucMotion == 0x7)
		{	
			*X = 0;
			*Y = 0;
			return true;
		}
	//}while(Retry++ < 2);

	return false;
}


/********************************************
PAT9125 设置分辨率
XRes  :  X  分辨率
YRes  :  Y  分辩率
********************************************/
void PAT9125SetResolution(uint8_t XRes, uint8_t YRes)
{
	PAT9125WriteReg(REG_RES_X, XRes);
	PAT9125WriteReg(REG_RES_Y, YRes);
}

/********************************************
使能物料检测软件中断
********************************************/
void EnableFilamentCheck()
{
	return;
	CRITICAL_SECTION_START;
	//清除数据
	PAT9125ReadDelta(&delX, &delY);
	PAT9125ReadDelta(&delX, &delY);
	PAT9125ReadDelta(&delX, &delY);
	PAT9125ReadDelta(&delX, &delY);
	EXTI->PR = THIS_EXTI_LINE;
	EXTI->IMR |= THIS_EXTI_LINE;
	FaultCounter = 0;
	SensorECounter = 0;
	FilamentCheckEnable = true;
	EStepsPerCheck = axis_steps_per_unit[E_AXIS] * FILAMENT_CHECK_LEN + 0.5f;
	MaxFi = 0;
	ErrorCount = 0;
	CRITICAL_SECTION_END;
}

/********************************************
禁能物料检测软件中断
********************************************/
void DisableFilamentCheck()
{
	EXTI->IMR &= ~THIS_EXTI_LINE;
	EXTI->PR = THIS_EXTI_LINE;
	FilamentCheckEnable = false;
}

/********************************************
读测试
********************************************/
long FilamentICReadTest(void)
{
	uint8_t delxlow,delylow,delxyhigh;
	long Count;
	Count = 0;
	for(int i=0;i<50000;i++)
	{
		#if(0)
		if(PAT9125CheckExisting() == false)
		{
			Count++;
		}
		#else
		delay_us(200);
		delxlow = PAT9125ReadReg(REG_DELTA_X_LOW);
		delylow = PAT9125ReadReg(REG_DELTA_Y_LOW);
		delxyhigh = PAT9125ReadReg(REG_DELTA_XY_HIGH);
		if((delxlow == 0xff) || (delylow==0xff) || (delxyhigh == 0xff))
			Count++;
		#endif
	}
	return Count;
}

#endif



/********************************************
初始化物料检测
********************************************/
void FilamentManagerInit()
{	
	#if(HAVE_FALIMENT_SENSOR == 1)
	{
		GPIO_InitTypeDef GPIO_InitStruct;
		
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;	
		GPIO_InitStruct.GPIO_Pin = FILAMNET_I2C_SDA_PIN;
		GPIO_Init(FILAMNET_I2C_SDA_PORT, &GPIO_InitStruct);

		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
		GPIO_InitStruct.GPIO_Pin = FILAMNET_I2C_SCL_PIN;
		GPIO_Init(FILAMNET_I2C_SCL_PORT, &GPIO_InitStruct);

		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_InitStruct.GPIO_Pin = FILAMNET_INT_PIN;
		GPIO_Init(FILAMNET_INT_PORT, &GPIO_InitStruct);

		FILAMNET_I2C_SCL_PORT->BSRR = FILAMNET_I2C_SCL_PIN;
		FILAMNET_I2C_SDA_PORT->BSRR = FILAMNET_I2C_SDA_PIN;


		//SysFaultFlag = FilamentICReadTest();

		//存在传感器
		if(PAT9125CheckExisting() == true)
		{
			for(int i=0;i<sizeof(PATInitRegs) / sizeof(PATInitRegs[0]);i++)
			{
				PAT9125WriteReg(PATInitRegs[i][0], PATInitRegs[i][1]);
			}

			//SysFaultFlag = FilamentICReadTest();

			EXTI_InitTypeDef EXTI_InitStruct;
			NVIC_InitTypeDef NVIC_InitStruct;
			
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);           
			RCC_APB2PeriphClockCmd(PortRccTable[SOURCE_PORT], ENABLE);

			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
			GPIO_InitStruct.GPIO_Pin = (1 << SOURCE_PIN);
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init((GPIO_TypeDef*)PortTable[SOURCE_PORT], &GPIO_InitStruct);

			((GPIO_TypeDef*)PortTable[SOURCE_PORT])->BSRR = (1 << SOURCE_PIN);
			GPIO_EXTILineConfig(SOURCE_PORT, SOURCE_PIN);
			

			EXTI_InitStruct.EXTI_Line = THIS_EXTI_LINE;
			EXTI_InitStruct.EXTI_LineCmd = ENABLE;
			EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
			EXTI_Init(&EXTI_InitStruct);
			

			NVIC_InitStruct.NVIC_IRQChannel = THIS_IRQN;
			NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
			NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
			NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
			NVIC_Init(&NVIC_InitStruct);

			EXTI->PR = THIS_EXTI_LINE;
			EXTI->IMR |= THIS_EXTI_LINE;
		}
	}
	#endif
}

/********************************************
逆初始化物料检测
********************************************/
void FilamentManagerDeinit()
{
	//EXTI->IMR = THIS_EXTI_LINE;
	#if(HAVE_FALIMENT_SENSOR == 1)
	DisableFilamentCheck();
	#endif
}


#if(HAVE_FALIMENT_SENSOR == 1)
extern "C"
{
void EXTI3_IRQHandler(void)
{
	static short tmpdely[3] = {0,0,0};
	int Check_Unit;
	int Tol_Unit;
	
	//读数成功
	if(PAT9125ReadDelta(&delX, &delY) == true)
	{
		Check_Unit = FILAMENT_CHECK_UNIT;
		Tol_Unit = TOLERANT_UNIT;

		TDely[FiCounter] = delY;
		FiDebugBuff[FiCounter] = Check_Unit + delY;
		if(delY < MaxFi)
			MaxFi = delY;
		FiCounter = (FiCounter + 1) % sizeof(FiDebugBuff);
		
		delY = -delY;
		if(delY < Check_Unit)
		{
			if((int16_t)(Check_Unit - delY) > Tol_Unit)
			{
				FaultCounter++;
				if(FaultCounter >= FAULT_TIME)
				{
					FaultCounter = 0;
					ErrorCount++;
					PauseTriggle(FilamentFaultPause);
				}
			}
			else
			{
				if(FaultCounter > 0)
					FaultCounter--;
			}
		}
		else
		{
			if(FaultCounter > 0)
				FaultCounter--;
		}
	}
	//读数失败
	/*
	else
	{
		FaultCounter++;
		if(FaultCounter >= FAULT_TIME)
		{
			FaultCounter = 0;
			PauseTriggle(FilamentFaultPause);
		}
	}
	*/
	EXTI->PR = THIS_EXTI_LINE;
}

}

#endif
