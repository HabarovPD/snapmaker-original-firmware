#define _IN_SHIELD
#include "Marlin.h"
#include "HMIScreen.h"
#include "HMI_Define.h"
#include "stepper.h"
#include "Udisk.h"
#include "stopwatch.h"
#include "temperature.h"
#include "mesh_bed_leveling.h"
#include "configuration_store.h"
#include "PowerManager.h"
#include "SystemControl.h"
#include "Shield.h"
#include "LaserPowerTable.h"
#include "Laser.h"

#include "stm32f10x_conf.h"

#if(SNAP_OLD_BASE == 0)
#define THIS_EXTI_LINE	EXTI_Line1
#define THIS_IRQN		EXTI1_IRQn
#define SOURCE_PORT		GPIO_PortSourceGPIOC
#define SOURCE_PIN		GPIO_PinSource1
#define PORT_X			GPIOC
#else
#define THIS_EXTI_LINE	EXTI_Line8
#define THIS_IRQN		EXTI9_5_IRQn
#define SOURCE_PORT		GPIO_PortSourceGPIOA
#define SOURCE_PIN		GPIO_PinSource8
#define PORT_X			GPIOA
#endif
//Для входа в прерывание необходимо просмотреть файл .S.

extern void sync_plan_position();
extern void gcode_M104();
extern void gcode_M109();
extern void gcode_M140();
extern void gcode_M190();
extern void gcode_G28();
extern void gcode_G0_G1(uint16_t Number);
extern void gcode_G92();

extern void set_current_to_destination();
extern void set_destination_to_current();

extern long pCounter_X,pCounter_Y,pCounter_Z,pCounter_E;
extern char* current_command, *current_command_args;
extern bool relative_mode;

static strPowerPanicSave tmpPowerPanicData;


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


/********************************************
Инициализация выключения питания
********************************************/
void ShieldInit(void)
{
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;

	//Старая основная плата управления обеспечивает обнаружение укрытий только в лазерном режиме.
	#if(SNAP_OLD_BASE == 1)
	if((MA_LASER == MachineSelected) || (MA_LASER_350 == MachineSelected))
	{
		//Дверь
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);           
		RCC_APB2PeriphClockCmd(PortRccTable[SOURCE_PORT], ENABLE);

		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_InitStruct.GPIO_Pin = (1 << SOURCE_PIN);
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		//GPIO_Init((GPIO_TypeDef*)PortTable[SOURCE_PORT], &GPIO_InitStruct);
		GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_EXTILineConfig(SOURCE_PORT, SOURCE_PIN);
		
		EXTI_InitStruct.EXTI_Line = THIS_EXTI_LINE;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
		EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		EXTI_Init(&EXTI_InitStruct);
		
		NVIC_InitStruct.NVIC_IRQChannel = THIS_IRQN;
		NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
		NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
		NVIC_Init(&NVIC_InitStruct);

		ShieldFanControlEn = true;
		ShieldLedControlEn = true;
		ShieldFanIsTurnOn = true;
		ShieldLedIsTurnOn = true;
		EnableDoorCheck();
		//DisableDoorCheck();
	}
	#else
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);           
	RCC_APB2PeriphClockCmd(PortRccTable[SOURCE_PORT], ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = (1 << SOURCE_PIN);
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	
	GPIO_EXTILineConfig(SOURCE_PORT, SOURCE_PIN);
	
	EXTI_InitStruct.EXTI_Line = THIS_EXTI_LINE;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel = THIS_IRQN;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
	
	ShieldFanControlEn = true;
	ShieldLedControlEn = true;
	ShieldFanIsTurnOn = true;
	ShieldLedIsTurnOn = true;

	DisableDoorCheck();
	#endif
}

/********************************************
Индикатор крышки горит
********************************************/
void ShieldLedOn(void)
{
	#if(SHIELD_AUTO_CONTROL == 0)
	return;
	#else
	if(ShieldLedControlEn == true)
	{
		ShieldLedOffTick = millis() + 30000;
		#ifdef SHIELD_LED_PORT
		SHIELD_LED_PORT->BSRR = SHIELD_LED_PIN;
		#endif
		ShieldLedIsTurnOn = true;
	}
	#endif
}

/********************************************
Вентилятор корпуса запускается
********************************************/
void ShieldFanOn(void)
{
	#if(SHIELD_AUTO_CONTROL == 0)
	return;
	#else
	if(ShieldFanControlEn == true)
	{
		ShieldFanOffTick = millis() + 30000;
		#ifdef SHIELD_FAN_PORT
		if(MA_CNC != MachineSelected)
			SHIELD_FAN_PORT->BSRR = SHIELD_FAN_PIN;
		#endif
		ShieldFanIsTurnOn = true;
	}
	#endif
}


/********************************************
Включить обнаружение отключения электроэнергии
********************************************/
void EnableDoorCheck()
{
	if(DoorCheckEnable == true)
	{
		EXTI->PR = THIS_EXTI_LINE;
		EXTI->IMR |= THIS_EXTI_LINE;
	}
}
/********************************************
Отключено обнаружение отключения электроэнергии
********************************************/
void DisableDoorCheck()
{
	if(DoorCheckEnable == true)
	{
		EXTI->IMR &= ~THIS_EXTI_LINE;
		EXTI->PR = THIS_EXTI_LINE;
	}
}



extern "C"
{
//Новая схема
#if(SNAP_OLD_BASE == 1)
void EXTI9_5_IRQHandler(void)
#else
void EXTI1_IRQHandler(void)
#endif
{
	CRITICAL_SECTION_START;
	
	//Восходящий край, действие открывания двери
	if(PORT_X->IDR & (1<<SOURCE_PIN))
	{
		if((MA_LASER == MachineSelected) || (MA_LASER_350 == MachineSelected))
		{
			if(GetCurrentPrinterStatus() == STAT_PRINTING)
			{
				PauseTriggle(DoorOpenPause);
				current_block = NULL;
				block_buffer_head = block_buffer_tail = 0;
				//Отключить обнаружение
				DisableDoorCheck();
				//лазер выключен
				SetLaserPower(0);
			}
		}
		ShieldLedOn();
	}
	//Задний фронт, закрывание двери
	else
	{
		if((MA_LASER == MachineSelected) || (MA_LASER_350 == MachineSelected))
		{
			if(GetCurrentPrinterStatus() == STAT_PAUSE)
			{
				PauseResume();
				//Отключить обнаружение
				DisableDoorCheck();
			}
		}
	}
	EXTI->PR = THIS_EXTI_LINE;
	CRITICAL_SECTION_END;
}
}
