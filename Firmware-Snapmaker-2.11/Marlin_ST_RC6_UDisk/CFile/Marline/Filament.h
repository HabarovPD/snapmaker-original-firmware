#ifndef _FILAMENT_H_
#define _FILAMENT_H_

#include "stm32f10x_conf.h"

#ifdef _IN_FILAMENT
#define EXT_FILAMENT
#else
#define EXT_FILAMENT	extern
#endif

EXT_FILAMENT void FilamentManagerInit();
EXT_FILAMENT void FilamentManagerDeinit();
EXT_FILAMENT void EnableFilamentCheck();
EXT_FILAMENT void DisableFilamentCheck();

#define FILAMENT_CHECK_LEN	3.0f
#define TOLERANT_LEN		3.0f

//#define MM_PER_UNIT			0.04285f
//#define TOLERANT_UNIT		(TOLERANT_LEN / MM_PER_UNIT)
#define TOLERANT_UNIT		50
//#define FILAMENT_CHECK_UNIT	(FILAMENT_CHECK_LEN / MM_PER_UNIT)
#define FILAMENT_CHECK_UNIT	120
#define FAULT_TIME			4

#if(HAVE_FALIMENT_SENSOR == 1)
#define TrigFilamentCheck()	{EXTI->SWIER |= EXTI_Line3;}
#endif

EXT_FILAMENT int32_t EStepsPerCheck;
EXT_FILAMENT bool FilamentRequestPause;
EXT_FILAMENT volatile int32_t SensorECounter;
EXT_FILAMENT bool FilamentCheckEnable;
EXT_FILAMENT int16_t FiDebugBuff[20];
EXT_FILAMENT int16_t MaxFi;
EXT_FILAMENT int16_t TDely[20];
EXT_FILAMENT uint8_t FiCounter;
#endif