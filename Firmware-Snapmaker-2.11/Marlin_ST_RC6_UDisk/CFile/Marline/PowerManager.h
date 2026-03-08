#ifndef _POWERMANAGER_H_
#define _POWERMANAGER_H_

#include "stm32f10x_conf.h"

#ifdef _IN_POWERMANAGER
#define EXT_POWERMANAGER
#else
#define EXT_POWERMANAGER	extern
#endif

typedef enum
{
	GCODE_SOURCE_UDISK = 0,
	GCODE_SOURCE_SCREEN,
	GCODE_SOURCE_PC
}GCodeSources;

typedef struct
{
	//проверять
	uint32_t CheckSum;
	//Температура экструзионной головки
	float HeaterTamp[4];
	//скорость
	float PrintFeedRate;
	//пустая скорость бега
	float TravelFeedRate;
	//Температура нагревательного слоя
	float BedTamp;
	//координировать
	float PositionData[NUM_AXIS];
	//расположение файла
	int FilePosition;
	//рекорд времени
	uint32_t accumulator;
	//скорость вентилятора
	uint8_t FanSpeed;
	//Флаг достоверности данных
	uint8_t Valid;
	//Тип машины
	uint8_t MachineType;
	//Распечатать источник данных
	uint8_t GCodeSource;
	//имя файла
	char FileName[270];
}strPowerPanicSave;

EXT_POWERMANAGER void PowerManagerInit(void);
EXT_POWERMANAGER void SaveBreakPointData(void);
EXT_POWERMANAGER void LoadBreakPointData(void);
EXT_POWERMANAGER void EnablePowerPanicCheck();
EXT_POWERMANAGER void DisablePowerPanicCheck();
EXT_POWERMANAGER bool PowerPanicResumeWork(uint8_t *Err);
EXT_POWERMANAGER void MaskBreadkPointData(void);
EXT_POWERMANAGER void ClearBreakPointData(void);
EXT_POWERMANAGER strPowerPanicSave PowerPanicData;

EXT_POWERMANAGER float CounterTest[NUM_AXIS];

EXT_POWERMANAGER char Current_Block_CMD[96];

EXT_POWERMANAGER bool PowerPanicCheckEnable;


#endif
