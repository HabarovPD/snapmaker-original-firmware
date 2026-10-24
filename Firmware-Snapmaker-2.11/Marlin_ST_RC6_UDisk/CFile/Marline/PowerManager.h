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
	//校验
	uint32_t CheckSum;
	//挤出头温度
	float HeaterTamp[4];
	//速度
	float PrintFeedRate;
	//空跑速度
	float TravelFeedRate;
	//热床温度
	float BedTamp;
	//坐标
	float PositionData[NUM_AXIS];
	//文件位置
	int FilePosition;
	//时间记录
	uint32_t accumulator;
	//风扇速度
	uint8_t FanSpeed;
	//数据有效标志
	uint8_t Valid;
	//机器类型
	uint8_t MachineType;
	//打印数据源
	uint8_t GCodeSource;
	//文件名
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
