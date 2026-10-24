#ifndef _SHIELD_H_
#define _SHIELD_H_

#include "stm32f10x_conf.h"

#ifdef _IN_SHIELD
#define EXT_SHIELD
#else
#define EXT_SHIELD	extern
#endif

EXT_SHIELD void ShieldInit(void);
EXT_SHIELD void EnableDoorCheck();
EXT_SHIELD void DisableDoorCheck();
EXT_SHIELD void ShieldLedOn(void);
EXT_SHIELD void ShieldFanOn(void);

//门是否启动检测
EXT_SHIELD bool DoorCheckEnable;
//外罩控制计时
EXT_SHIELD unsigned long ShieldFanOffTick;
EXT_SHIELD unsigned long ShieldLedOffTick;

//外罩设备是否可操作
EXT_SHIELD bool ShieldFanControlEn;
EXT_SHIELD bool ShieldLedControlEn;

//外罩设备开启状态
EXT_SHIELD bool ShieldFanIsTurnOn;
EXT_SHIELD bool ShieldLedIsTurnOn;




#endif
