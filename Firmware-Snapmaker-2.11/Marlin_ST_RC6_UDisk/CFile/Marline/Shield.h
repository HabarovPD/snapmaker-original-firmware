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

//Проверьте, активирована ли дверь
EXT_SHIELD bool DoorCheckEnable;
//Сроки контроля покрытия
EXT_SHIELD unsigned long ShieldFanOffTick;
EXT_SHIELD unsigned long ShieldLedOffTick;

//Оборудование корпуса работоспособно?
EXT_SHIELD bool ShieldFanControlEn;
EXT_SHIELD bool ShieldLedControlEn;

//Статус открытия крышки устройства
EXT_SHIELD bool ShieldFanIsTurnOn;
EXT_SHIELD bool ShieldLedIsTurnOn;




#endif
