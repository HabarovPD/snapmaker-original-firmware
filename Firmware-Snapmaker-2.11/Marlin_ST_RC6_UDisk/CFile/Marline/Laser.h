#ifndef _LASER_H_
#define _LASER_H_

#include "stm32f10x_conf.h"

#ifdef _IN_LASER
#define EXT_LASER
#else
#define EXT_LASER	extern
#endif

EXT_LASER void LaserInit(void);
EXT_LASER void LaserDeinit(void);
EXT_LASER void LaserProcess(void);
EXT_LASER uint16_t LaserPowerLERP(float v);

#if(HAVE_LASER_EEPROM == 1)
EXT_LASER bool SaveLaserFocusZValue(float ZValue);
EXT_LASER float LaserZValue;
#endif

//Мощность, потребляемая при настройке фокуса
EXT_LASER uint8_t LaserFocusPwm;
EXT_LASER float LaserSafePercent;//Порог безопасности, если он меньше этого значения, его можно считать фокусирующим, и свет все равно будет гореть при открытии двери.
EXT_LASER uint8_t LaserSafePwm;  //Порог безопасности, если он меньше этого значения, его можно считать фокусирующим, и свет все равно будет гореть при открытии двери.
//Последняя светлая сила
EXT_LASER uint16_t LastLaserPwm;

#endif