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

//调焦时使用的功率
EXT_LASER uint8_t LaserFocusPwm;
EXT_LASER float LaserSafePercent;//安全阈值，小于这个值可以认为是调焦，开门仍然亮
EXT_LASER uint8_t LaserSafePwm;  //安全阈值，小于这个值可以认为是调焦，开门仍然亮
//最后一次出光功率
EXT_LASER uint16_t LastLaserPwm;

#endif