#ifndef _CNC_H_
#define _CNC_H_

#include "stm32f10x_conf.h"

#ifdef _IN_CNC
#define EXT_CNC
#else
#define EXT_CNC	extern
#endif

EXT_CNC void CNCInit();
EXT_CNC void CNCDeint(void);
EXT_CNC void CncProcess(void);
EXT_CNC void CncSetRPM(uint16_t RPM);

#if(HAVE_CNC_MCU == 1)
//Текущая скорость ЧПУ
EXT_CNC uint16_t CurRPM;
#endif

#endif