#ifndef _PRINT_H_
#define _PRINT_H_

#include "stm32f10x_conf.h"

#ifdef _IN_PRINT
#define EXT_PRINT
#else
#define EXT_PRINT	extern
#endif

EXT_PRINT void PrintInit(void);
EXT_PRINT void PrintDeint(void);
EXT_PRINT void PrintProcess(void);

#endif