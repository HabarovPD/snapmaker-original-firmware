#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#ifdef _IN_SYSTEM
#define EXT_SYSTEM
#else
#define EXT_SYSTEM	extern
#endif



EXT_SYSTEM void PowerOnProcess(void);
EXT_SYSTEM void NormalProcess(void);

EXT_SYSTEM void SetLed(U8 parameter);
EXT_SYSTEM void SetBrightness(U8 Level);
EXT_SYSTEM void DisplaySwitch(U8 En);

EXT_SYSTEM U32 NoCommutionTick;


#endif
