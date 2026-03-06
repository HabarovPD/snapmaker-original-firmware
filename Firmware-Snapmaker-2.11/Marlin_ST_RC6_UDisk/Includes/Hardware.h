#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#ifdef _IN_HARDWARE
#define EXT_HARDWARE
#else
#define EXT_HARDWARE	extern
#endif

EXT_HARDWARE void HardwareInit(void);
EXT_HARDWARE void VariableInit(void);
EXT_HARDWARE void HardwareDeinit(void);
EXT_HARDWARE void IWdgInit(U8 Option);
#define	FeedDog()	{IWDG_ReloadCounter();}


EXT_HARDWARE volatile uint32_t Tick;

#endif
