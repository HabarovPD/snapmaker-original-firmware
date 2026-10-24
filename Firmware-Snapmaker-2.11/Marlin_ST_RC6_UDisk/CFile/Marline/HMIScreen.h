#ifndef _HMISCREEN_H_
#define _HMISCREEN_H_

#ifdef _IN_HMISCREEN
#define EXT_HMISCREEN
#else
#define EXT_HMISCREEN	extern
#endif

#include "ff.h"


EXT_HMISCREEN void HMICommandProcess();
EXT_HMISCREEN void HMIScreenShow();
EXT_HMISCREEN void HMI_ChangePage(uint8_t Page);
EXT_HMISCREEN void HMI_ShowPrintTime(void);
EXT_HMISCREEN void HMI_ShowPercentLable(uint8_t Value);
EXT_HMISCREEN void HMI_ShowProgressBar(uint8_t Value);
EXT_HMISCREEN void HMI_ShowTemperature(uint8_t Index, int CurrentTemp, int TargetTemp);
EXT_HMISCREEN void HMI_ShowFileName(uint8_t Index, char *FileName);
EXT_HMISCREEN void HMI_ZOffsetLable(float Value);
EXT_HMISCREEN void HMI_ShowPrintFileName(char * FileName);
EXT_HMISCREEN void HMI_ShowDialogText(char *String);
EXT_HMISCREEN void HMI_ShowConfirmFileName(char *FileName);
EXT_HMISCREEN void HMI_SetTFIcon(bool Enable);
EXT_HMISCREEN void HMI_StopEnable(bool Enable);
EXT_HMISCREEN void HMI_PauseEnable(bool Enable);
EXT_HMISCREEN void HMI_SetNextEnable(bool Enable);
EXT_HMISCREEN void HMI_SetControlEnable(bool Enable);
EXT_HMISCREEN void HMI_SetSettingEnable(bool Enable);
EXT_HMISCREEN void HMI_ShowChangeFilamentTamp();
EXT_HMISCREEN void HmiGetCurrentPage(void);
EXT_HMISCREEN void HMI_ShowLaserPower(float Power);
EXT_HMISCREEN void HmiComPortInit(int Baudary);
EXT_HMISCREEN void HMI_ChangePrinterStatus(U8 Status);
EXT_HMISCREEN void HMI_ChangeLaserCNCStatus(U8 Status);
EXT_HMISCREEN void HMI_SetAboutLabels(void);
EXT_HMISCREEN void HMI_SendGcode(char *GCode, uint8_t EventID);
EXT_HMISCREEN void HMI_SendContinuePrint(void);

#if(SNAP_OLD_BASE == 1)
EXT_HMISCREEN void HMI_SendScreenUpdate();
EXT_HMISCREEN void HMI_SendUpdateNoFile();
EXT_HMISCREEN void HMI_SendUpdateComplete();
EXT_HMISCREEN void HMI_SendUpdatePack(uint16_t PackIndex);
#else
EXT_HMISCREEN void HMI_SendBreakPointLine();
EXT_HMISCREEN void HMI_SendChDirResult(uint8_t Result);
EXT_HMISCREEN void HMI_SendMachineStatusChange(uint8_t Status, uint8_t Result);
EXT_HMISCREEN void HMI_SendMachineFaultFlag();
EXT_HMISCREEN void HMI_SendBreakPointData();
EXT_HMISCREEN void HMI_SendMachineStatus();
EXT_HMISCREEN void HMI_SendCurrentUDiskPath(uint8_t Result);
EXT_HMISCREEN void HMI_SendInitUdisk(uint8_t Result);
EXT_HMISCREEN uint8_t HMI_SendDirItems(uint8_t Result, uint16_t Offset);
EXT_HMISCREEN void HMI_SendSpecialData();
EXT_HMISCREEN void HMI_SendStartPrintReack(uint8_t Result);
EXT_HMISCREEN void HMI_BuffFlush(void);
#endif


EXT_HMISCREEN void FinishPrintProcess();
EXT_HMISCREEN void PauseProcess(void);
EXT_HMISCREEN void ScreenLockCheck(void);



//锁屏标置
EXT_HMISCREEN bool ScreenLock;
//锁屏计时器
EXT_HMISCREEN uint32_t ScreenLockTick;
EXT_HMISCREEN uint8_t CurrentHMIPage;
EXT_HMISCREEN FIL UpdateFile;
EXT_HMISCREEN FATFS UpdateFatfs;
EXT_HMISCREEN uint8_t HmiRequestStatus;

#endif
