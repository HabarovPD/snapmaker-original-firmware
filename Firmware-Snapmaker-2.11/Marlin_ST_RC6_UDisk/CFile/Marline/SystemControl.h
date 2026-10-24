#ifndef _SYSTEM_CONTROL_
#define _SYSTEM_CONTROL_

#include "stm32f10x_conf.h"

#ifdef _IN_SYSTEMCONTROL
#define EXT_SYSTEMCONTROL	
#else
#define EXT_SYSTEMCONTROL extern
#endif

#define MainPowerOn()	{MAIN_POWER_SUPPLY_PORT->BSRR = MAIN_POWER_SUPPLY_PIN;}
#define MainPowerOff()	{MAIN_POWER_SUPPLY_PORT->BRR = MAIN_POWER_SUPPLY_PIN;}
#define StepperPowerOn()	{STEPPER_POWER_SUPPLY_PORT->BSRR = STEPPER_POWER_SUPPLY_PIN;}
#define StepperPowerOff()	{STEPPER_POWER_SUPPLY_PORT->BRR = STEPPER_POWER_SUPPLY_PIN;}

EXT_SYSTEMCONTROL void FinishPrintProcess();
EXT_SYSTEMCONTROL void PauseProcess(void);
EXT_SYSTEMCONTROL void PauseTriggle(PausePrintType Type);
EXT_SYSTEMCONTROL void StopTriggle(StopPrintType Type);
EXT_SYSTEMCONTROL bool IsStoppingPrint(void);
EXT_SYSTEMCONTROL PrinterStatus GetCurrentPrinterStatus(void);
EXT_SYSTEMCONTROL void SetCurrentPrinterStatus(PrinterStatus NewStatus);
EXT_SYSTEMCONTROL void PauseResume(void);
EXT_SYSTEMCONTROL void SystemGpioInit(void);
EXT_SYSTEMCONTROL void AndroidScreenPowerManager();
EXT_SYSTEMCONTROL void SystemIdleProcess(void);
EXT_SYSTEMCONTROL void SystemSelfCheck(void);

//在线打印标置
EXT_SYSTEMCONTROL bool PrintingOnLine;
//在线打印指令行数
EXT_SYSTEMCONTROL uint32_t CommandLine4OnLine;
//暂停状态请求
//EXT_SYSTEMCONTROL uint8_t PrintStatusRequest;
//机器类型
EXT_SYSTEMCONTROL uint8_t MachineSelected;
//文件过滤器
EXT_SYSTEMCONTROL uint8_t FileFilter;

//外设插入检测标置
EXT_SYSTEMCONTROL uint8_t PeriphPlugFlag;
//机器工作模式
EXT_SYSTEMCONTROL WordMode MachineMode;
//半自动校准使能
EXT_SYSTEMCONTROL uint8_t HalfAutoCalibrateState;
EXT_SYSTEMCONTROL float HalfAutoCalibrateZPosition;

//暂停与终止打印时，保存的计算器
EXT_SYSTEMCONTROL volatile long StableCounter[NUM_AXIS];

//故障标置
EXT_SYSTEMCONTROL uint32_t SysFaultFlag;
//状态标置
EXT_SYSTEMCONTROL uint32_t SysStatusFlag;
#endif