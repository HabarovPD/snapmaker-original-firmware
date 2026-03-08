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

//Онлайн печать маркировки
EXT_SYSTEMCONTROL bool PrintingOnLine;
//Номер строки инструкции по онлайн-печати
EXT_SYSTEMCONTROL uint32_t CommandLine4OnLine;
//Приостановить запрос статуса
//EXT_SYSTEMCONTROL uint8_t PrintStatusRequest;
//Тип машины
EXT_SYSTEMCONTROL uint8_t MachineSelected;
//фильтр файлов
EXT_SYSTEMCONTROL uint8_t FileFilter;

//Маркировка обнаружения вставки периферийных устройств
EXT_SYSTEMCONTROL uint8_t PeriphPlugFlag;
//Режим работы машины
EXT_SYSTEMCONTROL WordMode MachineMode;
//Полуавтоматическая калибровка включена
EXT_SYSTEMCONTROL uint8_t HalfAutoCalibrateState;
EXT_SYSTEMCONTROL float HalfAutoCalibrateZPosition;

//Сохраняется калькулятор при приостановке и прекращении печати.
EXT_SYSTEMCONTROL volatile long StableCounter[NUM_AXIS];

//Настройка неисправности
EXT_SYSTEMCONTROL uint32_t SysFaultFlag;
//настройка статуса
EXT_SYSTEMCONTROL uint32_t SysStatusFlag;
#endif