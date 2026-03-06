#ifndef _SNAPMAKER_DEFINES_H_
#define _SNAPMAKER_DEFINES_H_

//老主控板PB1  为Gate
#define SNAP_OLD_BASE	1

//硬件接口定义
#define HAVE_HMI		1
#define HAVE_SHIELD_FAN	0
#define HAVE_SHIELD_LED	0
#define HAVE_FALIMENT_SENSOR	0
#define HAVE_CNC_MCU	0
#define HAVE_LASER_EEPROM	0
#define HAVE_ANDROID	0
#define HAVE_PROBE		0

#define IO_DOOR_EN	(1<<2)

//激光调光使用DAC
#define LASER_USE_DAC	0

//外罩自动控制
#define SHIELD_AUTO_CONTROL	0

//终止触发源
typedef enum
{
	NoneStop = 0,
	//脱机打印结束停止
	EndPrint = 1,
	//人机停止
	ManualStop,
	//断电停止
	PowerPanicStop
}StopPrintType;

//暂停触发源
typedef enum
{
	NonePause = 0,
	//物料错误
	FilamentFaultPause = 1,
	//人机停止
	ManualPause,
	//门中断
	DoorOpenPause,
	//执行头丢失
	ExecuterLostPause,
	//继续
	ContinuePrint
}PausePrintType;

//机器动作状态
typedef enum
{
	//待机状态
	STAT_IDLE = 0,
	//打印中
	STAT_PRINTING,
	//暂停
	STAT_PAUSE,
	//连机打印中
	STAT_PRINTING_ONLINE,
	//连机暂停
	STAT_PAUSE_ONLINE
}PrinterStatus;

//电源开关控制
typedef enum
{
	POWER_OPTION_MainPower = 0,
	POWER_OPTION_StepperPower
}PowerOption;

//安卓屏幕电源控制状态
enum
{
	ANDROID_PWR_STAT_IDLE = 0,
	ANDROID_PWR_STAT_POWERON,
	ANDROID_PWR_STAT_POWEROFF,
	ANDROID_PWR_STAT_KEYDOWN
};

//机器工作状态
typedef enum
{
	WORK_MODE_IDLE = 0,
	WORK_MODE_RUNNING
}WordMode;

//外围接入标置位定义
#define PERIPH_PC_FLAG		(1<<0)
#define PERIPH_SCREEN_FLAG	(1<<1)

//屏幕控制 
#define ANDROID_CTRL_NONE		0
#define ANDROID_CTRL_RESET		1
#define ANDROID_CTRL_POWEROFF	2

//错误标置
#define FAULT_FLAG_HEATER0	1
#define FAULT_FLAG_BED		(1<<1)
#define FAULT_FLAG_LOAD		(1<<2)
#define FAULT_FLAG_FILAMENT	(1<<3)

#endif
