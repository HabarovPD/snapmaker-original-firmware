#ifndef _SNAPMAKER_DEFINES_H_
#define _SNAPMAKER_DEFINES_H_

//Старая главная плата управления PB1 — это ворота.
#define SNAP_OLD_BASE	1

//Определение аппаратного интерфейса
#define HAVE_HMI		1
#define HAVE_SHIELD_FAN	0
#define HAVE_SHIELD_LED	0
#define HAVE_FALIMENT_SENSOR	0
#define HAVE_CNC_MCU	0
#define HAVE_LASER_EEPROM	0
#define HAVE_ANDROID	0
#define HAVE_PROBE		0

#define IO_DOOR_EN	(1<<2)

//Лазерное затемнение с помощью ЦАП
#define LASER_USE_DAC	0

//Автоматический контроль покрытия
#define SHIELD_AUTO_CONTROL	0

//Прекратить источник триггера
typedef enum
{
	NoneStop = 0,
	//Офлайн-печать завершается и останавливается
	EndPrint = 1,
	//Человеческая машина останавливается
	ManualStop,
	//Остановиться, когда питание отключено
	PowerPanicStop
}StopPrintType;

//источник триггера паузы
typedef enum
{
	NonePause = 0,
	//Ошибка материала
	FilamentFaultPause = 1,
	//Человеческая машина останавливается
	ManualPause,
	//прерывание двери
	DoorOpenPause,
	//Заголовок выполнения отсутствует
	ExecuterLostPause,
	//продолжать
	ContinuePrint
}PausePrintType;

//Статус действия машины
typedef enum
{
	//Состояние ожидания
	STAT_IDLE = 0,
	//Печать
	STAT_PRINTING,
	//пауза
	STAT_PAUSE,
	//Печать на машине
	STAT_PRINTING_ONLINE,
	//Соединение приостановлено
	STAT_PAUSE_ONLINE
}PrinterStatus;

//Управление выключателем питания
typedef enum
{
	POWER_OPTION_MainPower = 0,
	POWER_OPTION_StepperPower
}PowerOption;

//Статус управления питанием экрана Android
enum
{
	ANDROID_PWR_STAT_IDLE = 0,
	ANDROID_PWR_STAT_POWERON,
	ANDROID_PWR_STAT_POWEROFF,
	ANDROID_PWR_STAT_KEYDOWN
};

//Рабочее состояние машины
typedef enum
{
	WORK_MODE_IDLE = 0,
	WORK_MODE_RUNNING
}WordMode;

//Определение набора флагов доступа к периферийным устройствам
#define PERIPH_PC_FLAG		(1<<0)
#define PERIPH_SCREEN_FLAG	(1<<1)

//управление экраном 
#define ANDROID_CTRL_NONE		0
#define ANDROID_CTRL_RESET		1
#define ANDROID_CTRL_POWEROFF	2

//Маркировка ошибок
#define FAULT_FLAG_HEATER0	1
#define FAULT_FLAG_BED		(1<<1)
#define FAULT_FLAG_LOAD		(1<<2)
#define FAULT_FLAG_FILAMENT	(1<<3)

#endif
