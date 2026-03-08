#define _IN_SYSTEMCONTROL
#include "stm32f10x_conf.h"

#include "Marlin.h"
#include "HMIScreen.h"
#include "HMI_Define.h"
#include "stepper.h"
#include "Udisk.h"
#include "stopwatch.h"
#include "temperature.h"
#include "mesh_bed_leveling.h"
#include "configuration_store.h"
#include "PowerManager.h"
#include "Filament.h"
#include "SystemControl.h"
#include "Shield.h"
#include "LaserPowerTable.h"
#include "Laser.h"

extern void process_next_command();
extern void gcode_G28();
extern void gcode_G0_G1(uint16_t Number);
extern void gcode_G92();
extern void gcode_M206();
extern void gcode_M104();
extern char WaitUDisk(void);
extern void sync_plan_position();
extern long pCounter_X,pCounter_Y,pCounter_Z,pCounter_E;

extern bool cancel_heatup;
extern volatile bool DiskInserted;
extern char* current_command, *current_command_args;
extern bool relative_mode;

#if(SNAP_OLD_BASE == 0)
static const uint8_t BreathLedTable[] =
{50,84,104,119,130,139,147,153,159,164,169,173,177,181,184,188,191,193,196,199,201,203,206,208,210,212,213,215,217,219,220,222,223,225,226,228,229,230,232,233,234,235,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,251,252,253,254,255,255,255,254,253,252,251,251,250,249,248,247,246,245,244,243,242,241,240,239,238,237,235,234,233,232,230,229,228,226,225,223,222,220,219,217,215,213,212,210,208,206,203,201,199,196,193,191,188,184,181,177,173,169,164,159,153,147,139,130,119,104,84,50};

#endif

//Проверьте, пуст ли буфер инструкций
#define CMD_BUFF_EMPTY()	(GetCommandBuff()>0?false:true)
static char tmpBuff[96];

//Позиция во время паузы
static float PausePosition[4];
//Остановить тип печати
static StopPrintType StopType;
//Остановить тип печати
static PausePrintType PauseType;


//Немедленно прекратите выполнение настройки
static volatile bool SystemRequestStop;
//Статус печати
static PrinterStatus PrintingStatus;
//Режим, указанный хост-компьютером
//статический WordMode ModeFromCommando;


/********************************************
Включить обнаружение отключения исполнительной головки
********************************************/
void EnableExecuterCheck()
{
	CRITICAL_SECTION_START;
	EXTI->PR = EXTI_Line14 | EXTI_Line15;
	EXTI->IMR |= EXTI_Line14 | EXTI_Line15;
	CRITICAL_SECTION_END;
}

/********************************************
Отключить выполнение обнаружения отключения головки
********************************************/
void DisableExecuterCheck()
{
	EXTI->IMR &= ~EXTI_Line14 | EXTI_Line15;
	EXTI->PR = EXTI_Line14 | EXTI_Line15;
}


/***********************************************
Обработка G-кода в состоянии ожидания
***********************************************/
void SystemIdleGcodeProcess(void)
{
  current_command = command_queue[cmd_queue_index_r];

  if (DEBUGGING(ECHO)) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLN(current_command);
  }

  // Очистите текущую команду:
  //  -Пропустить ведущие пробелы
  //  -Обход N[-0-9][0-9]*[ ]*
  //  -Замените *нулем, чтобы отметить конец
  while (*current_command == ' ') ++current_command;
  if (*current_command == 'N' && NUMERIC_SIGNED(current_command[1])) {
    current_command += 2; // пропустить N[-0-9]
    while (NUMERIC(*current_command)) ++current_command; // пропустить [0-9]*
    while (*current_command == ' ') ++current_command; // пропускать [ ]*
  }
  char* starpos = strchr(current_command, '*');  // *всегда должен быть последним параметром
  if (starpos) while (*starpos == ' ' || *starpos == '*') *starpos-- = '\0'; // обнулить '*' и ' '

  char *cmd_ptr = current_command;

  uint16_t codenum = 0; // определить перед переходом

  bool code_is_good;

  // Получите код команды, который должен быть G, M или T.
  char command_code = *cmd_ptr++;

  // Пропускайте пробелы, чтобы получить числовую часть
  while (*cmd_ptr == ' ') cmd_ptr++;

  // Залог досрочно, если нет кода
  if(command_code == 'M')
  {
    code_is_good = NUMERIC(*cmd_ptr);
    if (!code_is_good) goto ExitUnknownCommand;
    
    // Получить и пропустить кодовый номер
    do {
    codenum = (codenum * 10) + (*cmd_ptr - '0');
    cmd_ptr++;
    } while (NUMERIC(*cmd_ptr));
  }

  // Пропустите все пробелы, чтобы перейти к первому аргументу или нулю.
  while (*cmd_ptr == ' ') cmd_ptr++;

  // Аргументы команды (если они есть) обязательно начинаются здесь!
  current_command_args = cmd_ptr;

  //KEEPALIVE_STATE(1);

  if(command_code == 'M')
  {
    switch(codenum)
	{
		//Войти в режим запуска
		case 4000:
			//ModeFromCommando = WORK_MODE_RUNNING;
		break;

		//Войти в режим ожидания
		case 4001:
			//ModeFromCommando = WORK_MODE_IDLE;
		break;

		//Статус заголовка выполнения запроса
		case 4002:
			
		break;
	}
  }

  //KEEPALIVE_STATE(0);

ExitUnknownCommand:

  // Все еще неизвестная команда? Выдать ошибку
  if (!code_is_good) unknown_command_error();

  ok_to_send();

}

#if(SNAP_OLD_BASE == 0)
/***********************************************
Поток резервной обработки
Обнаружение статуса вставки заголовка выполнения
Обработать некоторый указанный Gcode
Переключение режима на основе состояния вставленной исполнительной головки и состояния, указанного хост-компьютером.
***********************************************/
void SystemIdleProcess(void)
{
	uint8_t LastMachineSelected;
	millis_t CheckTick;
	uint8_t EIOCheck[3];
	LastMachineSelected = MA_UNDEFINED;
	//выключить питание
	MainPowerOff();
	StepperPowerOff();
	CheckTick = millis();
	while(MachineMode == WORK_MODE_IDLE)
	{
		EIOCheck[2] = EIOCheck[1];
		EIOCheck[1] = EIOCheck[0];
		EIOCheck[0] = 0;
		//Обнаружение вставки заголовка выполнения
		if((E_ID_0_PORT->IDR & E_ID_0_PIN))
			EIOCheck[0] |= 1;

		if((E_ID_1_PORT->IDR & E_ID_1_PIN))
			EIOCheck[0] |= 2;


		//отскакивать
		if((EIOCheck[0] == EIOCheck[1]) && (EIOCheck[1] == EIOCheck[2]))
		{
			if(EIOCheck[0] != 0x3)
			{
				switch(EIOCheck[0])
				{
					case 0:
						MachineSelected = MA_PRINT;
						MachineMode = WORK_MODE_RUNNING;
						//EnableExecuterCheck();
					break;

					case 1:
						MachineSelected = MA_CNC;
						MachineMode = WORK_MODE_RUNNING;
						//EnableExecuterCheck();
					break;

					case 2:
						MachineSelected = MA_LASER;
						MachineMode = WORK_MODE_RUNNING;
						//EnableExecuterCheck();
					break;
				}
			}
			else
			{
				MachineSelected = MA_UNDEFINED;
			}
		}
		else
		{
			MachineSelected = MA_UNDEFINED;
		}
		
		if(MachineSelected != MA_UNDEFINED)
		{
			if(LastMachineSelected != MachineSelected)
			{
				CheckTick = millis();
			}

			if((millis() - CheckTick) > (2 * TICK_PER_SECOND))
			{
				//Никаких системных ошибок
				if(!(SysFaultFlag & FAULT_FLAG_BED & FAULT_FLAG_HEATER0 & FAULT_FLAG_LOAD))
					break;
			}
		}

		#if(HAVE_ANDROID == 1)
		AndroidScreenPowerManager();
		#endif

		LastMachineSelected = MachineSelected;
		
		//Обработка G-кода
		/*
		get_serial_commands();
		если (GetCommandBuff() > 0) 
		{
			//Паладин
			SystemIdleGcodeProcess();
			DispatchComandIndex();
			ShieldLedOn();
			ShieldFanOn();
		}
		*/

		//Обнаружена исполнительная головка, и главный компьютер требует переключения статуса.
		/*
		if((MachineSelected != MA_UNDEFINED) && (ModeFromCommando == WORK_MODE_RUNNING))
		{
			MachineMode = WORK_MODE_RUNNING;
			//EnableExecuterCheck();
		}
		*/
		HMICommandProcess();
	}
}
#endif

#if (HAVE_ANDROID == 1)
uint8_t AndroidControlFlag;
uint8_t PowerStat;
/**************************************************
Управление питанием экрана Android
***************************************************/
void AndroidScreenPowerManager()
{
	static millis_t tmptick;
	static uint8_t DetIO[3] = {3,3,3};

	DetIO[2] = DetIO[1];
	DetIO[1] = DetIO[0];
	DetIO[0] = 0;
	if(ANDROID_DET_PORT->IDR & ANDROID_DET_PIN)
		DetIO[0] |= 1;

	if(PC_PLUG_DET_PORT->IDR & PC_PLUG_DET_PIN)
		DetIO[0] |= 2;
	

	//Экран подключен.
	if(!((DetIO[0] | DetIO[1] | DetIO[2]) & 1))
	{
		//Раньше он был отключен
		if(!(PeriphPlugFlag & PERIPH_SCREEN_FLAG))
		{
			//Экран перезапуска
			PowerStat = ANDROID_PWR_STAT_IDLE;
			AndroidControlFlag = ANDROID_CTRL_RESET;
			PeriphPlugFlag |= PERIPH_SCREEN_FLAG;
		}
	}
	//Экран отключен
	else if(DetIO[0] & DetIO[1] & DetIO[2] & 1)
	{
		//Раньше было подключено
		if(PeriphPlugFlag & PERIPH_SCREEN_FLAG)
		{
			AndroidControlFlag = ANDROID_CTRL_POWEROFF;
			//Очистить буфер последовательного порта экрана
			HMI_BuffFlush();
			//Разблокировать экран
			ScreenLock = false;
			//Машина печатает
			if(GetCurrentPrinterStatus() != STAT_IDLE)
			{
				//триггерная остановка
				StopTriggle(ManualStop);
			}
			PeriphPlugFlag &= ~PERIPH_SCREEN_FLAG;
		}
	}

	//ПК не подключен к устройству
	if(DetIO[0] & DetIO[1] & DetIO[2] & 2)
	{
		//Компьютер был подключен раньше
		if(PeriphPlugFlag & PERIPH_PC_FLAG)
		{
			//Переключить последовательный порт на экран
			UART1_SELK1_PORT->BRR = UART1_SELK1_PIN;
			PeriphPlugFlag &= ~PERIPH_PC_FLAG;
			//Машина печатает
			if(GetCurrentPrinterStatus() != STAT_IDLE)
			{
				//триггерная остановка
				StopTriggle(ManualStop);
			}
		}
	}
	//Устройство, подключенное к ПК
	else if(!((DetIO[0] | DetIO[1] | DetIO[2]) & 2))
	{
		//Компьютер был отключен раньше
		if(!(PeriphPlugFlag & PERIPH_PC_FLAG))
		{
			//Экран не заблокирован, и устройство не печатает
			if((ScreenLock == false) && (GetCurrentPrinterStatus() == STAT_IDLE))
				UART1_SELK1_PORT->BSRR = UART1_SELK1_PIN;
			PeriphPlugFlag |= PERIPH_PC_FLAG;
		}
	}
	
	switch(PowerStat)
	{
		case ANDROID_PWR_STAT_IDLE:
			//Экран отключен, питание выключено
			if(AndroidControlFlag == ANDROID_CTRL_POWEROFF)
			{
				//Поднимите кнопку питания
				ANDROID_START_PORT->BRR = ANDROID_START_PIN;
				//выключить питание
				ANDROID_PWR_PORT->BRR = ANDROID_PWR_PIN;
				AndroidControlFlag = ANDROID_CTRL_NONE;
				//Декодирование экрана
				ScreenLock = false;
			}
			//Переподключите экран и перезагрузите экран
			else if(AndroidControlFlag == ANDROID_CTRL_RESET)
			{
				//Поднимите кнопку питания
				ANDROID_START_PORT->BRR = ANDROID_START_PIN;
				//выключить питание
				ANDROID_PWR_PORT->BRR = ANDROID_PWR_PIN;
				//Статус переключения
				PowerStat = ANDROID_PWR_STAT_POWEROFF;
				//четкая отметка
				AndroidControlFlag = ANDROID_CTRL_NONE;
				tmptick = millis();
			}
		break;

		case ANDROID_PWR_STAT_POWEROFF:
			//Задержка 500 мс
			if((millis() - tmptick) > 300)
			{
				//Включите питание
				ANDROID_PWR_PORT->BSRR = ANDROID_PWR_PIN;
				//Статус переключения
				PowerStat = ANDROID_PWR_STAT_POWERON;
				tmptick = millis();
			}
		break;

		case ANDROID_PWR_STAT_POWERON:
			//Задержка 3000 мс
			if((millis() - tmptick) > 3000)
			{
				//Опустите кнопку питания
				ANDROID_START_PORT->BSRR = ANDROID_START_PIN;
				//Статус переключения
				PowerStat = ANDROID_PWR_STAT_KEYDOWN;
				tmptick = millis();
			}
		break;

		case ANDROID_PWR_STAT_KEYDOWN:
			if((millis() - tmptick) > 2000)
			{
				//Статус переключения
				PowerStat = ANDROID_PWR_STAT_IDLE;
				//Поднимите кнопку питания
				ANDROID_START_PORT->BRR = ANDROID_START_PIN;
			}
		break;
	}
}
#endif

#if(SNAP_OLD_BASE == 0)
/**********************************************************
Управление питанием системы
Опция: контрольная цель
               POWER_OPTION_MainPower: источник питания исполнительной головки и горячего стола.
               POWER_OPTION_StepperPower: источник питания шагового двигателя.
Включить: true для открытия, false для закрытия
**********************************************************/
void SystemPowerControl(PowerOption Option, bool Enable)
{
	switch(Option)
	{
		case POWER_OPTION_MainPower:
			if(Enable == true)
				MAIN_POWER_SUPPLY_PORT->BSRR = MAIN_POWER_SUPPLY_PIN;
			else
				MAIN_POWER_SUPPLY_PORT->BRR = MAIN_POWER_SUPPLY_PIN;
		break;

		case POWER_OPTION_StepperPower:
			if(Enable == true)
				STEPPER_POWER_SUPPLY_PORT->BSRR = STEPPER_POWER_SUPPLY_PIN;
			else
				STEPPER_POWER_SUPPLY_PORT->BRR = STEPPER_POWER_SUPPLY_PIN;
		break;
	}
}
#endif

/******************************************
Запросить, прекращена ли печать в данный момент
*****************************************/
bool IsStoppingPrint(void)
{
	return SystemRequestStop;
}

/******************************************
Запрос текущего состояния принтера
Возвращаемое значение: STAT_IDLE
                    STAT_PRINTG
                    STAT_PAUSE
*****************************************/
PrinterStatus GetCurrentPrinterStatus(void)
{
	return PrintingStatus;
}

/******************************************
Установить текущий статус принтера
Новый статус: STAT_IDLE
                    STAT_PRINTG
                    STAT_PAUSE
*****************************************/
void SetCurrentPrinterStatus(PrinterStatus NewStatus)
{
	PrintingStatus = NewStatus;
}


/******************************************
Приостановить печать триггера
Тип: тип триггера
*****************************************/
void PauseTriggle(PausePrintType Type)
{
	block_t *pBlock;
	PauseType = Type;
	StableCounter[0] = pCounter_X;
	StableCounter[1] = pCounter_Y;
	StableCounter[2] = pCounter_Z;
	StableCounter[3] = pCounter_E;
	//если (текущий_блок == NULL)
	//	current_block = plan_get_current_block();
	switch(Type)
	{
		case FilamentFaultPause:
			CRITICAL_SECTION_START;
			//Печать на USB-диске
			if(GetCurrentPrinterStatus() == STAT_PRINTING)
				card.sdprinting = false;
			SystemRequestStop = true;
			SysFaultFlag |= FAULT_FLAG_FILAMENT;
			
			//Блок движения пуст
			if(current_block != NULL)
			{
				PowerPanicData.FilePosition = current_block->FilePosition;
				pBlock = current_block;
			}
			else 
			{
				//Есть следующий блок движения
				if(blocks_queued() == true)
				{
					//Прочитайте следующий блок номеров строк
					pBlock = &block_buffer[block_buffer_tail];
					PowerPanicData.FilePosition = pBlock->FilePosition;
				}
				//Нет следующего блока
				else
				{
					//Прочитайте следующий блок номеров строк
					pBlock = &block_buffer[(block_buffer_tail + BLOCK_BUFFER_SIZE - 1) % BLOCK_BUFFER_SIZE];
					PowerPanicData.FilePosition = pBlock->FilePosition;
				}
			}
			#if(HAVE_FALIMENT_SENSOR == 1)
			if(MA_PRINT == MachineSelected)
				DisableFilamentCheck();
			#endif
			PowerPanicData.TravelFeedRate = pBlock->TravelFeedRate;
			PowerPanicData.PrintFeedRate = pBlock->PrintFeedRate;
			CRITICAL_SECTION_END;
		break;

		case ManualPause:
		case ExecuterLostPause:
			CRITICAL_SECTION_START;
			//Просьба остановиться
			SystemRequestStop = true;
			//Печать на USB-диске
			if(GetCurrentPrinterStatus() == STAT_PRINTING)
				card.sdprinting = false;
			
			//Блок движения не пуст
			if(current_block != NULL)
			{
				PowerPanicData.FilePosition = current_block->FilePosition;
				pBlock = current_block;
			}
			else 
			{
				//Есть следующий блок движения
				if(blocks_queued() == true)
				{
					//Прочитайте следующий блок номеров строк
					pBlock = &block_buffer[block_buffer_tail];
					PowerPanicData.FilePosition = pBlock->FilePosition;
				}
				//Нет следующего блока
				else
				{
					//Прочитайте следующий блок номеров строк
					pBlock = &block_buffer[(block_buffer_tail + BLOCK_BUFFER_SIZE - 1) % BLOCK_BUFFER_SIZE];
					PowerPanicData.FilePosition = pBlock->FilePosition;
				}
			}
			PowerPanicData.TravelFeedRate = pBlock->TravelFeedRate;
			PowerPanicData.PrintFeedRate = pBlock->PrintFeedRate;
			#if(HAVE_FALIMENT_SENSOR == 1)
			if(MA_PRINT == MachineSelected)
				DisableFilamentCheck();
			#endif
			CRITICAL_SECTION_END;
		break;

		case DoorOpenPause:
			//Распечатать онлайн
			if(GetCurrentPrinterStatus() == STAT_PRINTING)
				card.sdprinting = false;
			SystemRequestStop = true;
			//Блок движения не пуст
			if(current_block != NULL)
			{
				PowerPanicData.FilePosition = current_block->FilePosition;
				pBlock = current_block;
			}
			else 
			{
				//Есть следующий блок движения
				if(blocks_queued() == true)
				{
					//Прочитайте следующий блок номеров строк
					pBlock = &block_buffer[block_buffer_tail];
					PowerPanicData.FilePosition = pBlock->FilePosition;
				}
				//Нет следующего блока
				else
				{
					//Прочитайте следующий блок номеров строк
					pBlock = &block_buffer[(block_buffer_tail + BLOCK_BUFFER_SIZE - 1) % BLOCK_BUFFER_SIZE];
					PowerPanicData.FilePosition = pBlock->FilePosition;
				}
			}
			PowerPanicData.TravelFeedRate = pBlock->TravelFeedRate;
			PowerPanicData.PrintFeedRate = pBlock->PrintFeedRate;
		break;

		default:
			PauseType = NonePause;
		break;
	}
}

/******************************************
Продолжить печать триггера
Тип: тип триггера
*****************************************/
void PauseResume(void)
{
	PauseType = ContinuePrint;
}

/******************************************
Остановить печать триггера
Тип: тип триггера
*****************************************/
void StopTriggle(StopPrintType Type)
{
	StopType = Type;
	StableCounter[0] = pCounter_X;
	StableCounter[1] = pCounter_Y;
	StableCounter[2] = pCounter_Z;
	StableCounter[3] = pCounter_E;
	//если (текущий_блок == NULL)
	//	current_block = plan_get_current_block();
	switch(Type)
	{
		case EndPrint:
			
		break;

		case ManualStop:
			CRITICAL_SECTION_START;
			card.sdprinting = false;
			SystemRequestStop = true;
			if(current_block != NULL)
			{
				PowerPanicData.FilePosition = current_block->FilePosition;
				PowerPanicData.TravelFeedRate = current_block->TravelFeedRate;
				PowerPanicData.PrintFeedRate = current_block->PrintFeedRate;
			}
			else
			{	
				PowerPanicData.TravelFeedRate = GetG0FeedRate();
				PowerPanicData.PrintFeedRate = GetG1FeedRate();
			}
			CRITICAL_SECTION_END;
		break;

		case PowerPanicStop:
			//Запуск прерывания, критическая защита не требуется, критическая защита включена во время обработки прерывания
			//CRITICAL_SECTION_START;
			card.sdprinting = false;
			SystemRequestStop = true;
			if(current_block != NULL)
			{
				PowerPanicData.FilePosition = current_block->FilePosition;
				PowerPanicData.TravelFeedRate = current_block->TravelFeedRate;
				PowerPanicData.PrintFeedRate = current_block->PrintFeedRate;
			}
			else
			{	
				PowerPanicData.TravelFeedRate = GetG0FeedRate();
				PowerPanicData.PrintFeedRate = GetG1FeedRate();
			}
			//strcpy(PowerPanicData.LastCommand, current_block->Cmd);
			//CRITICAL_SECTION_END;
		break;

		default:
			StopType = NoneStop;
		break;
	}
}


/****************************************************************
Приостановить обработку
****************************************************************/
void PauseProcess(void)
{
	millis_t tmpmills;

	if((PrintingStatus == STAT_PRINTING) || (PrintingStatus == STAT_PRINTING_ONLINE))
	{
		if((PauseType == ManualPause) || (PauseType == FilamentFaultPause) || (PauseType == DoorOpenPause) || (PauseType == ExecuterLostPause))
		{
			//Установить статус паузы
			if(PrintingStatus == STAT_PRINTING_ONLINE)
				PrintingStatus = STAT_PAUSE_ONLINE;
			else
				PrintingStatus = STAT_PAUSE;

			block_buffer_head = block_buffer_tail = 0;
			current_block = NULL;

			//очистить команду
			FlushCommand();

			//температура хранения
			for(int i=0;i<4;i++)		
				PowerPanicData.HeaterTamp[i] = target_temperature[i];

			//Температура нагревательного слоя
			PowerPanicData.BedTamp = target_temperature_bed;
			//скорость вентилятора 
			//PowerPanicData.FanSpeed = TIM3->CCR1;
			//Сохранение модели
			PowerPanicData.MachineType = MachineSelected;

			//Настройка действительна
			PowerPanicData.Valid = 1;

			//имя файла
			if(PowerPanicData.GCodeSource == GCODE_SOURCE_UDISK)
			{
				strcpy(PowerPanicData.FileName, card.currentselectfilename);
			}
			else
			{
				PowerPanicData.FileName[0] = 0;
			}
			
			current_position[X_AXIS] = StableCounter[X_AXIS] / axis_steps_per_unit[X_AXIS];
			current_position[Y_AXIS] = StableCounter[Y_AXIS] / axis_steps_per_unit[Y_AXIS];
			current_position[Z_AXIS] = StableCounter[Z_AXIS] / axis_steps_per_unit[Z_AXIS];
			current_position[E_AXIS] = StableCounter[E_AXIS] / axis_steps_per_unit[E_AXIS];

			//Компенсация по оси Z
			if(MA_PRINT == MachineSelected)
			{
				float delZ = mbl.get_z(current_position[X_AXIS], current_position[Y_AXIS]);
				current_position[Z_AXIS] -= delZ;
			}
			
			sync_plan_position();

			//координировать
			for(int i=0;i<NUM_AXIS;i++)
				PowerPanicData.PositionData[i] = current_position[i];

			//Удалите запрос на остановку настройки. После очистки двигатель снова может двигаться.
			SystemRequestStop = false;

			//текущее удержание
			StepperKeepCurrent = true;

			//Статус отчета
			#if(SNAP_OLD_BASE == 0)
			if(PauseType == FilamentFaultPause)
				HMI_SendMachineFaultFlag();
			//HMI_SendMachineStatusChange(0x04, 0);
			#endif

			if(MA_PRINT == MachineSelected)
			{
				//Отзывать
				if(current_temperature[0] > 180)
				{
					//Переключиться в режим относительных координат
					relative_mode = true;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS] - 4, 40, active_extruder);
					while(blocks_queued())manage_heater();
					current_position[E_AXIS] -= 4;
				}
				
				//Переключиться в режим абсолютных координат
				relative_mode = false;

				if((HomeAfterReset[X_AXIS] == true) && (HomeAfterReset[Y_AXIS] == true) && (HomeAfterReset[Z_AXIS] == true))
				{
					//Подъем оси Z 30  
					if((current_position[Z_AXIS] + 30) < (home_offset[Z_AXIS] + base_max_pos[Z_AXIS]))
						sprintf(tmpBuff, "G0 Z%0.3f F2000", (current_position[Z_AXIS] + 30));
					else
						sprintf(tmpBuff, "G0 Z%0.3f F2000", (home_offset[Z_AXIS] + base_max_pos[Z_AXIS]));
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					while(blocks_queued())manage_heater();
					//Ось X перемещается в положение концевого выключателя
					sprintf(tmpBuff, "G0 X0 F2000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					while(blocks_queued())manage_heater();
					//Ось Y достигает максимального положения
					sprintf(tmpBuff, "G0 Y%0.3f F2000", (home_offset[Y_AXIS] + base_max_pos[Y_AXIS]));
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					while(blocks_queued())manage_heater();
				}
				//Переключиться на основной интерфейс
				HMI_ChangePage(PAGE_PRINT);
				//Отключить обнаружение разрыва материала
				#if(HAVE_FALIMENT_SENSOR == 1)
				DisableFilamentCheck();
				#endif
			}
			else if(MA_CNC == MachineSelected)
			{			
				//Подъем оси Z 30  
				if((current_position[Z_AXIS] + 30) < (home_offset[Z_AXIS] + base_max_pos[Z_AXIS]))
					sprintf(tmpBuff, "G0 Z%0.3f", (current_position[Z_AXIS] + 30));
				else
					sprintf(tmpBuff, "G0 Z%0.3f", (home_offset[Z_AXIS] + base_max_pos[Z_AXIS]));
				current_command_args = tmpBuff;
				gcode_G0_G1(0);
				while(blocks_queued())manage_heater();

				//Переход к началу координат заготовки
				strcpy(tmpBuff, "G0 X0 Y0 F3000");
				current_command_args = tmpBuff;
				gcode_G0_G1(0);
				while(blocks_queued())manage_heater();

				//Выключите двигатель
				digitalWrite(LASER_PORT, LASER_PIN, LOW);
				//Переключиться на основной интерфейс
				HMI_ChangePage(PAGE_CNC);
			}
			//лазер
			else
			{
				//Переключиться на основной интерфейс
				HMI_ChangePage(PAGE_LASER);
				//Начать обнаружение
				EnableDoorCheck();
			}

			//Ответ главному компьютеру
			#if(SNAP_OLD_BASE != 1)
			if((HmiRequestStatus == STAT_PAUSE) || (HmiRequestStatus == STAT_PAUSE_ONLINE))
			{
				HMI_SendMachineStatusChange(0x04, 0);
			}
			#endif
			//очистить флаг
			HmiRequestStatus = STAT_IDLE;
			
			//четкая отметка
			PauseType = NonePause;
		}
	}
	else if((PrintingStatus == STAT_PAUSE) || (PrintingStatus == STAT_PAUSE_ONLINE))
	{
		//Продолжить печать
		if(PauseType == ContinuePrint)
		{
			//Печать на USB-диске
			if(PrintingStatus == STAT_PAUSE)
			{
				//Переход к местоположению файла
				if(card.FileSeek(PowerPanicData.FilePosition) == false)
				{
					card.initsd();
					//попробуй открыть файл
				  	card.openFile(PowerPanicData.FileName, true);
					card.FileSeek(PowerPanicData.FilePosition);
				}
				
				//Продолжить чтение файла
				card.sdprinting = true;
			}
			
			//Вернуться в спорт
			SystemRequestStop = false;

			if(MA_PRINT == MachineSelected)
			{			
				//Предварительная экструзия
				if(current_temperature[0] > 175)
				{
					//Режим относительного положения
					relative_mode = true;
					strcpy(tmpBuff, "G0 E15 F800");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);

					strcpy(tmpBuff, "G0 E-4 F2400");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					while(blocks_queued())manage_heater();
					relative_mode = false;
				}

				//Положение синхронизации оси E
				current_position[E_AXIS] = PowerPanicData.PositionData[E_AXIS];
				//set_destination_to_current();
				sync_plan_position();

				//Режим абсолютного положения
				relative_mode = false;
				
				//Перейти к точке восстановления
				sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F4000", PowerPanicData.PositionData[X_AXIS], PowerPanicData.PositionData[Y_AXIS]);
				current_command_args = tmpBuff;
				gcode_G0_G1(0);
				while(blocks_queued())manage_heater();
				
				sprintf(tmpBuff, "G0 Z%0.2f F4000", PowerPanicData.PositionData[Z_AXIS]);
				current_command_args = tmpBuff;
				gcode_G0_G1(0);
				while(blocks_queued())manage_heater();

				//Переключиться на основной интерфейс
				HMI_ChangePage(PAGE_PRINT);
				//Включить обнаружение разрыва материала
				#if(HAVE_FALIMENT_SENSOR == 1)
				EnableFilamentCheck();
				#endif
			}
			else if(MA_CNC == MachineSelected)
			{
				//Включите двигатель
				digitalWrite(LASER_PORT, LASER_PIN, HIGH);
				//задерживать
				tmpmills = millis() + 500;
				while(millis() < tmpmills);

				//Перейти к точке восстановления
				sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F4000", PowerPanicData.PositionData[X_AXIS], PowerPanicData.PositionData[Y_AXIS]);
				current_command_args = tmpBuff;
				gcode_G0_G1(0);
				while(blocks_queued())manage_heater();

				sprintf(tmpBuff, "G0 Z%0.2f F4000", PowerPanicData.PositionData[Z_AXIS]);
				current_command_args = tmpBuff;
				gcode_G0_G1(0);
				while(blocks_queued())manage_heater();
				
				//Переключиться на основной интерфейс
				HMI_ChangePage(PAGE_CNC);
			}
			else if((MA_LASER == MachineSelected) || (MA_LASER_350 == MachineSelected))
			{
				//Переключиться на основной интерфейс
				HMI_ChangePage(PAGE_LASER);
			}

			//Скорость восстановления
			SetG0FeedRate(PowerPanicData.TravelFeedRate);
			SetG1FeedRate(PowerPanicData.PrintFeedRate);

			//Очистить буфер
			FlushCommand();
			//Переключиться в режим абсолютных координат
			relative_mode = false;
			while(blocks_queued())manage_heater();
			//Отметить статус печати
			if(PrintingStatus == STAT_PAUSE)
				PrintingStatus = STAT_PRINTING;
			else
				PrintingStatus = STAT_PRINTING_ONLINE;

			#if(SNAP_OLD_BASE == 0)
			HMI_SendMachineStatusChange(0x05, 0);
			#endif
			
			#if(HAVE_FALIMENT_SENSOR == 1)
			EnableFilamentCheck();
			#endif
			//Начать обнаружение
			if(MA_LASER == MachineSelected)
				EnableDoorCheck();

			//четкая отметка
			PauseType = NonePause;
		}
	}
	
}


/******************************************
Завершить обработку печати
3 ситуации
1] После печати материалы необходимо вытолкнуть.
2] При печати в автономном режиме нажмите, чтобы остановить печать.
3】Отключение питания системы
*****************************************/
void FinishPrintProcess()
{
	static uint8_t StopSteps = 0;
	static uint32_t nextcheckMill;
	switch(StopSteps)
	{
		case 0:
			//отключение электроэнергии
			if(StopType == PowerPanicStop)
			{
				//В состоянии печати
				if((PrintingStatus == STAT_PRINTING) || (PrintingStatus == STAT_PRINTING_ONLINE))
				{
					//температура хранения
					for(int i=0;i<4;i++)
					{
						PowerPanicData.HeaterTamp[i] = target_temperature[i];
						target_temperature[i] = 0;
					}
					PowerPanicData.BedTamp = target_temperature_bed;
					target_temperature_bed = 0;

					//скорость вентилятора 
					//PowerPanicData.FanSpeed = TIM3->CCR1;
					//Сохранение модели
					PowerPanicData.MachineType = MachineSelected;

					//Настройка действительна
					PowerPanicData.Valid = 1;

					//имя файла
					strcpy(PowerPanicData.FileName, card.currentselectfilename);
					//Время печати
					PowerPanicData.accumulator = print_job_timer.duration();
					
					
					current_position[X_AXIS] = StableCounter[X_AXIS] / axis_steps_per_unit[X_AXIS];
					current_position[Y_AXIS] = StableCounter[Y_AXIS] / axis_steps_per_unit[Y_AXIS];
					current_position[Z_AXIS] = StableCounter[Z_AXIS] / axis_steps_per_unit[Z_AXIS];
					current_position[E_AXIS] = StableCounter[E_AXIS] / axis_steps_per_unit[E_AXIS];

					//Компенсация по оси Z
					float delZ = mbl.get_z(current_position[X_AXIS], current_position[Y_AXIS]);
					current_position[Z_AXIS] -= delZ;

					block_buffer_head = block_buffer_tail = 0;
					current_block = NULL;
					
					sync_plan_position();

					//координировать
					for(int i=0;i<NUM_AXIS;i++)
						PowerPanicData.PositionData[i] = current_position[i];

					//сохранить данные
					SaveBreakPointData();

					//Удалите запрос на остановку настройки. После очистки двигатель снова может двигаться.
					SystemRequestStop = false;

					//Отзывать
					if(MA_PRINT == MachineSelected)
					{
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS] - 1, 40, active_extruder);
						while(blocks_queued())manage_heater();

						//Поднять ось Z
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + 5, current_position[E_AXIS] - 1, 40, active_extruder);
						while(blocks_queued())manage_heater();
					}
				}
				while(1);
			}
			//Печать завершена
			else if(StopType == EndPrint)
			{
				HMI_ChangePage(PAGE_PROCESSING);
				StopSteps = 1;
				nextcheckMill = millis() + 1000;
				card.LastPercent = 100;
				//Обнаружение закрытой двери
				if(isLaserMachine(MachineSelected) == 1)
					DisableDoorCheck();
			}
			//Экран прекращает печать
			else if(StopType == ManualStop)
			{
				StopSteps = 1;
				ClearCommand();
				nextcheckMill = millis() + 1000;
				HMI_ChangePage(PAGE_PROCESSING);
				current_position[X_AXIS] = StableCounter[X_AXIS] / axis_steps_per_unit[X_AXIS];
				current_position[Y_AXIS] = StableCounter[Y_AXIS] / axis_steps_per_unit[Y_AXIS];
				current_position[Z_AXIS] = StableCounter[Z_AXIS] / axis_steps_per_unit[Z_AXIS];
				sync_plan_position();
				//Обнаружение закрытой двери
				if(isLaserMachine(MachineSelected) == 1)
					DisableDoorCheck();
			}
		break;

		case 1:
			if(nextcheckMill < millis())
			{
				StopSteps = 2;
			}
		break;

		case 2:
			while(CMD_BUFF_EMPTY() == false)
			{
				manage_heater();
				process_next_command();
				DispatchComandIndex();
			}
			while(blocks_queued())
			{
				manage_heater();
			}
			
			StopSteps = 3;
			//Отключено обнаружение отключения электроэнергии
			DisablePowerPanicCheck();
			#if(SNAP_OLD_BASE == 1)
			if(isLaserMachine(MachineSelected) == 1)
				DisableDoorCheck();
			#endif
		break;
		
		case 3:
			//Режим абсолютных координат
			relative_mode = false;
			//Удалите запрос на остановку настройки. После очистки двигатель снова может двигаться.
			SystemRequestStop = false;
			block_buffer_head = block_buffer_tail = 0;
			current_block = NULL;
			//Переключиться на основной интерфейс
			if(MachineSelected == MA_PRINT)
			{
				disable_all_heaters();
				//Z поднимается до самого высокого уровня
				if((HomeAfterReset[0] == true) && (HomeAfterReset[1] == true) && (HomeAfterReset[2] == true))
				{
					float Zpos;
					if((current_position[Z_AXIS] + 15) < (base_max_pos[Z_AXIS] + home_offset[Z_AXIS]))
						Zpos = current_position[Z_AXIS] + 15;
					else
						Zpos = base_max_pos[Z_AXIS] + home_offset[Z_AXIS];
					
					sprintf(tmpBuff, PSTR("G0 Z%0.2f F1000"), Zpos);
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					while(blocks_queued())manage_heater();
					
					#if(SNAP_OLD_BASE == 0)
					strcpy(tmpBuff, "G0 X0");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					while(blocks_queued())manage_heater();
					//X Y раз начало
					strcpy(tmpBuff, "G28 Y");
					current_command_args = tmpBuff;
					gcode_G28();
					#else
					//X Y раз начало
					strcpy(tmpBuff, "G28 X Y");
					current_command_args = tmpBuff;
					gcode_G28();
					while(blocks_queued())manage_heater();
					//Y Перейти в максимальное положение
					sprintf(tmpBuff, "G0 Y%0.2f F2000", base_max_pos[Y_AXIS] + home_offset[Y_AXIS] - 20);
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					while(blocks_queued())manage_heater();
					#endif
				}
				HMI_ChangePage(PAGE_PRINT);
			}
			else if((MachineSelected == MA_CNC) || (isLaserMachine(MachineSelected)) )
			{
				if( isLaserMachine(MachineSelected) )
				{
					SetLaserPower(0);
					strcpy(tmpBuff, "G0 X0 Y0 Z0 S0");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					while(blocks_queued())manage_heater();
					Config_RetrieveSettings();
                    //Очистить данные блока ШИМ
                    for(int i=0;i<BLOCK_BUFFER_SIZE;i++)
                        block_buffer[i].LaserPwm = 0;
				}
				else if(MachineSelected == MA_CNC)
				{
					digitalWrite(LASER_PORT, LASER_PIN, 1);
					strcpy(tmpBuff, "G0 Z15");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					while(blocks_queued())manage_heater();
					strcpy(tmpBuff, "G0 X0 Y0");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					while(blocks_queued())manage_heater();
					//Выключите шпиндель
					digitalWrite(LASER_PORT, LASER_PIN, 0);
				}
				HMI_ChangePage(PAGE_CNC);
			}
			StopSteps = 0;
			//Отметить состояние ожидания
			PrintingStatus = STAT_IDLE;
			//четкая отметка
			StopType = NoneStop;
			
		break;
	}
}

/********************************************
Самотестирование системы
********************************************/
void SystemSelfCheck(void)
{
	#if(0)
	#if(SNAP_OLD_BASE == 0)
	uint32_t tick;
	//Главный выключатель обнаружения Mos
	//Главный выключатель питания выключен
	MainPowerOff();

	//Нижний конец MOS открыт
	HEATER_0_PORT->BSRR = HEATER_0_PIN;
	HEATER_BED_PORT->BSRR = HEATER_BED_PIN;
	tick = millis() + 100;
	while(tick > millis());
	//Обнаружена непрерывность
	if((!(HEATER0_DET_PORT->IDR & HEATER0_DET_PIN)) && (!(HEATBED_DET_PORT->IDR & HEATBED_DET_PIN)))
	{
		SysFaultFlag |= FAULT_FLAG_LOAD;
		HEATER_0_PORT->BRR = HEATER_0_PIN;
		HEATER_BED_PORT->BRR = HEATER_BED_PIN;
		return;
	}

	//МОП-обнаружение нагревательной головки
	//Главный выключатель включен
	MainPowerOn();
	//Нижний конец MOS закрыт
	HEATER_0_PORT->BRR = HEATER_0_PIN;
	HEATER_BED_PORT->BRR = HEATER_BED_PIN;
	tick = millis() + 100;
	while(tick > millis());
	//Обнаружена непрерывность
	if(!(HEATER0_DET_PORT->IDR & HEATER0_DET_PIN))
	{
		SysFaultFlag |= FAULT_FLAG_HEATER0;
		HEATER_0_PORT->BRR = HEATER_0_PIN;
		HEATER_BED_PORT->BRR = HEATER_BED_PIN;
		MainPowerOff();
		return;
	}
	
	if(!(HEATBED_DET_PORT->IDR & HEATBED_DET_PIN))
	{
		SysFaultFlag |= FAULT_FLAG_BED;
		HEATER_0_PORT->BRR = HEATER_0_PIN;
		HEATER_BED_PORT->BRR = HEATER_BED_PIN;
		MainPowerOff();
		return;
	}

	//Включите нижний предел МО
	HEATER_0_PORT->BSRR = HEATER_0_PIN;
	HEATER_BED_PORT->BSRR = HEATER_BED_PIN;
	tick = millis() + 100;
	while(tick > millis());
	if(HEATER0_DET_PORT->IDR & HEATER0_DET_PIN)
	{
		SysFaultFlag |= FAULT_FLAG_HEATER0;
		HEATER_0_PORT->BRR = HEATER_0_PIN;
		HEATER_BED_PORT->BRR = HEATER_BED_PIN;
		MainPowerOff();
		return;
	}
	if(HEATBED_DET_PORT->IDR & HEATBED_DET_PIN)
	{
		SysFaultFlag |= FAULT_FLAG_BED;
		HEATER_0_PORT->BRR = HEATER_0_PIN;
		HEATER_BED_PORT->BRR = HEATER_BED_PIN;
		MainPowerOff();
		return;
	}

	#endif
	#endif

	HEATER_0_PORT->BRR = HEATER_0_PIN;
	HEATER_BED_PORT->BRR = HEATER_BED_PIN;
	MainPowerOn();
	StepperPowerOn();
}


/********************************************
Единая инициализация системы
********************************************/
void SystemGpioInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	//Скажем, шаг
#ifdef X_ENABLE_PIN
	X_ENABLE_WRITE(1);
#endif
#ifdef Y_ENABLE_PIN
	Y_ENABLE_WRITE(1);
#endif
#ifdef Z_ENABLE_PIN
	Z_ENABLE_WRITE(1);
#endif
#ifdef X2_ENABLE_PIN
	X2_ENABLE_WRITE(1);
#endif
#ifdef Y2_ENABLE_PIN
	Y2_ENABLE_WRITE(1);
#endif
#ifdef Z2_ENABLE_PIN
	Z2_ENABLE_WRITE(1);
#endif
	E0_ENABLE_WRITE(1);

	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIOB->BRR = GPIO_Pin_1;

#ifdef HEATER_0_PORT
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = HEATER_0_PIN;
	GPIO_Init(HEATER_0_PORT, &GPIO_InitStruct);
#endif

#ifdef HEATER_BED_PORT
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = HEATER_BED_PIN;
	GPIO_Init(HEATER_BED_PORT, &GPIO_InitStruct);
#endif

	//Вел
#ifdef LED_R_PORT
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = LED_R_PIN;
	GPIO_Init(LED_R_PORT, &GPIO_InitStruct);
#endif

#ifdef LED_B_PORT
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = LED_B_PIN;
	GPIO_Init(LED_B_PORT, &GPIO_InitStruct);
#endif

//вентилятор
/*
Вентилятору 1 может потребоваться регулировка скорости.
#ifdef H_FAN1_PORT
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = H_FAN1_PIN;
	GPIO_Init(H_FAN1_PORT, &GPIO_InitStruct);
#endif
*/
	
#ifdef H_FAN2_PORT
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = H_FAN2_PIN;
	GPIO_Init(H_FAN2_PORT, &GPIO_InitStruct);
#endif
	//шаговый двигатель
#ifdef X_STEP_PIN
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = X_STEP_PIN;
	GPIO_Init(X_STEP_PORT, &GPIO_InitStruct);
#endif

#ifdef X_DIR_PIN
	GPIO_InitStruct.GPIO_Pin = X_DIR_PIN;
	GPIO_Init(X_DIR_PORT, &GPIO_InitStruct);
#endif

#ifdef X_ENABLE_PIN
	GPIO_InitStruct.GPIO_Pin = X_ENABLE_PIN;
	GPIO_Init(X_ENABLE_PORT, &GPIO_InitStruct);
#endif

#ifdef Y_STEP_PIN
	GPIO_InitStruct.GPIO_Pin = Y_STEP_PIN;
	GPIO_Init(Y_STEP_PORT, &GPIO_InitStruct);
#endif

#ifdef Y_DIR_PIN
	GPIO_InitStruct.GPIO_Pin = Y_DIR_PIN;
	GPIO_Init(Y_DIR_PORT, &GPIO_InitStruct);
#endif

#ifdef Y_ENABLE_PIN
	GPIO_InitStruct.GPIO_Pin = Y_ENABLE_PIN;
	GPIO_Init(Y_ENABLE_PORT, &GPIO_InitStruct);
#endif

#ifdef Z_STEP_PIN
	GPIO_InitStruct.GPIO_Pin = Z_STEP_PIN;
	GPIO_Init(Z_STEP_PORT, &GPIO_InitStruct);
#endif

#ifdef Z_DIR_PIN
	GPIO_InitStruct.GPIO_Pin = Z_DIR_PIN;
	GPIO_Init(Z_DIR_PORT, &GPIO_InitStruct);
#endif

#ifdef Z_ENABLE_PIN
	GPIO_InitStruct.GPIO_Pin = Z_ENABLE_PIN;
	GPIO_Init(Z_ENABLE_PORT, &GPIO_InitStruct);
#endif

#ifdef X2_STEP_PIN
	GPIO_InitStruct.GPIO_Pin = ZX2_STEP_PIN;
	GPIO_Init(X2_STEP_PORT, &GPIO_InitStruct);
#endif

#ifdef X2_DIR_PIN
	GPIO_InitStruct.GPIO_Pin = X2_DIR_PIN;
	GPIO_Init(X2_DIR_PORT, &GPIO_InitStruct);
#endif

#ifdef X2_ENABLE_PIN
	GPIO_InitStruct.GPIO_Pin = X2_ENABLE_PIN;
	GPIO_Init(X2_ENABLE_PORT, &GPIO_InitStruct);
#endif

#ifdef Y2_STEP_PIN
	GPIO_InitStruct.GPIO_Pin = Y2_STEP_PIN;
	GPIO_Init(Y2_STEP_PORT, &GPIO_InitStruct);
#endif

#ifdef Y2_DIR_PIN
	GPIO_InitStruct.GPIO_Pin = Y2_DIR_PIN;
	GPIO_Init(Y2_DIR_PORT, &GPIO_InitStruct);
#endif

#ifdef Y2_ENABLE_PIN
	GPIO_InitStruct.GPIO_Pin = Y2_ENABLE_PIN;
	GPIO_Init(Y2_ENABLE_PORT, &GPIO_InitStruct);
#endif

#ifdef Z2_STEP_PIN
	GPIO_InitStruct.GPIO_Pin = Z2_STEP_PIN;
	GPIO_Init(Z2_STEP_PORT, &GPIO_InitStruct);
#endif

#ifdef Z2_DIR_PIN
	GPIO_InitStruct.GPIO_Pin = Z2_DIR_PIN;
	GPIO_Init(Z2_DIR_PORT, &GPIO_InitStruct);
#endif

#ifdef Z2_ENABLE_PIN
	GPIO_InitStruct.GPIO_Pin = Z2_ENABLE_PIN;
	GPIO_Init(Z2_ENABLE_PORT, &GPIO_InitStruct);
#endif

	E0_DIR_WRITE(0);
//Здесь инициализируется старый мастер
#if(SNAP_OLD_BASE == 1)
#ifdef E0_STEP_PORT
	GPIO_InitStruct.GPIO_Pin = E0_STEP_PIN;
	GPIO_Init(E0_STEP_PORT, &GPIO_InitStruct);
#endif

#ifdef E0_ENABLE_PORT
	GPIO_InitStruct.GPIO_Pin = E0_ENABLE_PIN;
	GPIO_Init(E0_ENABLE_PORT, &GPIO_InitStruct);
#endif

#ifdef E0_DIR_PORT
	GPIO_InitStruct.GPIO_Pin = E0_DIR_PIN;
	GPIO_Init(E0_DIR_PORT, &GPIO_InitStruct);
#endif
#endif



#ifdef UART1_SELK1_PORT
	GPIO_InitStruct.GPIO_Pin = UART1_SELK1_PIN;
	GPIO_Init(UART1_SELK1_PORT, &GPIO_InitStruct);
	UART1_SELK1_PORT->BRR = UART1_SELK1_PIN;
#endif

#ifdef STEPPER_POWER_SUPPLY_PORT
	GPIO_InitStruct.GPIO_Pin = STEPPER_POWER_SUPPLY_PIN;
	GPIO_Init(STEPPER_POWER_SUPPLY_PORT, &GPIO_InitStruct);
#endif

#ifdef MAIN_POWER_SUPPLY_PORT
	GPIO_InitStruct.GPIO_Pin = MAIN_POWER_SUPPLY_PIN;
	GPIO_Init(MAIN_POWER_SUPPLY_PORT, &GPIO_InitStruct);
#endif


	//концевой выключатель
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
#if(SNAP_OLD_BASE == 1)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
#else
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
#endif

#ifdef X_MIN_PORT
	GPIO_InitStruct.GPIO_Pin = X_MIN_PIN;
	GPIO_Init(X_MIN_PORT, &GPIO_InitStruct);
#endif

#ifdef Y_MIN_PORT
	GPIO_InitStruct.GPIO_Pin = Y_MIN_PIN;
	GPIO_Init(Y_MIN_PORT, &GPIO_InitStruct);
#endif

#ifdef Z_MIN_PORT
	GPIO_InitStruct.GPIO_Pin = Z_MIN_PIN;
	GPIO_Init(Z_MIN_PORT, &GPIO_InitStruct);
#endif

#ifdef X2_MIN_PORT
	GPIO_InitStruct.GPIO_Pin = X2_MIN_PIN;
	GPIO_Init(X2_MIN_PORT, &GPIO_InitStruct);
#endif

#ifdef Y2_MIN_PORT
	GPIO_InitStruct.GPIO_Pin = Y2_MIN_PIN;
	GPIO_Init(Y2_MIN_PORT, &GPIO_InitStruct);
#endif

#ifdef Z2_MIN_PORT 
	GPIO_InitStruct.GPIO_Pin = Z2_MIN_PIN;
	GPIO_Init(Z2_MIN_PORT, &GPIO_InitStruct);
#endif

#ifdef X_MAX_PORT
	GPIO_InitStruct.GPIO_Pin = X_MAX_PIN;
	GPIO_Init(X_MAX_PORT, &GPIO_InitStruct);
#endif

#ifdef Y_MAX_PORT
	GPIO_InitStruct.GPIO_Pin = Y_MAX_PIN;
	GPIO_Init(Y_MAX_PORT, &GPIO_InitStruct);
#endif

#ifdef Z_MAX_PORT
	GPIO_InitStruct.GPIO_Pin = Z_MAX_PIN;
	GPIO_Init(Z_MAX_PORT, &GPIO_InitStruct);
#endif

#ifdef X2_MAX_PORT
	GPIO_InitStruct.GPIO_Pin = X2_MAX_PIN;
	GPIO_Init(X2_MAX_PORT, &GPIO_InitStruct);
#endif

#ifdef Y2_MAX_PORT
	GPIO_InitStruct.GPIO_Pin = Y2_MAX_PIN;
	GPIO_Init(Y2_MAX_PORT, &GPIO_InitStruct);
#endif

#ifdef Z2_MAX_PORT
	GPIO_InitStruct.GPIO_Pin = Z2_MAX_PIN;
	GPIO_Init(Z2_MAX_PORT, &GPIO_InitStruct);
#endif

#ifdef HEATER0_DET_PORT
	GPIO_InitStruct.GPIO_Pin = HEATER0_DET_PIN;
	GPIO_Init(HEATER0_DET_PORT, &GPIO_InitStruct);
#endif

#ifdef HEATBED_DET_PORT
	GPIO_InitStruct.GPIO_Pin = HEATBED_DET_PIN;
	GPIO_Init(HEATBED_DET_PORT, &GPIO_InitStruct);
#endif

#ifdef PROBE_PORT
	GPIO_InitStruct.GPIO_Pin = PROBE_PIN;
	GPIO_Init(PROBE_PORT, &GPIO_InitStruct);
#endif

#ifdef PC_PLUG_DET_PORT
	GPIO_InitStruct.GPIO_Pin = PC_PLUG_DET_PIN;
	GPIO_Init(PC_PLUG_DET_PORT, &GPIO_InitStruct);
#endif

#if(HAVE_ANDROID == 1)
#ifdef ANDROID_DET_PORT
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = ANDROID_DET_PIN;
	GPIO_Init(ANDROID_DET_PORT, &GPIO_InitStruct);
#endif

#ifdef ANDROID_START_PORT
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = ANDROID_START_PIN;
	GPIO_Init(ANDROID_START_PORT, &GPIO_InitStruct);
#endif

#ifdef ANDROID_PWR_PORT
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = ANDROID_PWR_PIN;
	GPIO_Init(ANDROID_PWR_PORT, &GPIO_InitStruct);
#endif
#endif

#ifdef SHIELD_FAN_PORT
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = SHIELD_FAN_PIN;
	GPIO_Init(SHIELD_FAN_PORT, &GPIO_InitStruct);
	SHIELD_FAN_PORT->BRR = SHIELD_FAN_PIN;
#endif

#ifdef SHIELD_LED_PORT
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = SHIELD_LED_PIN;
	GPIO_Init(SHIELD_LED_PORT, &GPIO_InitStruct);
	SHIELD_LED_PORT->BRR = SHIELD_LED_PIN;
#endif

	millis_t tmptick;
	tmptick = millis() + 10;
	while(tmptick > millis());

//Выполнить обнаружение поломки головы
#if(SNAP_OLD_BASE == 0)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = E_ID_0_PIN;
	GPIO_Init(E_ID_0_PORT, &GPIO_InitStruct);

	
	GPIO_InitStruct.GPIO_Pin = E_ID_1_PIN;
	GPIO_Init(E_ID_1_PORT, &GPIO_InitStruct);

	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource14);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource15);

	EXTI_InitStruct.EXTI_Line = EXTI_Line14 | EXTI_Line15;
	EXTI_InitStruct.EXTI_LineCmd = DISABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStruct);

	DisableExecuterCheck();

	//дышащий свет
	DAC_InitTypeDef DAC_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	DMA_InitTypeDef DMA_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC | RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Prescaler = SystemCoreClock / 30000 - 1;
	TIM_TimeBaseInitStruct.TIM_Period = 999;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
	TIM_SelectOnePulseMode(TIM4, TIM_OPMode_Repetitive);
	TIM_DMACmd(TIM4, TIM_DMA_Update, ENABLE);

	DMA_InitStruct.DMA_BufferSize = sizeof(BreathLedTable) / sizeof(BreathLedTable[0]);
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)BreathLedTable;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(DAC_BASE | 0x1C);
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
	DMA_Init(DMA1_Channel7, &DMA_InitStruct);
	DMA_Cmd(DMA1_Channel7, ENABLE);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits7_0;
	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_InitStruct.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_Init(DAC_Channel_2, &DAC_InitStruct);
	DAC_Cmd(DAC_Channel_2, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
#endif

	#if(SNAP_OLD_BASE == 0)
	SystemPowerControl(POWER_OPTION_StepperPower, true);
	SystemPowerControl(POWER_OPTION_MainPower, true);
	#endif

    //Питание двигателя включено
    enable_x();
    enable_y();
    enable_z();
}

#if(SNAP_OLD_BASE == 0)

extern "C"
{
void EXTI15_10_IRQHandler(void)
{
	EXTI->PR = EXTI_Line14 | EXTI_Line15;
	PauseTriggle(ExecuterLostPause);
	//Отключить обнаружение
	DisableExecuterCheck();
}
}
#endif

