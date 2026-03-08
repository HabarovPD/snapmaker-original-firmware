#define _IN_POWERMANAGER
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
#include "SystemControl.h"
#include "Shield.h"
#include "LaserPowerTable.h"

#include "stm32f10x_conf.h"

//Flash  保存位置定义 Сохранить определение местоположения
#define FLASH_POWERPANIC_ADDR	0x802A800
#define FLASH_PAGE_SIZE	2048
#define FLASH_RECORD_PAGES	2
#define RECORD_COUNT_PER_PAGE	(FLASH_PAGE_SIZE / (sizeof(strPowerPanicSave) + 2))

#define THIS_EXTI_LINE	EXTI_Line0
#define THIS_IRQN		EXTI0_IRQn
#define SOURCE_PORT		GPIO_PortSourceGPIOE
#define SOURCE_PIN		GPIO_PinSource0
//中断入口需要查看.S  文件 Точку входа прерывания необходимо проверить в файле .S.

extern void sync_plan_position();
extern void gcode_M104();
extern void gcode_M109();
extern void gcode_M140();
extern void gcode_M190();
extern void gcode_G28();
extern void gcode_G0_G1(uint16_t Number);
extern void gcode_G92();

extern void set_current_to_destination();
extern void set_destination_to_current();

extern long pCounter_X,pCounter_Y,pCounter_Z,pCounter_E;
extern char* current_command, *current_command_args;
extern bool relative_mode;

static strPowerPanicSave tmpPowerPanicData;

//Следующая запись индекса данных Следующая запись индекса данных
uint32_t WriteIndex;

const uint32_t PortTable[] = 
{
	(uint32_t)GPIOA,
	(uint32_t)GPIOB,
	(uint32_t)GPIOC,
	(uint32_t)GPIOD,
	(uint32_t)GPIOE,
	(uint32_t)GPIOF
};

const uint32_t PortRccTable[] = 
{
	RCC_APB2Periph_GPIOA,
	RCC_APB2Periph_GPIOB,
	RCC_APB2Periph_GPIOC,
	RCC_APB2Periph_GPIOD,
	RCC_APB2Periph_GPIOE,
	RCC_APB2Periph_GPIOF
};


/********************************************
断电初始化 Инициализация выключения питания
********************************************/
void PowerManagerInit(void)
{
	#if(SNAP_OLD_BASE == 0)
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);           
	RCC_APB2PeriphClockCmd(PortRccTable[SOURCE_PORT], ENABLE);

	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = (1 << SOURCE_PIN);
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIO_EXTILineConfig(SOURCE_PORT, SOURCE_PIN);

	
	EXTI_InitStruct.EXTI_Line = THIS_EXTI_LINE;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel = THIS_IRQN;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
	
	DisablePowerPanicCheck();
	LoadBreakPointData();
	#endif
	
}

/********************************************
使能断电检测 Включить обнаружение сбоев питания
********************************************/
void EnablePowerPanicCheck()
{
	return;
	#if(SNAP_OLD_BASE != 1)
	if(PowerPanicCheckEnable == true)
	{
		EXTI->PR = THIS_EXTI_LINE;
		EXTI->IMR |= THIS_EXTI_LINE;
	}
	#endif
}

/********************************************
禁能断电检测 Обнаружение отключения питания
********************************************/
void DisablePowerPanicCheck()
{
	#if(SNAP_OLD_BASE != 1)
	if(PowerPanicCheckEnable == true)
	{
		EXTI->PR = THIS_EXTI_LINE;
		EXTI->IMR &= ~THIS_EXTI_LINE;
	}
	#endif
}

/********************************************
清空所有断电数据 Удалите все данные об отключении электроэнергии.
********************************************/
void ClearBreakPointData(void)
{
	uint32_t addr;

	addr = FLASH_POWERPANIC_ADDR;
	FLASH_Unlock();
	for(int i=0;i<FLASH_RECORD_PAGES;i++)
	{
		FLASH_ErasePage(addr);
		addr += 2048;
	}
	FLASH_Lock();
}

/********************************************
保存断电数据 Сохранение данных о сбоях электропитания
********************************************/
void SaveBreakPointData(void)
{
	uint32_t addr;
	uint32_t RecordSize;
	uint32_t checksum;
	uint32_t u32data;
	uint8_t *pBuff;
	
	//Размер записи
	RecordSize = (sizeof(strPowerPanicSave) + 8);

	//Напишите половину письма
	addr = (WriteIndex / RECORD_COUNT_PER_PAGE) * 2048 + (WriteIndex % RECORD_COUNT_PER_PAGE) * RecordSize + FLASH_POWERPANIC_ADDR;
	u32data = 0x5555;
	FLASH_Unlock();
	FLASH_ProgramWord(addr, u32data);

	//Адрес адрес
	addr = (WriteIndex / RECORD_COUNT_PER_PAGE) * 2048 + (WriteIndex % RECORD_COUNT_PER_PAGE) * RecordSize + 8 + FLASH_POWERPANIC_ADDR;
	
	pBuff = (uint8_t*)&PowerPanicData;
	PowerPanicData.CheckSum = 0;
	checksum = 0;
	for(int i=0;i<sizeof(strPowerPanicSave);i++)
		checksum += pBuff[i];
	PowerPanicData.CheckSum = checksum;

	
	for(int i=0;i<sizeof(strPowerPanicSave);i=i+4)
	{
		u32data = (pBuff[i + 3] << 24) | (pBuff[i + 2] << 16) | (pBuff[i + 1] << 8) | pBuff[i];
		FLASH_ProgramWord(addr, u32data);
		addr = addr + 4;
	}

	//Копировать полный идентификатор. Записать полный идентификатор.
	addr = (WriteIndex / RECORD_COUNT_PER_PAGE) * 2048 + (WriteIndex % RECORD_COUNT_PER_PAGE) * RecordSize + FLASH_POWERPANIC_ADDR + 4;
	u32data = 0x5555;
	FLASH_ProgramHalfWord(addr, u32data);
	FLASH_Lock();
}

/********************************************
标置断电数据无效 Неверные данные о сбое электропитания
********************************************/
void MaskBreadkPointData(void)
{
	uint32_t RecordSize;
	uint32_t TotalCount;
	uint32_t addr;
	uint16_t preIndex;
	
	//Размер записи
	RecordSize = (sizeof(strPowerPanicSave) + 8);
	//Запишите общее пространство
	TotalCount = RECORD_COUNT_PER_PAGE * FLASH_RECORD_PAGES;
	
	preIndex = (WriteIndex + TotalCount - 1) % sizeof(TotalCount);

	//Рассчитать адрес
	addr = (preIndex / RECORD_COUNT_PER_PAGE) * 2048 + (preIndex % RECORD_COUNT_PER_PAGE) * RecordSize + FLASH_POWERPANIC_ADDR;
	FLASH_Unlock();
	FLASH_ProgramWord(addr, 0);
	FLASH_Lock();
}

/********************************************
Чтение данных об отключении электроэнергии
Используйте метод сохранения цикла
Первый байт содержит настройку
Флаг: 0x0000 указывает, что данные были прочитаны и очищены.
       0x5555 указывает на действительные данные.
       0xFFFF означает пустой (доступный для записи)
Принцип поиска: начните с начала, чтобы найти пустой блок, а затем вернитесь из пустого блока, чтобы найти недействительный блок или действительный блок, то есть последний непустой блок.

Чтение данных после отключения питания
Использование циклического метода сохранения
Первый байт хранит флаг
Флаг: 0x0000 указывает на то, что данные были прочитаны и очищены.
0x5555 указывает на допустимые данные
0xFFFF указывает на нулевой (доступен для записей)
Принцип поиска: Начинаем поиск пустых блоков с самого начала,
	затем возвращаемся из пустых блоков для замены недопустимых
	или допустимых блоков, т.е. последнего непустого блока.
********************************************/
void LoadBreakPointData(void)
{
	uint8_t *pSrcBuff, *pDstBuff;
	uint32_t i;
	uint32_t tmpIndex;
	uint32_t addr;
	uint32_t RecordSize;
	uint32_t TotalCount;
	uint32_t Flag;

	//Размер записи
	RecordSize = (sizeof(strPowerPanicSave) + 8);
	//记录总空间 Общее пространство для записей
	TotalCount = RECORD_COUNT_PER_PAGE * FLASH_RECORD_PAGES;

	//查找空块 Найти пустые блоки
	for(i=0;i<TotalCount;i++)
	{
		//计算地址 Рассчитать адрес
		addr = (i / RECORD_COUNT_PER_PAGE) * 2048 + (i % RECORD_COUNT_PER_PAGE) * RecordSize + FLASH_POWERPANIC_ADDR;
		Flag = *((uint32_t*)addr);
		if(Flag == 0xffffffff)
		{
			tmpIndex = (i + TotalCount - 1) % TotalCount;
			break;
		}
	}

	//Найти последний непустой блок. Найти последний непустой блок.
	for(i=0;i<TotalCount;i++)
	{
		//计算地址 Рассчитать адрес
		addr = (tmpIndex / RECORD_COUNT_PER_PAGE) * 2048 + (tmpIndex % RECORD_COUNT_PER_PAGE) * RecordSize + FLASH_POWERPANIC_ADDR;
		Flag = *((uint32_t*)addr);
		if(Flag != 0xffffffff)
		{
			break;
		}
		tmpIndex = (tmpIndex + TotalCount - 1) % TotalCount;
	}

	//чтение набора флагов
	Flag = *((uint32_t*)addr);
	//Flash пуст, что указывает на отсутствие записи.
	if(Flag == 0xffffffff)
	{
		//Текущее местоположение в качестве места записи
		WriteIndex = 0;
		//Неверная настройка
		PowerPanicData.Valid = 0;
	}
	else
	{
		//адрес
		addr = (tmpIndex / RECORD_COUNT_PER_PAGE) * 2048 + (tmpIndex % RECORD_COUNT_PER_PAGE) * RecordSize + 8 + FLASH_POWERPANIC_ADDR;
		//Чтение данных 
		pSrcBuff = (uint8_t*)addr;
		pDstBuff = (uint8_t*)&PowerPanicData;
		for(int i=0;i<sizeof(strPowerPanicSave);i++)
			*pDstBuff++ = *pSrcBuff++;

		//проверять
		uint32_t Checksum;
		uint32_t tmpChecksum;
		tmpChecksum = PowerPanicData.CheckSum;
		PowerPanicData.CheckSum = 0;
		Checksum = 0;
		pSrcBuff = (uint8_t*)&PowerPanicData;
		for(int i=0;i<sizeof(strPowerPanicSave);i++)
			Checksum += pSrcBuff[i];
		//Checksum = Checksum - (uint8_t)(PowerPanicData.CheckSum >> 24) - (uint8_t)(PowerPanicData.CheckSum >> 16) - (uint8_t)(PowerPanicData.CheckSum >> 8) - (uint8_t)(PowerPanicData.CheckSum);

		#if(0)
		tmpPowerPanicData = PowerPanicData;

		tmpPowerPanicData.CheckSum = 0;
		Checksum = 0;
		pSrcBuff = (uint8_t*)&tmpPowerPanicData;
		for(int i=0;i<sizeof(strPowerPanicSave);i++)
			Checksum += pSrcBuff[i];
		tmpPowerPanicData.CheckSum = Checksum;

		#else

		#endif

		//Проверка не удалась
		#if(1)
		if(Checksum != tmpChecksum)
		{
			//очистить флаг
			FLASH_Unlock();
			addr = (tmpIndex / RECORD_COUNT_PER_PAGE) * 2048 + (WriteIndex % RECORD_COUNT_PER_PAGE) * RecordSize + FLASH_POWERPANIC_ADDR;
			FLASH_ProgramWord(addr, 0);
			FLASH_Lock();
			//Неверная настройка
			PowerPanicData.Valid = 0;
		}
		//Проверка прошла успешно
		else
		{
			//Настройка действительна
			PowerPanicData.Valid = 1;
		}
		#endif

		if(Flag != 0x5555)
			PowerPanicData.Valid = 0;
		
		//Следующая позиция в качестве позиции письма
		WriteIndex = (tmpIndex + 1) % TotalCount;
		//Первый адрес
		if((WriteIndex % RECORD_COUNT_PER_PAGE) == 0)
		{
			//дедукционная вспышка
			FLASH_Unlock();
			addr = (tmpIndex / RECORD_COUNT_PER_PAGE) * 2048 + (WriteIndex % RECORD_COUNT_PER_PAGE) * RecordSize + FLASH_POWERPANIC_ADDR;
			FLASH_ErasePage(addr);
			FLASH_Lock();
		}
	}
}

/********************************************
Остановись и снова сражайся
Err: указывает на метку ошибки.
         1 означает, что сохраненные данные недоступны
         2 означает, что открытие файла не удалось
Возвращаемое значение: true означает нормальный запуск, false означает сбой при запуске, причину сбоя можно проверить по значению Err.
********************************************/
bool PowerPanicResumeWork(uint8_t *Err)
{
	char tmpBuff[32];
	

	//Временное расположение файла, при перемещении обновятся данные в PowerPanicData.
	tmpPowerPanicData = PowerPanicData;

	//Данные действительны
	if(tmpPowerPanicData.Valid != 0)
	{
		card.initsd();
	  	//попробуй открыть файл
	  	card.openFile(PowerPanicData.FileName, true);
		
		//Файл успешно открыт
		if(card.isFileOpen() == true)
		{
			//обогрев
			if(tmpPowerPanicData.BedTamp > 20)
			{
				sprintf(tmpBuff, "M140 S%0.2f", tmpPowerPanicData.BedTamp);
				current_command_args = tmpBuff;
				gcode_M140();
			}

			//В настоящее время рассматривается только нагревательная головка 1.
			sprintf(tmpBuff, "M104 S%0.2f", tmpPowerPanicData.HeaterTamp[0]);
			current_command_args = tmpBuff;
			gcode_M104();
			
			//X Y раз начало
			strcpy(tmpBuff, "G28");
			current_command_args = tmpBuff;
			gcode_G28();
			
			//В ожидании отопления
			if(tmpPowerPanicData.BedTamp > 20)
			{
				sprintf(tmpBuff, "M190 S%0.2f", tmpPowerPanicData.BedTamp);
				current_command_args = tmpBuff;
				gcode_M190();
			}

			//В настоящее время рассматривается только нагревательная головка 1.
			sprintf(tmpBuff, "M109 S%0.2f", tmpPowerPanicData.HeaterTamp[0]);
			current_command_args = tmpBuff;
			gcode_M109();

			//Предварительная экструзия 
			//Относительный режим
			relative_mode = true;
			strcpy(tmpBuff, "G0 E25 F200");
			current_command_args = tmpBuff;
			gcode_G0_G1(0);
			st_synchronize();

			//Абсолютный режим
			relative_mode = false;

			//Обновление координат
			current_position[E_AXIS] = tmpPowerPanicData.PositionData[E_AXIS];
			//текущая_позиция[Z_AXIS] = tmpPowerPanicData.PositionData[Z_AXIS];
			set_destination_to_current();
			sync_plan_position();

			//Перейти к точке восстановления
			//******************Временно прекратите перемещение оси Z.
			sprintf(tmpBuff, "G0 Z%0.2f F2000", tmpPowerPanicData.PositionData[Z_AXIS] + 2);
			current_command_args = tmpBuff;
			gcode_G0_G1(0);
			st_synchronize();
			
			sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F4000", tmpPowerPanicData.PositionData[X_AXIS], tmpPowerPanicData.PositionData[Y_AXIS]);
			current_command_args = tmpBuff;
			gcode_G0_G1(0);
			st_synchronize();

			sprintf(tmpBuff, "G0 Z%0.2f F2000", tmpPowerPanicData.PositionData[Z_AXIS]);
			current_command_args = tmpBuff;
			gcode_G0_G1(0);
			st_synchronize();

			print_job_timer.start();
			print_job_timer.accumulator = tmpPowerPanicData.accumulator;

			//Переход к файлу
			card.FileSeek(tmpPowerPanicData.FilePosition);
			//Начать печать
			card.sdprinting = true;
			
			//Абсолютный режим
			relative_mode = false;

			current_position[X_AXIS] = tmpPowerPanicData.PositionData[X_AXIS];
			current_position[Y_AXIS] = tmpPowerPanicData.PositionData[Y_AXIS];
			current_position[Z_AXIS] = tmpPowerPanicData.PositionData[Z_AXIS];
			current_position[E_AXIS] = tmpPowerPanicData.PositionData[E_AXIS];
			set_destination_to_current();
			
			sync_plan_position();

			Sync_PCounter();

			//Скорость восстановления
			SetG0FeedRate(PowerPanicData.TravelFeedRate);
			SetG1FeedRate(PowerPanicData.PrintFeedRate);

			*Err = 0;
			SetCurrentPrinterStatus(STAT_PRINTING);
			EnablePowerPanicCheck();
			if(MA_LASER == MachineSelected)
				EnableDoorCheck();
			FlushCommand();
			return 0;
		}
		else
		{
			if(Err != NULL)
				*Err = 1;
			return false;
		}
	}
	else
	{
		if(Err != NULL)
			*Err = 1;
		return false;
	}
}

extern "C"
{
//Новая схема
#if(SNAP_OLD_BASE == 0)
void EXTI0_IRQHandler(void)
{
	PrinterStatus curStatus;
	EnterCritical(1);

	if(GPIO_Pin_0 != (GPIOE->IDR & GPIO_Pin_0))
	{
		EXTI->PR = THIS_EXTI_LINE;
		EXTI->IMR &= ~EXTI_IMR_MR0;
		//Выключите исполнительную головку и питание подогрева стола.
		MainPowerOff();
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;

		//Выключить питание
		StopTriggle(PowerPanicStop);
		//Выключите экструдер
		disable_e0();
		disable_x();
		disable_y();
		disable_z();
		curStatus = GetCurrentPrinterStatus();
		if(curStatus != STAT_IDLE)
		{
			current_block = NULL;
			block_buffer_head = block_buffer_tail = 0;
		}
		EnterCritical(0);
	}
}
#endif
}
