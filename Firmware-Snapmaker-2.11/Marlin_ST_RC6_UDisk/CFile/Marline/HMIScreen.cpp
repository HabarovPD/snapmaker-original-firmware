#define _IN_HMISCREEN
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
extern void gcode_M203();
extern void gcode_M206();
extern void gcode_M104();
extern char WaitUDisk(void);
extern void sync_plan_position();
extern long pCounter_X,pCounter_Y,pCounter_Z,pCounter_E;



#define FILE_PER_PAGE (6)
uint8_t RecvLen1;
volatile uint16_t FileCounts;
uint8_t FilePages;
volatile uint8_t CurFilePage;


char CardFileNameSelected[LONG_FILENAME_LENGTH];
char tmpCardFileNameSelected[LONG_FILENAME_LENGTH];
char PrintFileNameSelected[FILENAME_LENGTH];
long UIKey;
#if(SNAP_OLD_BASE == 1)
static uint16_t FileIndex;
char CardFileName[FILE_PER_PAGE][LONG_FILENAME_LENGTH];
char PrintFileName[FILE_PER_PAGE][FILENAME_LENGTH];
#else
char FileListBuff[1028];
#endif
//Временный буфер инструкций. После корректного парсинга здесь сохраняются инструкции.
static char tmpBuff[372];
//Статус вставки SD
static bool SdInsert = false;

static volatile uint8_t ReadBuff[512];
static volatile uint16_t ReadTail;
static volatile uint16_t ReadHead;

static volatile uint8_t WriteBuff[2048];
static volatile uint16_t WriteTail;
static volatile uint16_t WriteHead;


static uint16_t Packs;


//Проверьте, пуст ли буфер инструкций
#define CMD_BUFF_EMPTY()	(GetCommandBuff()>0?false:true)
//Перечисление USB-дисков выполнено успешно
#define IS_UDISK_INSERTED DiskInserted

#define sprintf_P sprintf

//Индекс точки выравнивания
uint8_t PointIndex;
float MeshPointZ[4];
uint16_t ZHomeOffsetIndex;


#define WaitingStepper() {while(blocks_queued()){manage_heater();}}



#if(1==HAVE_HMI)

#if(SNAP_OLD_BASE == 1)
void ScreenLockCheck(void)
{
	//Экран заблокирован
	if(ScreenLock == true)
	{
		//Никаких блоков движения и стеков G-кода.
		if((CMD_BUFF_EMPTY() == true) && (blocks_queued() == false))
		{
			//тайм-аут
			if(ScreenLockTick < millis())
			{
				ScreenLockTick = millis() + 2000;
				ScreenLock = false;
			}
		}
		//Имеется новый код или блокировка движения.
		else
		{
			ScreenLockTick = millis() + 2000;
		}
	}
}


short HmiGetCommand(unsigned char *pBuff)
{
	uint16_t tmplen;
	uint16_t tmphead;
	uint16_t tmptail;
	uint8_t i;
	uint8_t checksum;

	tmphead = ReadHead;
	tmptail = ReadTail;
	
	//Нет данных
	if (tmphead == tmptail)
	{
		return (short)-1;
	}

	/*
	если (ReadBuff[tmptail] != 0xAA)
	{
		tmptail = (tmptail + 1) % sizeof(ReadBuff);
		//Обновить указатель чтения
		ReadTail = tmptail; если (ReadTail == ReadHead)
			return (char)-1;
	}
	*/

	tmplen = (uint16_t)((tmphead + sizeof(ReadBuff)- tmptail) % sizeof(ReadBuff));
	
	//Длина данных достаточна
	while(tmplen > 8)
	{
		if(ReadBuff[tmptail] != 0xAA)
		{
			tmptail = (tmptail + 1) % sizeof(ReadBuff);
			tmplen--;
			//Обновить указатель чтения
			ReadTail = tmptail;
			continue;
		}
		if(ReadBuff[(tmptail + 1) % sizeof(ReadBuff)] != 0x55)
		{
			tmptail = (tmptail + 2) % sizeof(ReadBuff);
			tmplen = tmplen - 2;
			//Обновить указатель чтения
			ReadTail = tmptail;
			continue;
		}
		//Чтение длины пакета
		uint8_t commandLen = ReadBuff[(tmptail + 2) % sizeof(ReadBuff)];

		//достаточно буферизованных данных
		if(commandLen <= (tmplen - 4))
		{
			//Копировать данные
			for(i=0;i<(commandLen + 4);i++)
			{
				pBuff[i] = ReadBuff[tmptail];
				tmptail = (tmptail + 1) % sizeof(ReadBuff);
			}

			//Обновить указатель чтения
			ReadTail = tmptail;

			//проверять
			checksum = 0;
			for(i=0;i<commandLen;i++)
				checksum = checksum + pBuff[i + 3];
			checksum = 0x100 - checksum;

			if(checksum != (pBuff[commandLen + 3]))
				return (short)-1;
			//_rx_buffer->tail = _rx_buffer->head = 0;
			return commandLen + 4;
		}
		//Недостаточная длина данных
		else
		{
			return (short)-1;
		}
		
	}
	return (short)-1;
}
#else
short HmiGetCommand(unsigned char *pBuff)
{
	uint16_t tmplen;
	uint16_t tmphead;
	uint16_t tmptail;
	uint16_t i;
	uint32_t checksum;

	tmphead = ReadHead;
	tmptail = ReadTail;
	
	//Нет данных
	if (tmphead == tmptail)
	{
		return (short)-1;
	}

	tmplen = (uint16_t)((tmphead + sizeof(ReadBuff)- tmptail) % sizeof(ReadBuff));
	
	//Длина данных достаточна
	while(tmplen > 9)
	{
		if(ReadBuff[tmptail] != 0xAA)
		{
			tmptail = (tmptail + 1) % sizeof(ReadBuff);
			tmplen--;
			//Обновить указатель чтения
			ReadTail = tmptail;
			continue;
		}
		if(ReadBuff[(tmptail + 1) % sizeof(ReadBuff)] != 0x55)
		{
			tmptail = (tmptail + 2) % sizeof(ReadBuff);
			tmplen = tmplen - 2;
			//Обновить указатель чтения
			ReadTail = tmptail;
			continue;
		}
		//Чтение длины пакета
		uint16_t commandLen = (ReadBuff[(tmptail + 2) % sizeof(ReadBuff)] << 8) | ReadBuff[(tmptail + 3) % sizeof(ReadBuff)];
		//Ошибка проверки длины пакета
		if((((commandLen >> 8) & 0xff) ^ (commandLen & 0xff)) != ReadBuff[(tmptail + 5) % sizeof(ReadBuff)])
		{
			tmptail = (tmptail + 2) % sizeof(ReadBuff);
			tmplen = tmplen - 2;
			//Обновить указатель чтения
			ReadTail = tmptail;
			continue;
		}

		//достаточно буферизованных данных
		if(commandLen <= (tmplen - 8))
		{
			//Копировать данные
			for(i=0;i<(commandLen + 8);i++)
			{
				pBuff[i] = ReadBuff[tmptail];
				tmptail = (tmptail + 1) % sizeof(ReadBuff);
			}

			//Обновить указатель чтения
			ReadTail = tmptail;

			//проверять
			checksum = 0;
			for(i=0;i<(commandLen-1);i=i+2)
				checksum += (pBuff[i + 8] << 8) | pBuff[i + 9];

			//паритетное решение
			if(commandLen % 2)
				checksum += pBuff[commandLen + 8 - 1];

			while(checksum > 0xffff)
				checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);

			checksum = ~checksum;
			if((uint16_t)checksum != (uint16_t)((pBuff[6] << 8) | pBuff[7]))
				return (short)-1;
			//_rx_buffer->tail = _rx_buffer->head = 0;
			return commandLen + 8;
		}
		//Недостаточная длина данных
		else
		{
			return (short)-1;
		}
		
	}
	return (short)-1;
}

#endif

void HmiWriteData(char *pData, uint16_t len)
{
	uint16_t tmphead;
	
	while(len--)
	{
		tmphead = (WriteHead + 1) % sizeof(WriteBuff);
		while(tmphead == WriteTail);
		WriteBuff[WriteHead] = *pData++;
		WriteHead = tmphead;
	} 
	USART2->CR1 |= USART_CR1_TXEIE;
}

#if(SNAP_OLD_BASE == 1)
void HMICommandProcess(void)
{
	static uint8_t ShowFileCount = 0;
	static uint8_t SeekFileCount = 0;
	static uint8_t SeekingFile = 0;
	static uint8_t ShowFileIndex = 0;
	uint32_t ID;
	uint16_t SetTamp;
	uint8_t eventId;
	short i;
	short j;
	i = HmiGetCommand((unsigned char*)tmpBuff);

	if(i == (short)-1)
	{
		/*
		if((millis() -CheckComTick) > 1000)
		{
			CheckComTick = миллис();
			HMI_GetCurrentPage();
			если (НоКомКаунт < 10)
				НоКомКаунт++;
		}
		*/
	}
	//Протокол обновления, только старые контроллеры поддерживают обновление экрана
	else if(tmpBuff[7] == 0x18)
	{
		uint16_t PackRequest;
		ShieldLedOn();
		PackRequest = (uint16_t)((tmpBuff[3] << 8) | tmpBuff[4]);
		if(PackRequest == 0)
		{
			WaitUDisk();
			f_mount(&UpdateFatfs, "0:\\", 0);
			WaitUDisk();
			f_mount(&UpdateFatfs, "0:\\", 1);
			WaitUDisk();
			if(FR_OK == f_open(&UpdateFile, "0:\\Screen.bin", FA_READ))
			{
				Packs = UpdateFile.fsize / 240;
				if(UpdateFile.fsize % 240)
					Packs++;
				HMI_SendUpdatePack(PackRequest);
			}
			else
			{
				HMI_SendUpdateNoFile();
			}
		}
		else
		{
			if(PackRequest >= (Packs))
				HMI_SendUpdateComplete();
			else
				HMI_SendUpdatePack(PackRequest);
		}
	}
	//протокол экрана
	else
	{
		ShieldLedOn();
		eventId = tmpBuff[7];
		//кнопка
		if((eventId == 1) && (ScreenLock == false))
		{
			ID = (uint16_t)(tmpBuff[3] << 8) | (uint8_t)tmpBuff[4];
			
			switch((uint16_t)ID)
			{
				case BUTTON_MOTOR_HOME:
					//Двигатель не синхронизирован
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						HMI_ChangePage(8);
						mbl.active = true;
						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();

						relative_mode = false;

						strcpy(tmpBuff, "G0 Z0");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						HMI_ChangePage(0);
					}
				break;

				//Изменить материалы
				case BUTTON_CHANGE_FILAMENT:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
						HMI_ChangePage(PAGE_CHANGEFILAMENT);
				break;

				//Остановить отопление
				case BUTTON_STOP_HEAT:
				case BUTTON_END_HEAD:
				case BUTTON_STOP_HEAT_2:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						enqueue_and_echo_commands_P(PSTR("M104 S0.0"));
						HMI_ChangePage(PAGE_CONTROL);
					}
				break;

				case BUTTON_CANCEL_CHANGE_FILAMENT:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						enqueue_and_echo_commands_P(PSTR("M104 S0"));
						HMI_ChangePage(PAGE_CONTROL);
					}
				break;
				
				case BUTTON_CONFIRM_CHANGE_FILAMENT:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						enqueue_and_echo_commands_P(PSTR("M104 S200.0"));
						HMI_ChangePage(PAGE_WAITHEAT);
					}
				break;

				//температура плюс
				case BUTTON_TAMP_INC:
					SetTamp = (int)target_temperature[0];
					if(SetTamp < 250)
					{
						SetTamp = SetTamp + 10;
						sprintf(tmpBuff, "M104 S%d", SetTamp);
						current_command_args = tmpBuff;
						gcode_M104();
						HMI_ShowChangeFilamentTamp();
					}
				break;

				//температура минус
				case BUTTON_TAMP_DEC:
					SetTamp = (int)target_temperature[0];
					if(SetTamp > 200)
					{
						SetTamp = SetTamp - 10;
						sprintf(tmpBuff, "M104 S%d", SetTamp);
						current_command_args = tmpBuff;
						gcode_M104();
						HMI_ShowChangeFilamentTamp();
					}
				break;
			
				case BUTTON_FILE1:
					if(movesplanned() <= 3)
					{
						if(CardFileName[0][0] != 0)
						{
							strcpy(tmpCardFileNameSelected, CardFileName[0]);
							strcpy(PrintFileNameSelected, PrintFileName[0]);
							//Перейти к диалоговому окну подтверждения
							HMI_ChangePage(PAGE_CONFIRMPRINT);
						}
					}
				break;

				case BUTTON_FILE2:
					if(movesplanned() <= 3)
					{
						if(CardFileName[1][0] != 0)
						{
							strcpy(tmpCardFileNameSelected, CardFileName[1]);
							strcpy(PrintFileNameSelected, PrintFileName[1]);
							//Перейти к диалоговому окну подтверждения
							HMI_ChangePage(PAGE_CONFIRMPRINT);
						}
					}
				break;

				case BUTTON_FILE3:
					if(movesplanned() <= 3)
					{
						if(CardFileName[2][0] != 0)
						{
							strcpy(tmpCardFileNameSelected, CardFileName[2]);
							strcpy(PrintFileNameSelected, PrintFileName[2]);
							//Перейти к диалоговому окну подтверждения
							HMI_ChangePage(PAGE_CONFIRMPRINT);
						}
					}
				break;

				case BUTTON_FILE4:
					if(movesplanned() <= 3)
					{
						if(CardFileName[3][0] != 0)
						{
							strcpy(tmpCardFileNameSelected, CardFileName[3]);
							strcpy(PrintFileNameSelected, PrintFileName[3]);
							//Перейти к диалоговому окну подтверждения
							HMI_ChangePage(PAGE_CONFIRMPRINT);
						}
					}
				break;

				case BUTTON_FILE5:
					if(movesplanned() <= 3)
					{
						if(CardFileName[4][0] != 0)
						{
							strcpy(tmpCardFileNameSelected, CardFileName[4]);
							strcpy(PrintFileNameSelected, PrintFileName[4]);
							//Перейти к диалоговому окну подтверждения
							HMI_ChangePage(PAGE_CONFIRMPRINT);
						}
					}
				break;

				case BUTTON_FILE6:
					if(movesplanned() <= 3)
					{
						if(CardFileName[5][0] != 0)
						{
							strcpy(tmpCardFileNameSelected, CardFileName[5]);
							strcpy(PrintFileNameSelected, PrintFileName[5]);
							//Перейти к диалоговому окну подтверждения
							HMI_ChangePage(PAGE_CONFIRMPRINT);
						}
					}
				break;

				case 0x0a2f:
				case BUTTON_FILE_CONFIRM_UP:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						if(MachineSelected == MA_PRINT)
							HMI_ChangePage(PAGE_PRINT);
						else
							HMI_ChangePage(PAGE_CNC);
							
						//Очистить количество файлов
						FileCounts = 0;
						//Начать печать выбранных файлов
						//card.StartSelectPrint(PrintFileNameSelected);
						strcpy(CardFileNameSelected, tmpCardFileNameSelected);
						card.openFile(PrintFileNameSelected, true);
						print_job_timer.start();
						card.LastPercent = 0;
						HMI_ShowPrintTime();
						HMI_ShowProgressBar(0);
						ShieldFanOn();
						
						
						if((MachineSelected == MA_LASER) || (MachineSelected == MA_LASER_350))
						{
							//дверь закрыта
							#ifdef DOOR_PORT
							if(DoorCheckEnable == true)
							{
								if(READ(DOOR_PORT, DOOR_PIN) == false)
								{
									card.startFileprint();
								}
								else
								{
									PauseTriggle(ManualPause);
									SetCurrentPrinterStatus(STAT_PAUSE);
								}
							}
							else
							{
								card.startFileprint();
							}
							#else
							card.startFileprint();
							#endif
						}
						else if(MachineSelected == MA_PRINT)
						{
							card.startFileprint();
						}
						else
						{
							card.startFileprint();
						}
						PowerPanicData.FilePosition = 0;
						PowerPanicData.accumulator = 0;
						PowerPanicData.HeaterTamp[0] = 0;
						PowerPanicData.BedTamp = 0;
						PowerPanicData.PositionData[0] = 0;
						PowerPanicData.PositionData[1] = 0;
						PowerPanicData.PositionData[2] = 0;
						PowerPanicData.PositionData[3] = 0;
						PowerPanicData.GCodeSource = GCODE_SOURCE_UDISK;
						PowerPanicData.MachineType = MachineSelected;
						//Включить обнаружение
						EnablePowerPanicCheck();
						if(isLaserMachine(MachineSelected) == 1)
							EnableDoorCheck();
					}
				break;

				case BUTTON_FILE_CANCEL_UP:
					PrintFileNameSelected[0] = 0;
					if(MachineSelected == MA_PRINT)
						HMI_ChangePage(PAGE_PRINT);
					else
						HMI_ChangePage(PAGE_CNC);
					//Очистить количество файлов
					FileCounts = 0;
				break;
				

				//Файл показывает предыдущую страницу
				case BUTTON_NEXT_PAGE_UP:
					//Нет файла или поиск файлов
					if((FileCounts == 0) || (SeekingFile == 1) || (ShowFileCount > 0))
					{
						break;
					}
					if(CurFilePage < (FilePages - 1))
					{
						CurFilePage++;
						FileIndex = CurFilePage * FILE_PER_PAGE;
						//последняя страница
						if(CurFilePage == (FilePages - 1))
						{
							//нецелый
							if((FileCounts % FILE_PER_PAGE) != 0)
								SeekFileCount = (FileCounts % FILE_PER_PAGE);
							else
								SeekFileCount = FILE_PER_PAGE;
						}
						else
						{
							SeekFileCount = FILE_PER_PAGE;
						}
						//Очистить список
						for(i=0;i<FILE_PER_PAGE;i++)
						{
							CardFileName[i][0] = 0;
							PrintFileName[i][0] = 0;
						}
						//Определяет файл, который нужно найти
						SeekingFile = 1;
						//Количество отображаемых файлов совпадает с количеством искомых файлов.
						ShowFileCount = FILE_PER_PAGE;
						//Индекс файла отображения начинается с 0
						ShowFileIndex = 0;
					}
				break;


				//Показать следующую страницу файла
				case BUTTON_PREV_PAGE_UP:
					//Нет файла или поиск файлов
					if((FileCounts == 0) || (SeekingFile == 1) || (ShowFileCount > 0))
					{
						break;
					}
					if(CurFilePage > 0)
					{
						CurFilePage--;
						FileIndex = CurFilePage * FILE_PER_PAGE;
						if(FileCounts > FILE_PER_PAGE)
						{
							SeekFileCount = FILE_PER_PAGE;
						}
						else
						{
							SeekFileCount = FileCounts;
						}
						//Очистить список
						for(i=0;i<FILE_PER_PAGE;i++)
						{
							CardFileName[i][0] = 0;
							PrintFileName[i][0] = 0;
						}
						//Определяет файл, который нужно найти
						SeekingFile = 1;
						//Количество отображаемых файлов совпадает с количеством искомых файлов.
						ShowFileCount = FILE_PER_PAGE;
						//Индекс файла отображения начинается с 0
						ShowFileIndex = 0;
					}
				break;

				//Выйти из выбора файла
				case BUTTON_SELECT_FILE_BACK:
				case BUTTON_SELECT_FILE_HOME:
					if(MachineSelected == MA_PRINT)
						HMI_ChangePage(PAGE_PRINT);
					else
						HMI_ChangePage(PAGE_CNC);
					//Очистить количество файлов
					FileCounts = 0;
				break;

				//Выйти из интерфейса управления
				case BUTTON_CONTROL_BACK:
				case BUTTON_CNC_CONTROL_BACK:
					if(MachineSelected == MA_PRINT)
					{
						HMI_ChangePage(PAGE_PRINT);
					}
					else if( isLaserMachine(MachineSelected) )
					{
						SetLaserPower(0);
						HMI_ChangePage(PAGE_LASER);
					}
					else if(MachineSelected == MA_CNC)
					{
						digitalWrite(LASER_PORT, LASER_PIN, 0);
						HMI_ChangePage(PAGE_CNC);
					}
				break;

				//Кнопка открытия файла
				case BUTTON_PRINT_SHOW_FILE:
				case BUTTON_CNC_SHOW_FILE:
					//if(card.isFileOpen() == false)
					if(GetCurrentPrinterStatus() == STAT_IDLE)
					{
						//SERIAL_PROTOCOLLNPGM("Button Push!\r\n");
						if(IS_UDISK_INSERTED)
						{
							card.initsd();
							SERIAL_PROTOCOLLNPGM("Card Insert!\r\n");
							if(card.cardOK)
							{
								SERIAL_PROTOCOLLNPGM("Card Inited");
								HMI_ChangePage(PAGE_FILELIST);
							}
						}
					}
					else
					{
						SERIAL_PROTOCOLLNPGM("Sd is printing\r\n");
					}			
				break;

				//движущаяся ось
				case BUTTON_MOVE_AXIS:
					//Двигатель не синхронизирован
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						{
							HMI_ChangePage(PAGE_MOVE100);
							relative_mode = true;
							HMICommandSave = 1;
						}
					}
				break;

				//Переместить ось, выход
				case BUTTON_MOVE_AXIS_BACK_001:
				case BUTTON_MOVE_AXIS_BACK_010:
				case BUTTON_MOVE_AXIS_BACK_100:
					//Двигатель не синхронизирован
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						if(MA_PRINT == MachineSelected)
						{
							HMI_ChangePage(PAGE_CONTROL);
						}
						else
						{
							HMI_ChangePage(PAGE_CNCCONTROL);
							if( isLaserMachine(MachineSelected) )
							{
								//Выключить лазер, ЧПУ
								SetLaserPower(0);
								LastLaserPwm = 0;
								digitalWrite(LASER_PORT, LASER_PIN, 0);
							}
						}
						relative_mode = false;
						HMICommandSave = 0;
					}
				break;
				
				case BUTTON_MOVE_AXIS_HOME_001:
				case BUTTON_MOVE_AXIS_HOME_010:
				case BUTTON_MOVE_AXIS_HOME_100:
					//Двигатель не синхронизирован
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						if(MA_PRINT == MachineSelected)
						{
							HMI_ChangePage(PAGE_PRINT);
						}
						else
						{
							HMI_ChangePage(PAGE_CNC);
							if( isLaserMachine(MachineSelected) )
							{
								//Выключить лазер, ЧПУ
								SetLaserPower(0);
								LastLaserPwm = 0;
							}
							else if(MA_CNC == MachineSelected)
							{
								digitalWrite(LASER_PORT, LASER_PIN, 0);
							}
						}

						relative_mode = false;
						HMICommandSave = 0;
					}
				break;
				
				//Двигайтесь в отрицательном направлении
				case BUTTON_XL_01:
					relative_mode = true;
					strcpy(tmpBuff, "G0 X-0.1 F3000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					WaitingStepper ();
					relative_mode = false;
				break;

				case BUTTON_YL_01:
					relative_mode = true;
					strcpy(tmpBuff, "G0 Y-0.1 F3000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					WaitingStepper ();
					relative_mode = false;
				break;

				case BUTTON_ZL_01:
					relative_mode = true;
					strcpy(tmpBuff, "G0 Z-0.1 F3000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					WaitingStepper ();
					relative_mode = false;
				break;

				case BUTTON_XL_10:
					relative_mode = true;
					strcpy(tmpBuff, "G0 X-1 F3000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					WaitingStepper ();
					relative_mode = false;
				break;

				case BUTTON_YL_10:
					relative_mode = true;
					strcpy(tmpBuff, "G0 Y-1 F3000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					WaitingStepper ();
					relative_mode = false;
				break;

				case BUTTON_ZL_10:
					relative_mode = true;
					strcpy(tmpBuff, "G0 Z-1 F3000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					WaitingStepper ();
					relative_mode = false;
				break;

				case BUTTON_XL_100:
					relative_mode = true;
					strcpy(tmpBuff, "G0 X-10 F3000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					WaitingStepper ();
					relative_mode = false;
				break;

				case BUTTON_YL_100:
					relative_mode = true;
					strcpy(tmpBuff, "G0 Y-10 F3000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					WaitingStepper ();
					relative_mode = false;
				break;

				case BUTTON_ZL_100:
					relative_mode = true;
					strcpy(tmpBuff, "G0 Z-10 F3000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					WaitingStepper ();
					relative_mode = false;
				break;

				//движение в положительном направлении
				case BUTTON_XR_01:
					relative_mode = true;
					strcpy(tmpBuff, "G0 X0.1 F3000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					WaitingStepper ();
					relative_mode = false;
				break;

				case BUTTON_YR_01:
					relative_mode = true;
					strcpy(tmpBuff, "G0 Y0.1 F3000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					WaitingStepper ();
					relative_mode = false;
				break;

				case BUTTON_ZR_01:
					relative_mode = true;
					strcpy(tmpBuff, "G0 Z0.1 F3000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					WaitingStepper ();
					relative_mode = false;
				break;

				case BUTTON_XR_10:
					relative_mode = true;
					strcpy(tmpBuff, "G0 X1 F3000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					WaitingStepper ();
					relative_mode = false;
				break;

				case BUTTON_YR_10:
					relative_mode = true;
					strcpy(tmpBuff, "G0 Y1 F3000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					WaitingStepper ();
					relative_mode = false;
				break;

				case BUTTON_ZR_10:
					relative_mode = true;
					strcpy(tmpBuff, "G0 Z1 F3000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					WaitingStepper ();
                    relative_mode = false;
				break;

				case BUTTON_XR_100:
					relative_mode = true;
					strcpy(tmpBuff, "G0 X10 F3000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					WaitingStepper ();
					relative_mode = false;
				break;

				case BUTTON_YR_100:
					relative_mode = true;
					strcpy(tmpBuff, "G0 Y10 F3000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					WaitingStepper ();
					relative_mode = false;
				break;

				case BUTTON_ZR_100:
					relative_mode = true;
					strcpy(tmpBuff, "G0 Z10 F3000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					WaitingStepper ();
					relative_mode = false;
				break;

				//контроль
				case BUTTON_CONTROLL:
					//if(card.isFileOpen() == false)
					if(GetCurrentPrinterStatus() == STAT_IDLE)
					{
						HMI_ChangePage(PAGE_CONTROL);
					}
				break;

				//настраивать
				case BUTTON_SETTING:
					//if(card.isFileOpen() == false)
					if(GetCurrentPrinterStatus() == STAT_IDLE)
					{
						HMI_ChangePage(PAGE_SETTING);
					}
				break;

				case BUTTON_SETTING_BACK:
					HMI_ChangePage(PAGE_PRINT);
				break;

				//Со смещением
				case BUTTON_ZOFFSET_SETUP:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						HMI_ChangePage(PAGE_ZCENTER);
						mbl.active = false;
						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();
						st_synchronize();

						//Компенсация неудачного выравнивания
						mbl.active = false;

						strcpy(tmpBuff, "G0 Z0 F1000");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						//Установите координаты точки уровня моря
						sprintf(tmpBuff, "G92 Z%0.4f", -home_offset[Z_AXIS]);
						current_command_args = tmpBuff;
						gcode_G92();
						//Относительный режим
						relative_mode = true;

						//После инициализации это точка отсчета
						PointIndex = 2;
						
						HMICommandSave = 1;

						MeshPointZ[0] = mbl.z_values[0][0] - home_offset[Z_AXIS];
						MeshPointZ[2] = mbl.z_values[1][0] - home_offset[Z_AXIS];
						MeshPointZ[1] = mbl.z_values[0][1] - home_offset[Z_AXIS];
						MeshPointZ[3] = mbl.z_values[1][1] - home_offset[Z_AXIS];

						HMI_ChangePage(PAGE_CALIBRATE1);
					}
					
				break;

				case BUTTON_ZOFFSET_HOME:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						HMI_ChangePage(PAGE_WAITHOME);
						relative_mode = false;

						mbl.active = true;
						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();
						st_synchronize();

						strcpy(tmpBuff, "G0 Z0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						HMI_ChangePage(PAGE_PRINT); 
						HMICommandSave = 0;
						//Переключиться в режим абсолютного положения
						relative_mode = false;
					}
				break;

				case BUTTON_CALIBRATE1_BACK:
				case BUTTON_CALIBRATE2_BACK:
				case BUTTON_CALIBRATE3_BACK:
				case BUTTON_ZOFFSET_BACK:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						HMI_ChangePage(PAGE_WAITHOME);
						relative_mode = false;

						mbl.active = true;
						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();
						st_synchronize();

						strcpy(tmpBuff, "G0 Z0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						HMI_ChangePage(PAGE_SETTING); 
						HMICommandSave = 0;
						//Переключиться в режим абсолютного положения
						relative_mode = false;
					}
				break;

				case BUTTON_CALIBRATE1_SAVE:
				case BUTTON_CALIBRATE2_SAVE:
				case BUTTON_CALIBRATE3_SAVE:
				case BUTTON_ZOFFSET_CONFIRM:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Интерфейс ожидания переключения
						HMI_ChangePage(PAGE_ZCENTER);

						//П1
						mbl.set_z(0, 1, MeshPointZ[2]);

						//П2
						mbl.set_z(1, 1, MeshPointZ[3]);
						
						//Продавать
						mbl.set_z(1, 0, MeshPointZ[1]);

						//P4
						mbl.set_z(0, 0, MeshPointZ[0]);

						//Установить zoffset
						float X, Y;
						X = (MESH_MAX_X + MESH_MIN_X) / 2.0f;
						Y = (MESH_MAX_Y + MESH_MIN_Y) / 2.0f;
						float DelZ = mbl.get_z(X, Y);

						sprintf(tmpBuff, "M206 Z%0.4f", -(DelZ));
						current_command_args = tmpBuff;
						gcode_M206();

						//П1
						mbl.set_z(0, 1, MeshPointZ[2] - DelZ);

						//П2
						mbl.set_z(1, 1, MeshPointZ[3] - DelZ);
						
						//Продавать
						mbl.set_z(1, 0, MeshPointZ[1] - DelZ);

						//P4
						mbl.set_z(0, 0, MeshPointZ[0] - DelZ);

						//начало вращения
						mbl.active = true;
						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();

						//сохранить данные
						Config_StoreSettings();
						HMICommandSave = 0;
						//Переключиться в режим абсолютного положения
						relative_mode = false;

						//Ось Z переходит в 0
						strcpy(tmpBuff, "G0 Z0"); 
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//Переключить основной интерфейс
						HMI_ChangePage(PAGE_PRINT);
					}
				break;
				
				case BUTTON_ZOFFSET_UP:
					strcpy(tmpBuff, "G0 Z0.05");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					st_synchronize();
					MeshPointZ[PointIndex] = current_position[Z_AXIS];
				break;

				case BUTTON_ZOFFSET_DOWN:
					strcpy(tmpBuff, "G0 Z-0.05");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					st_synchronize();
					MeshPointZ[PointIndex] = current_position[Z_AXIS];		
				break;

				case BUTTON_CALIBRATE_UP_005:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z0.05");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
					}
				break;

				case BUTTON_CALIBRATE_UP_020:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z0.2");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
					}
				break;

				case BUTTON_CALIBRATE_UP_050:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z0.5");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
					}
				break;

				case BUTTON_CALIBRATE_DOWN_005:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z-0.05");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
					}
				break;

				case BUTTON_CALIBRATE_DOWN_020:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z-0.2");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
					}
				break;

				case BUTTON_CALIBRATE_DOWN_050:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z-0.5");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
					}
				break;

				//Сбросить параметры выравнивания
				case BUTTON_CALIBRATE1_RESET:
				case BUTTON_CALIBRATE2_RESET:
				case BUTTON_CALIBRATE3_RESET:
					HMI_ChangePage(PAGE_RESETCALIBRATE);
				break;

				//Подтвердите сброс параметров выравнивания
				case BUTTON_CALIBRATE_RESET_CONFIRM:
					HMI_ChangePage(PAGE_WAITHOME);
					home_offset[Z_AXIS] = -8;
					mbl.reset();
					Config_StoreSettings();
					mbl.active = false;
					strcpy(tmpBuff, "G28");
					current_command_args = tmpBuff;
					gcode_G28();
					st_synchronize();
					//Компенсация неудачного выравнивания
					mbl.active = false;
					//Режим абсолютных координат
					relative_mode = false;

					//Назад к 0 часам
					strcpy(tmpBuff, "G0 Z0 F1000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					st_synchronize();

					sprintf(tmpBuff, "G92 Z%0.4f", -home_offset[Z_AXIS]);
					current_command_args = tmpBuff;
					gcode_G92();
					
					HMI_ChangePage(PAGE_CALIBRATE1);

					//Режим относительных координат
					relative_mode = true;

					//После инициализации это точка отсчета
					PointIndex = 2;
					
					HMICommandSave = 1;

					MeshPointZ[0] = -home_offset[Z_AXIS];
					MeshPointZ[1] = -home_offset[Z_AXIS];
					MeshPointZ[2] = -home_offset[Z_AXIS];
					MeshPointZ[3] = -home_offset[Z_AXIS];
					
					
					HMI_ChangePage(PAGE_CALIBRATE1);
				break;

				//Отменить сброс параметров выравнивания
				case BUTTON_CALIBRATE_RESET_BACK:
					HMI_ChangePage(PAGE_CALIBRATE1);
				break;

				case BUTTON_CALIBRATE1_P1:
				case BUTTON_CALIBRATE2_P1:
				case BUTTON_CALIBRATE3_P1:
				case BUTTON_ZOFFSET_P1:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						//Поднимитесь на 2 мм
						strcpy(tmpBuff, "G0 Z2.0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//Переключиться в режим абсолютного положения
						relative_mode = false;
						//в положение точки регулировки
						int X, Y;
						X = MESH_MIN_X;
						Y = MESH_MAX_Y;
						sprintf(tmpBuff, "G0 X%d Y%d F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						//Переключиться в режим абсолютного положения
						relative_mode = false;
						sprintf(tmpBuff, "G0 Z%0.2f", MeshPointZ[2]);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						
						//Переключиться в режим относительного положения
						relative_mode = false;
						
						//Совместим с M421 S2.
						PointIndex = 2;
					}
				break;

				case BUTTON_CALIBRATE1_P2:
				case BUTTON_CALIBRATE2_P2:
				case BUTTON_CALIBRATE3_P2:
				case BUTTON_ZOFFSET_P2:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						//Поднимитесь на 2 мм
						strcpy(tmpBuff, "G0 Z2.0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						
						//Переключиться в режим абсолютного положения
						relative_mode = false;
						//в положение точки регулировки
						int X, Y;
						X = MESH_MAX_X;
						Y = MESH_MAX_Y;
						sprintf(tmpBuff, "G0 X%d Y%d F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						//Переключиться в режим абсолютного положения
						relative_mode = false;
						sprintf(tmpBuff, "G0 Z%0.2f", MeshPointZ[3]);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//Переключиться в режим относительного положения
						relative_mode = true;
						//Совместим с M421 S3.
						PointIndex = 3;
					}
				break;

				case BUTTON_CALIBRATE1_P3:
				case BUTTON_CALIBRATE2_P3:
				case BUTTON_CALIBRATE3_P3:
				case BUTTON_ZOFFSET_P3:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						//Поднимитесь на 2 мм
						strcpy(tmpBuff, "G0 Z2.0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						
						//Переключиться в режим абсолютного положения
						relative_mode = false;
						//в положение точки регулировки
						int X, Y;
						X = MESH_MAX_X;
						Y = MESH_MIN_Y;
						sprintf(tmpBuff, "G0 X%d Y%d F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						//Переключиться в режим абсолютного положения
						relative_mode = false;
						sprintf(tmpBuff, "G0 Z%0.2f", MeshPointZ[1]);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//Переключиться в режим относительного положения
						relative_mode = true;
						//Соответствует M421 S1
						PointIndex = 1;
					}
				break;

				case BUTTON_CALIBRATE1_P4:
				case BUTTON_CALIBRATE2_P4:
				case BUTTON_CALIBRATE3_P4:
				case BUTTON_ZOFFSET_P4:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						//Поднимитесь на 2 мм
						strcpy(tmpBuff, "G0 Z2.0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						
						//Переключиться в режим абсолютного положения
						relative_mode = false;
						//в положение точки регулировки
						int X, Y;
						X = MESH_MIN_X;
						Y = MESH_MIN_Y;
						sprintf(tmpBuff, "G0 X%d Y%d F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						//Переключиться в режим абсолютного положения
						relative_mode = false;
						sprintf(tmpBuff, "G0 Z%0.2f", MeshPointZ[0]);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//Переключиться в режим относительного положения
						relative_mode = true;
						//Соответствует M421 S0
						PointIndex = 0;
					}
				break;

				//Остановить печать
				case BUTTON_STOP_PRINT:
					//Печать SD-карты
					//if(card.isFileOpen() == true)
					if(GetCurrentPrinterStatus() == STAT_PRINTING)
					{
						//Переключиться на остановку интерфейса
						HMI_ChangePage(PAGE_CONFIRMSTOP);
					}
				break;
				
				//Остановить печать подтверждения
				case BUTTON_STOP_PRINT_CONFIRM:
					//Эта кнопка действительна только в интерфейсе остановки работы.
					//если (ТекущийHMIPage == PAGE_CONFIRMSTOP)
					{
						//Оффлайн печать
						if(card.isFileOpen() == true)
						{
						    card.sdprinting = false;
						    card.closefile();
							if(MA_CNC != MachineSelected)
								disable_all_heaters();
							cancel_heatup = true;
							StopTriggle(ManualStop);
							HMI_ChangePage(PAGE_PROCESSING);
							//Отключено обнаружение отключения электроэнергии
							DisablePowerPanicCheck();
						}
						//Распечатать онлайн
						else if(GetCurrentPrinterStatus() == STAT_PRINTING)
						{
							if(MA_CNC != MachineSelected)
								disable_all_heaters();
							cancel_heatup = true;
							StopTriggle(ManualStop);
							HMI_ChangePage(PAGE_PROCESSING);
							//Отключено обнаружение отключения электроэнергии
							DisablePowerPanicCheck();
						}
					}
				break;

				//Остановить печать Отмена
				case BUTTON_STOP_PRINT_CANCEL:
					//Эта кнопка действительна только в интерфейсе остановки работы.
					//если (ТекущийHMIPage == PAGE_CONFIRMSTOP)
					{
						if(MachineSelected == MA_PRINT)
						{
							HMI_ChangePage(PAGE_PRINT);
						}
						else
						{
							HMI_ChangePage(PAGE_CNC);
						}
					}
				break;

				//Приостановить печать
				case BUTTON_PAUSE_PRINT:
					//Файл открыт
					if(card.isFileOpen() == true)
					{
						PauseTriggle(ManualPause);
						HMI_ChangePage(PAGE_PROCESSING);
					}
				break;

				//Продолжить печать
				case BUTTON_CONTINUE_PRINT:
					//Файл открыт
					if(card.isFileOpen() == true)
					{
						if(GetCurrentPrinterStatus() ==  STAT_PAUSE)
						{
							PauseResume();
							HMI_ChangePage(PAGE_PROCESSING);
						}
					}
				break;

				//ЧПУ 
				case BUTTON_CNC_PAUSE:
					//Печать на USB-диске
					if(card.isFileOpen() == true)
					{
						//Отключить обнаружение
						DisableDoorCheck();
						PauseTriggle(ManualPause); 
						current_block = NULL;
						//блок_буфер_хед = блок_буфер_хвост = 0;
						//лазер выключен
						SetLaserPower(0);
						//Включить обнаружение
						if(isLaserMachine(MachineSelected))
							EnableDoorCheck();
					}
				break;

				case BUTTON_CNC_RUN:
					//if(card.isFileOpen() == true)
					{
						//Функция ЧПУ
						if(MachineSelected == MA_CNC)
						{
							if(GetCurrentPrinterStatus() ==  STAT_PAUSE)
								PauseResume();
						}
						//Функция лазера
						else
						{
							if(GetCurrentPrinterStatus() ==  STAT_PAUSE)
							{
									//Отключить обнаружение
									//DisableDoorCheck();
									if(DoorCheckEnable == false)
									{
										PauseResume();
									}
									else 
									{
										if(READ(DOOR_PORT,DOOR_PIN) == false)
										{
											PauseResume();
										}
									}
									//Включить обнаружение
									EnableDoorCheck();
							}
							else
								PauseResume();
						}
					}
				break;

				case BUTTON_CNC_STOP:
					//Печать
					if((GetCurrentPrinterStatus() == STAT_PRINTING) || (GetCurrentPrinterStatus() == STAT_PAUSE))
					//if(card.isFileOpen() == true)
					{
						HMI_ChangePage(PAGE_CONFIRMSTOP);
					}
				break;

				case BUTTON_CNC_CONTROL:
					//if(card.isFileOpen() == false)
					if(GetCurrentPrinterStatus() == STAT_IDLE)
						HMI_ChangePage(PAGE_CNCCONTROL);
				break;

				case BUTTON_CNC_ON:
					if((MA_LASER == MachineSelected) || (MA_LASER_350 == MachineSelected))
					{
						#ifdef DOOR_PORT
						if((READ(DOOR_PORT, DOOR_PIN) == false) || (GetCurrentPrinterStatus() == STAT_IDLE))
							SetLaserPower(laserPercent);
						#else
						SetLaserPower(laserPercent);
						#endif
					}
					else if(MA_CNC == MachineSelected)
						digitalWrite(LASER_PORT, LASER_PIN, HIGH);
				break;

				case BUTTON_CNC_OFF:
					if( isLaserMachine(MachineSelected) )
						SetLaserPower(0);
					else if(MA_CNC == MachineSelected)
						digitalWrite(LASER_PORT, LASER_PIN, LOW);
				break;

				case BUTTON_CNC_SET_ORIGIN:
					strcpy(tmpBuff, "G92 X0 Y0 Z0");
					current_command_args = tmpBuff;
					gcode_G92();
					HMI_ChangePage(PAGE_ORIGINSAVED);
					millis_t tick;
					tick = millis();
					while(millis() < (tick + 800));
					HMI_ChangePage(PAGE_CNCCONTROL);
					//После установки начала координат удерживайте текущую
					StepperKeepCurrent = true;
					enable_x();
					enable_y();
					enable_z();
				break;

				case BUTTON_CNC_MOVE_AXIS:
					if( isLaserMachine(MachineSelected) )
					{
						SetLaserLowPower();
						LastLaserPwm = laserpwm;
					}
					HMI_ChangePage(PAGE_MOVE100);
				break;

				//Часто задаваемые вопросы
				case BUTTON_FAQ:
					HMI_ChangePage(PAGE_FAQ);
				break;

				//Часто задаваемые вопросы Возврат
				case BUTTON_FAQ_BACK:
					HMI_ChangePage(PAGE_SETTING);
				break;

				//Часто задаваемые вопросы Вернуться на главную страницу
				case BUTTON_FAQ_HOME:
					HMI_ChangePage(PAGE_PRINT);
				break;

				//О
				case BUTTON_ABOUT:
					HMI_ChangePage(PAGE_ABOUT);
				break;

				//О возвратах интерфейса
				case BUTTON_ABOUT_BACK:
					HMI_ChangePage(PAGE_SETTING);
				break;

				//Об интерфейсе возвращается на главную страницу
				case BUTTON_ABOUT_HOME:
					HMI_ChangePage(PAGE_PRINT);
				break;

				//Об экране обновления интерфейса
				case BUTTON_ABOUT_UPDATE:
					HMI_SendScreenUpdate();
				break;
				
				default:
					sprintf(tmpBuff, PSTR("Unknow:%x"), ID);
					SERIAL_ECHO(tmpBuff);
				break;
			}
		}
		//Инициализация интерфейса
		else if(eventId == 2)
		{
			ID = (uint16_t)tmpBuff[3];

			uint8_t i;
			//uint8_t tmpPage;
			//tmpPage = CurrentHMIPage;
			CurrentHMIPage = 0xff;
			//Получить идентификатор интерфейса
			for(i=0;i<sizeof(PageID);i++)
			{
				if(ID == PageID[i])
				{
					CurrentHMIPage = i;
					break;
				}
			}

			//Сохраните последний идентификатор интерфейса
			switch(CurrentHMIPage)
			{
				case PAGE_PRINT:
					//SD-печать
					if(card.isFileOpen()== true)
					{
						HMI_StopEnable(true);
						HMI_SetControlEnable(false);
						HMI_SetSettingEnable(false);
						//Этот тест был удален
						if(!(IS_UDISK_INSERTED))
						{
							//значок настроек
							HMI_SetTFIcon(false);
						}
						else
						{
							//значок настроек
							HMI_SetTFIcon(true);
						}
						//Печать
						if(card.sdprinting == true)
						{
							HMI_ChangePrinterStatus(1);
						}
						//Приостановить печать
						else
						{
							HMI_ChangePrinterStatus(2);
						}
					}
					//Нет SD-печати
					else
					{
						//Этот тест был удален
						if(!(IS_UDISK_INSERTED))
						{
							//значок настроек
							HMI_SetTFIcon(false);
						}
						else
						{
							//значок настроек
							HMI_SetTFIcon(true);
						}
					}
					HMI_ShowPrintTime();
					HMI_ShowPrintFileName(CardFileNameSelected);
					HMI_ShowProgressBar(card.LastPercent);
				break;

				case PAGE_ABOUT:
					HMI_SetAboutLabels();
				break;

				case PAGE_CNC:
					//SD-печать
					if(card.isFileOpen()== true)
					{
						HMI_StopEnable(true);
						HMI_PauseEnable(true);
						HMI_SetControlEnable(false);
						HMI_ChangePrinterStatus(1);
					}
					else
					{
						//Этот тест был удален
						if(!(IS_UDISK_INSERTED))
						{
							//значок настроек
							HMI_SetTFIcon(false);
						}
						else
						{
							//значок настроек
							HMI_SetTFIcon(true);
						}
					}
					//Мощность дисплея
					HMI_ShowPrintTime();
					HMI_ShowLaserPower(laserPercent);
					HMI_ShowPrintFileName(CardFileNameSelected);
					HMI_ShowProgressBar(card.LastPercent);
				break;
				
				case PAGE_FILELIST:
					//Инициализация диалога выбора файла
					if(FileCounts == 0)
					{
						if(card.cardOK)
						{
							FileCounts = card.getnrfilenames(FileFilter);
							FilePages = FileCounts / FILE_PER_PAGE;
							if((FileCounts % FILE_PER_PAGE) != 0)
								FilePages++;
							CurFilePage = 0;
						}
					}
					FileIndex = CurFilePage * FILE_PER_PAGE;
					if(FileCounts > FILE_PER_PAGE)
						SeekFileCount = FILE_PER_PAGE;
					else
						SeekFileCount = FileCounts;
					//Очистить список
					for(i=0;i<FILE_PER_PAGE;i++)
					{
						CardFileName[i][0] = 0;
						PrintFileName[i][0] = 0;
					}
					//Количество отображаемых файлов совпадает с количеством искомых файлов.
					ShowFileCount = FILE_PER_PAGE;
					//Определяет файл, который нужно найти
					SeekingFile = 1;
					//Индекс файла отображения начинается с 0
					ShowFileIndex = 0;		
				break;

				case PAGE_LOGO:
					if(MA_PRINT == MachineSelected)
						HMI_ChangePage(PAGE_PRINT);
					else
						HMI_ChangePage(PAGE_CNC);
				break;

				case PAGE_CONTROL:
				break;

				case PAGE_CNCCONTROL:
					if(MA_CNC == MachineSelected)
					{
						if(LASER_PORT->ODR & LASER_PIN)
							HMI_ChangeLaserCNCStatus(2);
						else
							HMI_ChangeLaserCNCStatus(4);
					}
					else if( isLaserMachine(MachineSelected) )
					{
						if(GetLaserPower() != 0)
							if(LASER_PORT->ODR & LASER_PIN)
								HMI_ChangeLaserCNCStatus(2);
						else
							HMI_ChangeLaserCNCStatus(4);
					}
				break;

				case PAGE_CONFIRMPRINT:
					HMI_ShowConfirmFileName(tmpCardFileNameSelected);
				break;

			}
		}
		//Страница запроса
		else if(eventId == 3)
		{
			ID = (uint16_t)tmpBuff[3];
			uint8_t i;
			uint8_t tmpPage;
			tmpPage = CurrentHMIPage;
			CurrentHMIPage = 0xff;
			//Получить идентификатор интерфейса
			for(i=0;i<sizeof(PageID);i++)
			{
				if(ID == PageID[i])
				{
					CurrentHMIPage = i;
					break;
				}
			}

			//Горячая замена экрана
			if(CurrentHMIPage == PAGE_LOGO)
			{
				switch(tmpPage)
				{
					case PAGE_ZOFFSET:
					case PAGE_MOVE001:
					case PAGE_MOVE010:
					case PAGE_MOVE100:
						HMI_ChangePage(tmpPage);
					break;

					default:
						if(MA_PRINT == MachineSelected)
							HMI_ChangePage(PAGE_PRINT);
						else
							HMI_ChangePage(PAGE_CNC);
					break;
				}
			}
		}
	}
	

	//Список файлов
	if(SeekingFile == 1)
	{
		//Число файлов, подлежащих перечислению, больше 0, что указывает на то, что его все еще необходимо перечислить.
		if(SeekFileCount > 0)
		{
			card.getfilename(FileIndex++, NULL, FileFilter);
			//Предотвратить переполнение
			card.longFilename[LONG_FILENAME_LENGTH - 1] = 0;
			if(card.longFilename[0] != 0)
			{
				for(j=0;j<strlen(card.longFilename);j++)
				{
					CardFileName[ShowFileIndex][j] = card.longFilename[j];
				}
			}
			else
			{
				for(j=0;j<strlen(card.filename);j++)
				{
					CardFileName[ShowFileIndex][j] = card.filename[j];
				}
			}
			CardFileName[ShowFileIndex][j] = 0;

			//Предотвратить переполнение
			card.filename[FILENAME_LENGTH - 1] = 0;
			for(j=0;j<strlen(card.filename);j++)
			{
				PrintFileName[ShowFileIndex][j] = card.filename[j];
			}
			PrintFileName[ShowFileIndex][j] = 0;
			//Уменьшить количество элементов в списке на 1.
			SeekFileCount--;
			//Индекс плюс 1
			ShowFileIndex++;
		}
		//Нет необходимости перечислять файлы
		else
		{
			//Очистить настройки из перечисления файлов
			SeekingFile = 0;
			//Индекс файла отображения начинается с 0
			ShowFileIndex = 0;
		}
	}
	//Файлы больше не отображаются в списке, а количество отображаемых файлов больше 0, что указывает на наличие файлов, которые необходимо отобразить.
	else if(ShowFileCount > 0)
	{
		HMI_ShowFileName(ShowFileIndex, CardFileName[ShowFileIndex]);
		//Номер дисплея минус 1
		ShowFileCount--;
		//Индекс отображения увеличен на 1.
		ShowFileIndex++;
	}
}
#else

//Очистить данные последовательного порта
void HMI_BuffFlush(void)
{
	ReadHead = ReadTail = 0;
	WriteHead = WriteTail = 0;
}

void HMICommandProcess(void)
{
	static uint16_t ListFileOffset = 0;
	PrinterStatus CurStatus;
	uint32_t ID;
	uint32_t u32Value;
	uint16_t SetTamp;
	uint8_t eventId;
	short i;
	uint16_t j;
	uint16_t cmdLen;
	i = HmiGetCommand((unsigned char*)tmpBuff);

	if(i == (short)-1)
	{
		/*
		if((millis() -CheckComTick) > 1000)
		{
			CheckComTick = миллис();
			HMI_GetCurrentPage();
			если (НоКомКаунт < 10)
				НоКомКаунт++;
		}
		*/
	}
	//протокол экрана
	else
	{
		cmdLen = (tmpBuff[2] << 8) | tmpBuff[3];
		
		eventId = tmpBuff[8];
		//кнопка
		if(eventId == 9)
		{
			ID = (uint16_t)(tmpBuff[9] << 8) | (uint8_t)tmpBuff[10];
			ShieldLedOn();
			
			switch((uint16_t)ID)
			{
				case BUTTON_MOTOR_HOME:
					//Двигатель не синхронизирован
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						HMI_ChangePage(8);
						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();
						st_synchronize();
						HMI_ChangePage(0);
					}
				break;

				case BUTTON_MOVE_AXIS_BACK_010:
					//Двигатель не синхронизирован
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						if(MA_PRINT == MachineSelected)
						{
							HMI_ChangePage(PAGE_CONTROL);
						}
						else
						{
							HMI_ChangePage(PAGE_CNCCONTROL);
							if( isLaserMachine(MachineSelected) )
							{
								//Выключить лазер, ЧПУ
								SetLaserPower(0);
								digitalWrite(LASER_PORT, LASER_PIN, 0);
							}
						}
						relative_mode = false;
						HMICommandSave = 0;
					}
				break;

				
				
				//Со смещением
				case BUTTON_ZOFFSET_SETUP:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						HMI_ChangePage(PAGE_ZCENTER);

						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();
						st_synchronize();

						mbl.active = false;

						//Режим абсолютных координат
						relative_mode = false;

						sprintf(tmpBuff, "G0 X%0.2f Y%0.2f", home_offset[X_AXIS], home_offset[Y_AXIS]);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						strcpy(tmpBuff, "G0 Z0");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						
						//Установите координаты точки уровня моря
						sprintf(tmpBuff, "G92 Z%0.2f", -home_offset[Z_AXIS]);
						current_command_args = tmpBuff;
						gcode_G92();
						
						HMI_ChangePage(PAGE_CALIBRATE1);

						relative_mode = true;

						//После инициализации это точка отсчета
						PointIndex = 2;
						
						HMICommandSave = 1;

						MeshPointZ[0] = mbl.z_values[0][0] - home_offset[Z_AXIS];
						MeshPointZ[2] = mbl.z_values[1][0] - home_offset[Z_AXIS];
						MeshPointZ[1] = mbl.z_values[0][1] - home_offset[Z_AXIS];
						MeshPointZ[3] = mbl.z_values[1][1] - home_offset[Z_AXIS];

					}
					
				break;
				

				case BUTTON_ZOFFSET_HOME:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						HMI_ChangePage(PAGE_WAITHOME);
						relative_mode = false;

						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();
						st_synchronize();

						strcpy(tmpBuff, "G0 Z0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						HMI_ChangePage(PAGE_PRINT); 
						HMICommandSave = 0;
						//Переключиться в режим абсолютного положения
						relative_mode = false;
					}
				break;

				case BUTTON_CALIBRATE1_BACK:
				case BUTTON_CALIBRATE2_BACK:
				case BUTTON_CALIBRATE3_BACK:
				case BUTTON_ZOFFSET_BACK:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						HMI_ChangePage(PAGE_WAITHOME);
						relative_mode = false;

						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();
						st_synchronize();

						strcpy(tmpBuff, "G0 Z0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						HMI_ChangePage(PAGE_SETTING); 
						HMICommandSave = 0;
						//Переключиться в режим абсолютного положения
						relative_mode = false;
					}
				break;

				case BUTTON_CALIBRATE1_SAVE:
				case BUTTON_CALIBRATE2_SAVE:
				case BUTTON_CALIBRATE3_SAVE:
				case BUTTON_ZOFFSET_CONFIRM:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
						//Интерфейс ожидания переключения
						HMI_ChangePage(PAGE_ZCENTER);

						//П1
						mbl.set_z(0, 1, MeshPointZ[2]);

						//П2
						mbl.set_z(1, 1, MeshPointZ[3]);
						
						//Продавать
						mbl.set_z(1, 0, MeshPointZ[1]);

						//P4
						mbl.set_z(0, 0, MeshPointZ[0]);

						//Установить zoffset
						float X, Y;
						X = (MESH_MAX_X + MESH_MIN_X) / 2.0f;
						Y = (MESH_MAX_Y + MESH_MIN_Y) / 2.0f;
						float DelZ = mbl.get_z(X, Y);

						sprintf(tmpBuff, "M206 Z%0.2f", -(DelZ));
						current_command_args = tmpBuff;
						gcode_M206();

						//П1
						mbl.set_z(0, 1, MeshPointZ[2] - DelZ);

						//П2
						mbl.set_z(1, 1, MeshPointZ[3] - DelZ);
						
						//Продавать
						mbl.set_z(1, 0, MeshPointZ[1] - DelZ);

						//P4
						mbl.set_z(0, 0, MeshPointZ[0] - DelZ);

						//начало вращения
						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();
						st_synchronize();

						//Переключиться в режим абсолютного положения
						relative_mode = false;
						//Переключить основной интерфейс
						HMI_ChangePage(PAGE_PRINT);

						//сохранить данные
						Config_StoreSettings();
						HMICommandSave = 0;
						//Переключиться в режим абсолютного положения
						relative_mode = false;
					}
				break;
				
				case BUTTON_ZOFFSET_UP:
					strcpy(tmpBuff, "G0 Z0.05");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					st_synchronize();
					MeshPointZ[PointIndex] = current_position[Z_AXIS];
				break;

				case BUTTON_ZOFFSET_DOWN:
					strcpy(tmpBuff, "G0 Z-0.05");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					st_synchronize();
					MeshPointZ[PointIndex] = current_position[Z_AXIS];		
				break;

				case BUTTON_CALIBRATE_UP_005:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z0.05");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
					}
				break;

				case BUTTON_CALIBRATE_UP_020:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z0.2");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
					}
				break;

				case BUTTON_CALIBRATE_UP_050:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z0.5");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
					}
				break;

				case BUTTON_CALIBRATE_DOWN_005:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z-0.05");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
					}
				break;

				case BUTTON_CALIBRATE_DOWN_020:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z-0.2");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
					}
				break;

				case BUTTON_CALIBRATE_DOWN_050:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z-0.5");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
					}
				break;

				//Сбросить параметры выравнивания
				case BUTTON_CALIBRATE1_RESET:
				case BUTTON_CALIBRATE2_RESET:
				case BUTTON_CALIBRATE3_RESET:
					HMI_ChangePage(PAGE_RESETCALIBRATE);
				break;

				//Подтвердите сброс параметров выравнивания
				case BUTTON_CALIBRATE_RESET_CONFIRM:
					HMI_ChangePage(PAGE_WAITHOME);
					home_offset[Z_AXIS] = -8;
					mbl.reset();
					Config_StoreSettings();
					strcpy(tmpBuff, "G28");
					current_command_args = tmpBuff;
					gcode_G28();
					st_synchronize();
					
					sprintf(tmpBuff, "G92 Z%0.2f", -home_offset[Z_AXIS]);
					current_command_args = tmpBuff;
					gcode_G92();
					
					
					HMI_ChangePage(PAGE_CALIBRATE1);

					relative_mode = true;

					//После инициализации это точка отсчета
					PointIndex = 2;
					
					HMICommandSave = 1;

					MeshPointZ[0] = -home_offset[Z_AXIS];
					MeshPointZ[1] = -home_offset[Z_AXIS];
					MeshPointZ[2] = -home_offset[Z_AXIS];
					MeshPointZ[3] = -home_offset[Z_AXIS];
					
					//Компенсация неудачного выравнивания
					mbl.active = false;
					HMI_ChangePage(PAGE_CALIBRATE1);
				break;

				//Отменить сброс параметров выравнивания
				case BUTTON_CALIBRATE_RESET_BACK:
					HMI_ChangePage(PAGE_CALIBRATE1);
				break;

				case BUTTON_CALIBRATE1_P1:
				case BUTTON_CALIBRATE2_P1:
				case BUTTON_CALIBRATE3_P1:
				case BUTTON_ZOFFSET_P1:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
						//Режим относительного положения
						relative_mode = true;
						//Поднимитесь на 2 мм
						strcpy(tmpBuff, "G0 Z2.0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//Переключиться в режим абсолютного положения
						relative_mode = false;
						//в положение точки регулировки
						int X, Y;
						X = MESH_MIN_X;
						Y = MESH_MAX_Y;
						sprintf(tmpBuff, "G0 X%d Y%d F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						//Переключиться в режим абсолютного положения
						relative_mode = false;
						sprintf(tmpBuff, "G0 Z%0.2f", MeshPointZ[2]);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						
						//Переключиться в режим относительного положения
						relative_mode = false;
						
						//Совместим с M421 S2.
						PointIndex = 2;
					}
				break;

				case BUTTON_CALIBRATE1_P2:
				case BUTTON_CALIBRATE2_P2:
				case BUTTON_CALIBRATE3_P2:
				case BUTTON_ZOFFSET_P2:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
						//Режим относительного положения
						relative_mode = true;
						//Поднимитесь на 2 мм
						strcpy(tmpBuff, "G0 Z2.0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						
						//Переключиться в режим абсолютного положения
						relative_mode = false;
						//в положение точки регулировки
						int X, Y;
						X = MESH_MAX_X;
						Y = MESH_MAX_Y;
						sprintf(tmpBuff, "G0 X%d Y%d F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						//Переключиться в режим абсолютного положения
						relative_mode = false;
						sprintf(tmpBuff, "G0 Z%0.2f", MeshPointZ[3]);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//Переключиться в режим относительного положения
						relative_mode = true;
						//Совместим с M421 S3.
						PointIndex = 3;
					}
				break;

				case BUTTON_CALIBRATE1_P3:
				case BUTTON_CALIBRATE2_P3:
				case BUTTON_CALIBRATE3_P3:
				case BUTTON_ZOFFSET_P3:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
						//Режим относительного положения
						relative_mode = true;
						//Поднимитесь на 2 мм
						strcpy(tmpBuff, "G0 Z2.0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						
						//Переключиться в режим абсолютного положения
						relative_mode = false;
						//в положение точки регулировки
						int X, Y;
						X = MESH_MAX_X;
						Y = MESH_MIN_Y;
						sprintf(tmpBuff, "G0 X%d Y%d F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						//Переключиться в режим абсолютного положения
						relative_mode = false;
						sprintf(tmpBuff, "G0 Z%0.2f", MeshPointZ[1]);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//Переключиться в режим относительного положения
						relative_mode = true;
						//Соответствует M421 S1
						PointIndex = 1;
					}
				break;

				case BUTTON_CALIBRATE1_P4:
				case BUTTON_CALIBRATE2_P4:
				case BUTTON_CALIBRATE3_P4:
				case BUTTON_ZOFFSET_P4:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
						//Режим относительного положения
						relative_mode = true;
						//Поднимитесь на 2 мм
						strcpy(tmpBuff, "G0 Z2.0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						
						//Переключиться в режим абсолютного положения
						relative_mode = false;
						//в положение точки регулировки
						int X, Y;
						X = MESH_MIN_X;
						Y = MESH_MIN_Y;
						sprintf(tmpBuff, "G0 X%d Y%d F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						//Переключиться в режим абсолютного положения
						relative_mode = false;
						sprintf(tmpBuff, "G0 Z%0.2f", MeshPointZ[0]);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//Переключиться в режим относительного положения
						relative_mode = true;
						//Соответствует M421 S0
						PointIndex = 0;
					}
				break;
				
				//Остановить печать подтверждения
				case BUTTON_STOP_PRINT_CONFIRM:
					//Эта кнопка действительна только в интерфейсе остановки работы.
					//если (ТекущийHMIPage == PAGE_CONFIRMSTOP)
					{
						//Оффлайн печать
						CurStatus = GetCurrentPrinterStatus();
						if((CurStatus == STAT_PRINTING) || (CurStatus == STAT_PAUSE))
						{
						    card.sdprinting = false;
						    card.closefile();
							if(MA_CNC != MachineSelected)
								disable_all_heaters();
							cancel_heatup = true;
							StopTriggle(ManualStop);
							HMI_ChangePage(PAGE_PROCESSING);
							//Отключено обнаружение отключения электроэнергии
							DisablePowerPanicCheck();
							if(MA_LASER == MachineSelected)
								DisableDoorCheck();
						}
						//Распечатать онлайн
						else if((CurStatus == STAT_PRINTING_ONLINE) || (CurStatus == STAT_PAUSE_ONLINE))
						{
							if(MA_CNC != MachineSelected)
								disable_all_heaters();
							cancel_heatup = true;
							StopTriggle(ManualStop);
							HMI_ChangePage(PAGE_PROCESSING);
							//Отключено обнаружение отключения электроэнергии
							DisablePowerPanicCheck();
							if(MA_LASER == MachineSelected)
								DisableDoorCheck();
						}
					}
				break;


				//Приостановить печать
				case BUTTON_PAUSE_PRINT:
					//Файл открыт
					if(GetCurrentPrinterStatus() == STAT_PRINTING)
					{
						PauseTriggle(ManualPause);
					}	
					//Оффлайн печать
					else if(GetCurrentPrinterStatus() == STAT_PRINTING_ONLINE)
					{
						PauseTriggle(ManualPause);
					}
				break;

				//Продолжить печать
				case BUTTON_CONTINUE_PRINT:
					//Печать на USB-диске
					if(GetCurrentPrinterStatus() ==  STAT_PAUSE)
					{
						//Лазерная исполнительная головка
						if(isLaserMachine(MachineSelected) == 1)
						{
							#ifdef DOOR_PORT
							//Отключить обнаружение
							//DisableDoorCheck();
							//дверь закрыта
							if(READ(DOOR_PORT,DOOR_PIN) == false)
							{
								//Начать печать
								PauseResume();
							}
							//Включить обнаружение
							EnableDoorCheck();
							#else
							PauseResume();
							#endif
						}
						else
						{
							PauseResume();
						}
						//Очистить флаг неисправности
						SysFaultFlag &= ~FAULT_FLAG_FILAMENT;
					}
					//Оффлайн печать
					else if(GetCurrentPrinterStatus() == STAT_PAUSE_ONLINE)
					{
						HMI_SendContinuePrint();
						//Очистить флаг неисправности
						SysFaultFlag &= ~FAULT_FLAG_FILAMENT;
					}
				break;

				case BUTTON_CNC_ON:
					if((MA_LASER == MachineSelected) || (MA_LASER_350 == MachineSelected))
					{
						#ifdef DOOR_PORT
						if(READ(DOOR_PORT, DOOR_PIN) == false)
							SetLaserPower(laserPercent);
						#else
						SetLaserPower(laserPercent);
						#endif
					}
					else if(MA_CNC == MachineSelected)
						digitalWrite(LASER_PORT, LASER_PIN, HIGH);
				break;

				case BUTTON_CNC_OFF:
					if( isLaserMachine(MachineSelected) == 1)
						SetLaserPower(0);
					else if(MA_CNC == MachineSelected)
						digitalWrite(LASER_PORT, LASER_PIN, LOW);
				break;

				case BUTTON_CNC_SET_ORIGIN:
					strcpy(tmpBuff, "G92 X0 Y0");
					current_command_args = tmpBuff;
					gcode_G92();
					HMI_ChangePage(PAGE_ORIGINSAVED);
					millis_t tick;
					tick = millis();
					while(millis() < (tick + 800));
					HMI_ChangePage(PAGE_CNCCONTROL);
					//После установки начала координат удерживайте текущую
					StepperKeepCurrent = true;
					enable_x();
					enable_y();
					enable_z();
				break;

				case BUTTON_SET_LASER_FOCUS_HIGH:
					LaserZValue = current_position[Z_AXIS];
					SaveLaserFocusZValue(LaserZValue);
					Config_StoreSettings();
					HomeAfterReset[Z_AXIS] = false;
					//Выключить лазер, ЧПУ
					SetLaserPower(0);
					digitalWrite(LASER_PORT, LASER_PIN, 0);
				break;

				case BUTTON_CNC_MOVE_AXIS:
					if( isLaserMachine(MachineSelected) )
						SetLaserLowPower();
					HMI_ChangePage(PAGE_MOVE100);
				break;


				case BUTTON_SHIELD_FAN_ON:
					SHIELD_FAN_PORT->BSRR = SHIELD_FAN_PIN;
				break;

				case BUTTON_SHIELD_FAN_OFF:
					SHIELD_FAN_PORT->BRR = SHIELD_FAN_PIN;
				break;

				case BUTTON_SHIELD_LIGHT_ON:
					SHIELD_LED_PORT->BSRR = SHIELD_LED_PIN;
				break;

				case BUTTON_SHIELD_LIGHT_OFF:
					SHIELD_LED_PORT->BRR = SHIELD_LED_PIN;
				break;


				/********************************************
				Ниже приведены тестовые элементы. Очищайте их, когда они не используются.
				********************************************/
				case BUTTON_ZHA_ZOFFSET_SETUP:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Выключите нагревательный слой и нагревательную головку.
						disable_all_heaters();
						
						HMI_ChangePage(PAGE_ZCENTER);
						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();
						while(blocks_queued());
						mbl.active = false;

						//Установить максимальную скорость оси Z
						strcpy(tmpBuff, "M203 Z50");
						current_command_args = tmpBuff;
						gcode_M203();

						//Установите координаты точки уровня моря
						current_position[Z_AXIS] = base_max_pos[Z_AXIS];
						sync_plan_position();

						//Пункт 1
						//Режим абсолютных координат
						relative_mode = false;
						strcpy(tmpBuff, "G0 Z13 F2000");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
						
						float X, Y;
						X = MESH_MIN_X + 25;
						Y = MESH_MIN_Y;
						sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());

						//Режим относительных координат
						relative_mode = true;
						HalfAutoCalibrateState = 1;
						strcpy(tmpBuff, "G0 Z-20 F70");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
						current_position[Z_AXIS] = pCounter_Z / axis_steps_per_unit[Z_AXIS];
						MeshPointZ[0] = current_position[Z_AXIS];
						HalfAutoCalibrateState = 0;
						sync_plan_position();

						//Пункт 2
						//Режим абсолютных координат
						relative_mode = false;
						strcpy(tmpBuff, "G0 Z13 F2000");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
						
						X = MESH_MAX_X + 25;
						Y = MESH_MIN_Y;
						sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());

						//Режим относительных координат
						relative_mode = true;
						HalfAutoCalibrateState = 1;
						strcpy(tmpBuff, "G0 Z-20 F70");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
						current_position[Z_AXIS] = pCounter_Z / axis_steps_per_unit[Z_AXIS];
						MeshPointZ[1] = current_position[Z_AXIS];
						HalfAutoCalibrateState = 0;
						sync_plan_position();

						//Пункт 3
						//Режим абсолютных координат
						relative_mode = false;
						strcpy(tmpBuff, "G0 Z13 F2000");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
						
						X = MESH_MIN_X + 25;
						Y = MESH_MAX_Y;
						sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());

						//Режим относительных координат
						relative_mode = true;
						HalfAutoCalibrateState = 1;
						strcpy(tmpBuff, "G0 Z-20 F70");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
						current_position[Z_AXIS] = pCounter_Z / axis_steps_per_unit[Z_AXIS];
						MeshPointZ[2] = current_position[Z_AXIS];
						HalfAutoCalibrateState = 0;
						sync_plan_position();

						//Пункт 4
						//Режим абсолютных координат
						relative_mode = false;
						strcpy(tmpBuff, "G0 Z13 F2000");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
						
						X = MESH_MAX_X + 25;
						Y = MESH_MAX_Y;
						sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());

						//Режим относительных координат
						relative_mode = true;
						HalfAutoCalibrateState = 1;
						strcpy(tmpBuff, "G0 Z-20 F70");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
						current_position[Z_AXIS] = pCounter_Z / axis_steps_per_unit[Z_AXIS];
						MeshPointZ[3] = current_position[Z_AXIS];
						HalfAutoCalibrateState = 0;
						sync_plan_position();

						//центральная точка
						relative_mode = false;
						strcpy(tmpBuff, "G0 Z13 F2000");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());

						X = (MESH_MAX_X + MESH_MIN_X) / 2.0f + 25;
						Y = (MESH_MAX_Y + MESH_MIN_Y) / 2.0f;
						sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());

						//Режим относительных координат
						relative_mode = true;
						HalfAutoCalibrateState = 1;
						strcpy(tmpBuff, "G0 Z-20 F70");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
						current_position[Z_AXIS] = pCounter_Z / axis_steps_per_unit[Z_AXIS];
						HalfAutoCalibrateZPosition = current_position[Z_AXIS];
						HalfAutoCalibrateState = 0;
						sync_plan_position();

						//Zoffset
						relative_mode = false;
						strcpy(tmpBuff, "G0 Z13 F2000");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());

						X = (MESH_MAX_X + MESH_MIN_X) / 2.0f;
						Y = (MESH_MAX_Y + MESH_MIN_Y) / 2.0f;
						sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
						
						HMI_ChangePage(PAGE_CALIBRATE_HA1);

						relative_mode = true;

						//Установить максимальную скорость оси Z
						strcpy(tmpBuff, "M203 Z5");
						current_command_args = tmpBuff;
						gcode_M203();
						
						HMICommandSave = 1;
					}
				break;
				
				case BUTTON_ZHA_CALIBRATE1_SAVE:
				case BUTTON_ZHA_CALIBRATE2_SAVE:
				case BUTTON_ZHA_CALIBRATE3_SAVE:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Интерфейс ожидания переключения
						HMI_ChangePage(PAGE_ZCENTER);

						//П1
						mbl.set_z(0, 0, MeshPointZ[0] - HalfAutoCalibrateZPosition);

						//П2
						mbl.set_z(0, 1, MeshPointZ[1] - HalfAutoCalibrateZPosition);
						
						//Продавать
						mbl.set_z(1, 0, MeshPointZ[2] - HalfAutoCalibrateZPosition);

						//P4
						mbl.set_z(1, 1, MeshPointZ[3] - HalfAutoCalibrateZPosition);

						sprintf(tmpBuff, "M206 Z%0.2f", -current_position[Z_AXIS]);
						current_command_args = tmpBuff;
						gcode_M206();

						//начало вращения
						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();
						while(blocks_queued());

						//Переключиться в режим абсолютного положения
						relative_mode = false;
						//Переключить основной интерфейс
						HMI_ChangePage(PAGE_PRINT);

						//сохранить данные
						Config_StoreSettings();
						HMICommandSave = 0;
						//Переключиться в режим абсолютного положения
						relative_mode = false;
					}
				break;
				
				//Сбросить параметры выравнивания
				case BUTTON_ZHA_CALIBRATE1_RESET:
				case BUTTON_ZHA_CALIBRATE2_RESET:
				case BUTTON_ZHA_CALIBRATE3_RESET:
					HMI_ChangePage(PAGE_PAGE_RESETCALIBRATE_HA);
				break;

				case BUTTON_ZHA_CALIBRATE1_BACK:
				case BUTTON_ZHA_CALIBRATE2_BACK:
				case BUTTON_ZHA_CALIBRATE3_BACK:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Load
						Config_RetrieveSettings();
						HMI_ChangePage(PAGE_WAITHOME);
						relative_mode = false;
						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();
						while(blocks_queued());
						strcpy(tmpBuff, "G0 Z0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());

						HMI_ChangePage(PAGE_SETTING); 
						HMICommandSave = 0;
						//Переключиться в режим абсолютного положения
						relative_mode = false;
					}
				break;

				//Подтвердите сброс параметров выравнивания
				case BUTTON_CALIBRATE_HA_RESET_CONFIRM:
					HMI_ChangePage(PAGE_WAITHOME);
					home_offset[Z_AXIS] = -8;
					mbl.reset();
					Config_StoreSettings();
					strcpy(tmpBuff, "G28");
					current_command_args = tmpBuff;
					gcode_G28();
					while(blocks_queued());
					
					sprintf(tmpBuff, "G92 Z%0.2f", -home_offset[Z_AXIS]);
					current_command_args = tmpBuff;
					gcode_G92();
					
					
					HMI_ChangePage(PAGE_CALIBRATE_HA1);

					relative_mode = true;
					
					HMICommandSave = 1;
					
					//Компенсация неудачного выравнивания
					mbl.active = false;
					HMI_ChangePage(PAGE_CALIBRATE_HA1);
				break;

				//Отменить сброс параметров выравнивания
				case BUTTON_CALIBRATE_HA_RESET_BACK:
					HMI_ChangePage(PAGE_CALIBRATE_HA1);
				break;

				
				case BUTTON_ZHA_CALIBRATE_UP_050:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z0.5");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
					}
				break;

				case BUTTON_ZHA_CALIBRATE_UP_020:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z0.2");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
					}
				break;

				case BUTTON_ZHA_CALIBRATE_UP_005:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z0.05");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
					}
				break;

				case BUTTON_ZHA_CALIBRATE_DOWN_050:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z-0.5");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
					}
				break;

				case BUTTON_ZHA_CALIBRATE_DOWN_020:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z-0.2");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
					}
				break;

				case BUTTON_ZHA_CALIBRATE_DOWN_005:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//Режим относительного положения
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z-0.05");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
					}
				break;
				/********************************************
				Вышеуказанные элементы являются тестовыми, очищайте их, когда они не используются.
				********************************************/

				//Получить данные точки останова
				case BUTTON_GET_BREAK_DATA:
					HMI_SendBreakPointData();
				break;

				//Возобновить работу с точки останова
				case BUTTON_RESUME_BREAK_PRINT:
					PowerPanicResumeWork(NULL);
				break;

				//Выдавливание
				case BUTTON_EXTRDE:
					if(MA_PRINT == MachineSelected)
					{
						//Относительный режим
						relative_mode = true;
						strcpy(tmpBuff, "G0 E5 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//Абсолютный режим
						relative_mode = false;
						
					}
				break;

				//Введите настройки высоты лазера
				case BUTTON_ENTER_LASER_FOCUS_SETTING:
					if(MA_LASER == MachineSelected)
					{
						float X,Y;
						X = (base_max_pos[X_AXIS] + home_offset[X_AXIS]) / 2.0f;
						Y = (base_max_pos[Y_AXIS] + home_offset[Y_AXIS]) / 2.0f;
						LaserZValue = 0;
						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();
						sprintf(tmpBuff, "G0 X%0.2f Y%0.2f", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						SetLaserLowPower();
					}
				break;
				
				default:
					sprintf(tmpBuff, PSTR("Unknow:%x"), ID);
					SERIAL_ECHO(tmpBuff);
				break;
			}
		}
		
		//Отладка инструкций верхнего уровня
		else if(eventId == 0x01)
		{
			ShieldLedOn();
			//Заполнение хвоста инструкции с помощью 0
			j = cmdLen + 12;
			tmpBuff[j] = 0;
			Screen_enqueue_and_echo_commands(&tmpBuff[13], 0xffffffff, 0x02);
		}
		//Печать GCode
		else if(eventId == 0x03)
		{
			ShieldLedOn();
			CurStatus = GetCurrentPrinterStatus();
			//Запуск печати с хоста
			if((CurStatus== STAT_PRINTING_ONLINE) || (CurStatus== STAT_PAUSE_ONLINE))
			{
				//лазерная головка
				if(isLaserMachine(MachineSelected) == 1)
				{
					//Номер строки
					ID = (tmpBuff[9] << 24) | (tmpBuff[10] << 16) | (tmpBuff[11] << 8) | tmpBuff[12];
					//Заполнение хвоста инструкции с помощью 0
					j = tmpBuff[3] + 8;
					tmpBuff[j] = 0;
					
					#ifdef DOOR_PORT
					if(DoorCheckEnable == true)
					{
						//дверь закрыта
						if(READ(DOOR_PORT, DOOR_PIN) == false)
						{
							Screen_enqueue_and_echo_commands(&tmpBuff[13], ID, 0x04);
						}
						else
						{
							PauseTriggle(ManualPause);
							SetCurrentPrinterStatus(STAT_PAUSE_ONLINE);
						}
					}
					#else
					Screen_enqueue_and_echo_commands(&tmpBuff[13], ID, 0x04);
					#endif
				}
				//Нелазерная головка
				else
				{
					//Номер строки
					ID = (tmpBuff[9] << 24) | (tmpBuff[10] << 16) | (tmpBuff[11] << 8) | tmpBuff[12];
					//Заполнение хвоста инструкции с помощью 0
					j = tmpBuff[3] + 8;
					tmpBuff[j] = 0;
					Screen_enqueue_and_echo_commands(&tmpBuff[13], ID, 0x04);
				}
			}
		}
		//Операции с файлами
		else if(eventId == 0x05)
		{
			ShieldLedOn();
			//управляющее слово
			switch(tmpBuff[9])
			{
				//Перемонтировать
				case 0:
					card.initsd();
					if(card.cardOK == true)
						HMI_SendInitUdisk(0);
					else
						HMI_SendInitUdisk(1);
					//Очистить смещение отображения проекта
					ListFileOffset = 0;
				break;

				//Получить текущий путь к каталогу
				case 1:
					if(card.getWorkDirPath() == true)
						HMI_SendCurrentUDiskPath(0);
					else
						HMI_SendCurrentUDiskPath(1);
				break;

				//Введите каталог
				case 2:
					ID = 0;
					FileListBuff[ID] = 0;
					//Очистить смещение отображения проекта
					ListFileOffset = 0;
					if(card.getWorkDirPath() == true)
					{
						for(int i=10;i<263;i++)
						{
							FileListBuff[ID++] = tmpBuff[i];
							if(tmpBuff[i] == 0)
								break;
						}
						if(card.chdir(FileListBuff) == 0)
						{
							HMI_SendChDirResult(0);
						}
						else
						{
							HMI_SendChDirResult(1);
						}
					}
					else
					{
						HMI_SendChDirResult(1);
					}
				break;

				//Вернуться на предыдущий уровень
				case 3:
				break;

				//Получить содержимое текущего каталога
				case 4:
					ListFileOffset += HMI_SendDirItems(0, ListFileOffset);
				break;

				//Получить специальное содержимое файла
				case 5:
					//Работает в режиме ожидания
					if(GetCurrentPrinterStatus() == STAT_IDLE)
					{
						ID = 0;
						FileListBuff[ID] = 0;
						if(card.getWorkDirPath() == true)
						{
							for(int i=10;i<263;i++)
							{
								FileListBuff[ID++] = tmpBuff[i];
								if(tmpBuff[i] == 0)
									break;
							}
						}
						HMI_SendSpecialData();
					}
				break;

				//Начать печать с U-диска
				case 6:
					CurStatus = GetCurrentPrinterStatus();
					//Работает в режиме ожидания
					if(CurStatus == STAT_IDLE)
					{
						ID = 0;
						FileListBuff[ID] = 0;
						if(card.getWorkDirPath() == true)
						{
							for(int i=10;i<263;i++)
							{
								FileListBuff[ID++] = tmpBuff[i];
								if(tmpBuff[i] == 0)
									break;
							}
						}
						card.StartSelectPrint(FileListBuff);
						//Файл успешно открыт
						if(card.isFileOpen() == false)
						{
							//Не удалось открыть файл отправки
							HMI_SendStartPrintReack(1);
							break;
						}
						PowerPanicData.FilePosition = 0;
						PowerPanicData.accumulator = 0;
						PowerPanicData.HeaterTamp[0] = 0;
						PowerPanicData.BedTamp = 0;
						PowerPanicData.PositionData[0] = 0;
						PowerPanicData.PositionData[1] = 0;
						PowerPanicData.PositionData[2] = 0;
						PowerPanicData.PositionData[3] = 0;
						PowerPanicData.GCodeSource = GCODE_SOURCE_UDISK;
						PowerPanicData.MachineType = MachineSelected;
						HMI_SendStartPrintReack(0);

						//Лазерная исполнительная головка
						if(isLaserMachine(MachineSelected) == 1)
						{
							//Z неизвестные координаты
							if(HomeAfterReset[Z_AXIS] == false)
							{
								//Ось Z возвращается в исходное положение
								strcpy(tmpBuff, "G28 Z");
								current_command_args = tmpBuff;
								gcode_G28();
							}
							//Перейти к координатам заготовки
							strcpy(tmpBuff, "G0 X0 Y0");
							current_command_args = tmpBuff;
							gcode_G0_G1(0);
							while(blocks_queued());
						}
						//Включить обнаружение
						EnablePowerPanicCheck();
						if(isLaserMachine(MachineSelected) == 1)
						{
							//дверь закрыта
							if(READ(DOOR_PORT, DOOR_PIN) == true)
							{
								PauseTriggle(ManualPause);
								SetCurrentPrinterStatus(STAT_PAUSE);
							}
						}
						//Запустить обнаружение двери
						if(isLaserMachine(MachineSelected) == 1)
							EnableDoorCheck();
					}
				break;
			}
		}
		//Отчеты о состоянии
		else if(eventId == 0x07)
		{
			uint8_t StatuID;
			block_t *pBlock;
			StatuID = tmpBuff[9];
			//Получить текущий статус
			CurStatus = GetCurrentPrinterStatus();
			//пауза
			if(StatuID == 0x04)
			{
				CurStatus = GetCurrentPrinterStatus();
				if(CurStatus == STAT_PRINTING)
				{
					HmiRequestStatus = STAT_PAUSE;
				}
				else if(CurStatus == STAT_PRINTING_ONLINE)
				{
					HmiRequestStatus = STAT_PAUSE_ONLINE;
				}
				PauseTriggle(ManualPause);
			}
			//продолжать
			else if(StatuID == 0x05)
			{
				CurStatus = GetCurrentPrinterStatus();
				//Печать на USB-диске
				if(CurStatus ==  STAT_PAUSE)
				{
					//Лазерная исполнительная головка
					if(isLaserMachine(MachineSelected) == 1)
					{
						#ifdef DOOR_PORT
						//Отключить обнаружение
						//DisableDoorCheck();
						//дверь закрыта
						if(READ(DOOR_PORT,DOOR_PIN) == false)
						{
							//Начать печать
							PauseResume();
						}
						//Включить обнаружение
						EnableDoorCheck();
						#else
						PauseResume();
						#endif
					}
					else
					{
						PauseResume();
					}
					//Очистить флаг неисправности
					SysFaultFlag &= ~FAULT_FLAG_FILAMENT;
					HmiRequestStatus = STAT_PRINTING;
				}//Онлайн-печать
				else if(CurStatus == STAT_PAUSE_ONLINE)
				{
					PauseResume();
					//Очистить флаг неисправности
					SysFaultFlag &= ~FAULT_FLAG_FILAMENT;
					HmiRequestStatus = STAT_PRINTING_ONLINE;
					
				}
			}
			//Распечатать онлайн
			else if(StatuID == 0x03)
			{
				//В режиме ожидания
				if(CurStatus == STAT_IDLE)
				{
					//Установить статус печати
					SetCurrentPrinterStatus(STAT_PRINTING_ONLINE);
					//Распечатать источник данных
					PowerPanicData.GCodeSource = GCODE_SOURCE_SCREEN;
					/*
					//Лазерная исполнительная головка
					если (isLaserMachine (MachineSelected) == 1)
					{
						//Z неизвестные координаты
						если (HomeAfterReset[Z_AXIS] == ложь)
						{
							//Ось Z возвращается в исходное положение
							strcpy(tmpBuff, "G28 Z");
							current_command_args = tmpBuff;
							gcode_G28();
						}
						//Перейти к координатам заготовки
						strcpy(tmpBuff, "G0 X0 Y0");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
					}
					*/
					//Включить обнаружение
					#if(HAVE_FALIMENT_SENSOR == 1)
					if(MA_PRINT == MachineSelected)
						EnableFilamentCheck();
					#endif
					HMI_SendMachineStatusChange(0x03, 0);
				}
			}
			//останавливаться
			else if(StatuID == 0x06)
			{
				if(CurStatus != STAT_IDLE)
				{
					//Существует два типа верхней остановки компьютера: немедленная остановка.
					StopTriggle(ManualStop);
				}
			}
			//Запросить самый последний номер строки
			else if(StatuID == 0x07)
			{
				HMI_SendBreakPointLine();
			}
			//Конец печати
			else if(StatuID == 0x08)
			{
				if(CurStatus != STAT_IDLE)
				{
					StopTriggle(ManualStop);
				}
			}
			//Статус запроса
			else if(StatuID == 0x01)
			{
				HMI_SendMachineStatus();
			}
		}
		//Инструкции по эксплуатации
		else if(eventId == 0x09)
		{
			switch(tmpBuff[9])
			{
				//движущаяся ось
				case 0x01:
					//буфер займа
					strcpy(FileListBuff, "G0 ");
					u32Value = (tmpBuff[10] << 24) | (tmpBuff[11] << 16) | (tmpBuff[12] << 8) | tmpBuff[13];
					sprintf(&FileListBuff[512], "X%0.2f", (float)u32Value / 1000.0f);
					strcat(FileListBuff, &FileListBuff[512]);

					u32Value = (tmpBuff[14] << 24) | (tmpBuff[15] << 16) | (tmpBuff[16] << 8) | tmpBuff[17];
					sprintf(&FileListBuff[512], "Y%0.2f", (float)u32Value / 1000.0f);
					strcat(FileListBuff, &FileListBuff[512]);

					u32Value = (tmpBuff[18] << 24) | (tmpBuff[19] << 16) | (tmpBuff[20] << 8) | tmpBuff[21];
					sprintf(&FileListBuff[512], "Z%0.2f", (float)u32Value / 1000.0f);
					strcat(FileListBuff, &FileListBuff[512]);

					u32Value = (tmpBuff[22] << 24) | (tmpBuff[23] << 16) | (tmpBuff[24] << 8) | tmpBuff[25];
					sprintf(&FileListBuff[512], "E%0.2f", (float)u32Value / 1000.0f);
					strcat(FileListBuff, &FileListBuff[512]);

					//Режим относительного положения
					relative_mode = true;
					current_command_args = FileListBuff;
					gcode_G0_G1(0);
					st_synchronize();
					//Режим абсолютного положения
					relative_mode = false;
				break;
			}
		}
		//ReadTail = ReadHead;
	}
	
}

/***********************************************
Ответить Приостановить печать номера строки
***********************************************/
void HMI_SendBreakPointLine()
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//Баотоу
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//менеджер пакетов
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//Версия протокола
	tmpBuff[i++] = 0x00;
	//Тест длины упаковки
	tmpBuff[i++] = 0x00;
	//проверять
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	//Номер строки
	tmpBuff[i++] = 0x07;

	//данные точки останова
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 24);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 16);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 8);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition);

	//Длина пополняемой упаковки
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//проверять
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}


/***********************************************
Отправить данные точки останова
************************************************/
void HMI_SendBreakPointData()
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//Баотоу
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//менеджер пакетов
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//Версия протокола
	tmpBuff[i++] = 0x00;
	//Тест длины упаковки
	tmpBuff[i++] = 0x00;
	//проверять
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x77;

	//данные точки останова
	tmpBuff[i++] = PowerPanicData.Valid;
	tmpBuff[i++] = PowerPanicData.GCodeSource;
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 24);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 16);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 8);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition);

	//Длина пополняемой упаковки
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//проверять
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}

/***********************************************
Отправить сигнал тревоги
************************************************/
void HMI_SendMachineFaultFlag()
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//Баотоу
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//менеджер пакетов
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//Версия протокола
	tmpBuff[i++] = 0x00;
	//Тест длины упаковки
	tmpBuff[i++] = 0x00;
	//проверять
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	//Аномальные отчеты
	tmpBuff[i++] = 0x02;
	//Флаг исключения
	tmpBuff[i++] = (uint8_t)(SysFaultFlag >> 24);
	tmpBuff[i++] = (uint8_t)(SysFaultFlag >> 16);
	tmpBuff[i++] = (uint8_t)(SysFaultFlag >> 8);
	tmpBuff[i++] = (uint8_t)(SysFaultFlag);

	//Распечатать исходный файл файла
	tmpBuff[i++] = PowerPanicData.GCodeSource;

	//Номер строки
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 24);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 16);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 8);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition);

	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//проверять
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}


/***********************************************
Отправить изменение статуса
************************************************/
void HMI_SendMachineStatusChange(uint8_t Status, uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//Баотоу
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//менеджер пакетов
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//Версия протокола
	tmpBuff[i++] = 0x00;
	//Тест длины упаковки
	tmpBuff[i++] = 0x00;
	//проверять
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	//текущий статус
	tmpBuff[i++] = Status;
	//Результаты обработки
	tmpBuff[i++] = Result;

	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//проверять
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}

/***********************************************
Отправить информацию о статусе
************************************************/
void HMI_SendMachineStatus()
{
	float fValue;
	uint32_t u32Value;
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//Баотоу
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//менеджер пакетов
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//Версия протокола
	tmpBuff[i++] = 0x00;
	//Тест длины упаковки
	tmpBuff[i++] = 0x00;
	//проверять
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	//Статус синхронизации
	tmpBuff[i++] = 0x01;

	//координировать
	fValue = pCounter_X / axis_steps_per_unit[X_AXIS];
	u32Value = (uint32_t)(fValue * 1000);
	tmpBuff[i++] = (uint8_t)(u32Value >> 24);
	tmpBuff[i++] = (uint8_t)(u32Value >> 16);
	tmpBuff[i++] = (uint8_t)(u32Value >> 8);
	tmpBuff[i++] = (uint8_t)(u32Value);

	fValue = pCounter_Y / axis_steps_per_unit[Y_AXIS];
	u32Value = (uint32_t)(fValue * 1000);
	tmpBuff[i++] = (uint8_t)(u32Value >> 24);
	tmpBuff[i++] = (uint8_t)(u32Value >> 16);
	tmpBuff[i++] = (uint8_t)(u32Value >> 8);
	tmpBuff[i++] = (uint8_t)(u32Value);

	fValue = pCounter_Z / axis_steps_per_unit[Z_AXIS];
	u32Value = (uint32_t)(fValue * 1000);
	tmpBuff[i++] = (uint8_t)(u32Value >> 24);
	tmpBuff[i++] = (uint8_t)(u32Value >> 16);
	tmpBuff[i++] = (uint8_t)(u32Value >> 8);
	tmpBuff[i++] = (uint8_t)(u32Value);

	fValue = pCounter_E / axis_steps_per_unit[E_AXIS];
	u32Value = (uint32_t)(fValue * 1000);
	tmpBuff[i++] = (uint8_t)(u32Value >> 24);
	tmpBuff[i++] = (uint8_t)(u32Value >> 16);
	tmpBuff[i++] = (uint8_t)(u32Value >> 8);
	tmpBuff[i++] = (uint8_t)(u32Value);

	//температура
	int16_t T0,TB,T0S,TBS;
	if(current_temperature[0] >= 0)
		T0 = (int16_t)current_temperature[0];
	else if(current_temperature[0] < 0)
		T0 = (int16_t)-current_temperature[0];

	T0S = target_temperature[0];
	
	if(current_temperature_bed >= 0)
		TB = (int16_t)current_temperature_bed;
	else if(current_temperature_bed < 0)
		TB = (int16_t)-current_temperature_bed;

	TBS = target_temperature_bed;
	
	tmpBuff[i++] = (uint8_t)((int)TB >> 8);
	tmpBuff[i++] = (uint8_t)((int)TB);
	tmpBuff[i++] = (uint8_t)((int)TBS >> 8);
	tmpBuff[i++] = (uint8_t)((int)TBS);
	tmpBuff[i++] = (uint8_t)((int)T0 >> 8);
	tmpBuff[i++] = (uint8_t)((int)T0);
	tmpBuff[i++] = (uint8_t)((int)T0S >> 8);
	tmpBuff[i++] = (uint8_t)((int)T0S);

	//Статус принтера
	j = GetCurrentPrinterStatus();
	tmpBuff[i++] = (uint8_t)(j);

	//Периферийный статус
	//tmpBuff[i++] = (uint8_t)(SysStatusFlag >> 24);
	//tmpBuff[i++] = (uint8_t)(SysStatusFlag >> 16);
	//tmpBuff[i++] = (uint8_t)(SysStatusFlag >> 8);
	tmpBuff[i++] = (uint8_t)(SysStatusFlag);

	//Тип заголовка выполнения
	tmpBuff[i++] = MachineSelected;

	//Длина заправки
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//проверять
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (tmpBuff[j] << 8) | tmpBuff[j + 1];

	if((i - 8) % 2)
		checksum += tmpBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	tmpBuff[6] = checksum >> 8;
	tmpBuff[7] = checksum;
	
	HmiWriteData(tmpBuff, i);
}

/***********************************************
Перемонтируйте USB-накопитель
************************************************/
void HMI_SendInitUdisk(uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//Баотоу
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//менеджер пакетов
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//Версия протокола
	FileListBuff[i++] = 0x00;
	//Тест длины упаковки
	FileListBuff[i++] = 0x00;
	//проверять
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = 0;
	FileListBuff[i++] = Result;
	
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//проверять
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (FileListBuff[j] << 8) | FileListBuff[j + 1];

	if((i - 8) % 2)
		checksum += FileListBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	FileListBuff[6] = checksum >> 8;
	FileListBuff[7] = checksum;
	
	HmiWriteData(FileListBuff, i);
}

/***********************************************
Введите ответ каталога
************************************************/
void HMI_SendChDirResult(uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//Баотоу
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//менеджер пакетов
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//Версия протокола
	FileListBuff[i++] = 0x00;
	//Тест длины упаковки
	FileListBuff[i++] = 0x00;
	//проверять
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = 0x02;
	FileListBuff[i++] = Result;
	
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//проверять
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (FileListBuff[j] << 8) | FileListBuff[j + 1];

	if((i - 8) % 2)
		checksum += FileListBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	FileListBuff[6] = checksum >> 8;
	FileListBuff[7] = checksum;
	
	HmiWriteData(FileListBuff, i);
}

/***********************************************
Получить ответ текущего каталога UDisk
************************************************/
void HMI_SendCurrentUDiskPath(uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//Баотоу
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//менеджер пакетов
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//Версия протокола
	FileListBuff[i++] = 0x00;
	//Тест длины упаковки
	FileListBuff[i++] = 0x00;
	//проверять
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = 0x01;
	if(Result == 0)
	{
		j = 0;
		while(card.workdirpath[j] != 0)
			FileListBuff[i++] = card.workdirpath[j++];
	}
	FileListBuff[i++] = 0;
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//проверять
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (FileListBuff[j] << 8) | FileListBuff[j + 1];

	if((i - 8) % 2)
		checksum += FileListBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	FileListBuff[6] = checksum >> 8;
	FileListBuff[7] = checksum;
	
	HmiWriteData(FileListBuff, i);
}

/***********************************************
Получить содержимое каталога
Возвращаемое значение: номер, отправленный на этот раз.
************************************************/
uint8_t HMI_SendDirItems(uint8_t Result, uint16_t Offset)
{
	char Res;
	uint8_t Count;
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	Count = 0;
	//Баотоу
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//менеджер пакетов
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//Версия протокола
	FileListBuff[i++] = 0x00;
	//Тест длины упаковки
	FileListBuff[i++] = 0x00;
	//проверять
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = 0x04;
	//результат
	FileListBuff[i++] = 0x00;
	//Offset
	FileListBuff[i++] = (uint8_t)(Offset >> 8);
	FileListBuff[i++] = (uint8_t)Offset;
	//число
	FileListBuff[i++] = 0x00;
	//Одновременно можно передать максимум 20 элементов, а общая длина данных находится в пределах размера буфера.
	while((Count < 10) && ((i + 257) < sizeof(FileListBuff)))
	{
		//Читать каталог 
		Res = card.getWorkDirItems();
		if(Res == 0)
			FileListBuff[i++] = 0;
		else if(Res == 1)
			FileListBuff[i++] = 1;
		else
			break;
		j = 0;
		if(card.longFilename[0] != 0)
		{
			while(card.longFilename[j] != 0)
				FileListBuff[i++] = card.longFilename[j++];
		}
		else
		{
			while(card.filename[j] != 0)
				FileListBuff[i++] = card.filename[j++];
		}
		FileListBuff[i++] = 0;
		Count++;
	}

	//Заполните результаты
	if(Res == (char)-2)
		FileListBuff[10] = 1;
		
	//Количество заправок
	FileListBuff[13] = Count;
	
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//проверять
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (FileListBuff[j] << 8) | FileListBuff[j + 1];

	if((i - 8) % 2)
		checksum += FileListBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	FileListBuff[6] = checksum >> 8;
	FileListBuff[7] = checksum;
	
	HmiWriteData(FileListBuff, i);

	return Count;
}

/***********************************************
Получить специальное содержимое файла
************************************************/
void HMI_SendSpecialData()
{
	float tmpTamp, tmpTampBed;
	int32_t ContentLen;
	
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	uint8_t dataValid;
	
	tmpTamp = current_temperature[0];
	tmpTampBed = current_temperature_bed;

	card.openFile(FileListBuff, true);
	dataValid = 1;
	//Файл успешно открыт
	if(card.isFileOpen() == true)
	{
		FileListBuff[0] = FileListBuff[1] = FileListBuff[2] = FileListBuff[3] = 0;
		card.ReadBytes((uint8_t*)FileListBuff, 20);
		if((FileListBuff[0] != 0x20) || (FileListBuff[1] != 0x18) || (FileListBuff[2] != 0x10) || (FileListBuff[3] != 0x01))
		{
			dataValid = 0;
		}
		else
		{
			//Рассчитать длину контента
			ContentLen = (uint32_t)((FileListBuff[19] << 24) | (FileListBuff[18] << 16) | (FileListBuff[17] << 8) | FileListBuff[16]) - 20;
		}
	}
	else
	{
		ContentLen = 0;
	}

	i=0;
	//Баотоу
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//менеджер пакетов
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//Версия протокола
	FileListBuff[i++] = 0x00;
	//Тест длины упаковки
	FileListBuff[i++] = 0x00;
	//проверять
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = 0x05;
	FileListBuff[i++] = dataValid;
	do
	{
		i = 11;
		//обычные специальные файлы
		if(dataValid == 1)
		{
			if(ContentLen > 1024)
			{
				j = 1024;
			}
			else
			{
				j = ContentLen;
				FileListBuff[10] = 2;
			}
			card.ReadBytes((uint8_t*)&FileListBuff[i], j);
			i = i + j;
			ContentLen -= j;
			
		}
		else
		{
			FileListBuff[9] = 2;
		}
		FileListBuff[2] = (uint8_t)((i - 8) >> 8);
		FileListBuff[3] = (uint8_t)(i - 8);
		FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
		//проверять
		checksum = 0;
		for(j = 8;j<(i - 1);j = j + 2)
			checksum += (FileListBuff[j] << 8) | FileListBuff[j + 1];

		if((i - 8) % 2)
			checksum += FileListBuff[i-1];
		while(checksum > 0xffff)
			checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
		checksum = ~checksum;

		FileListBuff[6] = checksum >> 8;
		FileListBuff[7] = checksum;
		
		HmiWriteData(FileListBuff, i);
	}while(ContentLen > 0);
	card.closefile();
	current_temperature[0] = tmpTamp;
	current_temperature_bed = tmpTampBed;
}

/***********************************************
Распечатать файл ответа
************************************************/
void HMI_SendStartPrintReack(uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//Баотоу
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//менеджер пакетов
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//Версия протокола
	FileListBuff[i++] = 0x00;
	//Тест длины упаковки
	FileListBuff[i++] = 0x00;
	//проверять
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = Result;
	
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//проверять
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (FileListBuff[j] << 8) | FileListBuff[j + 1];

	if((i - 8) % 2)
		checksum += FileListBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	FileListBuff[6] = checksum >> 8;
	FileListBuff[7] = checksum;
	
	HmiWriteData(FileListBuff, i);
}


//Отправить код
void HMI_SendGcode(char *GCode, uint8_t EventID)
{
	uint8_t i, j;
	uint32_t checksum;
	i = 0;

	//Баотоу
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//менеджер пакетов
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//Версия протокола
	FileListBuff[i++] = 0x00;
	//Тест длины упаковки
	FileListBuff[i++] = 0x00;
	//проверять
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = EventID;
	while(*GCode != 0)
		FileListBuff[i++] = *GCode++;
	
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//проверять
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (FileListBuff[j] << 8) | FileListBuff[j + 1];

	if((i - 8) % 2)
		checksum += FileListBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	FileListBuff[6] = checksum >> 8;
	FileListBuff[7] = checksum;
	
	HmiWriteData(FileListBuff, i);
}

//Отправить, чтобы продолжить печать
void HMI_SendContinuePrint(void)
{
	uint8_t i, j;
	uint32_t checksum;
	i = 0;
	
	//Баотоу
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//менеджер пакетов
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//Версия протокола
	FileListBuff[i++] = 0x00;
	//Тест длины упаковки
	FileListBuff[i++] = 0x00;
	//проверять
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x52;
	FileListBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 24);
	FileListBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 16);
	FileListBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 8);
	FileListBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 0);
	//менеджер пакетов
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//проверять
	checksum = 0;
	for(j = 8;j<(i - 1);j = j + 2)
		checksum += (FileListBuff[j] << 8) | FileListBuff[j + 1];

	if((i - 8) % 2)
		checksum += FileListBuff[i-1];
	while(checksum > 0xffff)
		checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);
	checksum = ~checksum;

	FileListBuff[6] = checksum >> 8;
	FileListBuff[7] = checksum;
	HmiWriteData(FileListBuff, i);
}


#endif

void HMIScreenShow(void)
{
	#if(SNAP_OLD_BASE == 1)
	uint8_t percent;
	static uint32_t nextShowMill;
	
	//Отправляется только в основном интерфейсе
	if(CurrentHMIPage == PAGE_PRINT)
	{
		if (nextShowMill < millis())
		{
			//Отображение температуры сопла
			HMI_ShowTemperature(0, (int)current_temperature[0], target_temperature[0]);
			//Отображение температуры кровати
			HMI_ShowTemperature(1, (int)current_temperature_bed, target_temperature_bed);
			
			//Файл уже открыт
			if (card.isFileOpen())
			{
				//Получить прогресс файла
				percent = card.percentDone();
				card.LastPercent = percent;

				//Показать индикатор выполнения
				HMI_ShowProgressBar(percent);

				//Показать время печати	
				HMI_ShowPrintTime();
				
			}
			nextShowMill = millis() + 1000;
		}
	}
	else if(CurrentHMIPage == PAGE_CNC)
	{
		if (nextShowMill < millis())
		{
			//Файл уже открыт
			if (card.isFileOpen())
			{
				//Получить прогресс файла
				percent = card.percentDone();
				card.LastPercent = percent;

				//Показать индикатор выполнения
				HMI_ShowProgressBar(percent);
				
				//Показать время печати	
				HMI_ShowPrintTime();
			}
			if(isLaserMachine(MachineSelected))
			{
				HMI_ShowLaserPower(laserPercent);//Обновить отображение экрана	
			}
			nextShowMill = millis() + 1000;
		}
	}
	else if(CurrentHMIPage == PAGE_WAITHEAT)
	{
		if (nextShowMill < millis())
		{
			HMI_ShowChangeFilamentTamp();
			nextShowMill = millis() + 1500;
		}
	}
	
	
	//Память вставлена
	if(SdInsert == true)
	{
		//Этот тест был удален
		if(!(IS_UDISK_INSERTED))
		{
			//значок настроек
			HMI_SetTFIcon(false);
			//Удостоверение личности было вытащено
			SdInsert = false;
			//Выйти из основного интерфейса
			if(CurrentHMIPage == PAGE_FILELIST)
			{
				nextShowMill = millis() + 50;
				FileCounts = 0;
				while (nextShowMill > millis());
				if(MA_PRINT == MachineSelected)
					HMI_ChangePage(PAGE_PRINT);
				else
					HMI_ChangePage(PAGE_CNC);
			}
		}
	}
	//SD-карта не вставлена
	else
	{
		//Этот тест был вставлен
		if(IS_UDISK_INSERTED)
		{
			//значок настроек
			HMI_SetTFIcon(true);
			//ID-карта вставлена
			SdInsert = true;
		}
	}
	#endif
}


//Add By Paladin
void HMI_ChangePage(uint8_t Page)
{
	uint8_t i;
	uint8_t pagenum;

	#if(SNAP_OLD_BASE == 0)
	return;
	#endif

	pagenum = PageID[Page];
	
	i = 0;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x05;
	tmpBuff[i++] = 0x03;
	tmpBuff[i++] = pagenum;
	tmpBuff[i++] = 0xFF;
	tmpBuff[i++] = 0xFF;
	tmpBuff[i++] = 0xFF;
	tmpBuff[i++] = 0x100 - pagenum;
	HmiWriteData(tmpBuff, i);
}

const uint32_t FileLable[6] = {LABEL_FILE1, LABEL_FILE2, LABEL_FILE3, LABEL_FILE4, LABEL_FILE5, LABEL_FILE6};
char Test[100];
void HMI_ShowFileName(uint8_t Index, char *FileName)
{
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4];
	id[0] = 0x01;
	id[1] = (uint8_t)(FileLable[Index] >> 8);
	id[2] = 0x04;
	id[3] = (uint8_t)(FileLable[Index]);
	i = 0;

	#if(SNAP_OLD_BASE == 1)
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	while(*FileName)
	{
		Test[i-8] = *FileName;
		tmpBuff[i++] = *FileName++;
		if(i > 80)
			break;
	}
	Test[i-8] = 0;
	tmpBuff[i++] = 0;

	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	#else
	//Баотоу
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//Версия протокола
	tmpBuff[i++] = 0x00;
	//менеджер пакетов
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//проверять
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	while(*FileName)
	{
		tmpBuff[i++] = *FileName++;
		if(i > 50)
			break;
	}
	tmpBuff[i++] = 0;
	tmpBuff[3] = (uint8_t)((i - 6) >> 8);
	tmpBuff[4] = (uint8_t)(i - 6);
	//проверять
	checksum = 0;
	for(j=6;j<i;j++)
		checksum = checksum + tmpBuff[j];

	checksum = 0x100 - checksum;
	tmpBuff[5] = checksum;
	#endif
	HmiWriteData(tmpBuff, i);
}

const uint32_t TemperatureLable[3] = {LABEL_TEMP_HEART0, LABEL_TEMP_BED};
void HMI_ShowTemperature(uint8_t Index, int CurrentTemp, int TargetTemp)
{
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4];
	char tmpStr[15];
	id[0] = 0x00;
	id[1] = (uint8_t)(TemperatureLable[Index] >> 8);
	id[2] = 0x04;
	id[3] = (uint8_t)(TemperatureLable[Index]);
	
	if(CurrentTemp <= 5)
		sprintf(tmpStr, PSTR("NA"));
	else
		sprintf(tmpStr, PSTR("%d/%d"), CurrentTemp, TargetTemp);

	i = 0;
	#if(SNAP_OLD_BASE == 1)
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	for(j=0;j<30;j++)
	{
		if(tmpStr[j] == 0)
			break;
		tmpBuff[i++] = tmpStr[j];
	}
	tmpBuff[i++] = 0;

	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	#else
	//Баотоу
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//Версия протокола
	tmpBuff[i++] = 0x00;
	//менеджер пакетов
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//проверять
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	for(j=0;j<30;j++)
	{
		if(tmpStr[j] == 0)
			break;
		tmpBuff[i++] = tmpStr[j];
	}
	tmpBuff[i++] = 0;
	tmpBuff[3] = (uint8_t)((i - 6) >> 8);
	tmpBuff[4] = (uint8_t)(i - 6);
	//проверять
	checksum = 0;
	for(j=6;j<i;j++)
		checksum = checksum + tmpBuff[j];

	checksum = 0x100 - checksum;
	tmpBuff[5] = checksum;
	#endif
	HmiWriteData(tmpBuff, i);
}

void HMI_ShowProgressBar(uint8_t Value)
{
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4];
	if(MA_PRINT == MachineSelected)
	{
		id[0] = 0;
		id[1] = (uint8_t)(PROGRESSBAR_PRINT >> 8);
		id[2] = 0x0C;
		id[3] = (uint8_t)(PROGRESSBAR_PRINT);
	}
	else
	{
		id[0] = 15;
		id[1] = (uint8_t)(PROGRESSBAR_CNC >> 8);
		id[2] = 0x0C;
		id[3] = (uint8_t)(PROGRESSBAR_CNC);
	}
	i = 0;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	tmpBuff[i++] = Value;

	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=3;j<i;j++)
		checksum = checksum + tmpBuff[j];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	HmiWriteData(tmpBuff, i);
}

void HMI_ShowPercentLable(uint8_t Value)
{
	
}

void HMI_SetControlEnable(bool Enable)
{
	uint8_t i=0;
	uint8_t j;
	uint8_t checksum;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x09;
	tmpBuff[i++] = 0x04;

	if(MachineSelected != MA_PRINT)
	{
		if(Enable == true)
		{
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x4D;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x04;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
		else
		{
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x04;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4D;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
	}
	else
	{
		if(Enable == true)
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4D;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x04;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
		else
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x04;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4D;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
	}
	
	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;

	HmiWriteData(tmpBuff, i);
}

void HMI_SetSettingEnable(bool Enable)
{
	#if(SNAP_OLD_BASE == 1)
	uint8_t i;
	uint8_t j;
	uint8_t checksum;

	i = 0;
	
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x09;
	tmpBuff[i++] = 0x04;
	if(MachineSelected != MA_PRINT)
	{
	}
	else
	{
		if(Enable == true)
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4E;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x05;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
		else
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x05;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4E;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
	}
	
	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	HmiWriteData(tmpBuff, i);
	#endif
}

void HMI_PauseEnable(bool Enable)
{
	#if(SNAP_OLD_BASE == 1)
	uint8_t i=0;
	uint8_t j;
	uint8_t checksum;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x09;
	tmpBuff[i++] = 0x04;
	if(MachineSelected != MA_PRINT)
	{
		if(Enable == true)
		{
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x4F;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x05;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
		else
		{
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x05;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4F;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
	}
	else
	{
		if(Enable == true)
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4F;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x05;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
		else
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x05;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4F;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
	}
	
	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;

	HmiWriteData(tmpBuff, i);
	#endif
}


void HMI_StopEnable(bool Enable)
{
	#if(SNAP_OLD_BASE == 1)
	uint8_t i=0;
	uint8_t j;
	uint8_t checksum;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x09;
	tmpBuff[i++] = 0x04;
	if(MachineSelected != MA_PRINT)
	{
		if(Enable == true)
		{
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x4B;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x02;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
		else
		{
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x02;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4B;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
	}
	else
	{
		if(Enable == true)
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4B;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x02;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
		else
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x02;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x4B;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
	}
	
	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;

	HmiWriteData(tmpBuff, i);
	#endif
}

void HMI_SetTFIcon(bool Enable)
{
	#if(SNAP_OLD_BASE == 1)
	uint8_t i = 0;
	uint8_t j;
	uint8_t checksum;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x09;
	tmpBuff[i++] = 0x04;
	if(MachineSelected != MA_PRINT)
	{
		if(Enable == true)
		{
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x3D;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x3C;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
		else
		{
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x3C;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x0F;
			tmpBuff[i++] = 0x3D;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
	}
	else
	{
		if(Enable == true)
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x43;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x03;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
		else
		{
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x03;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
			tmpBuff[i++] = 0x00;
			tmpBuff[i++] = 0x43;
			tmpBuff[i++] = 0x0B;
			tmpBuff[i++] = 0xFF;
		}
	}
	
	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;

	HmiWriteData(tmpBuff, i);
	#endif
}

void HMI_ShowConfirmFileName(char *FileName)
{
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4];
	id[0] = 0x0A;
	id[1] = (uint8_t)(LABEL_CONFIRM_PRINT_FILENAME >> 8);
	id[2] = 0x04;
	id[3] = (uint8_t)(LABEL_CONFIRM_PRINT_FILENAME);
	i = 0;
	#if(SNAP_OLD_BASE == 1)
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	while(*FileName)
	{
		tmpBuff[i++] = *FileName++;
		if(i > 50)
			break;
	}
	tmpBuff[i++] = 0;

	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	#else
	//Баотоу
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//Версия протокола
	tmpBuff[i++] = 0x00;
	//менеджер пакетов
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//проверять
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	while(*FileName)
	{
		tmpBuff[i++] = *FileName++;
		if(i > 50)
			break;
	}
	tmpBuff[i++] = 0;
	tmpBuff[3] = (uint8_t)((i - 6) >> 8);
	tmpBuff[4] = (uint8_t)(i - 6);
	//проверять
	checksum = 0;
	for(j=6;j<i;j++)
		checksum = checksum + tmpBuff[j];

	checksum = 0x100 - checksum;
	tmpBuff[5] = checksum;
	#endif
	HmiWriteData(tmpBuff, i);
}

void HMI_ShowPrintFileName(char *FileName)
{
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4];
	if(MachineSelected != MA_PRINT)
	{
		id[0] = 15;
		id[1] = (uint8_t)(LABEL_CNC_FILENAME >> 8);
		id[2] = 0x05;
		id[3] = (uint8_t)(LABEL_CNC_FILENAME);
	}
	else
	{
		id[0] = 0x00;
		id[1] = (uint8_t)(LABEL_PRINT_FILENAME >> 8);
		id[2] = 0x05;
		id[3] = (uint8_t)(LABEL_PRINT_FILENAME);
	}
	i = 0;
	#if(SNAP_OLD_BASE == 1)
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	#else
	//Баотоу
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//Версия протокола
	tmpBuff[i++] = 0x00;
	//менеджер пакетов
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//проверять
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	#endif

	//Получить длину имени файла
	uint8_t Len;
	Len = strlen(FileName);
	//Длина превышает диапазон отображения
	if(Len > 32)
	{
		//отображение планирования
		uint8_t extlen;
		uint8_t divlen;
		uint8_t dotcount;
		uint8_t sflen;
		uint8_t fl;
		char *p;
		p = &FileName[Len - 1];
		extlen = 0;
		while(*p != '.')
		{
			p--;
			extlen++;
		}
		//Расширение является нормальным, если оно находится в пределах 10 байт.
		if((extlen < 10) && (extlen > 0))
		{
			extlen++;
			//При расчете суффикс удаляется и отображается длина.
			sflen = 31 - extlen;
			//Вычислить имя файла
			fl = Len - extlen;
			//Подсчитайте эллипсы
			dotcount = fl - sflen;
			//Слишком много эллипсов
			if(dotcount > 3)
			{
				if((sflen % 2) != 0)
					dotcount = 3;
				else
					dotcount = 4;
			}

			//Посчитайте, сколько байт каждый занимает до и после 
			divlen = (sflen - dotcount) / 2;

			//Первая половина имени файла
			for(j=0;j<divlen;j++)
				tmpBuff[i++] = FileName[j];

			//  Эллипсис
			for(j=0;j<dotcount;j++)
				tmpBuff[i++] = '.';
			
			//Вторая половина имени файла
			p = &FileName[Len - extlen - divlen];
			for(j=0;j<divlen + extlen;j++)
			{
				tmpBuff[i++] = *p++;
			}
		}
		else
		{
			return;
		}
	}
	else
	{
		while(*FileName)
		{
			tmpBuff[i++] = *FileName++;
			if(i > 38)
				break;
		}
	}
	tmpBuff[i++] = 0;
	#if(SNAP_OLD_BASE == 1)
	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	#else
	tmpBuff[3] = (uint8_t)((i - 6) >> 8);
	tmpBuff[4] = (uint8_t)(i - 6);
	//проверять
	checksum = 0;
	for(j=6;j<i;j++)
		checksum = checksum + tmpBuff[j];

	checksum = 0x100 - checksum;
	tmpBuff[5] = checksum;
	#endif
	HmiWriteData(tmpBuff, i);
}

void HMI_ShowDialogText(char *String)
{
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4];
	id[0] = 0x08;
	id[1] = (uint8_t)(LABEL_DIALOG_FILENAME >> 8);
	id[2] = 0x04;
	id[3] = (uint8_t)(LABEL_DIALOG_FILENAME);
	i = 0;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	while(*String)
	{
		tmpBuff[i++] = *String++;
		if(i > 30)
			break;
	}
	tmpBuff[i++] = 0;

	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	HmiWriteData(tmpBuff, i);
}


void HMI_ZOffsetLable(float Value)
{
	int i, j;
	uint8_t checksum;
	uint8_t id[4];
	char tmpStr[10];

	i = (uint8_t)((Value) * 10.0f);
	j = (uint8_t)(Value);
	sprintf(tmpStr, PSTR("%d.%dmm"), j, i%10);
	id[0] = 0x07;
	id[1] = (uint8_t)(LABEL_ZOFFSET >> 8);
	id[2] = 0x04;
	id[3] = (uint8_t)(LABEL_ZOFFSET);
	i = 0;

	#if(SNAP_OLD_BASE == 1)
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	
	for(j=0;j<30;j++)
	{
		if(tmpStr[j] == 0)
			break;
		tmpBuff[i++] = tmpStr[j];
	}
	tmpBuff[i++] = 0;

	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	#else
	//Баотоу
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//Версия протокола
	tmpBuff[i++] = 0x00;
	//менеджер пакетов
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//проверять
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	for(j=0;j<30;j++)
	{
		if(tmpStr[j] == 0)
			break;
		tmpBuff[i++] = tmpStr[j];
	}
	tmpBuff[i++] = 0;
	tmpBuff[3] = (uint8_t)((i - 6) >> 8);
	tmpBuff[4] = (uint8_t)(i - 6);
	//проверять
	checksum = 0;
	for(j=6;j<i;j++)
		checksum = checksum + tmpBuff[j];

	checksum = 0x100 - checksum;
	tmpBuff[5] = checksum;
	#endif
	HmiWriteData(tmpBuff, i);
			
}


void HMI_ShowChangeFilamentTamp()
{
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4];
	char tmpStr[20];
	id[0] = 23;
	id[1] = (uint8_t)(LABEL_HEATER_TARGET_TEMP >> 8);
	id[2] = 0x04;
	id[3] = (uint8_t)(LABEL_HEATER_TARGET_TEMP);
	
	sprintf(tmpStr, PSTR("%d&"),(int)target_temperature[0]);
	i = 0;
	#if(SNAP_OLD_BASE == 1)
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	for(j=0;j<30;j++)
	{
		if(tmpStr[j] == 0)
			break;
		tmpBuff[i++] = tmpStr[j];
	}
	tmpBuff[i++] = 0;

	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	#else
	//Баотоу
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//Версия протокола
	tmpBuff[i++] = 0x00;
	//менеджер пакетов
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//проверять
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	for(j=0;j<30;j++)
	{
		if(tmpStr[j] == 0)
			break;
		tmpBuff[i++] = tmpStr[j];
	}
	tmpBuff[i++] = 0;
	tmpBuff[3] = (uint8_t)((i - 6) >> 8);
	tmpBuff[4] = (uint8_t)(i - 6);
	//проверять
	checksum = 0;
	for(j=6;j<i;j++)
		checksum = checksum + tmpBuff[j];

	checksum = 0x100 - checksum;
	tmpBuff[5] = checksum;
	#endif
	HmiWriteData(tmpBuff, i);

	id[0] = 23;
	id[1] = (uint8_t)(LABEL_HEATER_CUR_TEMP >> 8);
	id[2] = 0x04;
	id[3] = (uint8_t)(LABEL_HEATER_CUR_TEMP);
	
	sprintf(tmpStr, PSTR("%d&"),(int)current_temperature[0]);
	i = 0;
	#if(SNAP_OLD_BASE == 1)
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	for(j=0;j<30;j++)
	{
		if(tmpStr[j] == 0)
			break;
		tmpBuff[i++] = tmpStr[j];
	}
	tmpBuff[i++] = 0;

	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	#else
	//Баотоу
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//Версия протокола
	tmpBuff[i++] = 0x00;
	//менеджер пакетов
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//проверять
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	for(j=0;j<30;j++)
	{
		if(tmpStr[j] == 0)
			break;
		tmpBuff[i++] = tmpStr[j];
	}
	tmpBuff[i++] = 0;
	tmpBuff[3] = (uint8_t)((i - 6) >> 8);
	tmpBuff[4] = (uint8_t)(i - 6);
	//проверять
	checksum = 0;
	for(j=6;j<i;j++)
		checksum = checksum + tmpBuff[j];

	checksum = 0x100 - checksum;
	tmpBuff[5] = checksum;
	#endif
	HmiWriteData(tmpBuff, i);
}

void HMI_ShowLaserPower(float Power)
{
	char tmpStr[15];
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4];

	id[0] = 15;
	id[1] = (uint8_t)(LABEL_CNC_POWER >> 8);
	id[2] = 4;
	id[3] = (uint8_t)(LABEL_CNC_POWER);
	
	if( isLaserMachine(MachineSelected) )
		sprintf(tmpStr, "%.2f", Power);
	else
		strcpy(tmpStr, "100");
	strcat(tmpStr, "%");

	i = 0;
	#if(SNAP_OLD_BASE == 1)
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	for(j=0;j<30;j++)
	{
		if(tmpStr[j] == 0)
			break;
		tmpBuff[i++] = tmpStr[j];
	}
	tmpBuff[i++] = 0;
	
	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	#else
	//Баотоу
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//Версия протокола
	tmpBuff[i++] = 0x00;
	//менеджер пакетов
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//проверять
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	for(j=0;j<30;j++)
	{
		if(tmpStr[j] == 0)
			break;
		tmpBuff[i++] = tmpStr[j];
	}
	tmpBuff[i++] = 0;
	tmpBuff[3] = (uint8_t)((i - 6) >> 8);
	tmpBuff[4] = (uint8_t)(i - 6);
	//проверять
	checksum = 0;
	for(j=6;j<i;j++)
		checksum = checksum + tmpBuff[j];

	checksum = 0x100 - checksum;
	tmpBuff[5] = checksum;
	#endif
	
	HmiWriteData(tmpBuff, i);
}

void HMI_ShowPrintTime(void)
{
	char tmpStr[15];
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4];
	if(MA_PRINT == MachineSelected)
	{
		id[0] = 0;
		id[1] = (uint8_t)(LABEL_PRINT_TIME >> 8);
		id[2] = 4;
		id[3] = (uint8_t)(LABEL_PRINT_TIME);
	}
	else
	{
		id[0] = 15;
		id[1] = (uint8_t)(LABEL_CNC_TIME >> 8);
		id[2] = 4;
		id[3] = (uint8_t)(LABEL_CNC_TIME);
	}

	millis_t t = print_job_timer.duration();
    int hours = t / 60 / 60, minutes = (t / 60) % 60;

	sprintf(tmpStr, PSTR("%dh%dm"), hours, minutes);
	i = 0;
	#if(SNAP_OLD_BASE == 1)
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	for(j=0;j<30;j++)
	{
		if(tmpStr[j] == 0)
			break;
		tmpBuff[i++] = tmpStr[j];
	}
	tmpBuff[i++] = 0;
	
	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	#else
	//Баотоу
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//Версия протокола
	tmpBuff[i++] = 0x00;
	//менеджер пакетов
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//проверять
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	for(j=0;j<30;j++)
	{
		if(tmpStr[j] == 0)
			break;
		tmpBuff[i++] = tmpStr[j];
	}
	tmpBuff[i++] = 0;
	tmpBuff[3] = (uint8_t)((i - 6) >> 8);
	tmpBuff[4] = (uint8_t)(i - 6);
	//проверять
	checksum = 0;
	for(j=6;j<i;j++)
		checksum = checksum + tmpBuff[j];

	checksum = 0x100 - checksum;
	tmpBuff[5] = checksum;
	#endif
	
	HmiWriteData(tmpBuff, i);
}

void HMI_GetCurrentPage(void)
{
	#if(SNAP_OLD_BASE == 1)
	uint8_t i;
	//AA 55 02 06 03 F7
	i = 0;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x02;
	tmpBuff[i++] = 0x06;
	tmpBuff[i++] = 0x03;
	tmpBuff[i++] = 0xF7;
	
	HmiWriteData(tmpBuff, i);
	#endif
}

void HMI_ChangePrinterStatus(U8 Status)
{
	#if(SNAP_OLD_BASE == 1)
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4] = {0xff, 0xff, 0xff, 0xff};

	i = 0;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x0B;
	tmpBuff[i++] = Status;
	tmpBuff[i++] = ~Status;
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	
	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	HmiWriteData(tmpBuff, i);
	#endif
}


void HMI_ChangeLaserCNCStatus(U8 Status)
{
	#if(SNAP_OLD_BASE == 1)
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint8_t id[4] = {0xff, 0xff, 0xff, 0xff};

	i = 0;
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x0C;
	tmpBuff[i++] = Status;
	tmpBuff[i++] = ~Status;
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	
	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	HmiWriteData(tmpBuff, i);
	#endif
}

void HMI_SetAboutText(uint16_t u32LableID,char * Ver)
{
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t checksum;
	
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = 0x14;
	tmpBuff[i++] = (char)(u32LableID >> 8);
	tmpBuff[i++] = 0x04;
	tmpBuff[i++] = (char)u32LableID;

	for(j=0;j<30;j++)
	{
		if(Ver[j] == 0)
			break;
		tmpBuff[i++] = Ver[j];
	}
	tmpBuff[i++] = 0;
	
	tmpBuff[2] = i - 3;
	//проверять
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	HmiWriteData(tmpBuff, i);
}
void HMI_SetAboutLabels(void)
{
	HMI_SetAboutText(LABEL_ABOUT_1,FIRWARE_VER);
	//Display the size of external publicity
	if(base_max_pos[Z_AXIS] > Z_MAX_POS)
		HMI_SetAboutText(LABEL_ABOUT_3, "Build Volume: 125 x 125 x 221mm");
	else
		HMI_SetAboutText(LABEL_ABOUT_3, "Build Volume: 125 x 125 x 125mm");
	HMI_SetAboutText(LABEL_ABOUT_7, "Heated Bed: Up to 80&");
}


#if(SNAP_OLD_BASE == 1)
void HMI_SendScreenUpdate()
{
	char LSendbuff[20];
	uint8_t i;

	i = 0;
	LSendbuff[i++] = 0xAA;
	LSendbuff[i++] = 0x55;
	LSendbuff[i++] = 0x00;
	LSendbuff[i++] = 0x8E;
	LSendbuff[i++] = 0x00;
	LSendbuff[i++] = 0x00;
	LSendbuff[i++] = 0x00;
	LSendbuff[i++] = 0x00;

	LSendbuff[2] = (uint8_t)(i - 3);
	uint8_t Checksum = 0;
	for (int j = 3; j < i; j++)
	    Checksum = (uint8_t)(Checksum + LSendbuff[j]);
	Checksum = (uint8_t)(0x100 - Checksum);
	LSendbuff[i++] = Checksum;

	HmiWriteData(LSendbuff, i);
}

void HMI_SendUpdatePack(uint16_t PackIndex)
{
	unsigned long RN;
	uint32_t Checksum = 0;
	uint16_t tmpLen;
	char SendBuff[256];
	uint8_t ProChecksum;
	int j;
	int i = 0;
	SendBuff[i++] = 0xAA;
	SendBuff[i++] = 0x55;
	SendBuff[i++] = 0x00;
	SendBuff[i++] = 0x81;
	SendBuff[i++] = (uint8_t)(PackIndex>>8);
	SendBuff[i++] = (uint8_t)(PackIndex);
	SendBuff[i++] = (uint8_t)(Packs >> 8);
	SendBuff[i++] = (uint8_t)(Packs);
	i = i + 4;

	if (PackIndex != (Packs - 1))
	{
		tmpLen = 240;
	}
	else
	{
	    if ((UpdateFile.fsize % 240) == 0)
	        tmpLen = 240;
	    else
	        tmpLen = (UpdateFile.fsize % 240);
	}
	f_lseek(&UpdateFile, 240 * PackIndex);
	WaitUDisk();
	f_read(&UpdateFile, &SendBuff[i], tmpLen, &RN);
	i = i + tmpLen;

	Checksum = 0;
	for(j=0;j<tmpLen;j++)
		Checksum += SendBuff[12+j];

	SendBuff[8] = (uint8_t)(Checksum >> 24);
	SendBuff[9] = (uint8_t)(Checksum >> 16);
	SendBuff[10] = (uint8_t)(Checksum >> 8);
	SendBuff[11] = (uint8_t)(Checksum);
	SendBuff[2] = (uint8_t)(i - 3); 
	ProChecksum = 0;
	for (j = 3; j < i; j++)
	{
	    ProChecksum = (uint8_t)(ProChecksum + SendBuff[j]);
	}
	ProChecksum = (uint8_t)(0x100 - ProChecksum);
	SendBuff[i++] = ProChecksum;
	HmiWriteData(SendBuff, i);
}

void HMI_SendUpdateComplete()
{
    char SendBuff[30];
    int i = 0;
    SendBuff[i++] = 0xAA;
    SendBuff[i++] = 0x55;
    SendBuff[i++] = 0x00;
    SendBuff[i++] = 0x82;
    SendBuff[i++] = (uint8_t)((Packs - 1) >> 8);
    SendBuff[i++] = (uint8_t)(Packs - 1);
    SendBuff[i++] = (uint8_t)(Packs >> 8);
    SendBuff[i++] = (uint8_t)(Packs);

    SendBuff[2] = (uint8_t)(i - 3);

    uint8_t ProChecksum = 0;
    for (int j = 3; j < i; j++)
    {
        ProChecksum = (uint8_t)(ProChecksum + SendBuff[j]);
    }
    ProChecksum = (uint8_t)(0x100 - ProChecksum);
    SendBuff[i++] = ProChecksum;

    HmiWriteData(SendBuff, i);
}

void HMI_SendUpdateNoFile()
{
	char SendBuff[30];
    int i = 0;
    SendBuff[i++] = 0xAA;
    SendBuff[i++] = 0x55;
    SendBuff[i++] = 0x00;
    SendBuff[i++] = 0x83;
    SendBuff[i++] = (uint8_t)((Packs - 1) >> 8);
    SendBuff[i++] = (uint8_t)(Packs - 1);
    SendBuff[i++] = (uint8_t)(Packs >> 8);
    SendBuff[i++] = (uint8_t)(Packs);

    SendBuff[2] = (uint8_t)(i - 3);

    uint8_t ProChecksum = 0;
    for (int j = 3; j < i; j++)
    {
        ProChecksum = (uint8_t)(ProChecksum + SendBuff[j]);
    }
    ProChecksum = (uint8_t)(0x100 - ProChecksum);
    SendBuff[i++] = ProChecksum;

    HmiWriteData(SendBuff, i);
}
#endif

void HmiComPortInit(int Baudary)
{
	WriteHead = WriteTail = 0;
	ReadHead = ReadTail = 0;
	USART_DeInit(USART2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	USART_InitTypeDef USART_InitStruct;

	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	
	USART_InitStruct.USART_BaudRate = Baudary;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = 8;
	USART_Init(USART2, &USART_InitStruct);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
	

	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStruct);

	
}



extern "C"
{
void USART2_IRQHandler(void)
{
	uint16_t tmpTail;
	uint16_t tmpHead;
	uint8_t RecvData;
	
	if(USART2->SR & USART_SR_RXNE)
	{
		RecvData = USART2->DR;
		tmpHead = (ReadHead + 1) % sizeof(ReadBuff);
		if(tmpHead != ReadTail)
		{
			ReadBuff[ReadHead] = RecvData;
			ReadHead = tmpHead;
		}
	}
	else if(USART2->SR & USART_SR_TXE)
	{
		tmpTail = WriteTail;
		USART2->SR &= ~USART_SR_TXE;
		if(tmpTail == WriteHead)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;
			
		}
		else
		{
			USART2->DR = WriteBuff[tmpTail];
			WriteTail = (tmpTail + 1) % sizeof(WriteBuff);
		}
	}
}
}

#else

void HMICommandProcess(){}
void HMIScreenShow(){}
void HMI_ChangePage(uint8_t Page){}
void HMI_ShowPrintTime(void){}
void HMI_ShowPercentLable(uint8_t Value){}
void HMI_ShowProgressBar(uint8_t Value){}
void HMI_ShowTemperature(uint8_t Index, int CurrentTemp, int TargetTemp){}
void HMI_ShowFileName(uint8_t Index, char *FileName){}
void HMI_ZOffsetLable(float Value){}
void HMI_ShowPrintFileName(char * FileName){}
void HMI_ShowDialogText(char *String){}
void HMI_ShowConfirmFileName(char *FileName){}
void HMI_SetTFIcon(bool Enable){}
void HMI_StopEnable(bool Enable){}
void HMI_PauseEnable(bool Enable){}
void HMI_SetNextEnable(bool Enable){}
void HMI_SetControlEnable(bool Enable){}
void HMI_SetSettingEnable(bool Enable){}
void HMI_ShowChangeFilamentTamp(){}
void HmiGetCurrentPage(void){}
void HMI_ShowLaserPower(float Power){}
void HmiComPortInit(int Baudary){}
void HMI_ChangePrinterStatus(U8 Status){}
void HMI_ChangeLaserCNCStatus(U8 Status){}
void HMI_SetAboutLabels(void){}
void HMI_SendGcode(char *GCode, uint8_t EventID){}
void HMI_SendContinuePrint(void){}
void HMI_SendScreenUpdate(){}
void HMI_SendUpdateNoFile(){}
void HMI_SendUpdateComplete(){}
void HMI_SendUpdatePack(uint16_t PackIndex){}

#endif
