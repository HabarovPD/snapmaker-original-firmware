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
//指令暂存缓冲，正确解析之后，指令存放在这里
static char tmpBuff[372];
//SD  插入状态
static bool SdInsert = false;

static volatile uint8_t ReadBuff[512];
static volatile uint16_t ReadTail;
static volatile uint16_t ReadHead;

static volatile uint8_t WriteBuff[2048];
static volatile uint16_t WriteTail;
static volatile uint16_t WriteHead;


static uint16_t Packs;


//检测指令缓冲是否为空
#define CMD_BUFF_EMPTY()	(GetCommandBuff()>0?false:true)
//U  盘枚举成功
#define IS_UDISK_INSERTED DiskInserted

#define sprintf_P sprintf

//调平点索引
uint8_t PointIndex;
float MeshPointZ[4];
uint16_t ZHomeOffsetIndex;


#define WaitingStepper() {while(blocks_queued()){manage_heater();}}



#if(1==HAVE_HMI)

#if(SNAP_OLD_BASE == 1)
void ScreenLockCheck(void)
{
	//已锁屏
	if(ScreenLock == true)
	{
		//没有运动块也没有G  代码堆
		if((CMD_BUFF_EMPTY() == true) && (blocks_queued() == false))
		{
			//超时
			if(ScreenLockTick < millis())
			{
				ScreenLockTick = millis() + 2000;
				ScreenLock = false;
			}
		}
		//有新代码或运动块
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
	
	//没数据
	if (tmphead == tmptail)
	{
		return (short)-1;
	}

	/*
	if(ReadBuff[tmptail] != 0xAA)
	{
		tmptail = (tmptail + 1) % sizeof(ReadBuff);
		//更新读指针
		ReadTail = tmptail;
		if(ReadTail == ReadHead)
			return (char)-1;
	}
	*/

	tmplen = (uint16_t)((tmphead + sizeof(ReadBuff)- tmptail) % sizeof(ReadBuff));
	
	//数据长度足够
	while(tmplen > 8)
	{
		if(ReadBuff[tmptail] != 0xAA)
		{
			tmptail = (tmptail + 1) % sizeof(ReadBuff);
			tmplen--;
			//更新读指针
			ReadTail = tmptail;
			continue;
		}
		if(ReadBuff[(tmptail + 1) % sizeof(ReadBuff)] != 0x55)
		{
			tmptail = (tmptail + 2) % sizeof(ReadBuff);
			tmplen = tmplen - 2;
			//更新读指针
			ReadTail = tmptail;
			continue;
		}
		//读取包长
		uint8_t commandLen = ReadBuff[(tmptail + 2) % sizeof(ReadBuff)];

		//缓冲数据足够
		if(commandLen <= (tmplen - 4))
		{
			//复制数据
			for(i=0;i<(commandLen + 4);i++)
			{
				pBuff[i] = ReadBuff[tmptail];
				tmptail = (tmptail + 1) % sizeof(ReadBuff);
			}

			//更新读指针
			ReadTail = tmptail;

			//校验
			checksum = 0;
			for(i=0;i<commandLen;i++)
				checksum = checksum + pBuff[i + 3];
			checksum = 0x100 - checksum;

			if(checksum != (pBuff[commandLen + 3]))
				return (short)-1;
			//_rx_buffer->tail = _rx_buffer->head = 0;
			return commandLen + 4;
		}
		//数据长度不足
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
	
	//没数据
	if (tmphead == tmptail)
	{
		return (short)-1;
	}

	tmplen = (uint16_t)((tmphead + sizeof(ReadBuff)- tmptail) % sizeof(ReadBuff));
	
	//数据长度足够
	while(tmplen > 9)
	{
		if(ReadBuff[tmptail] != 0xAA)
		{
			tmptail = (tmptail + 1) % sizeof(ReadBuff);
			tmplen--;
			//更新读指针
			ReadTail = tmptail;
			continue;
		}
		if(ReadBuff[(tmptail + 1) % sizeof(ReadBuff)] != 0x55)
		{
			tmptail = (tmptail + 2) % sizeof(ReadBuff);
			tmplen = tmplen - 2;
			//更新读指针
			ReadTail = tmptail;
			continue;
		}
		//读取包长
		uint16_t commandLen = (ReadBuff[(tmptail + 2) % sizeof(ReadBuff)] << 8) | ReadBuff[(tmptail + 3) % sizeof(ReadBuff)];
		//包长效验错误
		if((((commandLen >> 8) & 0xff) ^ (commandLen & 0xff)) != ReadBuff[(tmptail + 5) % sizeof(ReadBuff)])
		{
			tmptail = (tmptail + 2) % sizeof(ReadBuff);
			tmplen = tmplen - 2;
			//更新读指针
			ReadTail = tmptail;
			continue;
		}

		//缓冲数据足够
		if(commandLen <= (tmplen - 8))
		{
			//复制数据
			for(i=0;i<(commandLen + 8);i++)
			{
				pBuff[i] = ReadBuff[tmptail];
				tmptail = (tmptail + 1) % sizeof(ReadBuff);
			}

			//更新读指针
			ReadTail = tmptail;

			//校验
			checksum = 0;
			for(i=0;i<(commandLen-1);i=i+2)
				checksum += (pBuff[i + 8] << 8) | pBuff[i + 9];

			//奇偶判断
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
		//数据长度不足
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
		if((millis() - CheckComTick) > 1000)
		{
			CheckComTick = millis();
			HMI_GetCurrentPage();
			if(NoComCount < 10)
				NoComCount++;
		}
		*/
	}
	//升级协议，老主控才支持升级屏幕
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
	//屏幕协议
	else
	{
		ShieldLedOn();
		eventId = tmpBuff[7];
		//按钮
		if((eventId == 1) && (ScreenLock == false))
		{
			ID = (uint16_t)(tmpBuff[3] << 8) | (uint8_t)tmpBuff[4];
			
			switch((uint16_t)ID)
			{
				case BUTTON_MOTOR_HOME:
					//电机不在同步中
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

				//换料
				case BUTTON_CHANGE_FILAMENT:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
						HMI_ChangePage(PAGE_CHANGEFILAMENT);
				break;

				//停止加热
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

				//温度加
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

				//温度减
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
							//跳转到确认对话框
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
							//跳转到确认对话框
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
							//跳转到确认对话框
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
							//跳转到确认对话框
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
							//跳转到确认对话框
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
							//跳转到确认对话框
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
							
						//清除文件个数
						FileCounts = 0;
						//开始打印选择的文件
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
							//门已关闭
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
						//使能检测
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
					//清除文件个数
					FileCounts = 0;
				break;
				

				//文件显示上一页
				case BUTTON_NEXT_PAGE_UP:
					//没有文件或正在搜文件中
					if((FileCounts == 0) || (SeekingFile == 1) || (ShowFileCount > 0))
					{
						break;
					}
					if(CurFilePage < (FilePages - 1))
					{
						CurFilePage++;
						FileIndex = CurFilePage * FILE_PER_PAGE;
						//最后一页
						if(CurFilePage == (FilePages - 1))
						{
							//非整数
							if((FileCounts % FILE_PER_PAGE) != 0)
								SeekFileCount = (FileCounts % FILE_PER_PAGE);
							else
								SeekFileCount = FILE_PER_PAGE;
						}
						else
						{
							SeekFileCount = FILE_PER_PAGE;
						}
						//清空列表
						for(i=0;i<FILE_PER_PAGE;i++)
						{
							CardFileName[i][0] = 0;
							PrintFileName[i][0] = 0;
						}
						//标识需要查找文件
						SeekingFile = 1;
						//显示文件个数与查找个数相同
						ShowFileCount = FILE_PER_PAGE;
						//显示文件索引从0开始
						ShowFileIndex = 0;
					}
				break;


				//显示文件下一页
				case BUTTON_PREV_PAGE_UP:
					//没有文件或正在搜文件中
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
						//清空列表
						for(i=0;i<FILE_PER_PAGE;i++)
						{
							CardFileName[i][0] = 0;
							PrintFileName[i][0] = 0;
						}
						//标识需要查找文件
						SeekingFile = 1;
						//显示文件个数与查找个数相同
						ShowFileCount = FILE_PER_PAGE;
						//显示文件索引从0开始
						ShowFileIndex = 0;
					}
				break;

				//退出文件选择
				case BUTTON_SELECT_FILE_BACK:
				case BUTTON_SELECT_FILE_HOME:
					if(MachineSelected == MA_PRINT)
						HMI_ChangePage(PAGE_PRINT);
					else
						HMI_ChangePage(PAGE_CNC);
					//清除文件个数
					FileCounts = 0;
				break;

				//退出控制界面
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

				//OpenFile按键
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

				//移动轴
				case BUTTON_MOVE_AXIS:
					//电机不在同步中
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						{
							HMI_ChangePage(PAGE_MOVE100);
							relative_mode = true;
							HMICommandSave = 1;
						}
					}
				break;

				//移动轴退出
				case BUTTON_MOVE_AXIS_BACK_001:
				case BUTTON_MOVE_AXIS_BACK_010:
				case BUTTON_MOVE_AXIS_BACK_100:
					//电机不在同步中
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
								//关闭激光，CNC
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
					//电机不在同步中
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
								//关闭激光，CNC
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
				
				//负方向移动
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

				//正方向移动
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

				//控制
				case BUTTON_CONTROLL:
					//if(card.isFileOpen() == false)
					if(GetCurrentPrinterStatus() == STAT_IDLE)
					{
						HMI_ChangePage(PAGE_CONTROL);
					}
				break;

				//设置
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

				//Z Offset
				case BUTTON_ZOFFSET_SETUP:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						HMI_ChangePage(PAGE_ZCENTER);
						mbl.active = false;
						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();
						st_synchronize();

						//失效调平补尝
						mbl.active = false;

						strcpy(tmpBuff, "G0 Z0 F1000");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						//设置海平面点的坐标
						sprintf(tmpBuff, "G92 Z%0.4f", -home_offset[Z_AXIS]);
						current_command_args = tmpBuff;
						gcode_G92();
						//相对模式
						relative_mode = true;

						//初始化之后为基准点
						PointIndex = 2;
						
						HMICommandSave = 1;

						//P4..P1 are the mesh corners
						MeshPointZ[0] = mbl.z_values[0][0] - home_offset[Z_AXIS];
						MeshPointZ[2] = mbl.z_values[MESH_NUM_Y_POINTS - 1][0] - home_offset[Z_AXIS];
						MeshPointZ[1] = mbl.z_values[0][MESH_NUM_X_POINTS - 1] - home_offset[Z_AXIS];
						MeshPointZ[3] = mbl.z_values[MESH_NUM_Y_POINTS - 1][MESH_NUM_X_POINTS - 1] - home_offset[Z_AXIS];

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
						//切换到绝对位置模式
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
						//切换到绝对位置模式
						relative_mode = false;
					}
				break;

				case BUTTON_CALIBRATE1_SAVE:
				case BUTTON_CALIBRATE2_SAVE:
				case BUTTON_CALIBRATE3_SAVE:
				case BUTTON_ZOFFSET_CONFIRM:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//切换等待界面
						HMI_ChangePage(PAGE_ZCENTER);

						//P4, P3, P1, P2 are the corners, the rest is interpolated
						mbl.set_from_corners(MeshPointZ[0], MeshPointZ[1],
						                     MeshPointZ[2], MeshPointZ[3]);

						//设置Zoffset
						float X, Y;
						X = (MESH_MAX_X + MESH_MIN_X) / 2.0f;
						Y = (MESH_MAX_Y + MESH_MIN_Y) / 2.0f;
						float DelZ = mbl.get_z(X, Y);

						sprintf(tmpBuff, "M206 Z%0.4f", -(DelZ));
						current_command_args = tmpBuff;
						gcode_M206();

						//P4, P3, P1, P2 are the corners, the rest is interpolated
						mbl.set_from_corners(MeshPointZ[0] - DelZ, MeshPointZ[1] - DelZ,
						                     MeshPointZ[2] - DelZ, MeshPointZ[3] - DelZ);

						//回原点
						mbl.active = true;
						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();

						//保存数据
						Config_StoreSettings();
						HMICommandSave = 0;
						//切换到绝对位置模式
						relative_mode = false;

						//Z  轴走到0
						strcpy(tmpBuff, "G0 Z0"); 
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//切换主界面
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
						//相对位置模式
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
						//相对位置模式
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
						//相对位置模式
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
						//相对位置模式
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
						//相对位置模式
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
						//相对位置模式
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z-0.5");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
					}
				break;

				//重置调平参数
				case BUTTON_CALIBRATE1_RESET:
				case BUTTON_CALIBRATE2_RESET:
				case BUTTON_CALIBRATE3_RESET:
					HMI_ChangePage(PAGE_RESETCALIBRATE);
				break;

				//确定重置调平参数
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
					//失效调平补尝
					mbl.active = false;
					//绝对坐标模式
					relative_mode = false;

					//回到0  点
					strcpy(tmpBuff, "G0 Z0 F1000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					st_synchronize();

					sprintf(tmpBuff, "G92 Z%0.4f", -home_offset[Z_AXIS]);
					current_command_args = tmpBuff;
					gcode_G92();
					
					HMI_ChangePage(PAGE_CALIBRATE1);

					//相对坐标模式
					relative_mode = true;

					//初始化之后为基准点
					PointIndex = 2;
					
					HMICommandSave = 1;

					MeshPointZ[0] = -home_offset[Z_AXIS];
					MeshPointZ[1] = -home_offset[Z_AXIS];
					MeshPointZ[2] = -home_offset[Z_AXIS];
					MeshPointZ[3] = -home_offset[Z_AXIS];
					
					
					HMI_ChangePage(PAGE_CALIBRATE1);
				break;

				//取消重置调平参数
				case BUTTON_CALIBRATE_RESET_BACK:
					HMI_ChangePage(PAGE_CALIBRATE1);
				break;

				case BUTTON_CALIBRATE1_P1:
				case BUTTON_CALIBRATE2_P1:
				case BUTTON_CALIBRATE3_P1:
				case BUTTON_ZOFFSET_P1:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//相对位置模式
						relative_mode = true;
						//向上走2  MM
						strcpy(tmpBuff, "G0 Z2.0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//切换到绝对位置模式
						relative_mode = false;
						//向到调整点位置
						int X, Y;
						X = MESH_MIN_X;
						Y = MESH_MAX_Y;
						sprintf(tmpBuff, "G0 X%d Y%d F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						//切换到绝对位置模式
						relative_mode = false;
						sprintf(tmpBuff, "G0 Z%0.2f", MeshPointZ[2]);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						
						//切换到相对位置模式
						relative_mode = false;
						
						//对应M421 S2
						PointIndex = 2;
					}
				break;

				case BUTTON_CALIBRATE1_P2:
				case BUTTON_CALIBRATE2_P2:
				case BUTTON_CALIBRATE3_P2:
				case BUTTON_ZOFFSET_P2:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//相对位置模式
						relative_mode = true;
						//向上走2  MM
						strcpy(tmpBuff, "G0 Z2.0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						
						//切换到绝对位置模式
						relative_mode = false;
						//向到调整点位置
						int X, Y;
						X = MESH_MAX_X;
						Y = MESH_MAX_Y;
						sprintf(tmpBuff, "G0 X%d Y%d F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						//切换到绝对位置模式
						relative_mode = false;
						sprintf(tmpBuff, "G0 Z%0.2f", MeshPointZ[3]);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//切换到相对位置模式
						relative_mode = true;
						//对应M421 S3
						PointIndex = 3;
					}
				break;

				case BUTTON_CALIBRATE1_P3:
				case BUTTON_CALIBRATE2_P3:
				case BUTTON_CALIBRATE3_P3:
				case BUTTON_ZOFFSET_P3:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//相对位置模式
						relative_mode = true;
						//向上走2  MM
						strcpy(tmpBuff, "G0 Z2.0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						
						//切换到绝对位置模式
						relative_mode = false;
						//向到调整点位置
						int X, Y;
						X = MESH_MAX_X;
						Y = MESH_MIN_Y;
						sprintf(tmpBuff, "G0 X%d Y%d F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						//切换到绝对位置模式
						relative_mode = false;
						sprintf(tmpBuff, "G0 Z%0.2f", MeshPointZ[1]);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//切换到相对位置模式
						relative_mode = true;
						//对应M421 S1
						PointIndex = 1;
					}
				break;

				case BUTTON_CALIBRATE1_P4:
				case BUTTON_CALIBRATE2_P4:
				case BUTTON_CALIBRATE3_P4:
				case BUTTON_ZOFFSET_P4:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//相对位置模式
						relative_mode = true;
						//向上走2  MM
						strcpy(tmpBuff, "G0 Z2.0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						
						//切换到绝对位置模式
						relative_mode = false;
						//向到调整点位置
						int X, Y;
						X = MESH_MIN_X;
						Y = MESH_MIN_Y;
						sprintf(tmpBuff, "G0 X%d Y%d F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						//切换到绝对位置模式
						relative_mode = false;
						sprintf(tmpBuff, "G0 Z%0.2f", MeshPointZ[0]);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//切换到相对位置模式
						relative_mode = true;
						//对应M421 S0
						PointIndex = 0;
					}
				break;

				//停止打印
				case BUTTON_STOP_PRINT:
					//SD  卡打印中
					//if(card.isFileOpen() == true)
					if(GetCurrentPrinterStatus() == STAT_PRINTING)
					{
						//切换到停止界面
						HMI_ChangePage(PAGE_CONFIRMSTOP);
					}
				break;
				
				//停止打印确认
				case BUTTON_STOP_PRINT_CONFIRM:
					//该按钮只在停止工作界面中有效
					//if(CurrentHMIPage == PAGE_CONFIRMSTOP)
					{
						//脱机打印
						if(card.isFileOpen() == true)
						{
						    card.sdprinting = false;
						    card.closefile();
							if(MA_CNC != MachineSelected)
								disable_all_heaters();
							cancel_heatup = true;
							StopTriggle(ManualStop);
							HMI_ChangePage(PAGE_PROCESSING);
							//禁能断电检测
							DisablePowerPanicCheck();
						}
						//联机打印
						else if(GetCurrentPrinterStatus() == STAT_PRINTING)
						{
							if(MA_CNC != MachineSelected)
								disable_all_heaters();
							cancel_heatup = true;
							StopTriggle(ManualStop);
							HMI_ChangePage(PAGE_PROCESSING);
							//禁能断电检测
							DisablePowerPanicCheck();
						}
					}
				break;

				//停止打印取消
				case BUTTON_STOP_PRINT_CANCEL:
					//该按钮只在停止工作界面中有效
					//if(CurrentHMIPage == PAGE_CONFIRMSTOP)
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

				//暂停打印
				case BUTTON_PAUSE_PRINT:
					//文件已打开
					if(card.isFileOpen() == true)
					{
						PauseTriggle(ManualPause);
						HMI_ChangePage(PAGE_PROCESSING);
					}
				break;

				//继续打印
				case BUTTON_CONTINUE_PRINT:
					//文件已打开
					if(card.isFileOpen() == true)
					{
						if(GetCurrentPrinterStatus() ==  STAT_PAUSE)
						{
							PauseResume();
							HMI_ChangePage(PAGE_PROCESSING);
						}
					}
				break;

				//CNC 
				case BUTTON_CNC_PAUSE:
					//U  盘打印
					if(card.isFileOpen() == true)
					{
						//关闭检测
						DisableDoorCheck();
						PauseTriggle(ManualPause); 
						current_block = NULL;
						//block_buffer_head = block_buffer_tail = 0;
						//激光关闭
						SetLaserPower(0);
						//开启检测
						if(isLaserMachine(MachineSelected))
							EnableDoorCheck();
					}
				break;

				case BUTTON_CNC_RUN:
					//if(card.isFileOpen() == true)
					{
						//CNC  功能
						if(MachineSelected == MA_CNC)
						{
							if(GetCurrentPrinterStatus() ==  STAT_PAUSE)
								PauseResume();
						}
						//激光功能
						else
						{
							if(GetCurrentPrinterStatus() ==  STAT_PAUSE)
							{
									//关闭检测
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
									//开启检测
									EnableDoorCheck();
							}
							else
								PauseResume();
						}
					}
				break;

				case BUTTON_CNC_STOP:
					//打印中
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
					//设置原点之后，保持电流
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

				//FAQ
				case BUTTON_FAQ:
					HMI_ChangePage(PAGE_FAQ);
				break;

				//FAQ  返回
				case BUTTON_FAQ_BACK:
					HMI_ChangePage(PAGE_SETTING);
				break;

				//FAQ  回主页
				case BUTTON_FAQ_HOME:
					HMI_ChangePage(PAGE_PRINT);
				break;

				//About
				case BUTTON_ABOUT:
					HMI_ChangePage(PAGE_ABOUT);
				break;

				//About  界面返回
				case BUTTON_ABOUT_BACK:
					HMI_ChangePage(PAGE_SETTING);
				break;

				//About  界面回主页
				case BUTTON_ABOUT_HOME:
					HMI_ChangePage(PAGE_PRINT);
				break;

				//About  界面升级屏幕
				case BUTTON_ABOUT_UPDATE:
					HMI_SendScreenUpdate();
				break;
				
				default:
					sprintf(tmpBuff, PSTR("Unknow:%x"), ID);
					SERIAL_ECHO(tmpBuff);
				break;
			}
		}
		//界面初始化
		else if(eventId == 2)
		{
			ID = (uint16_t)tmpBuff[3];

			uint8_t i;
			//uint8_t tmpPage;
			//tmpPage = CurrentHMIPage;
			CurrentHMIPage = 0xff;
			//获取界面ID
			for(i=0;i<sizeof(PageID);i++)
			{
				if(ID == PageID[i])
				{
					CurrentHMIPage = i;
					break;
				}
			}

			//保存最后界面ID
			switch(CurrentHMIPage)
			{
				case PAGE_PRINT:
					//SD  打印
					if(card.isFileOpen()== true)
					{
						HMI_StopEnable(true);
						HMI_SetControlEnable(false);
						HMI_SetSettingEnable(false);
						//本次检测被拔出来
						if(!(IS_UDISK_INSERTED))
						{
							//设置图标
							HMI_SetTFIcon(false);
						}
						else
						{
							//设置图标
							HMI_SetTFIcon(true);
						}
						//正在打印
						if(card.sdprinting == true)
						{
							HMI_ChangePrinterStatus(1);
						}
						//暂停打印
						else
						{
							HMI_ChangePrinterStatus(2);
						}
					}
					//没有SD  打印
					else
					{
						//本次检测被拔出来
						if(!(IS_UDISK_INSERTED))
						{
							//设置图标
							HMI_SetTFIcon(false);
						}
						else
						{
							//设置图标
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
					//SD  打印
					if(card.isFileOpen()== true)
					{
						HMI_StopEnable(true);
						HMI_PauseEnable(true);
						HMI_SetControlEnable(false);
						HMI_ChangePrinterStatus(1);
					}
					else
					{
						//本次检测被拔出来
						if(!(IS_UDISK_INSERTED))
						{
							//设置图标
							HMI_SetTFIcon(false);
						}
						else
						{
							//设置图标
							HMI_SetTFIcon(true);
						}
					}
					//显示功率
					HMI_ShowPrintTime();
					HMI_ShowLaserPower(laserPercent);
					HMI_ShowPrintFileName(CardFileNameSelected);
					HMI_ShowProgressBar(card.LastPercent);
				break;
				
				case PAGE_FILELIST:
					//文件选择对话框初始化
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
					//清空列表
					for(i=0;i<FILE_PER_PAGE;i++)
					{
						CardFileName[i][0] = 0;
						PrintFileName[i][0] = 0;
					}
					//显示文件个数与查找个数相同
					ShowFileCount = FILE_PER_PAGE;
					//标识需要查找文件
					SeekingFile = 1;
					//显示文件索引从0开始
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
		//查询页面
		else if(eventId == 3)
		{
			ID = (uint16_t)tmpBuff[3];
			uint8_t i;
			uint8_t tmpPage;
			tmpPage = CurrentHMIPage;
			CurrentHMIPage = 0xff;
			//获取界面ID
			for(i=0;i<sizeof(PageID);i++)
			{
				if(ID == PageID[i])
				{
					CurrentHMIPage = i;
					break;
				}
			}

			//屏幕热插拔
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
	

	//文件列举中
	if(SeekingFile == 1)
	{
		//需要列举的文件个数大于0，表示还要列举
		if(SeekFileCount > 0)
		{
			card.getfilename(FileIndex++, NULL, FileFilter);
			//防止溢出
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

			//防止溢出
			card.filename[FILENAME_LENGTH - 1] = 0;
			for(j=0;j<strlen(card.filename);j++)
			{
				PrintFileName[ShowFileIndex][j] = card.filename[j];
			}
			PrintFileName[ShowFileIndex][j] = 0;
			//列举个数减1
			SeekFileCount--;
			//索引加1
			ShowFileIndex++;
		}
		//不需要列举文件
		else
		{
			//清除文件列举中标置
			SeekingFile = 0;
			//显示文件索引从0开始
			ShowFileIndex = 0;
		}
	}
	//文件不在列举，显示文件个数大于0，表示有文件需要显示
	else if(ShowFileCount > 0)
	{
		HMI_ShowFileName(ShowFileIndex, CardFileName[ShowFileIndex]);
		//显示个数减1
		ShowFileCount--;
		//显示索引加1
		ShowFileIndex++;
	}
}
#else

//清除串口数据
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
		if((millis() - CheckComTick) > 1000)
		{
			CheckComTick = millis();
			HMI_GetCurrentPage();
			if(NoComCount < 10)
				NoComCount++;
		}
		*/
	}
	//屏幕协议
	else
	{
		cmdLen = (tmpBuff[2] << 8) | tmpBuff[3];
		
		eventId = tmpBuff[8];
		//按钮
		if(eventId == 9)
		{
			ID = (uint16_t)(tmpBuff[9] << 8) | (uint8_t)tmpBuff[10];
			ShieldLedOn();
			
			switch((uint16_t)ID)
			{
				case BUTTON_MOTOR_HOME:
					//电机不在同步中
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
					//电机不在同步中
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
								//关闭激光，CNC
								SetLaserPower(0);
								digitalWrite(LASER_PORT, LASER_PIN, 0);
							}
						}
						relative_mode = false;
						HMICommandSave = 0;
					}
				break;

				
				
				//Z Offset
				case BUTTON_ZOFFSET_SETUP:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						HMI_ChangePage(PAGE_ZCENTER);

						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();
						st_synchronize();

						mbl.active = false;

						//绝对坐标模式
						relative_mode = false;

						sprintf(tmpBuff, "G0 X%0.2f Y%0.2f", home_offset[X_AXIS], home_offset[Y_AXIS]);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						strcpy(tmpBuff, "G0 Z0");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						
						//设置海平面点的坐标
						sprintf(tmpBuff, "G92 Z%0.2f", -home_offset[Z_AXIS]);
						current_command_args = tmpBuff;
						gcode_G92();
						
						HMI_ChangePage(PAGE_CALIBRATE1);

						relative_mode = true;

						//初始化之后为基准点
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
						//切换到绝对位置模式
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
						//切换到绝对位置模式
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
						//切换等待界面
						HMI_ChangePage(PAGE_ZCENTER);

						//P1
						mbl.set_z(0, 1, MeshPointZ[2]);

						//P2
						mbl.set_z(1, 1, MeshPointZ[3]);
						
						//P3
						mbl.set_z(1, 0, MeshPointZ[1]);

						//P4
						mbl.set_z(0, 0, MeshPointZ[0]);

						//设置Zoffset
						float X, Y;
						X = (MESH_MAX_X + MESH_MIN_X) / 2.0f;
						Y = (MESH_MAX_Y + MESH_MIN_Y) / 2.0f;
						float DelZ = mbl.get_z(X, Y);

						sprintf(tmpBuff, "M206 Z%0.2f", -(DelZ));
						current_command_args = tmpBuff;
						gcode_M206();

						//P1
						mbl.set_z(0, 1, MeshPointZ[2] - DelZ);

						//P2
						mbl.set_z(1, 1, MeshPointZ[3] - DelZ);
						
						//P3
						mbl.set_z(1, 0, MeshPointZ[1] - DelZ);

						//P4
						mbl.set_z(0, 0, MeshPointZ[0] - DelZ);

						//回原点
						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();
						st_synchronize();

						//切换到绝对位置模式
						relative_mode = false;
						//切换主界面
						HMI_ChangePage(PAGE_PRINT);

						//保存数据
						Config_StoreSettings();
						HMICommandSave = 0;
						//切换到绝对位置模式
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
						//相对位置模式
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
						//相对位置模式
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
						//相对位置模式
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
						//相对位置模式
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
						//相对位置模式
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
						//相对位置模式
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z-0.5");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						MeshPointZ[PointIndex] = current_position[Z_AXIS];
					}
				break;

				//重置调平参数
				case BUTTON_CALIBRATE1_RESET:
				case BUTTON_CALIBRATE2_RESET:
				case BUTTON_CALIBRATE3_RESET:
					HMI_ChangePage(PAGE_RESETCALIBRATE);
				break;

				//确定重置调平参数
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

					//初始化之后为基准点
					PointIndex = 2;
					
					HMICommandSave = 1;

					MeshPointZ[0] = -home_offset[Z_AXIS];
					MeshPointZ[1] = -home_offset[Z_AXIS];
					MeshPointZ[2] = -home_offset[Z_AXIS];
					MeshPointZ[3] = -home_offset[Z_AXIS];
					
					//失效调平补尝
					mbl.active = false;
					HMI_ChangePage(PAGE_CALIBRATE1);
				break;

				//取消重置调平参数
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
						//相对位置模式
						relative_mode = true;
						//向上走2  MM
						strcpy(tmpBuff, "G0 Z2.0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//切换到绝对位置模式
						relative_mode = false;
						//向到调整点位置
						int X, Y;
						X = MESH_MIN_X;
						Y = MESH_MAX_Y;
						sprintf(tmpBuff, "G0 X%d Y%d F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						//切换到绝对位置模式
						relative_mode = false;
						sprintf(tmpBuff, "G0 Z%0.2f", MeshPointZ[2]);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						
						//切换到相对位置模式
						relative_mode = false;
						
						//对应M421 S2
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
						//相对位置模式
						relative_mode = true;
						//向上走2  MM
						strcpy(tmpBuff, "G0 Z2.0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						
						//切换到绝对位置模式
						relative_mode = false;
						//向到调整点位置
						int X, Y;
						X = MESH_MAX_X;
						Y = MESH_MAX_Y;
						sprintf(tmpBuff, "G0 X%d Y%d F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						//切换到绝对位置模式
						relative_mode = false;
						sprintf(tmpBuff, "G0 Z%0.2f", MeshPointZ[3]);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//切换到相对位置模式
						relative_mode = true;
						//对应M421 S3
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
						//相对位置模式
						relative_mode = true;
						//向上走2  MM
						strcpy(tmpBuff, "G0 Z2.0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						
						//切换到绝对位置模式
						relative_mode = false;
						//向到调整点位置
						int X, Y;
						X = MESH_MAX_X;
						Y = MESH_MIN_Y;
						sprintf(tmpBuff, "G0 X%d Y%d F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						//切换到绝对位置模式
						relative_mode = false;
						sprintf(tmpBuff, "G0 Z%0.2f", MeshPointZ[1]);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//切换到相对位置模式
						relative_mode = true;
						//对应M421 S1
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
						//相对位置模式
						relative_mode = true;
						//向上走2  MM
						strcpy(tmpBuff, "G0 Z2.0 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						
						//切换到绝对位置模式
						relative_mode = false;
						//向到调整点位置
						int X, Y;
						X = MESH_MIN_X;
						Y = MESH_MIN_Y;
						sprintf(tmpBuff, "G0 X%d Y%d F3000", X, Y);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();

						//切换到绝对位置模式
						relative_mode = false;
						sprintf(tmpBuff, "G0 Z%0.2f", MeshPointZ[0]);
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//切换到相对位置模式
						relative_mode = true;
						//对应M421 S0
						PointIndex = 0;
					}
				break;
				
				//停止打印确认
				case BUTTON_STOP_PRINT_CONFIRM:
					//该按钮只在停止工作界面中有效
					//if(CurrentHMIPage == PAGE_CONFIRMSTOP)
					{
						//脱机打印
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
							//禁能断电检测
							DisablePowerPanicCheck();
							if(MA_LASER == MachineSelected)
								DisableDoorCheck();
						}
						//联机打印
						else if((CurStatus == STAT_PRINTING_ONLINE) || (CurStatus == STAT_PAUSE_ONLINE))
						{
							if(MA_CNC != MachineSelected)
								disable_all_heaters();
							cancel_heatup = true;
							StopTriggle(ManualStop);
							HMI_ChangePage(PAGE_PROCESSING);
							//禁能断电检测
							DisablePowerPanicCheck();
							if(MA_LASER == MachineSelected)
								DisableDoorCheck();
						}
					}
				break;


				//暂停打印
				case BUTTON_PAUSE_PRINT:
					//文件已打开
					if(GetCurrentPrinterStatus() == STAT_PRINTING)
					{
						PauseTriggle(ManualPause);
					}	
					//脱机打印
					else if(GetCurrentPrinterStatus() == STAT_PRINTING_ONLINE)
					{
						PauseTriggle(ManualPause);
					}
				break;

				//继续打印
				case BUTTON_CONTINUE_PRINT:
					//U  盘打印
					if(GetCurrentPrinterStatus() ==  STAT_PAUSE)
					{
						//激光执行头
						if(isLaserMachine(MachineSelected) == 1)
						{
							#ifdef DOOR_PORT
							//关闭检测
							//DisableDoorCheck();
							//门已关闭
							if(READ(DOOR_PORT,DOOR_PIN) == false)
							{
								//启动打印
								PauseResume();
							}
							//开启检测
							EnableDoorCheck();
							#else
							PauseResume();
							#endif
						}
						else
						{
							PauseResume();
						}
						//清除故障标志
						SysFaultFlag &= ~FAULT_FLAG_FILAMENT;
					}
					//脱机打印
					else if(GetCurrentPrinterStatus() == STAT_PAUSE_ONLINE)
					{
						HMI_SendContinuePrint();
						//清除故障标志
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
					//设置原点之后，保持电流
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
					//关闭激光，CNC
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
				以下为测试项，不用时清除
				********************************************/
				case BUTTON_ZHA_ZOFFSET_SETUP:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//关闭热床和加热头
						disable_all_heaters();
						
						HMI_ChangePage(PAGE_ZCENTER);
						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();
						while(blocks_queued());
						mbl.active = false;

						//设置Z  轴最大速度
						strcpy(tmpBuff, "M203 Z50");
						current_command_args = tmpBuff;
						gcode_M203();

						//设置海平面点的坐标
						current_position[Z_AXIS] = base_max_pos[Z_AXIS];
						sync_plan_position();

						//第1  点
						//绝对坐标模式
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

						//相对坐标模式
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

						//第2  点
						//绝对坐标模式
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

						//相对坐标模式
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

						//第3  点
						//绝对坐标模式
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

						//相对坐标模式
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

						//第4  点
						//绝对坐标模式
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

						//相对坐标模式
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

						//中心点
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

						//相对坐标模式
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

						//设置Z  轴最大速度
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
						//切换等待界面
						HMI_ChangePage(PAGE_ZCENTER);

						//P1
						mbl.set_z(0, 0, MeshPointZ[0] - HalfAutoCalibrateZPosition);

						//P2
						mbl.set_z(0, 1, MeshPointZ[1] - HalfAutoCalibrateZPosition);
						
						//P3
						mbl.set_z(1, 0, MeshPointZ[2] - HalfAutoCalibrateZPosition);

						//P4
						mbl.set_z(1, 1, MeshPointZ[3] - HalfAutoCalibrateZPosition);

						sprintf(tmpBuff, "M206 Z%0.2f", -current_position[Z_AXIS]);
						current_command_args = tmpBuff;
						gcode_M206();

						//回原点
						strcpy(tmpBuff, "G28");
						current_command_args = tmpBuff;
						gcode_G28();
						while(blocks_queued());

						//切换到绝对位置模式
						relative_mode = false;
						//切换主界面
						HMI_ChangePage(PAGE_PRINT);

						//保存数据
						Config_StoreSettings();
						HMICommandSave = 0;
						//切换到绝对位置模式
						relative_mode = false;
					}
				break;
				
				//重置调平参数
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
						//切换到绝对位置模式
						relative_mode = false;
					}
				break;

				//确定重置调平参数
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
					
					//失效调平补尝
					mbl.active = false;
					HMI_ChangePage(PAGE_CALIBRATE_HA1);
				break;

				//取消重置调平参数
				case BUTTON_CALIBRATE_HA_RESET_BACK:
					HMI_ChangePage(PAGE_CALIBRATE_HA1);
				break;

				
				case BUTTON_ZHA_CALIBRATE_UP_050:
					if((StepperSync == false) && (CMD_BUFF_EMPTY() == true))
					{
						//相对位置模式
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
						//相对位置模式
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
						//相对位置模式
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
						//相对位置模式
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
						//相对位置模式
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
						//相对位置模式
						relative_mode = true;
						strcpy(tmpBuff, "G0 Z-0.05");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
					}
				break;
				/********************************************
				以上为测试项，不用时清除
				********************************************/

				//获取断点数据
				case BUTTON_GET_BREAK_DATA:
					HMI_SendBreakPointData();
				break;

				//断点续打
				case BUTTON_RESUME_BREAK_PRINT:
					PowerPanicResumeWork(NULL);
				break;

				//挤出
				case BUTTON_EXTRDE:
					if(MA_PRINT == MachineSelected)
					{
						//相对模式
						relative_mode = true;
						strcpy(tmpBuff, "G0 E5 F600");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						st_synchronize();
						//绝对模式
						relative_mode = false;
						
					}
				break;

				//进入激光高度设置
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
		
		//上位指令调试
		else if(eventId == 0x01)
		{
			ShieldLedOn();
			//指令尾补0
			j = cmdLen + 12;
			tmpBuff[j] = 0;
			Screen_enqueue_and_echo_commands(&tmpBuff[13], 0xffffffff, 0x02);
		}
		//GCode  打印
		else if(eventId == 0x03)
		{
			ShieldLedOn();
			CurStatus = GetCurrentPrinterStatus();
			//上位启动打印
			if((CurStatus== STAT_PRINTING_ONLINE) || (CurStatus== STAT_PAUSE_ONLINE))
			{
				//激光头
				if(isLaserMachine(MachineSelected) == 1)
				{
					//行号
					ID = (tmpBuff[9] << 24) | (tmpBuff[10] << 16) | (tmpBuff[11] << 8) | tmpBuff[12];
					//指令尾补0
					j = tmpBuff[3] + 8;
					tmpBuff[j] = 0;
					
					#ifdef DOOR_PORT
					if(DoorCheckEnable == true)
					{
						//门已关闭
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
				//非激光头
				else
				{
					//行号
					ID = (tmpBuff[9] << 24) | (tmpBuff[10] << 16) | (tmpBuff[11] << 8) | tmpBuff[12];
					//指令尾补0
					j = tmpBuff[3] + 8;
					tmpBuff[j] = 0;
					Screen_enqueue_and_echo_commands(&tmpBuff[13], ID, 0x04);
				}
			}
		}
		//文件操作
		else if(eventId == 0x05)
		{
			ShieldLedOn();
			//控制字
			switch(tmpBuff[9])
			{
				//重新挂载
				case 0:
					card.initsd();
					if(card.cardOK == true)
						HMI_SendInitUdisk(0);
					else
						HMI_SendInitUdisk(1);
					//清除项目显示偏移量
					ListFileOffset = 0;
				break;

				//获取当前目录路径
				case 1:
					if(card.getWorkDirPath() == true)
						HMI_SendCurrentUDiskPath(0);
					else
						HMI_SendCurrentUDiskPath(1);
				break;

				//进入目录
				case 2:
					ID = 0;
					FileListBuff[ID] = 0;
					//清除项目显示偏移量
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

				//返回上一级
				case 3:
				break;

				//获取当前目录的内容
				case 4:
					ListFileOffset += HMI_SendDirItems(0, ListFileOffset);
				break;

				//获取文件特殊内容
				case 5:
					//待机状态下可操作
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

				//启动U  盘打印
				case 6:
					CurStatus = GetCurrentPrinterStatus();
					//待机状态下可操作
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
						//文件打开成功
						if(card.isFileOpen() == false)
						{
							//发送文件打开失败
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

						//激光执行头
						if(isLaserMachine(MachineSelected) == 1)
						{
							//Z  未知坐标
							if(HomeAfterReset[Z_AXIS] == false)
							{
								//Z  轴回原点
								strcpy(tmpBuff, "G28 Z");
								current_command_args = tmpBuff;
								gcode_G28();
							}
							//走到工件坐标
							strcpy(tmpBuff, "G0 X0 Y0");
							current_command_args = tmpBuff;
							gcode_G0_G1(0);
							while(blocks_queued());
						}
						//使能检测
						EnablePowerPanicCheck();
						if(isLaserMachine(MachineSelected) == 1)
						{
							//门已关闭
							if(READ(DOOR_PORT, DOOR_PIN) == true)
							{
								PauseTriggle(ManualPause);
								SetCurrentPrinterStatus(STAT_PAUSE);
							}
						}
						//启动门检测
						if(isLaserMachine(MachineSelected) == 1)
							EnableDoorCheck();
					}
				break;
			}
		}
		//状态上报
		else if(eventId == 0x07)
		{
			uint8_t StatuID;
			block_t *pBlock;
			StatuID = tmpBuff[9];
			//获取当前状态
			CurStatus = GetCurrentPrinterStatus();
			//暂停
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
			//继续
			else if(StatuID == 0x05)
			{
				CurStatus = GetCurrentPrinterStatus();
				//U  盘打印
				if(CurStatus ==  STAT_PAUSE)
				{
					//激光执行头
					if(isLaserMachine(MachineSelected) == 1)
					{
						#ifdef DOOR_PORT
						//关闭检测
						//DisableDoorCheck();
						//门已关闭
						if(READ(DOOR_PORT,DOOR_PIN) == false)
						{
							//启动打印
							PauseResume();
						}
						//开启检测
						EnableDoorCheck();
						#else
						PauseResume();
						#endif
					}
					else
					{
						PauseResume();
					}
					//清除故障标志
					SysFaultFlag &= ~FAULT_FLAG_FILAMENT;
					HmiRequestStatus = STAT_PRINTING;
				}
				//连机打印
				else if(CurStatus == STAT_PAUSE_ONLINE)
				{
					PauseResume();
					//清除故障标志
					SysFaultFlag &= ~FAULT_FLAG_FILAMENT;
					HmiRequestStatus = STAT_PRINTING_ONLINE;
					
				}
			}
			//联机打印
			else if(StatuID == 0x03)
			{
				//待机状态中
				if(CurStatus == STAT_IDLE)
				{
					//设置打印状态
					SetCurrentPrinterStatus(STAT_PRINTING_ONLINE);
					//打印数据源
					PowerPanicData.GCodeSource = GCODE_SOURCE_SCREEN;
					/*
					//激光执行头
					if(isLaserMachine(MachineSelected) == 1)
					{
						//Z  未知坐标
						if(HomeAfterReset[Z_AXIS] == false)
						{
							//Z  轴回原点
							strcpy(tmpBuff, "G28 Z");
							current_command_args = tmpBuff;
							gcode_G28();
						}
						//走到工件坐标
						strcpy(tmpBuff, "G0 X0 Y0");
						current_command_args = tmpBuff;
						gcode_G0_G1(0);
						while(blocks_queued());
					}
					*/
					//使能检测
					#if(HAVE_FALIMENT_SENSOR == 1)
					if(MA_PRINT == MachineSelected)
						EnableFilamentCheck();
					#endif
					HMI_SendMachineStatusChange(0x03, 0);
				}
			}
			//停止
			else if(StatuID == 0x06)
			{
				if(CurStatus != STAT_IDLE)
				{
					//上位机停止分2  种，立即停止
					StopTriggle(ManualStop);
				}
			}
			//请求最近行号
			else if(StatuID == 0x07)
			{
				HMI_SendBreakPointLine();
			}
			//打印结束
			else if(StatuID == 0x08)
			{
				if(CurStatus != STAT_IDLE)
				{
					StopTriggle(ManualStop);
				}
			}
			//查询状态
			else if(StatuID == 0x01)
			{
				HMI_SendMachineStatus();
			}
		}
		//操作指令
		else if(eventId == 0x09)
		{
			switch(tmpBuff[9])
			{
				//移动轴
				case 0x01:
					//借用缓冲器
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

					//相对位置模式
					relative_mode = true;
					current_command_args = FileListBuff;
					gcode_G0_G1(0);
					st_synchronize();
					//绝对位置模式
					relative_mode = false;
				break;
			}
		}
		//ReadTail = ReadHead;
	}
	
}

/***********************************************
回复暂停打印行号
***********************************************/
void HMI_SendBreakPointLine()
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	//行号
	tmpBuff[i++] = 0x07;

	//断点数据
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 24);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 16);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 8);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition);

	//重填包长
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
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
发送断点数据
************************************************/
void HMI_SendBreakPointData()
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x77;

	//断点数据
	tmpBuff[i++] = PowerPanicData.Valid;
	tmpBuff[i++] = PowerPanicData.GCodeSource;
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 24);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 16);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 8);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition);

	//重填包长
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
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
发送报警
************************************************/
void HMI_SendMachineFaultFlag()
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	//异常上报
	tmpBuff[i++] = 0x02;
	//异常标志
	tmpBuff[i++] = (uint8_t)(SysFaultFlag >> 24);
	tmpBuff[i++] = (uint8_t)(SysFaultFlag >> 16);
	tmpBuff[i++] = (uint8_t)(SysFaultFlag >> 8);
	tmpBuff[i++] = (uint8_t)(SysFaultFlag);

	//打印文件源
	tmpBuff[i++] = PowerPanicData.GCodeSource;

	//行号
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 24);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 16);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 8);
	tmpBuff[i++] = (uint8_t)(PowerPanicData.FilePosition);

	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
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
发送状态切变
************************************************/
void HMI_SendMachineStatusChange(uint8_t Status, uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	//目前状态
	tmpBuff[i++] = Status;
	//处理结果
	tmpBuff[i++] = Result;

	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
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
发送状态信息
************************************************/
void HMI_SendMachineStatus()
{
	float fValue;
	uint32_t u32Value;
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长效验
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	//同步状态
	tmpBuff[i++] = 0x01;

	//坐标
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

	//温度
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

	//打印机状态
	j = GetCurrentPrinterStatus();
	tmpBuff[i++] = (uint8_t)(j);

	//外设状态
	//tmpBuff[i++] = (uint8_t)(SysStatusFlag >> 24);
	//tmpBuff[i++] = (uint8_t)(SysStatusFlag >> 16);
	//tmpBuff[i++] = (uint8_t)(SysStatusFlag >> 8);
	tmpBuff[i++] = (uint8_t)(SysStatusFlag);

	//执行头类型
	tmpBuff[i++] = MachineSelected;

	//重填长度
	tmpBuff[2] = (uint8_t)((i - 8) >> 8);
	tmpBuff[3] = (uint8_t)(i - 8);
	tmpBuff[5] = tmpBuff[2] ^ tmpBuff[3];
	//校验
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
重新挂载U  盘
************************************************/
void HMI_SendInitUdisk(uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//包长
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//协议版本
	FileListBuff[i++] = 0x00;
	//包长效验
	FileListBuff[i++] = 0x00;
	//校验
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = 0;
	FileListBuff[i++] = Result;
	
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//校验
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
进入目录应答
************************************************/
void HMI_SendChDirResult(uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//包长
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//协议版本
	FileListBuff[i++] = 0x00;
	//包长效验
	FileListBuff[i++] = 0x00;
	//校验
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = 0x02;
	FileListBuff[i++] = Result;
	
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//校验
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
获取UDisk  当前目录应答
************************************************/
void HMI_SendCurrentUDiskPath(uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//包长
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//协议版本
	FileListBuff[i++] = 0x00;
	//包长效验
	FileListBuff[i++] = 0x00;
	//校验
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
	//校验
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
获取目录内容
返回值:  本次发送的个数
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
	//包头
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//包长
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//协议版本
	FileListBuff[i++] = 0x00;
	//包长效验
	FileListBuff[i++] = 0x00;
	//校验
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = 0x04;
	//结果
	FileListBuff[i++] = 0x00;
	//Offset
	FileListBuff[i++] = (uint8_t)(Offset >> 8);
	FileListBuff[i++] = (uint8_t)Offset;
	//个数
	FileListBuff[i++] = 0x00;
	//一次最多传20  个，且总数据长度在缓冲大小之内
	while((Count < 10) && ((i + 257) < sizeof(FileListBuff)))
	{
		//读取目录 
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

	//填结果
	if(Res == (char)-2)
		FileListBuff[10] = 1;
		
	//重填个数
	FileListBuff[13] = Count;
	
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//校验
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
获取文件的特殊内容
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
	//文件打开成功
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
			//计算内容长度
			ContentLen = (uint32_t)((FileListBuff[19] << 24) | (FileListBuff[18] << 16) | (FileListBuff[17] << 8) | FileListBuff[16]) - 20;
		}
	}
	else
	{
		ContentLen = 0;
	}

	i=0;
	//包头
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//包长
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//协议版本
	FileListBuff[i++] = 0x00;
	//包长效验
	FileListBuff[i++] = 0x00;
	//校验
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = 0x05;
	FileListBuff[i++] = dataValid;
	do
	{
		i = 11;
		//正常的特殊文件
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
		//校验
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
打印文件应答
************************************************/
void HMI_SendStartPrintReack(uint8_t Result)
{
	uint16_t i;
	uint16_t j;
	uint32_t checksum;
	i=0;
	//包头
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//包长
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//协议版本
	FileListBuff[i++] = 0x00;
	//包长效验
	FileListBuff[i++] = 0x00;
	//校验
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = 0x06;
	FileListBuff[i++] = Result;
	
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//校验
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


//发送Gcode
void HMI_SendGcode(char *GCode, uint8_t EventID)
{
	uint8_t i, j;
	uint32_t checksum;
	i = 0;

	//包头
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//包长
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//协议版本
	FileListBuff[i++] = 0x00;
	//包长效验
	FileListBuff[i++] = 0x00;
	//校验
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = EventID;
	while(*GCode != 0)
		FileListBuff[i++] = *GCode++;
	
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//校验
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

//发送继续打印
void HMI_SendContinuePrint(void)
{
	uint8_t i, j;
	uint32_t checksum;
	i = 0;
	
	//包头
	FileListBuff[i++] = 0xAA;
	FileListBuff[i++] = 0x55;
	//包长
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//协议版本
	FileListBuff[i++] = 0x00;
	//包长效验
	FileListBuff[i++] = 0x00;
	//校验
	FileListBuff[i++] = 0x00;
	FileListBuff[i++] = 0x00;
	//EventID
	FileListBuff[i++] = 0x52;
	FileListBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 24);
	FileListBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 16);
	FileListBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 8);
	FileListBuff[i++] = (uint8_t)(PowerPanicData.FilePosition >> 0);
	//包长
	FileListBuff[2] = (uint8_t)((i - 8) >> 8);
	FileListBuff[3] = (uint8_t)(i - 8);
	FileListBuff[5] = FileListBuff[2] ^ FileListBuff[3];
	//校验
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
	
	//主界面才发送
	if(CurrentHMIPage == PAGE_PRINT)
	{
		if (nextShowMill < millis())
		{
			//显示喷头温度
			HMI_ShowTemperature(0, (int)current_temperature[0], target_temperature[0]);
			//显示床温度
			HMI_ShowTemperature(1, (int)current_temperature_bed, target_temperature_bed);
			
			//文件已经打开
			if (card.isFileOpen())
			{
				//获取文件进度
				percent = card.percentDone();
				card.LastPercent = percent;

				//显示进度条
				HMI_ShowProgressBar(percent);

				//显示打印时间	
				HMI_ShowPrintTime();
				
			}
			nextShowMill = millis() + 1000;
		}
	}
	else if(CurrentHMIPage == PAGE_CNC)
	{
		if (nextShowMill < millis())
		{
			//文件已经打开
			if (card.isFileOpen())
			{
				//获取文件进度
				percent = card.percentDone();
				card.LastPercent = percent;

				//显示进度条
				HMI_ShowProgressBar(percent);
				
				//显示打印时间	
				HMI_ShowPrintTime();
			}
			if(isLaserMachine(MachineSelected))
			{
				HMI_ShowLaserPower(laserPercent);//刷新屏幕显示	
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
	
	
	//存储器已经插入
	if(SdInsert == true)
	{
		//本次检测被拔出来
		if(!(IS_UDISK_INSERTED))
		{
			//设置图标
			HMI_SetTFIcon(false);
			//标识卡被拔出
			SdInsert = false;
			//退出主界面
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
	//SD  卡未插入
	else
	{
		//本次检测已插入
		if(IS_UDISK_INSERTED)
		{
			//设置图标
			HMI_SetTFIcon(true);
			//标识卡已插入
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
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	#else
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//校验
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
	//校验
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
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	#else
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//校验
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
	//校验
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
	//校验
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
	//校验
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
	//校验
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
	//校验
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
	//校验
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
	//校验
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
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	#else
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//校验
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
	//校验
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
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//校验
	tmpBuff[i++] = 0x00;
	//EventID
	tmpBuff[i++] = 0x08;
	tmpBuff[i++] = id[0];
	tmpBuff[i++] = id[1];
	tmpBuff[i++] = id[2];
	tmpBuff[i++] = id[3];
	#endif

	//获取文件名长度
	uint8_t Len;
	Len = strlen(FileName);
	//长度超过显示范围
	if(Len > 32)
	{
		//规划显示
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
		//扩展名在10  个字节以内表示正常
		if((extlen < 10) && (extlen > 0))
		{
			extlen++;
			//计算除去后缀，可显示长度
			sflen = 31 - extlen;
			//计算文件名
			fl = Len - extlen;
			//计算省略号
			dotcount = fl - sflen;
			//省略号过多
			if(dotcount > 3)
			{
				if((sflen % 2) != 0)
					dotcount = 3;
				else
					dotcount = 4;
			}

			//计算前后各占多少字节 
			divlen = (sflen - dotcount) / 2;

			//前半部份文件名
			for(j=0;j<divlen;j++)
				tmpBuff[i++] = FileName[j];

			//  省略号
			for(j=0;j<dotcount;j++)
				tmpBuff[i++] = '.';
			
			//后半部份文件名
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
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	#else
	tmpBuff[3] = (uint8_t)((i - 6) >> 8);
	tmpBuff[4] = (uint8_t)(i - 6);
	//校验
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
	//校验
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
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	#else
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//校验
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
	//校验
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
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	#else
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//校验
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
	//校验
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
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	#else
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//校验
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
	//校验
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
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	#else
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//校验
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
	//校验
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
	//校验
	checksum = 0;
	for(j=0;j<(i-3);j++)
		checksum = checksum + tmpBuff[j+3];

	checksum = 0x100 - checksum;
	tmpBuff[i++] = checksum;
	#else
	//包头
	tmpBuff[i++] = 0xAA;
	tmpBuff[i++] = 0x55;
	//协议版本
	tmpBuff[i++] = 0x00;
	//包长
	tmpBuff[i++] = 0x00;
	tmpBuff[i++] = 0x00;
	//校验
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
	//校验
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
	//校验
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
	//校验
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
	//校验
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
