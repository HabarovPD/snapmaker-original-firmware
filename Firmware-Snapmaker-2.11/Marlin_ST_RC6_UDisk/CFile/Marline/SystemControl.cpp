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

//检测指令缓冲是否为空
#define CMD_BUFF_EMPTY()	(GetCommandBuff()>0?false:true)
static char tmpBuff[96];

//暂停时的位置
static float PausePosition[4];
//停止打印类型
static StopPrintType StopType;
//停止打印类型
static PausePrintType PauseType;


//立即停止运行标置
static volatile bool SystemRequestStop;
//打机印状态
static PrinterStatus PrintingStatus;
//上位机指定的模式
//static WordMode ModeFromCommando;


/********************************************
使能执行头断开检测
********************************************/
void EnableExecuterCheck()
{
	CRITICAL_SECTION_START;
	EXTI->PR = EXTI_Line14 | EXTI_Line15;
	EXTI->IMR |= EXTI_Line14 | EXTI_Line15;
	CRITICAL_SECTION_END;
}

/********************************************
禁能执行头断开检测
********************************************/
void DisableExecuterCheck()
{
	EXTI->IMR &= ~EXTI_Line14 | EXTI_Line15;
	EXTI->PR = EXTI_Line14 | EXTI_Line15;
}


/***********************************************
待机状态 下处理G  代码
***********************************************/
void SystemIdleGcodeProcess(void)
{
  current_command = command_queue[cmd_queue_index_r];

  if (DEBUGGING(ECHO)) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLN(current_command);
  }

  // Sanitize the current command:
  //  - Skip leading spaces
  //  - Bypass N[-0-9][0-9]*[ ]*
  //  - Overwrite * with nul to mark the end
  while (*current_command == ' ') ++current_command;
  if (*current_command == 'N' && NUMERIC_SIGNED(current_command[1])) {
    current_command += 2; // skip N[-0-9]
    while (NUMERIC(*current_command)) ++current_command; // skip [0-9]*
    while (*current_command == ' ') ++current_command; // skip [ ]*
  }
  char* starpos = strchr(current_command, '*');  // * should always be the last parameter
  if (starpos) while (*starpos == ' ' || *starpos == '*') *starpos-- = '\0'; // nullify '*' and ' '

  char *cmd_ptr = current_command;

  uint16_t codenum = 0; // define ahead of goto

  bool code_is_good;

  // Get the command code, which must be G, M, or T
  char command_code = *cmd_ptr++;

  // Skip spaces to get the numeric part
  while (*cmd_ptr == ' ') cmd_ptr++;

  // Bail early if there's no code
  if(command_code == 'M')
  {
    code_is_good = NUMERIC(*cmd_ptr);
    if (!code_is_good) goto ExitUnknownCommand;
    
    // Get and skip the code number
    do {
    codenum = (codenum * 10) + (*cmd_ptr - '0');
    cmd_ptr++;
    } while (NUMERIC(*cmd_ptr));
  }

  // Skip all spaces to get to the first argument, or nul
  while (*cmd_ptr == ' ') cmd_ptr++;

  // The command's arguments (if any) start here, for sure!
  current_command_args = cmd_ptr;

  //KEEPALIVE_STATE(1);

  if(command_code == 'M')
  {
    switch(codenum)
	{
		//进入运行模式
		case 4000:
			//ModeFromCommando = WORK_MODE_RUNNING;
		break;

		//进入待机模式
		case 4001:
			//ModeFromCommando = WORK_MODE_IDLE;
		break;

		//查询执行头状态
		case 4002:
			
		break;
	}
  }

  //KEEPALIVE_STATE(0);

ExitUnknownCommand:

  // Still unknown command? Throw an error
  if (!code_is_good) unknown_command_error();

  ok_to_send();

}

#if(SNAP_OLD_BASE == 0)
/***********************************************
待机处理线程
检测执行头插入状态
处理指定的一些Gcode
根据执行头插入状态与上位机指定状态进行模式切换
***********************************************/
void SystemIdleProcess(void)
{
	uint8_t LastMachineSelected;
	millis_t CheckTick;
	uint8_t EIOCheck[3];
	LastMachineSelected = MA_UNDEFINED;
	//关闭电源
	MainPowerOff();
	StepperPowerOff();
	CheckTick = millis();
	while(MachineMode == WORK_MODE_IDLE)
	{
		EIOCheck[2] = EIOCheck[1];
		EIOCheck[1] = EIOCheck[0];
		EIOCheck[0] = 0;
		//检测执行头插入
		if((E_ID_0_PORT->IDR & E_ID_0_PIN))
			EIOCheck[0] |= 1;

		if((E_ID_1_PORT->IDR & E_ID_1_PIN))
			EIOCheck[0] |= 2;


		//消抖动
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
				//无系统错误
				if(!(SysFaultFlag & FAULT_FLAG_BED & FAULT_FLAG_HEATER0 & FAULT_FLAG_LOAD))
					break;
			}
		}

		#if(HAVE_ANDROID == 1)
		AndroidScreenPowerManager();
		#endif

		LastMachineSelected = MachineSelected;
		
		//Gcode  处理
		/*
		get_serial_commands();
		if (GetCommandBuff() > 0) 
		{
			//Paladin
			SystemIdleGcodeProcess();
			DispatchComandIndex();
			ShieldLedOn();
			ShieldFanOn();
		}
		*/

		//执行头已检测到并且上位机要求切换状态
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
安卓屏幕电源管理
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
	

	//屏幕已接入
	if(!((DetIO[0] | DetIO[1] | DetIO[2]) & 1))
	{
		//之前是断开的
		if(!(PeriphPlugFlag & PERIPH_SCREEN_FLAG))
		{
			//重启屏幕
			PowerStat = ANDROID_PWR_STAT_IDLE;
			AndroidControlFlag = ANDROID_CTRL_RESET;
			PeriphPlugFlag |= PERIPH_SCREEN_FLAG;
		}
	}
	//屏幕断开
	else if(DetIO[0] & DetIO[1] & DetIO[2] & 1)
	{
		//之前是连接的
		if(PeriphPlugFlag & PERIPH_SCREEN_FLAG)
		{
			AndroidControlFlag = ANDROID_CTRL_POWEROFF;
			//清除屏幕串口缓冲
			HMI_BuffFlush();
			//解除屏幕锁定
			ScreenLock = false;
			//机器正在打印中
			if(GetCurrentPrinterStatus() != STAT_IDLE)
			{
				//触发停止
				StopTriggle(ManualStop);
			}
			PeriphPlugFlag &= ~PERIPH_SCREEN_FLAG;
		}
	}

	//PC  未连接设备
	if(DetIO[0] & DetIO[1] & DetIO[2] & 2)
	{
		//之前PC  是连接状态
		if(PeriphPlugFlag & PERIPH_PC_FLAG)
		{
			//串口切换到屏幕
			UART1_SELK1_PORT->BRR = UART1_SELK1_PIN;
			PeriphPlugFlag &= ~PERIPH_PC_FLAG;
			//机器正在打印中
			if(GetCurrentPrinterStatus() != STAT_IDLE)
			{
				//触发停止
				StopTriggle(ManualStop);
			}
		}
	}
	//PC  已连接设备
	else if(!((DetIO[0] | DetIO[1] | DetIO[2]) & 2))
	{
		//之前PC  是断开状态
		if(!(PeriphPlugFlag & PERIPH_PC_FLAG))
		{
			//屏幕未锁定且机器不在打印状态中
			if((ScreenLock == false) && (GetCurrentPrinterStatus() == STAT_IDLE))
				UART1_SELK1_PORT->BSRR = UART1_SELK1_PIN;
			PeriphPlugFlag |= PERIPH_PC_FLAG;
		}
	}
	
	switch(PowerStat)
	{
		case ANDROID_PWR_STAT_IDLE:
			//屏幕断开，关闭电源
			if(AndroidControlFlag == ANDROID_CTRL_POWEROFF)
			{
				//拉高开机键
				ANDROID_START_PORT->BRR = ANDROID_START_PIN;
				//关闭电源
				ANDROID_PWR_PORT->BRR = ANDROID_PWR_PIN;
				AndroidControlFlag = ANDROID_CTRL_NONE;
				//屏幕解索
				ScreenLock = false;
			}
			//屏幕重新接入，复位屏幕
			else if(AndroidControlFlag == ANDROID_CTRL_RESET)
			{
				//拉高开机键
				ANDROID_START_PORT->BRR = ANDROID_START_PIN;
				//关闭电源
				ANDROID_PWR_PORT->BRR = ANDROID_PWR_PIN;
				//切换状态
				PowerStat = ANDROID_PWR_STAT_POWEROFF;
				//清除标置
				AndroidControlFlag = ANDROID_CTRL_NONE;
				tmptick = millis();
			}
		break;

		case ANDROID_PWR_STAT_POWEROFF:
			//延时500  ms
			if((millis() - tmptick) > 300)
			{
				//开启电源
				ANDROID_PWR_PORT->BSRR = ANDROID_PWR_PIN;
				//切换状态
				PowerStat = ANDROID_PWR_STAT_POWERON;
				tmptick = millis();
			}
		break;

		case ANDROID_PWR_STAT_POWERON:
			//延时3000 ms
			if((millis() - tmptick) > 3000)
			{
				//拉低开机键
				ANDROID_START_PORT->BSRR = ANDROID_START_PIN;
				//切换状态
				PowerStat = ANDROID_PWR_STAT_KEYDOWN;
				tmptick = millis();
			}
		break;

		case ANDROID_PWR_STAT_KEYDOWN:
			if((millis() - tmptick) > 2000)
			{
				//切换状态
				PowerStat = ANDROID_PWR_STAT_IDLE;
				//拉高开机键
				ANDROID_START_PORT->BRR = ANDROID_START_PIN;
			}
		break;
	}
}
#endif

#if(SNAP_OLD_BASE == 0)
/**********************************************************
系统电源控制
Option  :   控制目标
               POWER_OPTION_MainPower  :  执行头与热床电源
               POWER_OPTION_StepperPower  :  步进电机电源
Enable  :  true  打开 ，false  关闭
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
查询当前是否正在终止打印
*****************************************/
bool IsStoppingPrint(void)
{
	return SystemRequestStop;
}

/******************************************
查询当前打印机状态
返回值:  STAT_IDLE
                    STAT_PRINTG
                    STAT_PAUSE
*****************************************/
PrinterStatus GetCurrentPrinterStatus(void)
{
	return PrintingStatus;
}

/******************************************
设置当前打印机状态
NewStatus  :  STAT_IDLE
                    STAT_PRINTG
                    STAT_PAUSE
*****************************************/
void SetCurrentPrinterStatus(PrinterStatus NewStatus)
{
	PrintingStatus = NewStatus;
}


/******************************************
暂停打印触发
Type:触发类型
*****************************************/
void PauseTriggle(PausePrintType Type)
{
	block_t *pBlock;
	PauseType = Type;
	StableCounter[0] = pCounter_X;
	StableCounter[1] = pCounter_Y;
	StableCounter[2] = pCounter_Z;
	StableCounter[3] = pCounter_E;
	//if(current_block == NULL)
	//	current_block = plan_get_current_block();
	switch(Type)
	{
		case FilamentFaultPause:
			CRITICAL_SECTION_START;
			//U  盘打印
			if(GetCurrentPrinterStatus() == STAT_PRINTING)
				card.sdprinting = false;
			SystemRequestStop = true;
			SysFaultFlag |= FAULT_FLAG_FILAMENT;
			
			//运动块已空
			if(current_block != NULL)
			{
				PowerPanicData.FilePosition = current_block->FilePosition;
				pBlock = current_block;
			}
			else 
			{
				//存在下一个运动块
				if(blocks_queued() == true)
				{
					//读取下一块行号
					pBlock = &block_buffer[block_buffer_tail];
					PowerPanicData.FilePosition = pBlock->FilePosition;
				}
				//不存在下一个块
				else
				{
					//读取下一块行号
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
			//请求停止
			SystemRequestStop = true;
			//U  盘打印
			if(GetCurrentPrinterStatus() == STAT_PRINTING)
				card.sdprinting = false;
			
			//运动块非空
			if(current_block != NULL)
			{
				PowerPanicData.FilePosition = current_block->FilePosition;
				pBlock = current_block;
			}
			else 
			{
				//存在下一个运动块
				if(blocks_queued() == true)
				{
					//读取下一块行号
					pBlock = &block_buffer[block_buffer_tail];
					PowerPanicData.FilePosition = pBlock->FilePosition;
				}
				//不存在下一个块
				else
				{
					//读取下一块行号
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
			//在线打印
			if(GetCurrentPrinterStatus() == STAT_PRINTING)
				card.sdprinting = false;
			SystemRequestStop = true;
			//运动块非空
			if(current_block != NULL)
			{
				PowerPanicData.FilePosition = current_block->FilePosition;
				pBlock = current_block;
			}
			else 
			{
				//存在下一个运动块
				if(blocks_queued() == true)
				{
					//读取下一块行号
					pBlock = &block_buffer[block_buffer_tail];
					PowerPanicData.FilePosition = pBlock->FilePosition;
				}
				//不存在下一个块
				else
				{
					//读取下一块行号
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
继续打印触发
Type:触发类型
*****************************************/
void PauseResume(void)
{
	PauseType = ContinuePrint;
}

/******************************************
停止打印触发
Type:触发类型
*****************************************/
void StopTriggle(StopPrintType Type)
{
	StopType = Type;
	StableCounter[0] = pCounter_X;
	StableCounter[1] = pCounter_Y;
	StableCounter[2] = pCounter_Z;
	StableCounter[3] = pCounter_E;
	//if(current_block == NULL)
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
			//中断触发，无需临界保护，在中断处理中已开启临界保护
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
暂停处理
****************************************************************/
void PauseProcess(void)
{
	millis_t tmpmills;

	if((PrintingStatus == STAT_PRINTING) || (PrintingStatus == STAT_PRINTING_ONLINE))
	{
		if((PauseType == ManualPause) || (PauseType == FilamentFaultPause) || (PauseType == DoorOpenPause) || (PauseType == ExecuterLostPause))
		{
			//标置暂停状态
			if(PrintingStatus == STAT_PRINTING_ONLINE)
				PrintingStatus = STAT_PAUSE_ONLINE;
			else
				PrintingStatus = STAT_PAUSE;

			block_buffer_head = block_buffer_tail = 0;
			current_block = NULL;

			//清除指令
			FlushCommand();

			//保存温度
			for(int i=0;i<4;i++)		
				PowerPanicData.HeaterTamp[i] = target_temperature[i];

			//热床温度
			PowerPanicData.BedTamp = target_temperature_bed;
			//风扇速度 
			//PowerPanicData.FanSpeed = TIM3->CCR1;
			//机型保存
			PowerPanicData.MachineType = MachineSelected;

			//标置有效
			PowerPanicData.Valid = 1;

			//文件名
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

			//Z  轴补偿
			if(MA_PRINT == MachineSelected)
			{
				float delZ = mbl.get_z(current_position[X_AXIS], current_position[Y_AXIS]);
				current_position[Z_AXIS] -= delZ;
			}
			
			sync_plan_position();

			//坐标
			for(int i=0;i<NUM_AXIS;i++)
				PowerPanicData.PositionData[i] = current_position[i];

			//清除请求停止标置，清除之后，电机才可重新运动
			SystemRequestStop = false;

			//电流保持
			StepperKeepCurrent = true;

			//上报状态
			#if(SNAP_OLD_BASE == 0)
			if(PauseType == FilamentFaultPause)
				HMI_SendMachineFaultFlag();
			//HMI_SendMachineStatusChange(0x04, 0);
			#endif

			if(MA_PRINT == MachineSelected)
			{
				//回抽
				if(current_temperature[0] > 180)
				{
					//切换到相对坐标模式
					relative_mode = true;
					plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS] - 4, 40, active_extruder);
					while(blocks_queued())manage_heater();
					current_position[E_AXIS] -= 4;
				}
				
				//切换到绝对坐标模式
				relative_mode = false;

				if((HomeAfterReset[X_AXIS] == true) && (HomeAfterReset[Y_AXIS] == true) && (HomeAfterReset[Z_AXIS] == true))
				{
					//Z  轴抬升30  
					if((current_position[Z_AXIS] + 30) < (home_offset[Z_AXIS] + base_max_pos[Z_AXIS]))
						sprintf(tmpBuff, "G0 Z%0.3f F2000", (current_position[Z_AXIS] + 30));
					else
						sprintf(tmpBuff, "G0 Z%0.3f F2000", (home_offset[Z_AXIS] + base_max_pos[Z_AXIS]));
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					while(blocks_queued())manage_heater();
					//X  轴走到限位开关位置
					sprintf(tmpBuff, "G0 X0 F2000");
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					while(blocks_queued())manage_heater();
					//Y  轴走到最大位置
					sprintf(tmpBuff, "G0 Y%0.3f F2000", (home_offset[Y_AXIS] + base_max_pos[Y_AXIS]));
					current_command_args = tmpBuff;
					gcode_G0_G1(0);
					while(blocks_queued())manage_heater();
				}
				//切换到主界面
				HMI_ChangePage(PAGE_PRINT);
				//关闭断料检测
				#if(HAVE_FALIMENT_SENSOR == 1)
				DisableFilamentCheck();
				#endif
			}
			else if(MA_CNC == MachineSelected)
			{			
				//Z  轴抬升30  
				if((current_position[Z_AXIS] + 30) < (home_offset[Z_AXIS] + base_max_pos[Z_AXIS]))
					sprintf(tmpBuff, "G0 Z%0.3f", (current_position[Z_AXIS] + 30));
				else
					sprintf(tmpBuff, "G0 Z%0.3f", (home_offset[Z_AXIS] + base_max_pos[Z_AXIS]));
				current_command_args = tmpBuff;
				gcode_G0_G1(0);
				while(blocks_queued())manage_heater();

				//走到工件原点
				strcpy(tmpBuff, "G0 X0 Y0 F3000");
				current_command_args = tmpBuff;
				gcode_G0_G1(0);
				while(blocks_queued())manage_heater();

				//关闭电机
				digitalWrite(LASER_PORT, LASER_PIN, LOW);
				//切换到主界面
				HMI_ChangePage(PAGE_CNC);
			}
			//激光
			else
			{
				//切换到主界面
				HMI_ChangePage(PAGE_LASER);
				//启动检测
				EnableDoorCheck();
			}

			//回应上位机
			#if(SNAP_OLD_BASE != 1)
			if((HmiRequestStatus == STAT_PAUSE) || (HmiRequestStatus == STAT_PAUSE_ONLINE))
			{
				HMI_SendMachineStatusChange(0x04, 0);
			}
			#endif
			//清除标志
			HmiRequestStatus = STAT_IDLE;
			
			//清除标置
			PauseType = NonePause;
		}
	}
	else if((PrintingStatus == STAT_PAUSE) || (PrintingStatus == STAT_PAUSE_ONLINE))
	{
		//继续打印
		if(PauseType == ContinuePrint)
		{
			//U  盘打印
			if(PrintingStatus == STAT_PAUSE)
			{
				//文件位置跳转
				if(card.FileSeek(PowerPanicData.FilePosition) == false)
				{
					card.initsd();
					//尝试打开文件
				  	card.openFile(PowerPanicData.FileName, true);
					card.FileSeek(PowerPanicData.FilePosition);
				}
				
				//继续 读取文件
				card.sdprinting = true;
			}
			
			//恢复运动
			SystemRequestStop = false;

			if(MA_PRINT == MachineSelected)
			{			
				//预挤出
				if(current_temperature[0] > 175)
				{
					//相对位置模式
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

				//E  轴同步位置
				current_position[E_AXIS] = PowerPanicData.PositionData[E_AXIS];
				//set_destination_to_current();
				sync_plan_position();

				//绝对位置模式
				relative_mode = false;
				
				//移动到恢复点
				sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F4000", PowerPanicData.PositionData[X_AXIS], PowerPanicData.PositionData[Y_AXIS]);
				current_command_args = tmpBuff;
				gcode_G0_G1(0);
				while(blocks_queued())manage_heater();
				
				sprintf(tmpBuff, "G0 Z%0.2f F4000", PowerPanicData.PositionData[Z_AXIS]);
				current_command_args = tmpBuff;
				gcode_G0_G1(0);
				while(blocks_queued())manage_heater();

				//切换到主界面
				HMI_ChangePage(PAGE_PRINT);
				//开启断料检测
				#if(HAVE_FALIMENT_SENSOR == 1)
				EnableFilamentCheck();
				#endif
			}
			else if(MA_CNC == MachineSelected)
			{
				//打开电机
				digitalWrite(LASER_PORT, LASER_PIN, HIGH);
				//延时
				tmpmills = millis() + 500;
				while(millis() < tmpmills);

				//移动到恢复点
				sprintf(tmpBuff, "G0 X%0.2f Y%0.2f F4000", PowerPanicData.PositionData[X_AXIS], PowerPanicData.PositionData[Y_AXIS]);
				current_command_args = tmpBuff;
				gcode_G0_G1(0);
				while(blocks_queued())manage_heater();

				sprintf(tmpBuff, "G0 Z%0.2f F4000", PowerPanicData.PositionData[Z_AXIS]);
				current_command_args = tmpBuff;
				gcode_G0_G1(0);
				while(blocks_queued())manage_heater();
				
				//切换到主界面
				HMI_ChangePage(PAGE_CNC);
			}
			else if((MA_LASER == MachineSelected) || (MA_LASER_350 == MachineSelected))
			{
				//切换到主界面
				HMI_ChangePage(PAGE_LASER);
			}

			//速度还原
			SetG0FeedRate(PowerPanicData.TravelFeedRate);
			SetG1FeedRate(PowerPanicData.PrintFeedRate);

			//清空缓冲
			FlushCommand();
			//切换到绝对坐标模式
			relative_mode = false;
			while(blocks_queued())manage_heater();
			//标置打印中状态
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
			//启动检测
			if(MA_LASER == MachineSelected)
				EnableDoorCheck();

			//清除标置
			PauseType = NonePause;
		}
	}
	
}


/******************************************
结束打印处理
3  种情况
1】打印完毕，需要把物料向外推送
2】脱机打印时，点击停止打印
3】系统掉电
*****************************************/
void FinishPrintProcess()
{
	static uint8_t StopSteps = 0;
	static uint32_t nextcheckMill;
	switch(StopSteps)
	{
		case 0:
			//断电
			if(StopType == PowerPanicStop)
			{
				//打印状态下
				if((PrintingStatus == STAT_PRINTING) || (PrintingStatus == STAT_PRINTING_ONLINE))
				{
					//保存温度
					for(int i=0;i<4;i++)
					{
						PowerPanicData.HeaterTamp[i] = target_temperature[i];
						target_temperature[i] = 0;
					}
					PowerPanicData.BedTamp = target_temperature_bed;
					target_temperature_bed = 0;

					//风扇速度 
					//PowerPanicData.FanSpeed = TIM3->CCR1;
					//机型保存
					PowerPanicData.MachineType = MachineSelected;

					//标置有效
					PowerPanicData.Valid = 1;

					//文件名
					strcpy(PowerPanicData.FileName, card.currentselectfilename);
					//打印时间
					PowerPanicData.accumulator = print_job_timer.duration();
					
					
					current_position[X_AXIS] = StableCounter[X_AXIS] / axis_steps_per_unit[X_AXIS];
					current_position[Y_AXIS] = StableCounter[Y_AXIS] / axis_steps_per_unit[Y_AXIS];
					current_position[Z_AXIS] = StableCounter[Z_AXIS] / axis_steps_per_unit[Z_AXIS];
					current_position[E_AXIS] = StableCounter[E_AXIS] / axis_steps_per_unit[E_AXIS];

					//Z  轴补偿
					float delZ = mbl.get_z(current_position[X_AXIS], current_position[Y_AXIS]);
					current_position[Z_AXIS] -= delZ;

					block_buffer_head = block_buffer_tail = 0;
					current_block = NULL;
					
					sync_plan_position();

					//坐标
					for(int i=0;i<NUM_AXIS;i++)
						PowerPanicData.PositionData[i] = current_position[i];

					//保存数据
					SaveBreakPointData();

					//清除请求停止标置，清除之后，电机才可重新运动
					SystemRequestStop = false;

					//回抽
					if(MA_PRINT == MachineSelected)
					{
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS] - 1, 40, active_extruder);
						while(blocks_queued())manage_heater();

						//抬升Z  轴
						plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + 5, current_position[E_AXIS] - 1, 40, active_extruder);
						while(blocks_queued())manage_heater();
					}
				}
				while(1);
			}
			//打印完成
			else if(StopType == EndPrint)
			{
				HMI_ChangePage(PAGE_PROCESSING);
				StopSteps = 1;
				nextcheckMill = millis() + 1000;
				card.LastPercent = 100;
				//关闭门检测
				if(isLaserMachine(MachineSelected) == 1)
					DisableDoorCheck();
			}
			//屏幕终止打印
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
				//关闭门检测
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
			//禁能断电检测
			DisablePowerPanicCheck();
			#if(SNAP_OLD_BASE == 1)
			if(isLaserMachine(MachineSelected) == 1)
				DisableDoorCheck();
			#endif
		break;
		
		case 3:
			//绝对坐标模式
			relative_mode = false;
			//清除请求停止标置，清除之后，电机才可重新运动
			SystemRequestStop = false;
			block_buffer_head = block_buffer_tail = 0;
			current_block = NULL;
			//切换到主界面
			if(MachineSelected == MA_PRINT)
			{
				disable_all_heaters();
				//Z  升到最高
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
					//X  Y  回原点
					strcpy(tmpBuff, "G28 Y");
					current_command_args = tmpBuff;
					gcode_G28();
					#else
					//X  Y  回原点
					strcpy(tmpBuff, "G28 X Y");
					current_command_args = tmpBuff;
					gcode_G28();
					while(blocks_queued())manage_heater();
					//Y  走到最大位置
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
                    //清除Block的PWM数据
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
					//关闭主轴
					digitalWrite(LASER_PORT, LASER_PIN, 0);
				}
				HMI_ChangePage(PAGE_CNC);
			}
			StopSteps = 0;
			//标置待机状态
			PrintingStatus = STAT_IDLE;
			//清除标置
			StopType = NoneStop;
			
		break;
	}
}

/********************************************
系统自检
********************************************/
void SystemSelfCheck(void)
{
	#if(0)
	#if(SNAP_OLD_BASE == 0)
	uint32_t tick;
	//总开关Mos  检测
	//总电开关关闭
	MainPowerOff();

	//下端MOS  打开
	HEATER_0_PORT->BSRR = HEATER_0_PIN;
	HEATER_BED_PORT->BSRR = HEATER_BED_PIN;
	tick = millis() + 100;
	while(tick > millis());
	//测到导通
	if((!(HEATER0_DET_PORT->IDR & HEATER0_DET_PIN)) && (!(HEATBED_DET_PORT->IDR & HEATBED_DET_PIN)))
	{
		SysFaultFlag |= FAULT_FLAG_LOAD;
		HEATER_0_PORT->BRR = HEATER_0_PIN;
		HEATER_BED_PORT->BRR = HEATER_BED_PIN;
		return;
	}

	//加热头MOS  检测
	//总电开关打开
	MainPowerOn();
	//下端MOS  关闭
	HEATER_0_PORT->BRR = HEATER_0_PIN;
	HEATER_BED_PORT->BRR = HEATER_BED_PIN;
	tick = millis() + 100;
	while(tick > millis());
	//测到导通
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

	//开启下端Mos
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
系统统一初始化
********************************************/
void SystemGpioInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	//En,Dir,Step
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

	//LED
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

//风扇
/*
风扇1  可能需要调速
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
	//步进电机
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
//老主控在此初始化
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


	//限位开关
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

//执行头断开检测
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

	//呼吸灯
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

    //电机上电
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
	//关闭检测
	DisableExecuterCheck();
}
}
#endif

