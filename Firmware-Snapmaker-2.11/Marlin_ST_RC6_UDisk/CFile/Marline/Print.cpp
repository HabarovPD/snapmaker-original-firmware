#define _IN_PRINT
#include "Marlin.h"
#include "HMIScreen.h"
#include "HMI_Define.h"
#include "stepper.h"
#include "Udisk.h"
#include "stopwatch.h"
#include "temperature.h"
#include "mesh_bed_leveling.h"
#include "configuration_store.h"
#include "SystemControl.h"
#include "Print.h"

#include "stm32f10x_conf.h"

/*****************************************************
打印变量初始化
*****************************************************/
void PrintVariableInit(void)
{
	CRITICAL_SECTION_START;
	ClearCommand();
	current_block = NULL;
	block_buffer_tail = block_buffer_head = 0;
	FlushCommand();
	CRITICAL_SECTION_END;
}


/********************************************
初始化3D  打印
********************************************/
void PrintInit(void)
{
	TIM_DeInit(TIM1);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);

	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;

	#if(SNAP_OLD_BASE == 1)
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIOB->BRR = GPIO_Pin_1;
	#endif

	//挤出步进 电机初始化
	#if(SNAP_OLD_BASE == 0)
	#ifdef E0_STEP_PORT
	GPIO_InitStruct.GPIO_Pin = E0_STEP_PIN;
	GPIO_Init(E0_STEP_PORT, &GPIO_InitStruct);
	#endif

	#ifdef E0_DIR_PORT
	GPIO_InitStruct.GPIO_Pin = E0_DIR_PIN;
	GPIO_Init(E0_DIR_PORT, &GPIO_InitStruct);
	#endif

	#ifdef E0_ENABLE_PORT
	GPIO_InitStruct.GPIO_Pin = E0_ENABLE_PIN;
	GPIO_Init(E0_ENABLE_PORT, &GPIO_InitStruct);
	#endif
	#endif

	

	#ifdef HEATER_0_PIN
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = HEATER_0_PIN;
	GPIO_Init(HEATER_0_PORT, &GPIO_InitStruct);
	#endif

	#ifdef HEATER_BED_PIN
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = HEATER_BED_PIN;
	GPIO_Init(HEATER_BED_PORT, &GPIO_InitStruct);
	#endif


	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_BDTRInitTypeDef TIM_BDTRInitStruct;
	//78HZ ，128  Level
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Prescaler = SystemCoreClock / 1000000 - 1;
	TIM_TimeBaseInitStruct.TIM_Period = 128;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);
	TIM_ARRPreloadConfig(TIM1, DISABLE);
	TIM_SelectOnePulseMode(TIM1, TIM_OPMode_Repetitive);

	TIM_BDTRStructInit(&TIM_BDTRInitStruct);
	TIM_BDTRInitStruct.TIM_OSSRState = ENABLE;
	TIM_BDTRInitStruct.TIM_OSSIState = ENABLE;
	TIM_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStruct.TIM_DeadTime = 0;
	TIM_BDTRInitStruct.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStruct);
	
	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OC1Init(TIM1, &TIM_OCInitStruct);
	TIM_OC2Init(TIM1, &TIM_OCInitStruct);
	//TIM_OC3Init(TIM1, &TIM_OCInitStruct);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_Cmd(TIM1, ENABLE);

	FileFilter = FIL_GCODE;
	MachineSelected = MA_PRINT;
	#if(HAVE_HMI == 1)
	HMI_ChangePage(PAGE_PRINT);
	#endif

	//开启温度检测
	#if(SNAP_OLD_BASE == 0)
	DMA_Cmd(DMA1_Channel1, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	#endif
	
	//物料检测
	FilamentManagerInit();

	//关闭门外罩检测
	DisableDoorCheck();

	E0_ENABLE_PORT->BRR = E0_ENABLE_PIN;
	//Add by Paladin:Update Acc according the machine
	gcode_M201();

	//自检
	SystemSelfCheck();
	//开启电源
	if((SysFaultFlag & FAULT_FLAG_LOAD & FAULT_FLAG_BED & FAULT_FLAG_HEATER0))
	{
		MachineMode = WORK_MODE_IDLE;
	}
	//变量初始化
	PrintVariableInit();

	#ifdef H_FAN2_PORT
	H_FAN2_PORT->BSRR = H_FAN2_PIN;
	#endif
}

/********************************************
逆始化3D  打印
********************************************/
void PrintDeint(void)
{
	TIM_DeInit(TIM1);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, DISABLE);

	GPIO_StructInit(&GPIO_InitStruct);

	#if(SNAP_OLD_BASE == 1)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIOB->BRR = GPIO_Pin_1;
	#endif

	#ifdef HEATER_0_PIN
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = HEATER_0_PIN;
	GPIO_Init(HEATER_0_PORT, &GPIO_InitStruct);
	#endif

	#ifdef HEATER_BED_PIN
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = HEATER_BED_PIN;
	GPIO_Init(HEATER_BED_PORT, &GPIO_InitStruct);
	#endif

	//关闭温度检测
	DMA_Cmd(DMA1_Channel1, DISABLE);
	TIM_Cmd(TIM3, DISABLE);

	FilamentManagerDeinit();
}


/*****************************************************
打印处理进程
*****************************************************/
void PrintProcess(void)
{
	PrintInit();
	while(MachineMode == WORK_MODE_RUNNING)
	{
		UDiskIdleProceess();
		if (commands_in_queue < (BUFSIZE - 4)) get_available_commands();

		//Modify by Paladin:系统不要求停止，并且队列中有指令待执行
		if ((commands_in_queue > 0) && (IsStoppingPrint() == false)) 
		{
			//Paladin
			process_next_command();
			DispatchComandIndex();
			ShieldLedOn();
			ShieldFanOn();
		}
		else
		{
			if((target_temperature[0] > 0) || (target_temperature[1] > 0) || (target_temperature_bed > 0))
			{
		 		ShieldLedOn();
		 		ShieldFanOn();
			}
		}
		//Paladin
		FinishPrintProcess();
		PauseProcess();
		checkHitEndstops();
		TampErrorCheck();
		#if(HAVE_ANDROID == 1)
		AndroidScreenPowerManager();
		#endif
		idle();
	}
	PrintDeint();
}

