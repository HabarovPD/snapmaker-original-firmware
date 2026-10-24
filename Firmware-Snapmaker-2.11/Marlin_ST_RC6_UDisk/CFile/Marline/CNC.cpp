#define _IN_CNC
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
#include "CNC.h"

#include "stm32f10x_conf.h"


//中断入口需要查看.S  文件
#define THIS_USART	USART3


/********************************************
初始化CNC   检测
********************************************/
void CNCInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	
	#if(HAVE_CNC_MCU == 1)
	USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);           
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);


	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	USART_InitStruct.USART_BaudRate = 115200;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(THIS_USART, &USART_InitStruct);
	USART_ITConfig(THIS_USART, USART_IT_RXNE, ENABLE);
	USART_Cmd(THIS_USART, ENABLE);
	USART_ClearITPendingBit(THIS_USART, USART_IT_RXNE);

	NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);

	#else

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = LASER_PIN;
	GPIO_Init(LASER_PORT, &GPIO_InitStruct);
	LASER_PORT->BRR = LASER_PIN;

	#if(SNAP_OLD_BASE == 1)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIOB->BRR = GPIO_Pin_1;
	#endif

	#endif

	FileFilter = FIL_CNC;
	MachineSelected = MA_CNC;

	//关闭门外罩检测
	DisableDoorCheck();

	#if(HAVE_HMI == 1)
	HMI_ChangePage(PAGE_CNC);
	#endif

	//Add by Paladin:Update Acc according the machine
	gcode_M201();

	//自检
	SystemSelfCheck();
	//开启电源
	if((SysFaultFlag & FAULT_FLAG_LOAD & FAULT_FLAG_BED & FAULT_FLAG_HEATER0))
	{
		MachineMode = WORK_MODE_IDLE;
	}
}

/********************************************
逆始化CNC
********************************************/
void CNCDeint(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_StructInit(&GPIO_InitStruct);
	
	#if(HAVE_CNC_MCU == 1)
	//USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, DISABLE);           
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);


	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = DISABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);

	USART_DeInit(USART3);

	#else

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = LASER_PIN;
	GPIO_Init(LASER_PORT, &GPIO_InitStruct);
	LASER_PORT->BRR = LASER_PIN;

	#endif
}


/*****************************************************
Cnc  变量初始化
*****************************************************/
void CncVariableInit(void)
{
	CRITICAL_SECTION_START;
	ClearCommand();
	current_block = NULL;
	block_buffer_tail = block_buffer_head = 0;
	FlushCommand();
	CRITICAL_SECTION_END;
}

#if(HAVE_CNC_MCU == 1)


static uint8_t ReadBuff[128];
static uint8_t SendBuff[32];
static uint8_t DataBuff[16];
static uint8_t ReadHead;
static uint8_t ReadTail;
static uint8_t SendLen;
static uint8_t SendIndex;

/*****************************************************
Cnc 发送改变转速
*****************************************************/
void CncSetRPM(uint16_t RPM)
{
	uint32_t statreg0, statreg1;
	//等待发送数据寄存器为空
	do
	{
		statreg0 = THIS_USART->SR;
		statreg1 = THIS_USART->SR;
	}while((statreg0 != statreg1) && ((statreg0 & USART_SR_TXE) == 0));
	char strRPM[6];
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	i = 0;
	//包头
	SendBuff[i++] = 0xcc;
	//长度
	SendBuff[i++] = 0;
	//命令
	SendBuff[i++] = 'S';
	//数据
	sprintf(strRPM, "%d", RPM);
	j = 0;
	while((strRPM[j] != 0) && (j < 5))
	{
		SendBuff[i++] = strRPM[j++];
	}

	//重填长度
	SendBuff[1] = i;
	//效验
	checksum = 0;
	for(j=3;j<i;j++)
		checksum += SendBuff[j];
	SendBuff[i++] = checksum;
	
	//包尾
	SendBuff[i++] = 0xcd;

	//重填长度
	SendLen = i;
	SendIndex = 0;
	THIS_USART->CR1 |= USART_CR1_TXEIE;
}

/*****************************************************
Cnc  读取当前转速
*****************************************************/
void CncGetRPM()
{
	uint8_t i;
	uint8_t j;
	uint8_t checksum;
	uint32_t statreg0, statreg1;
	//等待发送数据寄存器为空
	do
	{
		statreg0 = THIS_USART->SR;
		statreg1 = THIS_USART->SR;
	}while((statreg0 != statreg1) && ((statreg0 & USART_SR_TXE) == 0));
	i = 0;
	//包头
	SendBuff[i++] = 0xcc;
	//长度
	SendBuff[i++] = 0;
	//命令
	SendBuff[i++] = 'R';
	
	//效验
	checksum = 0;
	for(j=0;j<i;i++)
		checksum += SendBuff[j];
	SendBuff[i++] = checksum;
	
	//包尾
	SendBuff[i++] = 0xcd;

	//重填长度
	SendLen = i;
	
	THIS_USART->CR1 |= USART_CR1_TXEIE;
}

/*****************************************************
Cnc  解析指令
*****************************************************/
short CNCGetCommand(unsigned char *pBuff)
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
		if(ReadBuff[tmptail] != 0xCC)
		{
			tmptail = (tmptail + 1) % sizeof(ReadBuff);
			tmplen--;
			//更新读指针
			ReadTail = tmptail;
			continue;
		}
	
		//读取包长
		uint8_t commandLen = ReadBuff[(tmptail + 1) % sizeof(ReadBuff)];
		/*
		//包长效验错误
		if((((commandLen >> 8) & 0xff) ^ (commandLen & 0xff)) != ReadBuff[(tmptail + 5) % sizeof(ReadBuff)])
		{
			tmptail = (tmptail + 2) % sizeof(ReadBuff);
			tmplen = tmplen - 2;
			//更新读指针
			ReadTail = tmptail;
			continue;
		}
		*/

		//缓冲数据足够
		if(commandLen <= (tmplen - 2))
		{
			//复制数据
			for(i=0;i<(commandLen + 2);i++)
			{
				pBuff[i] = ReadBuff[tmptail];
				tmptail = (tmptail + 1) % sizeof(ReadBuff);
			}

			//更新读指针
			ReadTail = tmptail;

			//校验
			checksum = 0;
			for(i=0;i<(commandLen + 2);i++)
				checksum += pBuff[i];

			if((uint16_t)checksum != pBuff[commandLen + 2])
				return (short)-1;
			return commandLen + 2;
		}
		//数据长度不足
		else
		{
			return (short)-1;
		}
		
	}
	return (short)-1;
}

/*****************************************************
Cnc  读取当前转速
*****************************************************/
void CncCpuCommunicationProcess()
{
	uint8_t DataLen;
	uint8_t Command;
	uint8_t dataindex;
	uint16_t u16Value;
	if(CNCGetCommand(DataBuff) < 0)
		return;

	dataindex = 3;
	DataLen = DataBuff[1];
	switch(DataBuff[2])
	{
		case 'R':
			u16Value = 0;
			while(DataLen > 0)
			{
				u16Value = u16Value * 10 + (DataBuff[dataindex++] - '0');
				DataLen--;
			}
			CurRPM = u16Value;
		break;

		case 'S':
			u16Value = 0;
			while(DataLen > 0)
			{
				u16Value = u16Value * 10 + (DataBuff[dataindex++] - '0');
				DataLen--;
			}
			CurRPM = u16Value;
		break;
	}

}

#endif

/*****************************************************
CNC  处理进程
*****************************************************/
void CncProcess(void)
{
	CNCInit();
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
		}
		//Paladin
		FinishPrintProcess();
		PauseProcess();
		checkHitEndstops();
		#if(HAVE_CNC_MCU == 1)
		CncCpuCommunicationProcess();
		#endif
		#if(HAVE_ANDROID == 1)
		AndroidScreenPowerManager();
		#endif
		idle();
	}
	CNCDeint();
}


#if(HAVE_CNC_MCU == 1)
extern "C"
{

void USART3_IRQHandler(void)
{
	uint8_t recvdata;
	uint16_t tmpTail;
	uint16_t tmpHead;
	uint8_t RecvData;
	
	if(THIS_USART->SR & USART_SR_RXNE)
	{
		RecvData = THIS_USART->DR;
		tmpHead = (ReadHead + 1) % sizeof(ReadBuff);
		if(tmpHead != ReadTail)
		{
			ReadBuff[ReadHead] = RecvData;
			ReadHead = tmpHead;
		}
	}
	else if(THIS_USART->SR & USART_FLAG_TXE)
	{
		if(SendLen == 0)
		{
			THIS_USART->CR1 &= ~USART_CR1_TXEIE;
			THIS_USART->SR &= USART_FLAG_TXE;
		}
		else
		{
			THIS_USART->DR = SendBuff[SendIndex];
			SendIndex = (SendIndex + 1) % sizeof(SendBuff);
			SendLen--;
		}
	}
}


}

#endif
