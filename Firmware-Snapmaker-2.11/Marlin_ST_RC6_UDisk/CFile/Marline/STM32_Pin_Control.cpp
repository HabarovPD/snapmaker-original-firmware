#include "stm32f10x_conf.h"

uint32_t Tick;

//ФЛЭШ-определение
#define BOOT_CODE_SIZE	(32*1024)
#define BOOT_PARA_SIZE	(6*1024)
#define MARLIN_CODE_SIZE	(128*1024)
#define MARLIN_PARA_SIZE	(4*1024)
#define GRBL_CODE_SIZE	(64*1024)
#define GRBL_PARA_SIZE	(4*1024)

//#define FLASH_BASE 0x08000000
#define FLASH_BOOT_PARA	(FLASH_BASE+BOOT_CODE_SIZE)
#define FLASH_MARLIN	(FLASH_BOOT_PARA+BOOT_PARA_SIZE)
#define FLASH_MARLIN_PARA	(FLASH_MARLIN+MARLIN_CODE_SIZE)
#define FLASH_GRBL	(FLASH_MARLIN_PARA+MARLIN_PARA_SIZE)
#define FLASH_GRBL_PARA (FLASH_GRBL+GRBL_CODE_SIZE)


inline uint32_t SaveSR(void)
{
	__asm
	(
	"mrs r0, primask\n"
	"CPSID I\n"
	"bx lr"
	);
}

inline void RestoreSR(uint32_t Sr)
{
	__asm
	(
	"msr primask, r0\n"
	"bx lr"
	);
}

void EnterCritical(uint8_t option)
{
	static uint32_t cpu_sr = 0;
	if(option == 0)
		RestoreSR(cpu_sr);
	else
		cpu_sr = SaveSR();
}


void memset(void *pBuff, uint8_t value, int size)
{
	uint8_t *p = (uint8_t*)pBuff;
	while(size--)
		*p++ = value;
}

void SET_OUTPUT(GPIO_TypeDef * GPIOx, uint16_t Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin = Pin;
	GPIO_Init(GPIOx, &GPIO_InitStruct);
}

uint8_t LoadOption()
{
	uint8_t CurrentOption;
	uint32_t Address;
	Address = FLASH_BOOT_PARA;
	CurrentOption = *((uint8_t*)Address);
	return CurrentOption;
}


void SaveOption(uint8_t StartOption)
{
	uint8_t CurrentOption;
	uint32_t Address;
	Address = FLASH_BOOT_PARA;
	CurrentOption = *((uint8_t*)Address);
	if(CurrentOption != StartOption)
	{
		FLASH_Unlock();
		FLASH_ErasePage(Address);
		FLASH_ProgramHalfWord(Address, StartOption);
		FLASH_Lock();
	}
}

void SetStepperCurrent(float Current)
{
	uint16_t Value;
	float Vol;
	Vol = Current * 0.8f / 0.36486f;
	
	Value = (uint16_t)(Vol / 3.3f * 4095);
	if(Value > 4095)
		Value = 4095;
	DAC_SetChannel2Data(DAC_Align_12b_R, Value);
}
