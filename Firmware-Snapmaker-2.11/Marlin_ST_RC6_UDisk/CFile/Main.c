#include "includes.h"

#include "Marlin.h"

extern void setup();
extern void loop();
USB_OTG_CORE_HANDLE USB_OTG_Core;
USBH_HOST USB_Host;


void HeatTickInit(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVicInit;

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

	SysTick_Config(SystemCoreClock / 1000);

	#if(SNAP_OLD_BASE == 1)
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 7199;
	TIM_TimeBaseInitStruct.TIM_Period = 999;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
	TIM4->SR = 0;
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_SelectOnePulseMode(TIM4, TIM_OPMode_Repetitive);

	NVicInit.NVIC_IRQChannel = TIM4_IRQn;
	NVicInit.NVIC_IRQChannelSubPriority = 0;
	NVicInit.NVIC_IRQChannelPreemptionPriority = 2;
	NVicInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVicInit);
	TIM_Cmd(TIM4, ENABLE);
	#endif

	/*
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 71;
	TIM_TimeBaseInitStruct.TIM_Period = 999;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStruct);
	TIM6->SR = 0;
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	TIM_SelectOnePulseMode(TIM6, TIM_OPMode_Repetitive);

	NVicInit.NVIC_IRQChannel = TIM6_IRQn;
	NVicInit.NVIC_IRQChannelSubPriority = 0;
	NVicInit.NVIC_IRQChannelPreemptionPriority = 2;
	NVicInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVicInit);
	//TIM_Cmd(TIM6, ENABLE);
	*/
}

void SystemDeinit(void)
{
	TIM_DeInit(TIM1);
	TIM_DeInit(TIM2);
	TIM_DeInit(TIM3);
	TIM_DeInit(TIM4);
	TIM_DeInit(TIM5);
	TIM_DeInit(TIM6);
	TIM_DeInit(TIM7);
	#ifdef STM32F10X_CL
	USBH_DeInit(&USB_OTG_Core, &USB_Host);
	#endif
	USART_DeInit(USART1);
	USART_DeInit(USART2);
	USART_DeInit(USART3);
	ADC_DeInit(ADC1);
	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);
	GPIO_DeInit(GPIOD);
	GPIO_DeInit(GPIOE);
	TIM1->CNT = 0;
	TIM2->CNT = 0;
	TIM3->CNT = 0;
	TIM4->CNT = 0;
	TIM5->CNT = 0;
	TIM6->CNT = 0;
	TIM7->CNT = 0;
}

extern char WaitUDisk(void);

#include "ff.h"
void DelUpdateFile(void)
{
	unsigned long RN;
	uint32_t tmptick;
	uint8_t Retry;
	uint8_t ReadBuff[16];
	
	Retry = 0;
	tmptick = Tick;
	while((Tick - tmptick) < (TICK_PER_SECOND / 5));
	do
	{
		//操作超时
		if(WaitUDisk() == (char)-1)
			break;
		if(FR_OK == f_mount(&UpdateFatfs, "0:\\", 1))
		{
			if(FR_OK == f_open(&UpdateFile, "0:\\Update.bin", FA_READ))
			{
				f_lseek(&UpdateFile, 0x24);
				if(FR_OK == f_read(&UpdateFile, ReadBuff, 16, &RN))
				{
					if(ReadBuff[0] == 0xea)
					{
						f_unlink("0:\\Update.bin");
					}
				}
			}
			else
                break;
		}
        else
          break;
		Retry++;
	}while(Retry < 2);
	f_mount(&UpdateFatfs, "0:\\", 0);
}


void GpioCheck(void)
{
  //#if (SNAP_OLD_BASE == 0)
  #if(0)
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

  GPIO_InitTypeDef GPIO_InitStruct;
  //Output
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;

  GPIOA->BRR = GPIO_Pin_All;
  GPIOB->BRR = GPIO_Pin_All;
  GPIOC->BRR = GPIO_Pin_All;
  GPIOD->BRR = GPIO_Pin_All;
  GPIOE->BRR = GPIO_Pin_All;

  #ifdef X_STEP_PIN
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

  #ifdef E0_STEP_PIN
  GPIO_InitStruct.GPIO_Pin = E0_STEP_PIN;
  GPIO_Init(E0_STEP_PORT, &GPIO_InitStruct);
  #endif

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOE, &GPIO_InitStruct);

  #ifdef E0_DIR_PIN
  GPIO_InitStruct.GPIO_Pin = E0_DIR_PIN;
  GPIO_Init(E0_DIR_PORT, &GPIO_InitStruct);
  #endif

  #ifdef E0_ENABLE_PIN
  GPIO_InitStruct.GPIO_Pin = E0_ENABLE_PIN;
  GPIO_Init(E0_ENABLE_PORT, &GPIO_InitStruct);
  #endif

  #ifdef H_FAN1_PORT
  GPIO_InitStruct.GPIO_Pin = H_FAN1_PIN;
  GPIO_Init(H_FAN1_PORT, &GPIO_InitStruct);
  #endif

  #ifdef H_FAN2_PORT
  GPIO_InitStruct.GPIO_Pin = H_FAN2_PIN;
  GPIO_Init(H_FAN2_PORT, &GPIO_InitStruct);
  #endif

  #ifdef E0_ENABLE_PIN
  GPIO_InitStruct.GPIO_Pin = E0_ENABLE_PIN;
  GPIO_Init(E0_ENABLE_PORT, &GPIO_InitStruct);
  #endif

  #ifdef UART1_SELK1_PORT
  GPIO_InitStruct.GPIO_Pin = UART1_SELK1_PIN;
  GPIO_Init(UART1_SELK1_PORT, &GPIO_InitStruct);
  #endif

  GPIO_InitStruct.GPIO_Pin = SHIELD_LED_PIN;
  GPIO_Init(SHIELD_LED_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = SHIELD_FAN_PIN;
  GPIO_Init(SHIELD_FAN_PORT, &GPIO_InitStruct);

  #if(HAVE_ANDROID == 1)
  GPIO_InitStruct.GPIO_Pin = ANDROID_PWR_PIN;
  GPIO_Init(ANDROID_PWR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = ANDROID_START_PIN;
  GPIO_Init(ANDROID_START_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = HEATER_BED_PIN;
  GPIO_Init(HEATER_BED_PORT, &GPIO_InitStruct);
  #endif

  #ifdef MAIN_POWER_SUPPLY_PORT
  GPIO_InitStruct.GPIO_Pin = MAIN_POWER_SUPPLY_PIN;
  GPIO_Init(MAIN_POWER_SUPPLY_PORT, &GPIO_InitStruct);
  #endif

  #ifdef STEPPER_POWER_SUPPLY_PORT
  GPIO_InitStruct.GPIO_Pin = STEPPER_POWER_SUPPLY_PIN;
  GPIO_Init(STEPPER_POWER_SUPPLY_PORT, &GPIO_InitStruct);
  #endif

  GPIO_InitStruct.GPIO_Pin = HEATER_0_PIN;
  GPIO_Init(HEATER_0_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*******************************************************/
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
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

  GPIO_InitStruct.GPIO_Pin = DOOR_PIN;
  GPIO_Init(DOOR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = PC_PLUG_DET_PIN;
  GPIO_Init(PC_PLUG_DET_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = ANDROID_DET_PIN;
  GPIO_Init(ANDROID_DET_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = E_ID_0_PIN;
  GPIO_Init(E_ID_0_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = E_ID_1_PIN;
  GPIO_Init(E_ID_1_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = EXE_INT_PIN;
  GPIO_Init(EXE_INT_PORT, &GPIO_InitStruct);

  /*******************************************************/
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;

  GPIO_InitStruct.GPIO_Pin = HEATER_CHECK_PIN;
  GPIO_Init(HEATER_CHECK_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = BED_CHECK_PIN;
  GPIO_Init(BED_CHECK_PORT, &GPIO_InitStruct);

  /******************************************************/

  


  HEATER_BED_PORT->BSRR = HEATER_BED_PIN;
  HEATER_BED_PORT->BRR = HEATER_BED_PIN;
  HEATER_0_PORT->BSRR = HEATER_0_PIN;
  HEATER_0_PORT->BRR = HEATER_0_PIN;

  X_STEP_PORT->BSRR = X_STEP_PIN;
  X_STEP_PORT->BRR = X_STEP_PIN;
  X_DIR_PORT->BSRR = X_DIR_PIN;
  X_DIR_PORT->BRR = X_DIR_PIN;
  X_ENABLE_PORT->BSRR = X_ENABLE_PIN;
  X_ENABLE_PORT->BRR = X_ENABLE_PIN;

  Y_STEP_PORT->BSRR = Y_STEP_PIN;
  Y_STEP_PORT->BRR = Y_STEP_PIN;
  Y_DIR_PORT->BSRR = Y_DIR_PIN;
  Y_DIR_PORT->BRR = Y_DIR_PIN;
  Y_ENABLE_PORT->BSRR = Y_ENABLE_PIN;
  Y_ENABLE_PORT->BRR = Y_ENABLE_PIN;

  Z_STEP_PORT->BSRR = Z_STEP_PIN;
  Z_STEP_PORT->BRR = Z_STEP_PIN;
  Z_DIR_PORT->BSRR = Z_DIR_PIN;
  Z_DIR_PORT->BRR = Z_DIR_PIN;
  Z_ENABLE_PORT->BSRR = Z_ENABLE_PIN;
  Z_ENABLE_PORT->BRR = Z_ENABLE_PIN;

  E0_STEP_PORT->BSRR = E0_STEP_PIN;
  E0_STEP_PORT->BRR = E0_STEP_PIN;
  E0_DIR_PORT->BSRR = E0_DIR_PIN;
  E0_DIR_PORT->BRR = E0_DIR_PIN;
  E0_ENABLE_PORT->BSRR = E0_ENABLE_PIN;
  E0_ENABLE_PORT->BRR = E0_ENABLE_PIN;

  MAIN_POWER_SUPPLY_PORT->BSRR = MAIN_POWER_SUPPLY_PIN;
  SHIELD_FAN_PORT->BSRR = SHIELD_FAN_PIN;
  SHIELD_FAN_PORT->BRR = SHIELD_FAN_PIN;

  SHIELD_LED_PORT->BSRR = SHIELD_LED_PIN;
  SHIELD_LED_PORT->BRR = SHIELD_LED_PIN;

  #if(HAVE_ANDROID == 1)
  ANDROID_PWR_PORT->BSRR = ANDROID_PWR_PIN;
  ANDROID_PWR_PORT->BRR = ANDROID_PWR_PIN;

  ANDROID_START_PORT->BSRR = ANDROID_START_PIN;
  ANDROID_START_PORT->BRR = ANDROID_START_PIN;
  #endif

  
  MAIN_POWER_SUPPLY_PORT->BRR = MAIN_POWER_SUPPLY_PIN;
  STEPPER_POWER_SUPPLY_PORT->BSRR = STEPPER_POWER_SUPPLY_PIN;
  STEPPER_POWER_SUPPLY_PORT->BRR = STEPPER_POWER_SUPPLY_PIN;

  UART1_SELK1_PORT->BSRR = UART1_SELK1_PIN;
  UART1_SELK1_PORT->BRR = UART1_SELK1_PIN;

  GPIOA->BSRR = GPIO_Pin_2;
  GPIOA->BRR = GPIO_Pin_2;

  GPIOA->BSRR = GPIO_Pin_3;
  GPIOA->BRR = GPIO_Pin_3;

  GPIOA->BSRR = GPIO_Pin_9;
  GPIOA->BRR = GPIO_Pin_9;

  GPIOA->BSRR = GPIO_Pin_10;
  GPIOA->BRR = GPIO_Pin_10;

  GPIOB->BSRR = GPIO_Pin_10;
  GPIOB->BRR = GPIO_Pin_10;

  GPIOB->BSRR = GPIO_Pin_11;
  GPIOB->BRR = GPIO_Pin_11;

  GPIOA->BSRR = GPIO_Pin_5;

  H_FAN1_PORT->BSRR = H_FAN1_PIN;
  H_FAN1_PORT->BRR = H_FAN1_PIN;

  H_FAN2_PORT->BSRR = H_FAN2_PIN;
  H_FAN2_PORT->BRR = H_FAN2_PIN;

  ANDROID_PWR_PORT->BSRR = ANDROID_PWR_PIN;
  UART1_SELK1_PORT->BSRR = UART1_SELK1_PIN;

  GPIOA->BSRR = GPIO_Pin_9;
  GPIOA->BRR = GPIO_Pin_9;

  GPIOA->BSRR = GPIO_Pin_10;
  GPIOA->BRR = GPIO_Pin_10;

  UART1_SELK1_PORT->BRR = UART1_SELK1_PIN;

  GPIOA->BSRR = GPIO_Pin_9;
  GPIOA->BRR = GPIO_Pin_9;

  GPIOA->BSRR = GPIO_Pin_10;
  GPIOA->BRR = GPIO_Pin_10;

  for(int i=0;i<10000;i++);

 
  while(DOOR_PORT->IDR & DOOR_PIN);

  /*
  while(X_MAX_PORT->IDR & X_MAX_PIN);
  while(Y_MAX_PORT->IDR & Y_MAX_PIN);
  while(Z_MAX_PORT->IDR & Z_MAX_PIN);
  while(DOOR_PORT->IDR & DOOR_PIN);
  while(PC_PLUG_DET_PORT->IDR & PC_PLUG_DET_PIN);
  while(ANDROID_DET_PORT->IDR & ANDROID_DET_PIN);
  while(AC_PLUGIN_PORT->IDR & AC_PLUGIN_PIN);
  while(E_ID_0_PORT->IDR & E_ID_0_PIN);
  while(E_ID_1_PORT->IDR & E_ID_1_PIN);
  while(HEATER_CHECK_PORT->IDR & HEATER_CHECK_PIN);
  while(BED_CHECK_PORT->IDR & BED_CHECK_PIN);
  */
  #endif
}

//GD32  专用，提高代码效率
void SetCodeEfficiency()
{
	uint32_t tmpReadback;
	tmpReadback = *((volatile uint32_t *)0x4002103C);
	tmpReadback |= (1<<7);
	*((volatile uint32_t *)0x4002103C) = tmpReadback;
}

void Test()
{

	TIM_DeInit(TIM1);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);

	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = E0_ENABLE_PIN;
	GPIO_Init(E0_ENABLE_PORT, &GPIO_InitStruct);

	
	#ifdef STEPPER_POWER_SUPPLY_PORT
		GPIO_InitStruct.GPIO_Pin = STEPPER_POWER_SUPPLY_PIN;
		GPIO_Init(STEPPER_POWER_SUPPLY_PORT, &GPIO_InitStruct);
	#endif
	
	#ifdef MAIN_POWER_SUPPLY_PORT
		GPIO_InitStruct.GPIO_Pin = MAIN_POWER_SUPPLY_PIN;
		GPIO_Init(MAIN_POWER_SUPPLY_PORT, &GPIO_InitStruct);
	#endif

	#ifdef HEATER_0_PIN
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = HEATER_0_PIN;
	GPIO_Init(HEATER_0_PORT, &GPIO_InitStruct);
	#endif

	MAIN_POWER_SUPPLY_PORT->BSRR = MAIN_POWER_SUPPLY_PIN;
	STEPPER_POWER_SUPPLY_PORT->BSRR = STEPPER_POWER_SUPPLY_PIN;
	
	E0_ENABLE_PORT->BRR = E0_ENABLE_PIN;

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_BDTRInitTypeDef TIM_BDTRInitStruct;
	//78HZ ，128  Level
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 11999;
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
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OC1Init(TIM1, &TIM_OCInitStruct);
	TIM_OC2Init(TIM1, &TIM_OCInitStruct);
	//TIM_OC3Init(TIM1, &TIM_OCInitStruct);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_Cmd(TIM1, ENABLE);

	TIM1->CCR2 = 127;

	
	millis_t tmptick;
	tmptick = Tick + 3000;
	while(Tick < tmptick);
}

int main(void)
{
	//double A,B,C;

	SystemInit();
	SystemDeinit();

	//SetCodeEfficiency();

	//GpioCheck();

	

	#if(SNAP_OLD_BASE == 1)
	
	#endif

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0009800);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM7 | RCC_APB1Periph_TIM6 | RCC_APB1Periph_USART2, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_TIM1 | RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, DISABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM7 | RCC_APB1Periph_TIM6 | RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_TIM1 | RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	HeatTickInit();

	//Test();
	#ifdef STM32F10X_CL
	USBH_Init(&USB_OTG_Core, USB_OTG_FS_CORE_ID,&USB_Host,&USBH_MSC_cb, &USR_cb);
	DelUpdateFile();
	#endif

	
	setup();
	loop();
}


extern "C"
{
void SysTick_Handler(void)
{
	Tick++;
}

void TIM6_IRQHandler(void)
{
	TIM6->SR = 0;
	Tick++;
}

#if(SNAP_OLD_BASE == 1)
void TIM4_IRQHandler(void)
{
	TIM4->SR = 0;
	#ifdef LED_B_PORT
	if(LED_B_PORT->ODR & LED_B_PIN)
		LED_B_PORT->BRR = LED_B_PIN;
	else
		LED_B_PORT->BSRR = LED_B_PIN;
	#endif
}
#endif

}
