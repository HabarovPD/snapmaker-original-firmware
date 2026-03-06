#ifndef _STM32_PIN_CONTROL_
#define _STM32_PIN_CONTROL_

#include "stm32f10x_conf.h"

#define digitalWrite(Port, Pin, digit) digit!=LOW?Port->BSRR=Pin:Port->BRR=Pin
#define READ(PORT, Pin)	((PORT->IDR & Pin) == 0?false:true)

extern void set_PRIMASK(uint32_t priMask);
extern void memset(void *pBuff, uint8_t value, int size);
extern void SET_OUTPUT(GPIO_TypeDef * GPIOx, uint16_t Pin);
extern void EnterCritical(uint8_t option);
extern void SaveOption(uint8_t StartOption);
extern uint8_t LoadOption();
extern void SetStepperCurrent(float Current);

#endif