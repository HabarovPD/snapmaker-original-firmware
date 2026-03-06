#ifndef _PINS_H_
#define _PINS_H_

#include "SnapmakerDefines.h"

//Heater Pins
#define HEATER_0_PIN	GPIO_Pin_0
#define HEATER_0_PORT	GPIOB

#define TEMP_0_PIN	GPIO_Pin_5
#define TEMP_0_PORT	GPIOC

#define HEATER_BED_PIN	GPIO_Pin_7
#define HEATER_BED_PORT	GPIOA

#define TEMP_BED_PIN	GPIO_Pin_4
#define TEMP_BED_PORT	GPIOC

//Laser
#define GATE_CTRL_PIN	GPIO_Pin_10
#define GATE_CTRL_PORT	GPIOB
#define LASER_PIN	GPIO_Pin_0
#define LASER_PORT	GPIOB


//Stepper Pins
#define X_STEP_PIN	GPIO_Pin_5
#define X_STEP_PORT	GPIOB
#define X_DIR_PIN	GPIO_Pin_6
#define X_DIR_PORT	GPIOB
#define X_ENABLE_PIN	GPIO_Pin_7
#define X_ENABLE_PORT	GPIOB

#define Y_STEP_PIN	GPIO_Pin_8
#define Y_STEP_PORT	GPIOB
#define Y_DIR_PIN	GPIO_Pin_9
#define Y_DIR_PORT	GPIOB
#define Y_ENABLE_PIN	GPIO_Pin_4
#define Y_ENABLE_PORT	GPIOB

#define Z_STEP_PIN	GPIO_Pin_2
#define Z_STEP_PORT	GPIOC
#define Z_DIR_PIN	GPIO_Pin_1
#define Z_DIR_PORT	GPIOC
#define Z_ENABLE_PIN	GPIO_Pin_3
#define	Z_ENABLE_PORT	GPIOC


#if ENABLED(Z_DUAL_STEPPER_DRIVERS)
#define Z2_STEP_PIN	GPIO_Pin_3
#define Z2_STEP_PORT	GPIOB
#define Z2_DIR_PIN	GPIO_Pin_2
#define Z2_DIR_PORT	GPIOD
#define Z2_ENABLE_PIN	GPIO_Pin_12
#define Z2_ENABLE_PORT	GPIOC
#endif

//Limits
#define X_MIN_PIN	GPIO_Pin_11
#define X_MIN_PORT	GPIOC

#define Y_MIN_PIN	GPIO_Pin_10
#define Y_MIN_PORT	GPIOC

#define Z_MIN_PIN	GPIO_Pin_0
#define Z_MIN_PORT	GPIOC

#define X2_MIN_PIN	-1
//#define Z2_MIN_PORT	GPIOC

#define Y2_MIN_PIN	-1
//#define Z2_MIN_PORT	GPIOC

#if ENABLED(Z_DUAL_STEPPER_DRIVERS)
#define Z2_MIN_PIN	GPIO_Pin_8
#define Z2_MIN_PORT	GPIOA
#endif

#define X_MAX_PIN	-1
//#define X_MAX_PORT	GPIOC

#define Y_MAX_PIN	-1
//#define Y_MAX_PORT	GPIOC

#define Z_MAX_PIN	-1
//#define Z_MAX_PORT	GPIOC

//#define X2_MAX_PIN	-1
//#define X_MAX_PORT	GPIOC

//#define Y2_MAX_PIN	-1
//#define Y_MAX_PORT	GPIOC

//#define Z2_MAX_PIN	-1
//#define Z_MAX_PORT	GPIOC


#define E0_STEP_PIN	GPIO_Pin_4
#define E0_STEP_PORT	GPIOA
#define E0_DIR_PIN	GPIO_Pin_1
#define E0_DIR_PORT	GPIOA
#define E0_ENABLE_PIN	GPIO_Pin_0
#define E0_ENABLE_PORT	GPIOA


//LED
#define LED_R_PIN	GPIO_Pin_7
#define LED_R_PORT	GPIOC
#define LED_B_PIN	GPIO_Pin_8
#define LED_B_PORT	GPIOC

//Door & Filament
#define DOOR_PIN	GPIO_Pin_8
#define DOOR_PORT	GPIOA

#if EXTRUDERS > 1
#define FILAMENT2_PIN	-1
//#define FILAMENT2_PORT	GPIOC
#endif

#if (HAVE_SHIELD_FAN == 1)
#define SHIELD_FAN_PIN	GPIO_Pin_1
#define SHIELD_FAN_PORT	GPIOC
#endif

#if (HAVE_SHIELD_LED == 1)
#define SHIELD_LED_PIN	GPIO_Pin_2
#define SHIELD_LED_PORT	GPIOC
#endif

#if (HAVE_FALIMENT_SENSOR == 1)
#define FILAMNET_I2C_SDA_PORT	GPIOB
#define FILAMNET_I2C_SDA_PIN	GPIO_Pin_11

#define FILAMNET_I2C_SCL_PORT	GPIOB
#define FILAMNET_I2C_SCL_PIN	GPIO_Pin_10

#endif

#if (HAVE_LASER_EEPROM == 1)
#define LASER_I2C_SDA_PORT	GPIOB
#define LASER_I2C_SDA_PIN	GPIO_Pin_11

#define LASER_I2C_SCL_PORT	GPIOB
#define LASER_I2C_SCL_PIN	GPIO_Pin_10

#endif

#define E_ID_0_PIN		GPIO_Pin_15
#define E_ID_0_PORT		GPIOE

#define E_ID_1_PIN		GPIO_Pin_14
#define E_ID_1_PORT		GPIOE

#define MAIN_POWER_SUPPLY_PIN		GPIO_Pin_2
#define MAIN_POWER_SUPPLY_PORT		GPIOF

#define STEPPER_POWER_SUPPLY_PIN	GPIO_Pin_1
#define STEPPER_POWER_SUPPLY_PORT	GPIOF

#if(HAVE_ANDROID == 1)
#define ANDROID_DET_PIN		GPIO_Pin_1
#define ANDROID_DET_PORT	GPIOF

#define ANDROID_START_PIN	GPIO_Pin_1
#define ANDROID_START_PORT	GPIOF

#define ANDROID_PWR_PIN		GPIO_Pin_1
#define ANDROID_PWR_PORT	GPIOF
#endif

#define PROBE_PIN			GPIO_Pin_0
#define PROBE_PORT			GPIOC

#endif
