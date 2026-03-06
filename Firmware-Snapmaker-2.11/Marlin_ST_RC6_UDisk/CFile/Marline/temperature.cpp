/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
  temperature.cpp - temperature control
  Part of Marlin

 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Marlin.h"
#include "ultralcd.h"
#include "temperature.h"
#include "language.h"
#include "HMIScreen.h"
#include "LaserPowerTable.h"
#include "Laser.h"
#include "Systemcontrol.h"
//===========================================================================
//================================== macros =================================
//===========================================================================

#ifdef K1 // Defined in Configuration.h in the PID settings
  #define K2 (1.0-K1)
#endif

#if ENABLED(PIDTEMPBED) || ENABLED(PIDTEMP)
  #define PID_dT 246//160MS
#endif


static volatile uint16_t tmpADC[32];
//===========================================================================
//============================= public variables ============================
//===========================================================================

int target_temperature[4] = { 0 };
int target_temperature_bed = 0;
int current_temperature_raw[4] = { 0 };
float current_temperature[4] = { 0.0 };
int current_temperature_bed_raw = 0;
float current_temperature_bed = 0.0;
#if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
  int redundant_temperature_raw = 0;
  float redundant_temperature = 0.0;
#endif

#if ENABLED(PIDTEMPBED)
  float bedKp = DEFAULT_bedKp;
  float bedKi = (DEFAULT_bedKi* PID_dT);
  float bedKd = (DEFAULT_bedKd / PID_dT);
#endif //PIDTEMPBED

#if ENABLED(FAN_SOFT_PWM)
  unsigned char fanSpeedSoftPwm[FAN_COUNT];
#endif

unsigned char soft_pwm_bed;

#if ENABLED(BABYSTEPPING)
  volatile int babystepsTodo[3] = { 0 };
#endif

#if ENABLED(FILAMENT_WIDTH_SENSOR)
  int current_raw_filwidth = 0;  //Holds measured filament diameter - one extruder only
#endif

#if ENABLED(THERMAL_PROTECTION_HOTENDS) || ENABLED(THERMAL_PROTECTION_BED)
  enum TRState { TRReset, TRInactive, TRFirstHeating, TRStable, TRRunaway };
  void thermal_runaway_protection(TRState* state, millis_t* timer, float temperature, float target_temperature, int heater_id, int period_seconds, int hysteresis_degc);
  #if ENABLED(THERMAL_PROTECTION_HOTENDS)
    static TRState thermal_runaway_state_machine[4] = { TRReset, TRReset, TRReset, TRReset };
    static millis_t thermal_runaway_timer[4]; // = {0,0,0,0};
  #endif
  #if ENABLED(THERMAL_PROTECTION_BED) && TEMP_SENSOR_BED != 0
    static TRState thermal_runaway_bed_state_machine = TRReset;
    static millis_t thermal_runaway_bed_timer;
  #endif
#endif

//===========================================================================
//============================ private variables ============================
//===========================================================================

static volatile bool temp_meas_ready = false;

#if ENABLED(PIDTEMP)
  //static cannot be external:
  static float temp_iState[EXTRUDERS] = { 0 };
  static float temp_dState[EXTRUDERS] = { 0 };
  static float pTerm[EXTRUDERS];
  static float iTerm[EXTRUDERS];
  static float dTerm[EXTRUDERS];
  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    static float cTerm[EXTRUDERS];
    static long last_position[EXTRUDERS];
    static long lpq[LPQ_MAX_LEN];
    static int lpq_ptr = 0;
  #endif
  //int output;
  static float pid_error[EXTRUDERS];
  static float temp_iState_min[EXTRUDERS];
  static float temp_iState_max[EXTRUDERS];
  static bool pid_reset[EXTRUDERS];
#endif //PIDTEMP
#if ENABLED(PIDTEMPBED)
  //static cannot be external:
  static float temp_iState_bed = { 0 };
  static float temp_dState_bed = { 0 };
  static float pTerm_bed;
  static float iTerm_bed;
  static float dTerm_bed;
  //int output;
  static float pid_error_bed;
  static float temp_iState_min_bed;
  static float temp_iState_max_bed;
#else //PIDTEMPBED
  static millis_t  next_bed_check_ms;
#endif //PIDTEMPBED
static unsigned char soft_pwm[EXTRUDERS];

#if ENABLED(FAN_SOFT_PWM)
  static unsigned char soft_pwm_fan[FAN_COUNT];
#endif
#if HAS_AUTO_FAN
  static millis_t next_auto_fan_check_ms;
#endif

#if ENABLED(PIDTEMP)
  #if ENABLED(PID_PARAMS_PER_EXTRUDER)
    float Kp[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(DEFAULT_Kp);
    float Ki[EXTRUDERS] = ARRAY_BY_EXTRUDERS1((DEFAULT_Ki) * (PID_dT));
    float Kd[EXTRUDERS] = ARRAY_BY_EXTRUDERS1((DEFAULT_Kd) / (PID_dT));
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      float Kc[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(DEFAULT_Kc);
    #endif // PID_ADD_EXTRUSION_RATE
  #else //PID_PARAMS_PER_EXTRUDER
    float Kp = DEFAULT_Kp;
    float Ki = (DEFAULT_Ki) * (PID_dT);
    float Kd = (DEFAULT_Kd) / (PID_dT);
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      float Kc = DEFAULT_Kc;
    #endif // PID_ADD_EXTRUSION_RATE
  #endif // PID_PARAMS_PER_EXTRUDER
#endif //PIDTEMP

// Init min and max temp with extreme values to prevent false errors during startup
static int minttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS(HEATER_0_RAW_LO_TEMP , HEATER_1_RAW_LO_TEMP , HEATER_2_RAW_LO_TEMP, HEATER_3_RAW_LO_TEMP);
static int maxttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS(HEATER_0_RAW_HI_TEMP , HEATER_1_RAW_HI_TEMP , HEATER_2_RAW_HI_TEMP, HEATER_3_RAW_HI_TEMP);
static int minttemp[EXTRUDERS] = { 0 };
static int maxttemp[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(65519);
#ifdef BED_MINTEMP
  static int bed_minttemp_raw = HEATER_BED_RAW_LO_TEMP;
#endif
#ifdef BED_MAXTEMP
  static int bed_maxttemp_raw = HEATER_BED_RAW_HI_TEMP;
#endif

#if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
  static void* heater_ttbl_map[2] = {(void*)HEATER_0_TEMPTABLE, (void*)HEATER_1_TEMPTABLE };
  static uint8_t heater_ttbllen_map[2] = { HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN };
#else
  static void* heater_ttbl_map[EXTRUDERS] = ARRAY_BY_EXTRUDERS((void*)HEATER_0_TEMPTABLE, (void*)HEATER_1_TEMPTABLE, (void*)HEATER_2_TEMPTABLE, (void*)HEATER_3_TEMPTABLE);
  static uint8_t heater_ttbllen_map[EXTRUDERS] = ARRAY_BY_EXTRUDERS(HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN, HEATER_2_TEMPTABLE_LEN, HEATER_3_TEMPTABLE_LEN);
#endif

//Paladin:温度失常标志
bool MaxTampErrorTrig = false;
bool MinTampErrorTrig = false;

static float analog2temp(int raw, uint8_t e);
static float analog2tempBed(int raw);
static void updateTemperaturesFromRawValues();

#if ENABLED(THERMAL_PROTECTION_HOTENDS)
  int watch_target_temp[EXTRUDERS] = { 0 };
  millis_t watch_heater_next_ms[EXTRUDERS] = { 0 };
#endif

#ifndef SOFT_PWM_SCALE
  #define SOFT_PWM_SCALE 0
#endif

//===========================================================================
//================================ Functions ================================
//===========================================================================

#if HAS_PID_HEATING

  void PID_autotune(float temp, int extruder, int ncycles, bool set_result/*=false*/) {
    float input = 0.0;
    int cycles = 0;
    bool heating = true;

    millis_t temp_ms = millis(), t1 = temp_ms, t2 = temp_ms;
    long t_high = 0, t_low = 0;

    long bias, d;
    float Ku, Tu;
    float workKp = 0, workKi = 0, workKd = 0;
    float max = 0, min = 10000;

    #if HAS_AUTO_FAN
      millis_t next_auto_fan_check_ms = temp_ms + 2500UL;
    #endif

    if (false
      #if ENABLED(PIDTEMP)
         || extruder >= EXTRUDERS
      #else
         || extruder >= 0
      #endif
      #if DISABLED(PIDTEMPBED)
         || extruder < 0
      #endif
    ) {
      SERIAL_ECHOLN(MSG_PID_BAD_EXTRUDER_NUM);
      return;
    }

    SERIAL_ECHOLN(MSG_PID_AUTOTUNE_START);

    disable_all_heaters(); // switch off all heaters.

    #if HAS_PID_FOR_BOTH
      if (extruder < 0)
        soft_pwm_bed = bias = d = (MAX_BED_POWER) / 2;
      else
        soft_pwm[extruder] = bias = d = (PID_MAX) / 2;
    #elif ENABLED(PIDTEMP)
      soft_pwm[extruder] = bias = d = (PID_MAX) / 2;
    #else
      soft_pwm_bed = bias = d = (MAX_BED_POWER) / 2;
    #endif

	//Paladin
	if((extruder == 0) && (MA_PRINT == MachineSelected))
		TemperaturePwm = soft_pwm[extruder];

    // PID Tuning loop
    for (;;) {

      millis_t ms = millis();

      if (temp_meas_ready) { // temp sample ready
        updateTemperaturesFromRawValues();

        input =
          #if HAS_PID_FOR_BOTH
            extruder < 0 ? current_temperature_bed : current_temperature[extruder]
          #elif ENABLED(PIDTEMP)
            current_temperature[extruder]
          #else
            current_temperature_bed
          #endif
        ;

        max = max(max, input);
        min = min(min, input);

        #if HAS_AUTO_FAN
          if (ELAPSED(ms, next_auto_fan_check_ms)) {
            checkExtruderAutoFans();
            next_auto_fan_check_ms = ms + 2500UL;
          }
        #endif

        if (heating && input > temp) {
          if (ELAPSED(ms, t2 + 5000UL)) {
            heating = false;
            #if HAS_PID_FOR_BOTH
              if (extruder < 0)
                soft_pwm_bed = (bias - d) >> 1;
              else
                soft_pwm[extruder] = (bias - d) >> 1;
            #elif ENABLED(PIDTEMP)
              soft_pwm[extruder] = (bias - d) >> 1;
            #elif ENABLED(PIDTEMPBED)
              soft_pwm_bed = (bias - d) >> 1;
            #endif
            t1 = ms;
            t_high = t1 - t2;
            max = temp;
			//Paladin
			if((extruder == 0) && (MA_PRINT == MachineSelected))
			  TemperaturePwm = soft_pwm[extruder];
          }
        }

        if (!heating && input < temp) {
          if (ELAPSED(ms, t1 + 5000UL)) {
            heating = true;
            t2 = ms;
            t_low = t2 - t1;
            if (cycles > 0) {
              long max_pow =
                #if HAS_PID_FOR_BOTH
                  extruder < 0 ? MAX_BED_POWER : PID_MAX
                #elif ENABLED(PIDTEMP)
                  PID_MAX
                #else
                  MAX_BED_POWER
                #endif
              ;
              bias += (d * (t_high - t_low)) / (t_low + t_high);
              bias = constrain(bias, 20, max_pow - 20);
              d = (bias > max_pow / 2) ? max_pow - 1 - bias : bias;

              SERIAL_PROTOCOLPGM(MSG_BIAS); SERIAL_PROTOCOL(bias);
              SERIAL_PROTOCOLPGM(MSG_D);    SERIAL_PROTOCOL(d);
              SERIAL_PROTOCOLPGM(MSG_T_MIN);  SERIAL_PROTOCOL(min);
              SERIAL_PROTOCOLPGM(MSG_T_MAX);  SERIAL_PROTOCOLLN(max);
              if (cycles > 2) {
                Ku = (4.0 * d) / (3.14159265 * (max - min) / 2.0);
                Tu = ((float)(t_low + t_high) / 1000.0);
                SERIAL_PROTOCOLPGM(MSG_KU); SERIAL_PROTOCOL(Ku);
                SERIAL_PROTOCOLPGM(MSG_TU); SERIAL_PROTOCOLLN(Tu);
                workKp = 0.6 * Ku;
                workKi = 2 * workKp / Tu;
                workKd = workKp * Tu / 8;
                SERIAL_PROTOCOLLNPGM(MSG_CLASSIC_PID);
                SERIAL_PROTOCOLPGM(MSG_KP); SERIAL_PROTOCOLLN(workKp);
                SERIAL_PROTOCOLPGM(MSG_KI); SERIAL_PROTOCOLLN(workKi);
                SERIAL_PROTOCOLPGM(MSG_KD); SERIAL_PROTOCOLLN(workKd);
                /**
                workKp = 0.33*Ku;
                workKi = workKp/Tu;
                workKd = workKp*Tu/3;
                SERIAL_PROTOCOLLNPGM(" Some overshoot ");
                SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(workKp);
                SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(workKi);
                SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(workKd);
                workKp = 0.2*Ku;
                workKi = 2*workKp/Tu;
                workKd = workKp*Tu/3;
                SERIAL_PROTOCOLLNPGM(" No overshoot ");
                SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(workKp);
                SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(workKi);
                SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(workKd);
                */
              }
            }
            #if HAS_PID_FOR_BOTH
              if (extruder < 0)
                soft_pwm_bed = (bias + d) >> 1;
              else
                soft_pwm[extruder] = (bias + d) >> 1;
            #elif ENABLED(PIDTEMP)
              soft_pwm[extruder] = (bias + d) >> 1;
            #else
              soft_pwm_bed = (bias + d) >> 1;
            #endif

			//Paladin
			if((extruder == 0) && (MA_PRINT == MachineSelected))
		      TemperaturePwm = soft_pwm[extruder];
            cycles++;
            min = temp;
          }
        }
      }
      #define MAX_OVERSHOOT_PID_AUTOTUNE 20
      if (input > temp + MAX_OVERSHOOT_PID_AUTOTUNE) {
        SERIAL_PROTOCOLLNPGM(MSG_PID_TEMP_TOO_HIGH);
        return;
      }
      // Every 2 seconds...
      if (ELAPSED(ms, temp_ms + 2000UL)) {
        #if HAS_TEMP_HOTEND || HAS_TEMP_BED
          print_heaterstates();
          SERIAL_EOL;
        #endif

        temp_ms = ms;
      } // every 2 seconds
      // Over 2 minutes?
      if (((ms - t1) + (ms - t2)) > (10L * 60L * 1000L * 2L)) {
        SERIAL_PROTOCOLLNPGM(MSG_PID_TIMEOUT);
        return;
      }
      if (cycles > ncycles) {
        SERIAL_PROTOCOLLNPGM(MSG_PID_AUTOTUNE_FINISHED);

        #if HAS_PID_FOR_BOTH
          const char* estring = extruder < 0 ? "bed" : "";
          SERIAL_PROTOCOLPGM("#define  DEFAULT_"); SERIAL_PROTOCOL(estring); SERIAL_PROTOCOLPGM("Kp "); SERIAL_PROTOCOLLN(workKp);
          SERIAL_PROTOCOLPGM("#define  DEFAULT_"); SERIAL_PROTOCOL(estring); SERIAL_PROTOCOLPGM("Ki "); SERIAL_PROTOCOLLN(workKi);
          SERIAL_PROTOCOLPGM("#define  DEFAULT_"); SERIAL_PROTOCOL(estring); SERIAL_PROTOCOLPGM("Kd "); SERIAL_PROTOCOLLN(workKd);
        #elif ENABLED(PIDTEMP)
          SERIAL_PROTOCOLPGM("#define  DEFAULT_Kp "); SERIAL_PROTOCOLLN(workKp);
          SERIAL_PROTOCOLPGM("#define  DEFAULT_Ki "); SERIAL_PROTOCOLLN(workKi);
          SERIAL_PROTOCOLPGM("#define  DEFAULT_Kd "); SERIAL_PROTOCOLLN(workKd);
        #else
          SERIAL_PROTOCOLPGM("#define  DEFAULT_bedKp "); SERIAL_PROTOCOLLN(workKp);
          SERIAL_PROTOCOLPGM("#define  DEFAULT_bedKi "); SERIAL_PROTOCOLLN(workKi);
          SERIAL_PROTOCOLPGM("#define  DEFAULT_bedKd "); SERIAL_PROTOCOLLN(workKd);
        #endif

        #define _SET_BED_PID() \
          bedKp = workKp; \
          bedKi = scalePID_i(workKi); \
          bedKd = scalePID_d(workKd); \
          updatePID()

        #define _SET_EXTRUDER_PID() \
          PID_PARAM(Kp, extruder) = workKp; \
          PID_PARAM(Ki, extruder) = scalePID_i(workKi); \
          PID_PARAM(Kd, extruder) = scalePID_d(workKd); \
          updatePID()

        // Use the result? (As with "M303 U1")
        if (set_result) {
          #if HAS_PID_FOR_BOTH
            if (extruder < 0) {
              _SET_BED_PID();
            }
            else {
              _SET_EXTRUDER_PID();
            }
          #elif ENABLED(PIDTEMP)
            _SET_EXTRUDER_PID();
          #else
            _SET_BED_PID();
          #endif
        }
        return;
      }
	  //Add by Paladin
	  HMICommandProcess();
	  HMIScreenShow();
      lcd_update();
    }
  }

#endif // PIDTEMP

void updatePID() {
  #if ENABLED(PIDTEMP)
    for (int e = 0; e < EXTRUDERS; e++) {
      temp_iState_max[e] = (PID_INTEGRAL_DRIVE_MAX) / PID_PARAM(Ki, e);
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        last_position[e] = 0;
      #endif
    }
  #endif
  #if ENABLED(PIDTEMPBED)
    temp_iState_max_bed = (PID_BED_INTEGRAL_DRIVE_MAX) / bedKi;
  #endif
}

int getHeaterPower(int heater) {
  return heater < 0 ? soft_pwm_bed : soft_pwm[heater];
}

#if HAS_AUTO_FAN

void setExtruderAutoFanState(int pin, bool state) {
  unsigned char newFanSpeed = (state != 0) ? EXTRUDER_AUTO_FAN_SPEED : 0;
  // this idiom allows both digital and PWM fan outputs (see M42 handling).
  digitalWrite(pin, newFanSpeed);
  analogWrite(pin, newFanSpeed);
}

void checkExtruderAutoFans() {
  uint8_t fanState = 0;

  // which fan pins need to be turned on?
  #if HAS_AUTO_FAN_0
    if (current_temperature[0] > EXTRUDER_AUTO_FAN_TEMPERATURE)
      fanState |= 1;
  #endif
  #if HAS_AUTO_FAN_1
    if (current_temperature[1] > EXTRUDER_AUTO_FAN_TEMPERATURE) {
      if (EXTRUDER_1_AUTO_FAN_PIN == EXTRUDER_0_AUTO_FAN_PIN)
        fanState |= 1;
      else
        fanState |= 2;
    }
  #endif
  #if HAS_AUTO_FAN_2
    if (current_temperature[2] > EXTRUDER_AUTO_FAN_TEMPERATURE) {
      if (EXTRUDER_2_AUTO_FAN_PIN == EXTRUDER_0_AUTO_FAN_PIN)
        fanState |= 1;
      else if (EXTRUDER_2_AUTO_FAN_PIN == EXTRUDER_1_AUTO_FAN_PIN)
        fanState |= 2;
      else
        fanState |= 4;
    }
  #endif
  #if HAS_AUTO_FAN_3
    if (current_temperature[3] > EXTRUDER_AUTO_FAN_TEMPERATURE) {
      if (EXTRUDER_3_AUTO_FAN_PIN == EXTRUDER_0_AUTO_FAN_PIN)
        fanState |= 1;
      else if (EXTRUDER_3_AUTO_FAN_PIN == EXTRUDER_1_AUTO_FAN_PIN)
        fanState |= 2;
      else if (EXTRUDER_3_AUTO_FAN_PIN == EXTRUDER_2_AUTO_FAN_PIN)
        fanState |= 4;
      else
        fanState |= 8;
    }
  #endif

  // update extruder auto fan states
  #if HAS_AUTO_FAN_0
    setExtruderAutoFanState(EXTRUDER_0_AUTO_FAN_PIN, (fanState & 1) != 0);
  #endif
  #if HAS_AUTO_FAN_1
    if (EXTRUDER_1_AUTO_FAN_PIN != EXTRUDER_0_AUTO_FAN_PIN)
      setExtruderAutoFanState(EXTRUDER_1_AUTO_FAN_PIN, (fanState & 2) != 0);
  #endif
  #if HAS_AUTO_FAN_2
    if (EXTRUDER_2_AUTO_FAN_PIN != EXTRUDER_0_AUTO_FAN_PIN
        && EXTRUDER_2_AUTO_FAN_PIN != EXTRUDER_1_AUTO_FAN_PIN)
      setExtruderAutoFanState(EXTRUDER_2_AUTO_FAN_PIN, (fanState & 4) != 0);
  #endif
  #if HAS_AUTO_FAN_3
    if (EXTRUDER_3_AUTO_FAN_PIN != EXTRUDER_0_AUTO_FAN_PIN
        && EXTRUDER_3_AUTO_FAN_PIN != EXTRUDER_1_AUTO_FAN_PIN
        && EXTRUDER_3_AUTO_FAN_PIN != EXTRUDER_2_AUTO_FAN_PIN)
      setExtruderAutoFanState(EXTRUDER_3_AUTO_FAN_PIN, (fanState & 8) != 0);
  #endif
}

#endif // HAS_AUTO_FAN



//Paladin:测试板专用，没有KILL
#ifdef NO_KILL

//
// Temperature Error Handlers
//
inline void _temp_error(int e, const char* serial_msg, const char* lcd_msg) {
  static bool killed = false;
  if (IsRunning()) {
    SERIAL_ERROR_START;
    serialprintPGM(serial_msg);
    SERIAL_ERRORPGM(MSG_STOPPED_HEATER);
    if (e >= 0) SERIAL_ERRORLN((int)e); else SERIAL_ERRORLNPGM(MSG_HEATER_BED);
  }
  #if DISABLED(BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE)
    #if(SNAP_OLD_BASE == 1)
    if (!killed) {
      //Running = false;
      //killed = true;
      //kill(lcd_msg);
    }
    else
    {
      //disable_all_heaters(); // paranoia
    }
	#endif
  #endif
}

void max_temp_error(uint8_t e) {
  if(GetCurrentPrinterStatus() == STAT_PRINTING)
  {
  	PauseTriggle(ManualPause);
  }
}
void min_temp_error(uint8_t e) {
  
  if(GetCurrentPrinterStatus() == STAT_PRINTING)
  {
  	PauseTriggle(ManualPause);
  }
  StopTriggle(ManualStop);
  #if(SNAP_OLD_BASE == 1)
  //Paladin:显示NA
  if(e == 0)
	  HMI_ShowTemperature(0, -27, -27);
  #endif
}
#else

inline void _temp_error(int e, const char* serial_msg, const char* lcd_msg) {
  static bool killed = false;
  if (IsRunning()) {
    SERIAL_ERROR_START;
    serialprintPGM(serial_msg);
    SERIAL_ERRORPGM(MSG_STOPPED_HEATER);
    if (e >= 0) SERIAL_ERRORLN((int)e); else SERIAL_ERRORLNPGM(MSG_HEATER_BED);
  }
  #if DISABLED(BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE)
    #if(SNAP_OLD_BASE == 1)
    if (!killed) {
      Running = false;
      killed = true;
      kill(lcd_msg);
    }
    else
    {
      disable_all_heaters(); // paranoia
    }
	#endif
  #endif
}


void max_temp_error(uint8_t e) {
  _temp_error(e, PSTR(MSG_T_MAXTEMP), PSTR(MSG_ERR_MAXTEMP));
}
void min_temp_error(uint8_t e) {
  //Paladin:显示NA
  if(e == 0)
	  HMI_ShowTemperature(0, -27, -27);
  _temp_error(e, PSTR(MSG_T_MINTEMP), PSTR(MSG_ERR_MINTEMP));
}

#endif

float get_pid_output(int e) {
  static float pid_output;
  #if ENABLED(PIDTEMP)
    #if DISABLED(PID_OPENLOOP)
      pid_error[e] = target_temperature[e] - current_temperature[e];
      dTerm[e] = K2 * PID_PARAM(Kd, e) * (current_temperature[e] - temp_dState[e]) + K1 * dTerm[e];
      temp_dState[e] = current_temperature[e];
      if (pid_error[e] > PID_FUNCTIONAL_RANGE) {
        pid_output = BANG_MAX;
        pid_reset[e] = true;
      }
      else if (pid_error[e] < -(PID_FUNCTIONAL_RANGE) || target_temperature[e] == 0) {
        pid_output = 0;
        pid_reset[e] = true;
      }
      else {
        if (pid_reset[e]) {
          temp_iState[e] = 0.0;
          pid_reset[e] = false;
        }
        pTerm[e] = PID_PARAM(Kp, e) * pid_error[e];
        temp_iState[e] += pid_error[e];
        temp_iState[e] = constrain(temp_iState[e], temp_iState_min[e], temp_iState_max[e]);
        iTerm[e] = PID_PARAM(Ki, e) * temp_iState[e];

        pid_output = pTerm[e] + iTerm[e] - dTerm[e];

        #if ENABLED(PID_ADD_EXTRUSION_RATE)
          cTerm[e] = 0;
          if (e == active_extruder) {
            long e_position = st_get_position(E_AXIS);
            if (e_position > last_position[e]) {
              lpq[lpq_ptr++] = e_position - last_position[e];
              last_position[e] = e_position;
            }
            else {
              lpq[lpq_ptr++] = 0;
            }
            if (lpq_ptr >= lpq_len) lpq_ptr = 0;
            cTerm[e] = (lpq[lpq_ptr] / axis_steps_per_unit[E_AXIS]) * PID_PARAM(Kc, e);
            pid_output += cTerm[e];
          }
        #endif //PID_ADD_EXTRUSION_RATE

        if (pid_output > PID_MAX) {
          if (pid_error[e] > 0) temp_iState[e] -= pid_error[e]; // conditional un-integration
          pid_output = PID_MAX;
        }
        else if (pid_output < 0) {
          if (pid_error[e] < 0) temp_iState[e] -= pid_error[e]; // conditional un-integration
          pid_output = 0;
        }
      }
    #else
      pid_output = constrain(target_temperature[e], 0, PID_MAX);
    #endif //PID_OPENLOOP

    #if ENABLED(PID_DEBUG)
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR(MSG_PID_DEBUG, e);
      SERIAL_ECHOPAIR(MSG_PID_DEBUG_INPUT, current_temperature[e]);
      SERIAL_ECHOPAIR(MSG_PID_DEBUG_OUTPUT, pid_output);
      SERIAL_ECHOPAIR(MSG_PID_DEBUG_PTERM, pTerm[e]);
      SERIAL_ECHOPAIR(MSG_PID_DEBUG_ITERM, iTerm[e]);
      SERIAL_ECHOPAIR(MSG_PID_DEBUG_DTERM, dTerm[e]);
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        SERIAL_ECHOPAIR(MSG_PID_DEBUG_CTERM, cTerm[e]);
      #endif
      SERIAL_EOL;
    #endif //PID_DEBUG

  #else /* PID off */
    pid_output = (current_temperature[e] < target_temperature[e]) ? PID_MAX : 0;
  #endif

  return pid_output;
}

#if ENABLED(PIDTEMPBED)
  float get_pid_output_bed() {
    float pid_output;
    #if DISABLED(PID_OPENLOOP)
      pid_error_bed = target_temperature_bed - current_temperature_bed;
      pTerm_bed = bedKp * pid_error_bed;
      temp_iState_bed += pid_error_bed;
      temp_iState_bed = constrain(temp_iState_bed, temp_iState_min_bed, temp_iState_max_bed);
      iTerm_bed = bedKi * temp_iState_bed;

      dTerm_bed = K2 * bedKd * (current_temperature_bed - temp_dState_bed) + K1 * dTerm_bed;
      temp_dState_bed = current_temperature_bed;

      pid_output = pTerm_bed + iTerm_bed - dTerm_bed;
      if (pid_output > MAX_BED_POWER) {
        if (pid_error_bed > 0) temp_iState_bed -= pid_error_bed; // conditional un-integration
        pid_output = MAX_BED_POWER;
      }
      else if (pid_output < 0) {
        if (pid_error_bed < 0) temp_iState_bed -= pid_error_bed; // conditional un-integration
        pid_output = 0;
      }
    #else
      pid_output = constrain(target_temperature_bed, 0, MAX_BED_POWER);
    #endif // PID_OPENLOOP

    #if ENABLED(PID_BED_DEBUG)
      SERIAL_ECHO_START;
      SERIAL_ECHO(" PID_BED_DEBUG ");
      SERIAL_ECHO(": Input ");
      SERIAL_ECHO(current_temperature_bed);
      SERIAL_ECHO(" Output ");
      SERIAL_ECHO(pid_output);
      SERIAL_ECHO(" pTerm ");
      SERIAL_ECHO(pTerm_bed);
      SERIAL_ECHO(" iTerm ");
      SERIAL_ECHO(iTerm_bed);
      SERIAL_ECHO(" dTerm ");
      SERIAL_ECHOLN(dTerm_bed);
    #endif //PID_BED_DEBUG

    return pid_output;
  }
#endif //PIDTEMPBED

/**
 * Manage heating activities for extruder hot-ends and a heated bed
 *  - Acquire updated temperature readings
 *  - Invoke thermal runaway protection
 *  - Manage extruder auto-fan
 *  - Apply filament width to the extrusion rate (may move)
 *  - Update the heated bed PID output value
 */
void manage_heater() 
{
  if (!temp_meas_ready) return;

  updateTemperaturesFromRawValues();

  #if ENABLED(HEATER_0_USES_MAX6675)
    float ct = current_temperature[0];
    if (ct > min(HEATER_0_MAXTEMP, 4095)) max_temp_error(0);
    if (ct < max(HEATER_0_MINTEMP, 0.01)) min_temp_error(0);
  #endif

  #if ENABLED(THERMAL_PROTECTION_HOTENDS) || DISABLED(PIDTEMPBED) || HAS_AUTO_FAN
    millis_t ms = millis();
  #endif

  // Loop through all extruders
  for (int e = 0; e < EXTRUDERS; e++) {

    #if ENABLED(THERMAL_PROTECTION_HOTENDS)
      thermal_runaway_protection(&thermal_runaway_state_machine[e], &thermal_runaway_timer[e], current_temperature[e], target_temperature[e], e, THERMAL_PROTECTION_PERIOD, THERMAL_PROTECTION_HYSTERESIS);
    #endif

    float pid_output = get_pid_output(e);

    // Check if temperature is within the correct range
    soft_pwm[e] = current_temperature[e] > minttemp[e] && current_temperature[e] < maxttemp[e] ? (int)pid_output >> 1 : 0;

    // Check if the temperature is failing to increase
    #if ENABLED(THERMAL_PROTECTION_HOTENDS)

      // Is it time to check this extruder's heater?
      if (watch_heater_next_ms[e] && ELAPSED(ms, watch_heater_next_ms[e])) {
        // Has it failed to increase enough?
        if (degHotend(e) < watch_target_temp[e]) {
          // Stop!
          _temp_error(e, PSTR(MSG_T_HEATING_FAILED), PSTR(MSG_HEATING_FAILED_LCD));
        }
        else {
          // Start again if the target is still far off
          start_watching_heater(e);
        }
      }
	  //Paladin
	  //if(MachineSelected == MA_PRINT)
		  //TempDropCheck(0);
    #endif // THERMAL_PROTECTION_HOTENDS

    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      if (fabs(current_temperature[0] - redundant_temperature) > MAX_REDUNDANT_TEMP_SENSOR_DIFF) {
        _temp_error(0, PSTR(MSG_REDUNDANCY), PSTR(MSG_ERR_REDUNDANT_TEMP));
      }
    #endif

  } // Extruders Loop


  //Paladin
  //if((target_temperature[0] - current_temperature[0]) > PID_FUNCTIONAL_RANGE)
  //	TIM1->CCR2 = 150;
  //else
  if(MA_PRINT == MachineSelected)
  	TemperaturePwm = soft_pwm[0];
  

  #if HAS_AUTO_FAN
    if (ELAPSED(ms, next_auto_fan_check_ms)) { // only need to check fan state very infrequently
      checkExtruderAutoFans();
      next_auto_fan_check_ms = ms + 2500UL;
    }
  #endif

  // Control the extruder rate based on the width sensor
  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    if (filament_sensor) {
      meas_shift_index = filwidth_delay_index1 - meas_delay_cm;
      if (meas_shift_index < 0) meas_shift_index += MAX_MEASUREMENT_DELAY + 1;  //loop around buffer if needed

      // Get the delayed info and add 100 to reconstitute to a percent of
      // the nominal filament diameter then square it to get an area
      meas_shift_index = constrain(meas_shift_index, 0, MAX_MEASUREMENT_DELAY);
      float vm = pow((measurement_delay[meas_shift_index] + 100.0) / 100.0, 2);
      NOLESS(vm, 0.01);
      volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM] = vm;
    }
  #endif //FILAMENT_WIDTH_SENSOR

  #if DISABLED(PIDTEMPBED)
    if (PENDING(ms, next_bed_check_ms)) return;
    next_bed_check_ms = ms + BED_CHECK_INTERVAL;
  #endif

  #if TEMP_SENSOR_BED != 0

    #if ENABLED(THERMAL_PROTECTION_BED)
      thermal_runaway_protection(&thermal_runaway_bed_state_machine, &thermal_runaway_bed_timer, current_temperature_bed, target_temperature_bed, -1, THERMAL_PROTECTION_BED_PERIOD, THERMAL_PROTECTION_BED_HYSTERESIS);
    #endif

    #if ENABLED(PIDTEMPBED)
      float pid_output = get_pid_output_bed();

      soft_pwm_bed = current_temperature_bed > BED_MINTEMP && current_temperature_bed < BED_MAXTEMP ? (int)pid_output >> 1 : 0;

    #elif ENABLED(BED_LIMIT_SWITCHING)
      // Check if temperature is within the correct band
      if (current_temperature_bed > BED_MINTEMP && current_temperature_bed < BED_MAXTEMP) {
        if (current_temperature_bed >= target_temperature_bed + BED_HYSTERESIS)
          soft_pwm_bed = 0;
        else if (current_temperature_bed <= target_temperature_bed - (BED_HYSTERESIS))
          soft_pwm_bed = MAX_BED_POWER >> 1;
      }
      else {
        soft_pwm_bed = 0;
        WRITE_HEATER_BED(LOW);
      }
    #else // !PIDTEMPBED && !BED_LIMIT_SWITCHING
      // Check if temperature is within the correct range
      //Paladin:如果热床温度设置大于30  度才有效
      if ((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP) && (target_temperature_bed > 30)) {
        soft_pwm_bed = current_temperature_bed < target_temperature_bed ? (MAX_BED_POWER / 2) : 10;
      }
      else {
        soft_pwm_bed = 0;
        //WRITE_HEATER_BED(LOW);
      }

	  TIM1->CCR1 = soft_pwm_bed;
    #endif
  #endif //TEMP_SENSOR_BED != 0
}

#define PGM_RD_W(x)   *((long*)(x))
// Derived from RepRap FiveD extruder::getTemperature()
// For hot end temperature measurement.
static float analog2temp(int raw, uint8_t e) {
  #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
    if (e > EXTRUDERS)
  #else
    if (e >= EXTRUDERS)
  #endif
    {
      SERIAL_ERROR_START;
      SERIAL_ERROR((int)e);
      SERIAL_ERRORLNPGM(MSG_INVALID_EXTRUDER_NUM);
      kill(PSTR(MSG_KILLED));
      return 0.0;
    }

  #if ENABLED(HEATER_0_USES_MAX6675)
    if (e == 0) return 0.25 * raw;
  #endif

  if (heater_ttbl_map[e] != NULL) {
    float celsius = 0;
    uint16_t i;
	long *tt = (long*)heater_ttbl_map[e];
	long tableAnalogValue;
    for (i = 1; i < heater_ttbllen_map[e]; i++) 
	{
	  tableAnalogValue = tt[i * 2];
      if (tableAnalogValue > raw) {
        celsius  = tt[i * 2 + 1] +
                   ((tt[i * 2] - raw) *
                   (float)(tt[(i - 1) * 2 + 1] - tt[i * 2 + 1]) /
                   (float)(tt[i * 2] - tt[(i - 1) * 2]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == heater_ttbllen_map[e]) celsius = tt[(i-1) * 2 + 1];

    return celsius;
  }
  return ((raw * ((5.0 * 100.0) / 4095.0) / OVERSAMPLENR) * (TEMP_SENSOR_AD595_GAIN)) + TEMP_SENSOR_AD595_OFFSET;
}

// Derived from RepRap FiveD extruder::getTemperature()
// For bed temperature measurement.
static float analog2tempBed(int raw) {
  #if ENABLED(BED_USES_THERMISTOR)
    float celsius = 0;
    uint16_t i;
	long *tt;
	tt = (long*)BEDTEMPTABLE;
	long tableAnalogValue;
    for (i = 1; i < BEDTEMPTABLE_LEN; i++) 
	{
	  tableAnalogValue = tt[i * 2];
      if (tableAnalogValue > raw) 
	  	{
        celsius  = tt[i * 2 + 1] +
                   ((tt[i * 2] - raw) *
                   (float)(tt[(i - 1) * 2 + 1] - tt[i * 2 + 1]) /
                   (float)(tt[i * 2] - tt[(i - 1) * 2]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == BEDTEMPTABLE_LEN) celsius = tt[(i-1) * 2 + 1];

    return celsius;

  #elif defined(BED_USES_AD595)

    return ((raw * ((5.0 * 100.0) / 4095.0) / OVERSAMPLENR) * (TEMP_SENSOR_AD595_GAIN)) + TEMP_SENSOR_AD595_OFFSET;

  #else

    UNUSED(raw);
    return 0;

  #endif
}

/* Called to get the raw values into the the actual temperatures. The raw values are created in interrupt context,
    and this function is called from normal context as it is too slow to run in interrupts and will block the stepper routine otherwise */
static void updateTemperaturesFromRawValues() {
  #if ENABLED(HEATER_0_USES_MAX6675)
    current_temperature_raw[0] = read_max6675();
  #endif
  for (uint8_t e = 0; e < EXTRUDERS; e++) {
    current_temperature[e] = analog2temp(current_temperature_raw[e], e);
	//Paladin :  温度测试
	//current_temperature[e] = 20;
  }
  current_temperature_bed = analog2tempBed(current_temperature_bed_raw);
  //current_temperature_bed = 50;
  #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
    redundant_temperature = analog2temp(redundant_temperature_raw, 1);
  #endif
  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    filament_width_meas = analog2widthFil();
  #endif

  #if ENABLED(USE_WATCHDOG)
    // Reset the watchdog after we know we have a temperature measurement.
    watchdog_reset();
  #endif

  CRITICAL_SECTION_START;
  temp_meas_ready = false;
  CRITICAL_SECTION_END;
}


#if ENABLED(FILAMENT_WIDTH_SENSOR)

  // Convert raw Filament Width to millimeters
  float analog2widthFil() {
    return current_raw_filwidth / 16383.0 * 5.0;
    //return current_raw_filwidth;
  }

  // Convert raw Filament Width to a ratio
  int widthFil_to_size_ratio() {
    float temp = filament_width_meas;
    if (temp < MEASURED_LOWER_LIMIT) temp = filament_width_nominal;  //assume sensor cut out
    else NOMORE(temp, MEASURED_UPPER_LIMIT);
    return filament_width_nominal / temp * 100;
  }

#endif


/**
 * Initialize the temperature manager
 * The manager is implemented by periodic calls to manage_heater()
 */
void tp_init() {
  // Finish init of mult extruder arrays
  for (int e = 0; e < EXTRUDERS; e++) {
    // populate with the first value
    maxttemp[e] = maxttemp[0];
    #if ENABLED(PIDTEMP)
      temp_iState_min[e] = 0.0;
      temp_iState_max[e] = (PID_INTEGRAL_DRIVE_MAX) / PID_PARAM(Ki, e);
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        last_position[e] = 0;
      #endif
    #endif //PIDTEMP
    #if ENABLED(PIDTEMPBED)
      temp_iState_min_bed = 0.0;
      temp_iState_max_bed = (PID_BED_INTEGRAL_DRIVE_MAX) / bedKi;
    #endif //PIDTEMPBED
  }

  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;

  #ifdef TEMP_BED_PIN
  GPIO_InitStruct.GPIO_Pin = TEMP_BED_PIN;
  GPIO_Init(TEMP_BED_PORT, &GPIO_InitStruct);
  TEMP_BED_PORT->BRR = TEMP_BED_PIN;
  #endif

  #ifdef TEMP_0_PIN
  GPIO_InitStruct.GPIO_Pin = TEMP_0_PIN;
  GPIO_Init(TEMP_0_PORT, &GPIO_InitStruct);
  TEMP_0_PORT->BRR = TEMP_0_PIN;
  #endif

  #ifdef GATE_CTRL_PORT
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStruct.GPIO_Pin = GATE_CTRL_PIN;
  GPIO_Init(GATE_CTRL_PORT, &GPIO_InitStruct);
  #endif

  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  #ifdef HEATER_BED_PIN
  digitalWrite(HEATER_BED_PORT, HEATER_BED_PIN, LOW);
  GPIO_InitStruct.GPIO_Pin = HEATER_BED_PIN;
  GPIO_Init(HEATER_BED_PORT, &GPIO_InitStruct);
  #endif

  #if(SNAP_OLD_BASE == 1)
  digitalWrite(GPIOB, GPIO_Pin_1, LOW);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
  #endif
  
  // Set analog inputs
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
  
  #ifdef TEMP_BED_PIN
  GPIO_InitStruct.GPIO_Pin = TEMP_BED_PIN;
  GPIO_Init(TEMP_BED_PORT, &GPIO_InitStruct);
  #endif

  #ifdef TEMP_0_PIN
  GPIO_InitStruct.GPIO_Pin = TEMP_0_PIN;
  GPIO_Init(TEMP_0_PORT, &GPIO_InitStruct);
  #endif

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

  TIM_DeInit(TIM3);
  ADC_DeInit(ADC1);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  //ADC Init
  TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_Prescaler = SystemCoreClock / 1000000 - 1;
  TIM_TimeBaseInitStruct.TIM_Period = 2399;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, DISABLE);

  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
  
  ADC_InitTypeDef ADC_InitStruct;
  ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  ADC_InitStruct.ADC_Mode =ADC_Mode_Independent;
  ADC_InitStruct.ADC_NbrOfChannel = 2;
  ADC_InitStruct.ADC_ScanConvMode = ENABLE;
  ADC_Init(ADC1, &ADC_InitStruct);
  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_239Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 2, ADC_SampleTime_239Cycles5);
  
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  ADC_DMACmd(ADC1, ENABLE);
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1) == SET);
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1) == SET);

  for(int i=0;i<32;i++)
  	tmpADC[i] = 0;

  DMA_InitTypeDef DMA_InitStruct;
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
  DMA_InitStruct.DMA_BufferSize = 32;
  DMA_InitStruct.DMA_MemoryBaseAddr = (U32)tmpADC;
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStruct.DMA_PeripheralBaseAddr = (U32)(ADC1_BASE | 0x4c);
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_Priority = DMA_Priority_High;
  DMA_Init(DMA1_Channel1, &DMA_InitStruct);
  DMA_Cmd(DMA1_Channel1, ENABLE);


  //DMA1_Channel1
  NVIC_InitTypeDef NVicInit;
  NVicInit.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVicInit.NVIC_IRQChannelSubPriority = 0;
  NVicInit.NVIC_IRQChannelPreemptionPriority = 2;
  NVicInit.NVIC_IRQChannelCmd = ENABLE;
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
  NVIC_Init(&NVicInit);
  
  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt

  #define TEMP_MIN_ROUTINE(NR) \
    minttemp[NR] = HEATER_ ## NR ## _MINTEMP; \
    while(analog2temp(minttemp_raw[NR], NR) < HEATER_ ## NR ## _MINTEMP) { \
      if (HEATER_ ## NR ## _RAW_LO_TEMP < HEATER_ ## NR ## _RAW_HI_TEMP) \
        minttemp_raw[NR] += OVERSAMPLENR; \
      else \
        minttemp_raw[NR] -= OVERSAMPLENR; \
    }
  #define TEMP_MAX_ROUTINE(NR) \
    maxttemp[NR] = HEATER_ ## NR ## _MAXTEMP; \
    while(analog2temp(maxttemp_raw[NR], NR) > HEATER_ ## NR ## _MAXTEMP) { \
      if (HEATER_ ## NR ## _RAW_LO_TEMP < HEATER_ ## NR ## _RAW_HI_TEMP) \
        maxttemp_raw[NR] -= OVERSAMPLENR; \
      else \
        maxttemp_raw[NR] += OVERSAMPLENR; \
    }

  #ifdef HEATER_0_MINTEMP
    TEMP_MIN_ROUTINE(0);
  #endif
  #ifdef HEATER_0_MAXTEMP
    TEMP_MAX_ROUTINE(0);
  #endif
  #if EXTRUDERS > 1
    #ifdef HEATER_1_MINTEMP
      TEMP_MIN_ROUTINE(1);
    #endif
    #ifdef HEATER_1_MAXTEMP
      TEMP_MAX_ROUTINE(1);
    #endif
    #if EXTRUDERS > 2
      #ifdef HEATER_2_MINTEMP
        TEMP_MIN_ROUTINE(2);
      #endif
      #ifdef HEATER_2_MAXTEMP
        TEMP_MAX_ROUTINE(2);
      #endif
      #if EXTRUDERS > 3
        #ifdef HEATER_3_MINTEMP
          TEMP_MIN_ROUTINE(3);
        #endif
        #ifdef HEATER_3_MAXTEMP
          TEMP_MAX_ROUTINE(3);
        #endif
      #endif // EXTRUDERS > 3
    #endif // EXTRUDERS > 2
  #endif // EXTRUDERS > 1

  #ifdef BED_MINTEMP
    while(analog2tempBed(bed_minttemp_raw) < BED_MINTEMP) {
      #if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
        bed_minttemp_raw += OVERSAMPLENR;
      #else
        bed_minttemp_raw -= OVERSAMPLENR;
      #endif
    }
  #endif //BED_MINTEMP
  #ifdef BED_MAXTEMP
    while (analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) {
      #if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
        bed_maxttemp_raw -= OVERSAMPLENR;
      #else
        bed_maxttemp_raw += OVERSAMPLENR;
      #endif
    }
  #endif //BED_MAXTEMP

  //Paladin  开始ADC  采样
  #if(SNAP_OLD_BASE == 1)
  TIM3->CR1 |= TIM_CR1_CEN;
  #endif
}

#if ENABLED(THERMAL_PROTECTION_HOTENDS)
  /**
   * Start Heating Sanity Check for hotends that are below
   * their target temperature by a configurable margin.
   * This is called when the temperature is set. (M104, M109)
   */
  void start_watching_heater(int e) {
    if (degHotend(e) < degTargetHotend(e) - (WATCH_TEMP_INCREASE + TEMP_HYSTERESIS + 1)) {
      watch_target_temp[e] = degHotend(e) + WATCH_TEMP_INCREASE;
      watch_heater_next_ms[e] = millis() + (WATCH_TEMP_PERIOD) * 1000UL;
    }
    else
      watch_heater_next_ms[e] = 0;
  }


  #ifdef NO_KILL
  void TempDropCheck(int e)
  {

  }
  #else
  //Paladin
  void TempDropCheck(int e)
  {
  	static float degree[EXTRUDERS][2];
	static millis_t checkmills;

	//检测时间已到
	if(checkmills < millis())
	{
		checkmills = millis() + 2000UL;
		degree[e][0] = degree[e][1];
		degree[e][1] = degHotend(e);
		//加热过程中
		//if(degTargetHotend(e) > 125)
		{
			//之前温度比当前温度高
			if(degree[e][0] > degree[e][1])
			{
				//温度差超过15  度
				if(degree[e][0] - degree[e][1] > 20)
				{
					kill(PSTR("Drop Tamp"));
				}
			}
		}
	}
  }
  #endif
#endif

#if ENABLED(THERMAL_PROTECTION_HOTENDS) || ENABLED(THERMAL_PROTECTION_BED)

  void thermal_runaway_protection(TRState* state, millis_t* timer, float temperature, float target_temperature, int heater_id, int period_seconds, int hysteresis_degc) {

    static float tr_target_temperature[EXTRUDERS + 1] = { 0.0 };

    /**
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("Thermal Thermal Runaway Running. Heater ID: ");
        if (heater_id < 0) SERIAL_ECHOPGM("bed"); else SERIAL_ECHOPGM(heater_id);
        SERIAL_ECHOPGM(" ;  State:");
        SERIAL_ECHOPGM(*state);
        SERIAL_ECHOPGM(" ;  Timer:");
        SERIAL_ECHOPGM(*timer);
        SERIAL_ECHOPGM(" ;  Temperature:");
        SERIAL_ECHOPGM(temperature);
        SERIAL_ECHOPGM(" ;  Target Temp:");
        SERIAL_ECHOPGM(target_temperature);
        SERIAL_EOL;
    */

    int heater_index = heater_id >= 0 ? heater_id : EXTRUDERS;

    // If the target temperature changes, restart
    if (tr_target_temperature[heater_index] != target_temperature)
      *state = TRReset;

    switch (*state) {
      case TRReset:
        *timer = 0;
        *state = TRInactive;
      // Inactive state waits for a target temperature to be set
      case TRInactive:
        if (target_temperature > 0) {
          tr_target_temperature[heater_index] = target_temperature;
          *state = TRFirstHeating;
        }
        break;
      // When first heating, wait for the temperature to be reached then go to Stable state
      case TRFirstHeating:
        if (temperature >= tr_target_temperature[heater_index]) *state = TRStable;
        break;
      // While the temperature is stable watch for a bad temperature
      case TRStable:
        // If the temperature is over the target (-hysteresis) restart the timer
        if (temperature >= tr_target_temperature[heater_index] - hysteresis_degc)
          *timer = millis();
        // If the timer goes too long without a reset, trigger shutdown
        else if (ELAPSED(millis(), *timer + period_seconds * 1000UL))
          *state = TRRunaway;
        break;
      case TRRunaway:
        _temp_error(heater_id, PSTR(MSG_T_THERMAL_RUNAWAY), PSTR(MSG_THERMAL_RUNAWAY));
    }
  }

#endif // THERMAL_PROTECTION_HOTENDS || THERMAL_PROTECTION_BED

void disable_all_heaters() {
  for (int i = 0; i < EXTRUDERS; i++) setTargetHotend(0, i);
  setTargetBed(0);

  // If all heaters go down then for sure our print job has stopped
  print_job_timer.stop();
  WRITE_HEATER_BED(LOW);
  WRITE_HEATER_0(LOW);
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  SetLaserPower(0);
  digitalWrite(LASER_PORT, LASER_PIN, 0);
}

/**
 * Stages in the ISR loop
 */
enum TempState {
  PrepareTemp_0,
  MeasureTemp_0,
  PrepareTemp_BED,
  MeasureTemp_BED,
  PrepareTemp_1,
  MeasureTemp_1,
  PrepareTemp_2,
  MeasureTemp_2,
  PrepareTemp_3,
  MeasureTemp_3,
  Prepare_FILWIDTH,
  Measure_FILWIDTH,
  StartupDelay // Startup, delay initial temp reading a tiny bit so the hardware can settle
};

static unsigned long raw_temp_value[4] = { 0 };
static unsigned long raw_temp_bed_value = 0;

static void set_current_temp_raw() {
  #if HAS_TEMP_0 && DISABLED(HEATER_0_USES_MAX6675)
    current_temperature_raw[0] = raw_temp_value[0];
  #endif
  #if HAS_TEMP_1
    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      redundant_temperature_raw = raw_temp_value[1];
    #else
      current_temperature_raw[1] = raw_temp_value[1];
    #endif
    #if HAS_TEMP_2
      current_temperature_raw[2] = raw_temp_value[2];
      #if HAS_TEMP_3
        current_temperature_raw[3] = raw_temp_value[3];
      #endif
    #endif
  #endif
  current_temperature_bed_raw = raw_temp_bed_value;
  temp_meas_ready = true;
}

/**
 * Timer 0 is shared with millies
 *  - Manage PWM to all the heaters and fan
 *  - Update the raw temperature values
 *  - Check new temperature values for MIN/MAX errors
 *  - Step the babysteps value for each axis towards 0
 */

extern "C"
{
void DMA1_Channel1_IRQHandler(void)
{
	uint8_t i;
	static unsigned char MaxTempCount = 0;
	static unsigned char MinTempCount = 0;
	if(DMA1->ISR & DMA_ISR_TCIF1)
	{
		TIM3->CR1 &= ~TIM_CR1_CEN;
		TIM3->CNT = 0;
		DMA1->IFCR |= DMA_IFCR_CTCIF1;

		// Update the raw values if they've been read. Else we could be updating them during reading.
		for (i = 0; i < 4; i++) 
			raw_temp_value[i] = 0;
		raw_temp_bed_value = 0;
		
		//if (temp_meas_ready == false) 
		{
			for(i=0;i<32;i=i+2)
			{
				raw_temp_value[0] += tmpADC[i];
				raw_temp_bed_value += tmpADC[i+1];
			}
			set_current_temp_raw();
		}
		
		for (i = 0; i < 4; i++) 
			raw_temp_value[i] = 0;
		raw_temp_bed_value = 0;

		#if(1)
		//Add by Paladin:Check Max Tamp and Min Tamp for Printer
		if(MA_PRINT == MachineSelected)
		{

		  #if HAS_TEMP_0 && DISABLED(HEATER_0_USES_MAX6675)
		    #if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
		      #define GE0 <=
		    #else
		      #define GE0 >=
		    #endif
		    //if (current_temperature_raw[0] GE0 maxttemp_raw[0]) max_temp_error(0);
		    //if (minttemp_raw[0] GE0 current_temperature_raw[0]) min_temp_error(0);

			if (current_temperature_raw[0] GE0 maxttemp_raw[0]) 
			{
				if(MaxTempCount < 10)
					MaxTempCount++;
		    }
			else
			{
				MaxTempCount = 0;
			}
			
			if(MaxTempCount > 2)
			{
				if( isLaserMachine(MachineSelected) )
				{
					//max_temp_error(0);
					MaxTampErrorTrig = true;
				}
			}
			
		    if (minttemp_raw[0] GE0 current_temperature_raw[0]) 
		    {
		    	if(MinTempCount < 10)
					MinTempCount++;
		    }
			else
			{
				MinTempCount = 0;
			}
			if(MinTempCount > 2)
			{
				MinTampErrorTrig = true;
				//min_temp_error(0);
			}
		  #endif

		  #if HAS_TEMP_1 && EXTRUDERS > 1
		    #if HEATER_1_RAW_LO_TEMP > HEATER_1_RAW_HI_TEMP
		      #define GE1 <=
		    #else
		      #define GE1 >=
		    #endif
		    if (current_temperature_raw[1] GE1 maxttemp_raw[1]) max_temp_error(1);
		    if (minttemp_raw[1] GE1 current_temperature_raw[1]) min_temp_error(1);
		  #endif // TEMP_SENSOR_1

		  #if HAS_TEMP_2 && EXTRUDERS > 2
		    #if HEATER_2_RAW_LO_TEMP > HEATER_2_RAW_HI_TEMP
		      #define GE2 <=
		    #else
		      #define GE2 >=
		    #endif
		    if (current_temperature_raw[2] GE2 maxttemp_raw[2]) max_temp_error(2);
		    if (minttemp_raw[2] GE2 current_temperature_raw[2]) min_temp_error(2);
		  #endif // TEMP_SENSOR_2

		  #if HAS_TEMP_3 && EXTRUDERS > 3
		    #if HEATER_3_RAW_LO_TEMP > HEATER_3_RAW_HI_TEMP
		      #define GE3 <=
		    #else
		      #define GE3 >=
		    #endif
		    if (current_temperature_raw[3] GE3 maxttemp_raw[3]) max_temp_error(3);
		    if (minttemp_raw[3] GE3 current_temperature_raw[3]) min_temp_error(3);
		  #endif // TEMP_SENSOR_3

		  #if HAS_TEMP_BED
		    #if HEATER_BED_RAW_LO_TEMP > HEATER_BED_RAW_HI_TEMP
		      #define GEBED <=
		    #else
		      #define GEBED >=
		    #endif
		    if (current_temperature_bed_raw GEBED bed_maxttemp_raw) _temp_error(-1, PSTR(MSG_T_MAXTEMP), PSTR(MSG_ERR_MAXTEMP_BED));
			//Paladin:热床没插上时，显示NA
		    if (bed_minttemp_raw GEBED current_temperature_bed_raw){HMI_ShowTemperature(1, -27, -27); _temp_error(-1, PSTR(MSG_T_MINTEMP), PSTR(MSG_ERR_MINTEMP_BED));}
		  #endif
		}
		else if((MA_CNC == MachineSelected) || (isLaserMachine(MachineSelected)) )
		{
			#if HAS_TEMP_0 && DISABLED(HEATER_0_USES_MAX6675)
		    #if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
		      #define GE0 <=
		    #else
		      #define GE0 >=
		    #endif

			//Paladin:多次检测
	        if (current_temperature_raw[0] GE0 maxttemp_raw[0]) 
			{
				if(MaxTempCount < 10)
					MaxTempCount++;
	        }
			else
			{
				MaxTempCount = 0;
			}
			
			if(MaxTempCount > 2)
			{
				if( isLaserMachine(MachineSelected) )
				{
					disable_all_steppers();
					//max_temp_error(0);
					MaxTampErrorTrig = true;
				}
			}
			
	        if (minttemp_raw[0] GE0 current_temperature_raw[0]) 
	        {
	        	if(MinTempCount < 10)
					MinTempCount++;
	        }
			else
			{
				MinTempCount = 0;
			}
			if(MinTempCount > 2)
			{
			    #ifndef NO_KILL
				disable_all_steppers();
                #endif
				MinTampErrorTrig = true;
				//min_temp_error(0);
			}
		    #endif
		}
		#endif
		//启动检测
		TIM3->CR1 |= TIM_CR1_CEN;
	}
}


}

#if ENABLED(PIDTEMP)
  // Apply the scale factors to the PID values
  float scalePID_i(float i)   { return i * PID_dT; }
  float unscalePID_i(float i) { return i / PID_dT; }
  float scalePID_d(float d)   { return d / PID_dT; }
  float unscalePID_d(float d) { return d * PID_dT; }
#endif //PIDTEMP
