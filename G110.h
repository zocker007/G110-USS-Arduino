/**
  * copyright (c) 2020, Merlin Krümmel
  * SPDX-License-Identifier: LGPL-3.0-or-later
  */

/**
  * This library is free software: you can redistribute it and/or modify
  * it under the terms of the GNU Lesser General Public License as published by
  * the Free Software Foundation, version 3 or later.
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  * Lesser General Public License for more details.  
  *
  * You should have received a copy of the GNU Lesser General Public License
  * along with this program. If not, see <http://www.gnu.org/licenses/>.
  */

/**
  *   @file   G110.h
  *
  *   @brief  class definition for SINAMICS G100 drive inverter
  *
  *   @author Merlin Krümmel
  *
  *   @date   22.07.2020
  */

#ifndef G110_H
#define G110_H

#include "USS.h"

#define CTL_WORD_REVERSE_FALG               0x0800

#define STATUS_WORD_CURRENT_LIMIT_FLAG      0x0800
#define STATUS_WORD_MOTOR_OVERLOAD_FLAG     0x2000
#define STATUS_WORD_MOTOR_RUNS_RIGHT_FLAG   0x4000
#define STATUS_WORD_INVERTER_OVERLOAD_FLAG  0x8000

#define USER_ACCESS_LEVEL_STD               (uint16_t)1
#define USER_ACCESS_LEVEL_EXT               (uint16_t)2
#define USER_ACCESS_LEVEL_EXPERT            (uint16_t)3
#define USER_ACCESS_LEVEL_RESERVED          (uint16_t)4

#define QUICK_COMMISSIONING_READY           (uint16_t)0
#define QUICK_COMMISSIONING_QUICK_COMM      (uint16_t)1
#define QUICK_COMMISSIONING_INVERTER        (uint16_t)2
#define QUICK_COMMISSIONING_DOWNLOAD        (uint16_t)29
#define QUICK_COMMISSIONING_FACTORY_SETTING (uint16_t)30

#define POWER_SETTING_EUROPE                (uint16_t)0
#define POWER_SETTING_NORTH_AMERICA_HP      (uint16_t)1
#define POWER_SETTING_NORTH_AMERICA_KW      (uint16_t)2

#define MOTOR_COOLING_SELF_COOLED           (uint16_t)0
#define MOTOR_COOLING_FORCE_COOLED          (uint16_t)1

#define COMMAND_SOURCE_DEFAULT              (uint16_t)0
#define COMMAND_SOURCE_BOP                  (uint16_t)1
#define COMMAND_SOURCE_TERMINAL             (uint16_t)2
#define COMMAND_SOURCE_USS                  (uint16_t)5

#define FREQ_SETPOINT_NONE                  (uint16_t)0
#define FREQ_SETPOINT_MOP                   (uint16_t)1
#define FREQ_SETPOINT_ANALOG                (uint16_t)2
#define FREQ_SETPOINT_FIXED                 (uint16_t)3
#define FREQ_SETPOINT_USS                   (uint16_t)5

#define FACTORY_RESET_PARAMETER_RESET       (uint16_t)1

#define CTL_MODE_V_F_LINEAR                 (uint16_t)0
#define CTL_MODE_V_F_QUADRATIC              (uint16_t)2
#define CTL_MODE_V_F_PROGRAMMABLE           (uint16_t)3

#define END_QUICK_COMM_NONE                 (uint16_t)0
#define END_QUICK_COMM_RESET                (uint16_t)1
#define END_QUICK_COMM_NORMAL               (uint16_t)2
#define END_QUICK_COMM_ONLY_MOTOR_DATA      (uint16_t)3

#define USS_PKW_LENGTH_NONE                 (uint16_t)0
#define USS_PKW_LENGTH_3_WORDS              (uint16_t)3
#define USS_PKW_LENGTH_4_WORDS              (uint16_t)4
#define USS_PKW_LENGTH_VARIABLE             (uint16_t)127

#define USS_BAUDRATE_1200_BAUD              (uint16_t)3
#define USS_BAUDRATE_2400_BAUD              (uint16_t)4
#define USS_BAUDRATE_4800_BAUD              (uint16_t)5
#define USS_BAUDRATE_9600_BAUD              (uint16_t)6
#define USS_BAUDRATE_19200_BAUD             (uint16_t)7
#define USS_BAUDRATE_38400_BAUD             (uint16_t)8
#define USS_BAUDRATE_57600_BAUD             (uint16_t)9

#define CALC_MOTOR_PARAMS_NONE              (uint16_t)0
#define CALC_MOTOR_PARAMS_COMPLETE          (uint16_t)1

#define FUN_DIGITAL_IN_DISABLED             (uint16_t)0
#define FUN_DIGITAL_IN_ON_OFF1              (uint16_t)1
#define FUN_DIGITAL_IN_ON_REV_OFF1          (uint16_t)2
#define FUN_DIGITAL_IN_OFF2                 (uint16_t)3
#define FUN_DIGITAL_IN_OFF3                 (uint16_t)4
#define FUN_DIGITAL_IN_FAULT_ACK            (uint16_t)9
#define FUN_DIGITAL_IN_JOG_RIGHT            (uint16_t)10
#define FUN_DIGITAL_IN_JOG_LEFT             (uint16_t)11
#define FUN_DIGITAL_IN_REVERSE              (uint16_t)12
#define FUN_DIGITAL_IN_MOP_UP               (uint16_t)13
#define FUN_DIGITAL_IN_MOP_DOWN             (uint16_t)14
#define FUN_DIGITAL_IN_FIXED_FREQ           (uint16_t)15
#define FUN_DIGITAL_IN_FIXED_FREQ_ON        (uint16_t)16
#define FUN_DIGITAL_IN_LOCA_REMOTE          (uint16_t)21
#define FUN_DIGITAL_IN_DC_BRAKE             (uint16_t)25
#define FUN_DIGITAL_IN_EXT_TRIP             (uint16_t)29

#define PARAM_NR_END_QUICK_COMM             3900
#define PARAM_NR_USS_PKW_LENGTH             2013
#define PARAM_NR_USS_ADDRESS                2011
#define PARAM_NR_USS_BAUDRATE               2010
#define PARAM_NR_PULSE_FREQ_KHZ             1800
#define PARAM_NR_CTL_MODE                   1300
#define PARAM_NR_OFF3_RAMP_DOWN_TIME_S      1135
#define PARAM_NR_ROUNDING_TIME_S            1130
#define PARAM_NR_RAMP_DOWN_TIME_S           1121
#define PARAM_NR_RAMP_UP_TIME_S             1120
#define PARAM_NR_MAX_FREQ_HZ                1082
#define PARAM_NR_MIN_FREQ_HZ                1080
#define PARAM_NR_SEL_FREQ_SETPOINT          1000
#define PARAM_NR_FACTORY_RESET              970
#define PARAM_NR_FUN_DIGITAL_IN_3           704
#define PARAM_NR_FUN_DIGITAL_IN_2           703
#define PARAM_NR_FUN_DIGITAL_IN_1           702
#define PARAM_NR_FUN_DIGITAL_IN_0           701
#define PARAM_NR_SEL_CMD_SOURCE             700
#define PARAM_NR_MOTOR_OVERLOAD_FACTOR      640
#define PARAM_NR_CALC_MOTOR_PARAMS          340           
#define PARAM_NR_MOTOR_COOLING              335
#define PARAM_NR_MOTOR_SPEED_PER_MINUTE     311
#define PARAM_NR_MOTOR_FREQ_HZ              310
#define PARAM_NR_MOTOR_EFFICIENCY_FACTOR    309
#define PARAM_NR_MOTOR_COS_PHI              308
#define PARAM_NR_MOTOR_POWER_KW_HP          307
#define PARAM_NR_MOTOR_CURRENT_A            305
#define PARAM_NR_MOTOR_VOLTAGE_V            304
#define PARAM_NR_POWER_SETING               100
#define PARAM_NR_COMMISSIONING_PARAM        10
#define PARAM_NR_USER_ACCESS_LEVEL          3

#define FREQUENCY_CALC_BASE                 0x4000

typedef struct
{
    uint16_t powerSetting;
    uint16_t motorVoltage;
    float motorCurrent;
    float motorPower;
    float motorCosPhi;
    float motorEff;
    float motorFreq;
    uint16_t motorSpeed;
    uint16_t motorCooling;
    float motorOverload;
    uint16_t cmdSource;
    uint16_t setpointSource;
    float minFreq;
    float maxFreq;
    float rampupTime;
    float rampdownTime;
    float OFF3rampdownTime;
    uint16_t ctlMode;
    uint16_t endQuickComm;
} quickCommissioning_t;

class G110
{
    public:

    G110();
    int start(USS *pinterface, quickCommissioning_t quickCommData, byte pindex);
    void setFrequency(float freq);
    void setON();
    void setOFF1();
    void setCtlFlag(uint16_t flag);
    void clearCtlFlag(uint16_t flag);
    bool running();
    bool getOFF2();
    bool getOFF3();
    bool setpointReached();
    bool reverse();
    bool checkStatusFlag(uint16_t flag);
    float getFrequency();
    void reset();
    int setParameter(uint16_t param, uint16_t value);
    int setParameter(uint16_t param, uint32_t value);
    int setParameter(uint16_t param, float value);
    
    private:

    USS *interface;
    float refFreq;
    byte index;
};

#endif