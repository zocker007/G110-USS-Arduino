/**
 * Copyright (c) 2020, Merlin Kr�mmel
 * SPDX-License-Identifier: LGPL-3.0-or-later
 */

/**
 * @section LICENSE
 * 
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, version 3 or (at your option) any later version.
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
 *   @brief  class definition for SINAMICS G110 drive inverter, uses USS class as
 *           base communication layer.
 *   @author Merlin Kr�mmel
 *   @date   22.07.2020
 */

#ifndef G110_H
#define G110_H

#include "USS.h"

/**
 * control word flags specific to the inverter
 */
#define CTL_WORD_REVERSE_FALG               0x0800

/**
 * Status word flags specific to the inverter
 */
#define STATUS_WORD_CURRENT_LIMIT_FLAG      0x0800
#define STATUS_WORD_MOTOR_OVERLOAD_FLAG     0x2000
#define STATUS_WORD_MOTOR_RUNS_RIGHT_FLAG   0x4000
#define STATUS_WORD_INVERTER_OVERLOAD_FLAG  0x8000

/**
 * Parameter values user access level for parameters, refer to G110 user manual
 */
#define USER_ACCESS_LEVEL_STD               (uint16_t)1
#define USER_ACCESS_LEVEL_EXT               (uint16_t)2
#define USER_ACCESS_LEVEL_EXPERT            (uint16_t)3
#define USER_ACCESS_LEVEL_RESERVED          (uint16_t)4

/**
 * Parameter values quick commisioning operating modes, refer to G110 user manual
 */
#define QUICK_COMMISSIONING_READY           (uint16_t)0
#define QUICK_COMMISSIONING_QUICK_COMM      (uint16_t)1
#define QUICK_COMMISSIONING_INVERTER        (uint16_t)2
#define QUICK_COMMISSIONING_DOWNLOAD        (uint16_t)29
#define QUICK_COMMISSIONING_FACTORY_SETTING (uint16_t)30

/**
 * Parameter values power settings
 */
#define POWER_SETTING_EUROPE                (uint16_t)0
#define POWER_SETTING_NORTH_AMERICA_HP      (uint16_t)1
#define POWER_SETTING_NORTH_AMERICA_KW      (uint16_t)2

/**
 * Parameter values motor cooling
 */
#define MOTOR_COOLING_SELF_COOLED           (uint16_t)0
#define MOTOR_COOLING_FORCE_COOLED          (uint16_t)1

/**
 * Parameter values command source
 */
#define COMMAND_SOURCE_DEFAULT              (uint16_t)0
#define COMMAND_SOURCE_BOP                  (uint16_t)1
#define COMMAND_SOURCE_TERMINAL             (uint16_t)2
#define COMMAND_SOURCE_USS                  (uint16_t)5

/**
 * Parameter values frequency setpoint
 */
#define FREQ_SETPOINT_NONE                  (uint16_t)0
#define FREQ_SETPOINT_MOP                   (uint16_t)1
#define FREQ_SETPOINT_ANALOG                (uint16_t)2
#define FREQ_SETPOINT_FIXED                 (uint16_t)3
#define FREQ_SETPOINT_USS                   (uint16_t)5

/**
 * Parameter value reset
 */
#define FACTORY_RESET_PARAMETER_RESET       (uint16_t)1

/**
 * Parameter values control mode
 */
#define CTL_MODE_V_F_LINEAR                 (uint16_t)0
#define CTL_MODE_V_F_QUADRATIC              (uint16_t)2
#define CTL_MODE_V_F_PROGRAMMABLE           (uint16_t)3

/**
 * Parameter values end quick commisioning mode
 */
#define END_QUICK_COMM_NONE                 (uint16_t)0
#define END_QUICK_COMM_RESET                (uint16_t)1
#define END_QUICK_COMM_NORMAL               (uint16_t)2
#define END_QUICK_COMM_ONLY_MOTOR_DATA      (uint16_t)3

/**
 * Parameter values PKW length
 */
#define USS_PKW_LENGTH_NONE                 (uint16_t)0
#define USS_PKW_LENGTH_3_WORDS              (uint16_t)3
#define USS_PKW_LENGTH_4_WORDS              (uint16_t)4
#define USS_PKW_LENGTH_VARIABLE             (uint16_t)127

/**
 * Parameter values USS baudrate
 */
#define USS_BAUDRATE_1200_BAUD              (uint16_t)3
#define USS_BAUDRATE_2400_BAUD              (uint16_t)4
#define USS_BAUDRATE_4800_BAUD              (uint16_t)5
#define USS_BAUDRATE_9600_BAUD              (uint16_t)6
#define USS_BAUDRATE_19200_BAUD             (uint16_t)7
#define USS_BAUDRATE_38400_BAUD             (uint16_t)8
#define USS_BAUDRATE_57600_BAUD             (uint16_t)9

/**
 * Parameter values calculate motor parameters
 */
#define CALC_MOTOR_PARAMS_NONE              (uint16_t)0
#define CALC_MOTOR_PARAMS_COMPLETE          (uint16_t)1

/**
 * Parameter values function of digital input pin
 */
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

/**
 * Parameter numbers
 */
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

/**
 * Number used in calculation of main setpoint from given frequency in Hz as floating point
 */
#define FREQUENCY_CALC_BASE                 0x4000

/**
 * @struct structure definition for G110 quick commissioning parameters
 */
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

    /**
     * @brief Constructor for G110 class, initializes the members
     * 
     * @return none
     */
    G110();

    /**
     * @brief Function to configure the G110 instance, called in setup of arduino sketch
     * 
     * @param interface instance of USS interface used for communication over serial
     * @param quickCommData structure of parameter values for quick commissioning
     * @param index index in array of USS slave addresses used on creation of USS instance
     * @return 0 on success
     * 
     * Runs quick commissioning mode with commsioning values given and triggers calculation of
     * motor parameters. Sets reference frequency for calculation of main setpoint to given motor
     * frequency. Sets control word to operating conditions.
     */
    int begin(USS *interface, const quickCommissioning_t &quickCommData, const int index);

    /**
     * @brief Set frequency of inverter, calculates main setpoint and sets reverse flag appropriately
     * 
     * @param freq frequency in Hz as floating point number
     * @return none
     */
    void setFrequency(float freq) const;

    /**
     * @brief Set power stage in operating mode, sets ON/OFF1 flag, OFF2 and OFF3 appropriately
     * 
     * @return none
     */
    void setON() const;

    /**
     * @brief Disable power stage, clears ON/OFF1 flag appropriately
     * 
     * @return none
     */
    void setOFF1() const;

    /**
     * @brief Set one or more flags in control word
     * 
     * @param flags control flags to set, for flags refer to G110 user manual
     * @return none
     */
    void setCtlFlag(const uint16_t flags) const;

    /**
     * @brief clear one or more flags in control word
     * 
     * @param flags control flags to clear, for flags refer to G110 user manual
     * @return none
     */
    void clearCtlFlag(const uint16_t flags) const;

    /**
     * @brief Is the motor running? Checks the enabled flag in status word
     * 
     * @return boolean is the motor running?
     */
    bool running() const;

    /**
     * @brief Checks if OFF2 is set via flag in status word
     * 
     * @return boolean is OFF2 set?
     */
    bool getOFF2() const;

    /**
     * @brief Checks if OFF3 is set via flag in status word
     * 
     * @return boolean is OFF3 set?
     */
    bool getOFF3() const;

    /**
     * @brief Checks if setpoint from main actual value is reached via flag in status word
     * 
     * @return boolean is setpoint reached?
     */
    bool setpointReached() const;

    /**
     * @brief Checks if motor rotates in reverse via flag in status word
     * 
     * @return boolean rotates the motor in reverse?
     */
    bool reverse() const;

    /**
     * @brief Check a flag in status word
     * 
     * @param flag status flag to check, for flags refer to G110 user manual
     * @return boolean is the flag set?
     */
    bool checkStatusFlag(const uint16_t flag) const;

    /**
     * @brief Get actual frequency of motor from main actualvalue
     * 
     * @return actual frequency in Hz as floating point number, -1.0 on error
     */
    float getFrequency() const;

    /**
     * @brief Reset/restart the inverter over USS
     * 
     * @return none
     */
    void reset() const;

    /**
     * @brief Set parameter as word value (2 byte) on G110
     * 
     * @param param parameter number, for parameters refer to G110 user manual
     * @param value parameter value as word (2 byte), for parameters refer to G110 user manual
     * @return USS error code
     * @retval 0: success
     * @retval -1: no response
     * @retval -2: access denied
     * @retval -3: illegal parameter number
     */
    int setParameter(const uint16_t param, const uint16_t value) const;

    /**
     * @brief Set parameter as double word value (4 byte) on G110
     * 
     * @param param parameter number, for parameters refer to G110 user manual
     * @param value parameter value as double word (4 byte), for parameters refer to G110 user manual
     * @return USS error code
     * @retval 0: success
     * @retval -1: no response
     * @retval -2: access denied
     * @retval -3: illegal parameter number
     */
    int setParameter(const uint16_t param, const uint32_t value) const;

    /**
     * @brief Set parameter as float (single precision) on G110
     * 
     * @param param parameter number, for parameters refer to G110 user manual
     * @param value parameter value as float (single precision), for parameters refer to G110 user manual
     * @return USS error code
     * @retval 0: success
     * @retval -1: no response
     * @retval -2: access denied
     * @retval -3: illegal parameter number
     */
    int setParameter(const uint16_t param, const float value) const;
    
    private:

    USS *m_interface;
    float m_refFreq;
    int m_index;
};

#endif