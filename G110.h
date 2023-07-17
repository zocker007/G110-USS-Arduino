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
#include "USS.cpp"

/**
 * control word flags specific to the inverter
 */
enum CtlWordFlagsG110 : uint16_t
{
    CTL_WORD_REVERSE_FALG              = 0x0800,
};

/**
 * Status word flags specific to the inverter
 */
enum StatusWordFlagsG110 : uint16_t
{
    STATUS_WORD_CURRENT_LIMIT_FLAG      = 0x0800,
    STATUS_WORD_MOTOR_OVERLOAD_FLAG     = 0x2000,
    STATUS_WORD_MOTOR_RUNS_RIGHT_FLAG   = 0x4000,
    STATUS_WORD_INVERTER_OVERLOAD_FLAG  = 0x8000,
};

/**
 * Parameter values user access level for parameters, refer to G110 user manual
 */
enum UserLevel : uint16_t
{
    USER_ACCESS_LEVEL_STD        = 1,
    USER_ACCESS_LEVEL_EXT        = 2,
    USER_ACCESS_LEVEL_EXPERT     = 3,
    USER_ACCESS_LEVEL_RESERVED   = 4,
};

/**
 * Parameter values quick commisioning operating modes, refer to G110 user manual
 */
enum QuickCommModes : uint16_t
{
    QUICK_COMMISSIONING_READY           = 0,
    QUICK_COMMISSIONING_QUICK_COMM      = 1,
    QUICK_COMMISSIONING_INVERTER        = 2,
    QUICK_COMMISSIONING_DOWNLOAD        = 29,
    QUICK_COMMISSIONING_FACTORY_SETTING = 30,
};

/**
 * Parameter values power settings
 */
enum PowerSetting : uint16_t
{
    POWER_SETTING_EUROPE             = 0,
    POWER_SETTING_NORTH_AMERICA_HP   = 1,
    POWER_SETTING_NORTH_AMERICA_KW   = 2,
};

/**
 * Parameter values motor cooling
 */
enum MotorCooling : uint16_t
{
    MOTOR_COOLING_SELF_COOLED   = 0,
    MOTOR_COOLING_FORCE_COOLED  = 1,
};

/**
 * Parameter values command source
 */
enum CmdSource : uint16_t
{
    COMMAND_SOURCE_DEFAULT   = 0,
    COMMAND_SOURCE_BOP       = 1,
    COMMAND_SOURCE_TERMINAL  = 2,
    COMMAND_SOURCE_USS       = 5,
};

/**
 * Parameter values frequency setpoint
 */
enum FreqSetpoint : uint16_t
{
    FREQ_SETPOINT_NONE       = 0,
    FREQ_SETPOINT_MOP        = 1,
    FREQ_SETPOINT_ANALOG     = 2,
    FREQ_SETPOINT_FIXED      = 3,
    FREQ_SETPOINT_USS        = 5,
};

/**
 * Parameter value reset
 */
constexpr const uint16_t FACTORY_RESET_PARAMETER_RESET{1};

/**
 * Parameter values control mode
 */
enum CtlMode : uint16_t
{
    CTL_MODE_V_F_LINEAR           = 0,
    CTL_MODE_V_F_QUADRATIC        = 2,
    CTL_MODE_V_F_PROGRAMMABLE     = 3,
};

/**
 * Parameter values end quick commisioning mode
 */
enum EndQuickCommissioning : uint16_t
{
    END_QUICK_COMM_NONE             = 0,
    END_QUICK_COMM_RESET            = 1,
    END_QUICK_COMM_NORMAL           = 2,
    END_QUICK_COMM_ONLY_MOTOR_DATA  = 3,
};

/**
 * Parameter values PKW length
 */
enum PKWLength : uint16_t
{
    USS_PKW_LENGTH_NONE            = 0,
    USS_PKW_LENGTH_3_WORDS         = 3,
    USS_PKW_LENGTH_4_WORDS         = 4,
    USS_PKW_LENGTH_VARIABLE        = 127,
};

/**
 * Parameter values USS baudrate
 */
enum USSBaudrates : uint16_t
{
    USS_BAUDRATE_1200_BAUD         = 3,
    USS_BAUDRATE_2400_BAUD         = 4,
    USS_BAUDRATE_4800_BAUD         = 5,
    USS_BAUDRATE_9600_BAUD         = 6,
    USS_BAUDRATE_19200_BAUD        = 7,
    USS_BAUDRATE_38400_BAUD        = 8,
    USS_BAUDRATE_57600_BAUD        = 9,
};

/**
 * Parameter values calculate motor parameters
 */
enum CalcMotorParameters : uint16_t
{
    CALC_MOTOR_PARAMS_NONE           = 0,
    CALC_MOTOR_PARAMS_COMPLETE       = 1,
};

/**
 * Parameter values function of digital input pin
 */
enum DigitalInputFct : uint16_t
{
    FUN_DIGITAL_IN_DISABLED          = 0,
    FUN_DIGITAL_IN_ON_OFF1           = 1,
    FUN_DIGITAL_IN_ON_REV_OFF1       = 2,
    FUN_DIGITAL_IN_OFF2              = 3,
    FUN_DIGITAL_IN_OFF3              = 4,
    FUN_DIGITAL_IN_FAULT_ACK         = 9,
    FUN_DIGITAL_IN_JOG_RIGHT         = 10,
    FUN_DIGITAL_IN_JOG_LEFT          = 11,
    FUN_DIGITAL_IN_REVERSE           = 12,
    FUN_DIGITAL_IN_MOP_UP            = 13,
    FUN_DIGITAL_IN_MOP_DOWN          = 14,
    FUN_DIGITAL_IN_FIXED_FREQ        = 15,
    FUN_DIGITAL_IN_FIXED_FREQ_ON     = 16,
    FUN_DIGITAL_IN_LOCA_REMOTE       = 21,
    FUN_DIGITAL_IN_DC_BRAKE          = 25,
    FUN_DIGITAL_IN_EXT_TRIP          = 29,
};

/**
 * Parameter numbers
 */
enum ParameterNr : uint16_t
{
    PARAM_NR_END_QUICK_COMM           = 3900,
    PARAM_NR_USS_PKW_LENGTH           = 2013,
    PARAM_NR_USS_ADDRESS              = 2011,
    PARAM_NR_USS_BAUDRATE             = 2010,
    PARAM_NR_PULSE_FREQ_KHZ           = 1800,
    PARAM_NR_CTL_MODE                 = 1300,
    PARAM_NR_OFF3_RAMP_DOWN_TIME_S    = 1135,
    PARAM_NR_ROUNDING_TIME_S          = 1130,
    PARAM_NR_RAMP_DOWN_TIME_S         = 1121,
    PARAM_NR_RAMP_UP_TIME_S           = 1120,
    PARAM_NR_MAX_FREQ_HZ              = 1082,
    PARAM_NR_MIN_FREQ_HZ              = 1080,
    PARAM_NR_SEL_FREQ_SETPOINT        = 1000,
    PARAM_NR_FACTORY_RESET            = 970,
    PARAM_NR_FUN_DIGITAL_IN_3         = 704,
    PARAM_NR_FUN_DIGITAL_IN_2         = 703,
    PARAM_NR_FUN_DIGITAL_IN_1         = 702,
    PARAM_NR_FUN_DIGITAL_IN_0         = 701,
    PARAM_NR_SEL_CMD_SOURCE           = 700,
    PARAM_NR_MOTOR_OVERLOAD_FACTOR    = 640,
    PARAM_NR_CALC_MOTOR_PARAMS        = 340,
    PARAM_NR_MOTOR_COOLING            = 335,
    PARAM_NR_MOTOR_SPEED_PER_MINUTE   = 311,
    PARAM_NR_MOTOR_FREQ_HZ            = 310,
    PARAM_NR_MOTOR_EFFICIENCY_FACTOR  = 309,
    PARAM_NR_MOTOR_COS_PHI            = 308,
    PARAM_NR_MOTOR_POWER_KW_HP        = 307,
    PARAM_NR_MOTOR_CURRENT_A          = 305,
    PARAM_NR_MOTOR_VOLTAGE_V          = 304,
    PARAM_NR_POWER_SETING             = 100,
    PARAM_NR_COMMISSIONING_PARAM      = 10,
    PARAM_NR_USER_ACCESS_LEVEL        = 3,
};

/**
 * Number used in calculation of main setpoint from given frequency in Hz as floating point
 */
constexpr const uint16_t FREQUENCY_CALC_BASE{0x4000U};

class G110
{
    public:

    /**
     * @struct structure definition for G110 quick commissioning parameters
     */
    struct quickCommissioning_t
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
    };

    /**
     * @brief Constructor for G110 class, initializes the members
     * 
     * @return none
     */
    G110(USS<USS_SLAVES> &interface,  const int index);

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
    int begin(const quickCommissioning_t &quickCommData);

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

    USS<USS_SLAVES> &m_interface;
    float m_refFreq;
    int m_index;
};

#endif