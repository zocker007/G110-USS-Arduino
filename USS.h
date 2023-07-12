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
 *   @file   USS.h
 *   @brief  class definition for Siemens USS protocol, implements the low level
 *           functionality of communication protocol as baselayer for higher layers
 *           like inverters.
 *   @author Merlin Kr�mmel
 *   @date   22.07.2020
 */

#ifndef USS_H
#define USS_H

#include "Arduino.h"

/**
 * @brief STX start byte
 */
constexpr const uint8_t STX_BYTE_STX{0x02};

/**
 * @brief Define for empty parameter value to check if parameter was written on the bus
 */
constexpr const uint16_t PARAM_VALUE_EMPTY{0xA000};

/**
 * @brief Flags in USS address byte (address has only 5 bits)
 */
enum USSAddressFlags : uint8_t
{
    ADDR_BYTE_BROADCAST_FLAG  = 0x20,
    ADDR_BYTE_MIRROR_FLAG     = 0x40,
    ADDR_BYTE_SPECIAL_FLAG    = 0x80,
};

/**
 * @brief Bitmasks for PKE field and address byte
 */
constexpr const uint8_t ADDR_BYTE_ADDR_MASK{0x1F};

enum PKEBitmasks : uint16_t
{
    PKE_WORD_PARAM_MASK      = 0x7FF,
    PKE_WORD_SP_FLAG         = 0x800,
    PKE_WORD_AK_MASK         = 0xF000,
    PKE_WORD_AK_NO_TASK      = 0x0000,
    PKE_WORD_AK_REQ_PWE      = 0x1000,
    PKE_WORD_AK_CHW_PWE      = 0x2000,
    PKE_WORD_AK_CHD_PWE      = 0x3000,
};

/**
 * @brief Response code for PKE operations
 */
enum PKEResponseCodes : uint16_t
{
    PKE_WORD_AK_NO_RESP      = 0x0000,
    PKE_WORD_AK_TRW_PWE      = 0x1000,
    PKE_WORD_AK_TRD_PWE      = 0x2000,
    PKE_WORD_AK_NO_RIGHTS    = 0x8000,
    PKE_WORD_AK_CANT_EXECUTE = 0x7000,
};

/**
 * @brief Control word flags and descriptions
 */
enum CtlWordFlags : uint16_t
{
    CTL_WORD_ON_OFF1_FLAG           = 0x0001,
    CTL_WORD_ON_OFF1_OFF1           = 0x0000,
    CTL_WORD_ON_OFF1_ON             = 0x0001,
    CTL_WORD_OFF2_FLAG              = 0x0002,
    CTL_WORD_OFF2_OFF2              = 0x0000,
    CTL_WORD_OFF2_OP_COND           = 0x0002,
    CTL_WORD_OFF3_FLAG              = 0x0004,
    CTL_WORD_OFF3_OFF3              = 0x0000,
    CTL_WORD_OFF3_OP_COND           = 0x0004,
    CTL_WORD_ENABLE_FLAG            = 0x0008,
    CTL_WORD_ENABLE_INHIBIT         = 0x0000,
    CTL_WORD_ENABLE_ENABLE          = 0x0008,
    CTL_WORD_INHIBIT_RAMP_FLAG      = 0x0010,
    CTL_WORD_INHIBIT_RAMP_INHIBIT   = 0x0000,
    CTL_WORD_INHIBIT_RAMP_OP_COND   = 0x0010,
    CTL_WORD_ENABLE_RAMP_FLAG       = 0x0020,
    CTL_WORD_ENABLE_RAMP_HOLD       = 0x0000,
    CTL_WORD_ENABLE_RAMP_ENABLE     = 0x0020,
    CTL_WORD_ENABLE_SETPOINT_FLAG   = 0x0040,
    CTL_WORD_ENABLE_SETPOINT_INHIBIT= 0x0000,
    CTL_WORD_ENABLE_SETPOINT_ENABLE = 0x0040,
    CTL_WORD_ACK_FLAG           	= 0x0080,
    CTL_WORD_CTL_PLC_FLAG       	= 0x0400,
    CTL_WORD_CTL_PLC_NO_CTL     	= 0x0000,
    CTL_WORD_CTL_PLC_CTL_PLC    	= 0x0400,
};

/**
 * @brief Status word flags and descriptions
 */
enum StatusWordFlags : uint16_t
{
    STATUS_WORD_SWITCH_READY_FLAG         = 0x0001,
    STATUS_WORD_SWITCH_READY              = 0x0001,
    STATUS_WORD_SWITCH_NOT_READY          = 0x0000,
    STATUS_WORD_READY_FLAG                = 0x0002,
    STATUS_WORD_READY                     = 0x0002,
    STATUS_WORD_NOT_READY                 = 0x0000,
    STATUS_WORD_OP_ENABLED_FLAG           = 0x0004,
    STATUS_WORD_OP_ENABLED_ENABLED        = 0x0004,
    STATUS_WORD_OP_ENABLED_INHIBIT        = 0x0000,
    STATUS_WORD_FAULT_FLAG                = 0x0008,
    STATUS_WORD_FAULT_FAULT               = 0x0008,
    STATUS_WORD_FAULT_FAUlT_FREE          = 0x0000,
    STATUS_WORD_OFF2_FLAG                 = 0x0010,
    STATUS_WORD_OFF2_NO_OFF2              = 0x0010,
    STATUS_WORD_OFF2_OFF2                 = 0x0000,
    STATUS_WORD_OFF3_FLAG                 = 0x0020,
    STATUS_WORD_OFF3_NO_OFF3              = 0x0020,
    STATUS_WORD_OFF3_OFF3                 = 0x0000,
    STATUS_WORD_SWITCH_INHIBIT_FLAG       = 0x0040,
    STATUS_WORD_SWITCH_INHIBIT_INHIBIT    = 0x0040,
    STATUS_WORD_SWITCH_INHIBIT_NO_INHIBIT = 0x0000,
    STATUS_WORD_ALARM_FLAG                = 0x0080,
    STATUS_WORD_ALARM_ALARM               = 0x0080,
    STATUS_WORD_ALARM_NO_ALARM            = 0x0000,
    STATUS_WORD_SETPOINT_TOL_FLAG         = 0x0100,
    STATUS_WORD_SETPOINT_TOL_IN_RANGE     = 0x0100,
    STATUS_WORD_SETPOINT_TOL_NOT_IN_RANGE = 0x0000,
    STATUS_WORD_CTL_REQ_FLAG              = 0x0200,
    STATUS_WORD_CTL_REQ_CTL_REQ           = 0x0200,
    STATUS_WORD_CTL_REQ_LOCAL_OP          = 0x0000,
    STATUS_WORD_F_N_REACHED_FLAG          = 0x0400,
    STATUS_WORD_F_N_REACHED_REACHED       = 0x0400,
    STATUS_WORD_F_N_REACHED_FALLEN_BELOW  = 0x0000,
};

/**
 * @brief Max number of USS Slaves
 */
constexpr const int USS_SLAVES{2};

/**
 * @brief number of PZD and PKW fields and length in bytes
 */
constexpr const int PZD_ANZ{1};
constexpr const int PZD_LENGTH_CHARACTERS{4};
constexpr const int PKW_ANZ{1};
constexpr const int PKW_LENGTH_CHARACTERS{8};

/**
 * @brief USS parameters
 */
constexpr const int CHARACTER_RUNTIME_BASE_US{1150};
constexpr const int BAUDRATE_BASE{9600};
constexpr const int MAX_RESP_DELAY_TIME_MS{20};
constexpr const int MASTER_COMPUTE_DELAY_MS{20};
constexpr const int START_DELAY_LENGTH_CHARACTERS{2};
constexpr const int TELEGRAM_OVERHEAD_CHARACTERS{4};

constexpr const int USS_BUFFER_LENGTH{(TELEGRAM_OVERHEAD_CHARACTERS + (PKW_LENGTH_CHARACTERS * PKW_ANZ) + (PZD_LENGTH_CHARACTERS * PZD_ANZ))};

class USS
{
    public:

    /**
     * @brief Constructor for USS class, sets first byte in send buffer to STX start byte
     *        and the fixed length in second byte
     * 
     * @return none
     */
    USS();

    /**
     * @brief Function to configure the USS instance, called in setup of arduino sketch
     * 
     * @param speed Baudrate of serial peripheral used for USS communication
     * @param slaves array with USS slave addresses that are on the bus
     * @param nrSlaves Number fo USS slaves on the bus
     * @retval 0: success
     * @retval -1: failure
     */
    int begin(const long speed, const byte slaves[], const int nrSlaves);

    /**
     * @brief Function to configure the USS instance, called in setup of arduino sketch
     * 
     * @param speed Baudrate of serial peripheral used for USS communication
     * @param slaves array with USS slave addresses that are on the bus
     * @param nrSlaves Number fo USS slaves on the bus
     * @param dePin Driver enable pin for RS485 level converters that need it (like MAX485)
     * @retval 0: success
     * @retval -1: failure
     */
    int begin(const long speed, const byte slaves[], const int nrSlaves, const int dePin);

    /**
     * @brief Set parameter as word value (2 byte) to a given USS slave
     * 
     * @param param Parameter number to set, these are specific to the concrete device (like inverter model)
     *              on the USS bus and therefore defined in a higher layer
     * @param value Parameter value to set as word (2 byte), the meanings are specific to the concrete device
     *              on the USS bus and therefore defined in a higher layer
     * @param slaveIndex Index of the slave the parameter should be set, index number acording to pslaves array
     *                   from begin()
     * @return USS error code
     * @retval 0: success
     * @retval -1: no response
     * @retval -2: access denied
     * @retval -3: illegal parameter number
     */
    int setParameter(const uint16_t param, const uint16_t value, const int slaveIndex);

    /**
     * @brief Set parameter as double word value (4 byte) to a given USS slave
     * 
     * @param param Parameter number to set, these are specific to the concrete device (like inverter model)
     *              on the USS bus and therefore defined in a higher layer
     * @param value Parameter value to set as double word (4 byte), the meanings are specific to the concrete device
     *              on the USS bus and therefore defined in a higher layer
     * @param slaveIndex Index of the slave the parameter should be set, index number acording to pslaves array
     *                   from begin()
     * @return USS error code
     * @retval 0: success
     * @retval -1: no response
     * @retval -2: access denied
     * @retval -3: illegal parameter number
     */
    int setParameter(const uint16_t param, const uint32_t value, const int slaveIndex);

    /**
     * @brief Set parameter as float (single precision) to a given USS slave
     * 
     * @param param Parameter number to set, these are specific to the concrete device (like inverter model)
     *              on the USS bus and therefore defined in a higher layer
     * @param value Parameter value to set as float (single precision), the meanings are specific to the concrete device
     *              on the USS bus and therefore defined in a higher layer
     * @param slaveIndex Index of the slave the parameter should be set, index number acording to pslaves array
     *                   from begin()
     * @return USS error code
     * @retval 0: success
     * @retval -1: no response
     * @retval -2: access denied
     * @retval -3: illegal parameter number
     */
    int setParameter(const uint16_t param, const float value, const int slaveIndex);

    /**
     * @brief Set main setpoint of PZD field
     * 
     * @param value Main setpoint as word (2 byte)
     * @param slaveIndex Index of the slave the setpoint should be set, index number acording to pslaves array
     *                   from begin()
     * @return none
     */
    void setMainsetpoint(const uint16_t value, const int slaveIndex);

    /**
     * @brief Set one or more flags in control word of PZD field
     * 
     * @param flags Flags to set in the control word, these are specific for the concrete device (like inverter model)
     *              on the USS bus and therefore defined in a higher layer
     * @param slaveIndex Index of the slave the flags should be set, index number acording to pslaves array
     *                   from begin()
     * @return none
     */
    void setCtlFlag(const uint16_t flags, const int slaveIndex);

    /**
     * @brief Clear one or more flags in control word of PZD field
     * 
     * @param flags Flags to clear from the control word, these are specific for the concrete device (like inverter model)
     *              on the USS bus and therefore defined in a higher layer
     * @param slaveIndex Index of the slave the flags should be cleared, index number acording to pslaves array
     *                   from begin()
     * @return none
     */
    void clearCtlFlag(const uint16_t flags, const int slaveIndex);

    /**
     * @brief Get main actual value from specified USS slave
     * 
     * @param slaveIndex Index of the slave the actual value should be get from, index number acording to pslaves array
     *                   from begin()
     * @return Main actual value as word (2 byte)
     */
    uint16_t getActualvalue(const int slaveIndex) const;

    /**
     * @brief Check flag in status word from specified USS slave
     * 
     * @param flag Flag to check, these are specific for the concrete device (like inverter model)
     *             on the USS bus and therefore defined in a higher layer
     * @param slaveIndex Index of the slave the flag should be checked, index number acording to pslaves array
     *                   from begin()
     * @return Boolean is the flag set?
     */
    bool checkStatusFlag(const uint16_t flag, const int slaveIndex) const;

    /**
     * @brief Fill the send buffer and send over serial.
     * 
     * @return true if telegram was send, else false
     * 
     * Fill the send buffer with control word, main setpoint, address of actual slave and parameter number and value,
     * when configured for this slave via setParameter() functions, and send over serial. Generates BCC (Block Check
     * Character), blocks until the cycle time is over from the last send. Must be called in loop() from application
     */
    bool send();

    /**
     * @brief Receives the response from USS salves over serial.
     * 
     * @return USS error code
     * @retval 0: success
     * @retval -1: no response
     * @retval -2: access denied
     * @retval -3: illegal parameter number
     * 
     * Receives the response from USS salves checks the length and address and the BCC (Block Check character).
     * When all is correct, updates the status word and main actualvalue of the slave from which is the response.
     * When parameter was requested to set, error code from USS spec. is returned.
     */
    int receive();

    private:
    
    /**
     * @brief Calculates the Block Check Character (BCC) like in USS spec.
     * 
     * @param buffer data over which the BCC is calculated
     * @param length length in bytes over which the BCC is calculated
     * @return the BCC value
     */
    byte BCC(const byte buffer[], const int length) const;

    byte m_slaves[USS_SLAVES];
    int m_nrSlaves;
    int m_actualSlave;
    byte m_sendBuffer[USS_BUFFER_LENGTH];
    byte m_recvBuffer[USS_BUFFER_LENGTH];
    uint16_t m_mainsetpoint[USS_SLAVES];
    uint16_t m_mainactualvalue[USS_SLAVES];
    uint16_t m_ctlword[USS_SLAVES];
    uint16_t m_statusword[USS_SLAVES];
    uint16_t m_paramValue[PKW_LENGTH_CHARACTERS / 2][USS_SLAVES];
    unsigned long m_nextSend;             // timestamp of next send in ms, compare to millis()
    unsigned long m_period;               // cycle time between sending frames in ms
    int m_characterRuntime;
    int m_dePin;
};

#endif