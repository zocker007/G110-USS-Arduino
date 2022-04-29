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
 *   @file   G110.cpp
 *   @brief  class implementation for SINAMICS G110 drive inverter
 *   @author Merlin Kr�mmel
 *   @date   22.07.2020
 */

#include "G110.h"

G110::G110() :
    m_interface(nullptr),
    m_refFreq(0.0),
    m_index(0)
{
}

int G110::begin(USS *interface, const G110::quickCommissioning_t &quickCommData, const int index)
{
    if(interface == nullptr)
        return -1;

    m_interface = interface;
    m_refFreq = quickCommData.motorFreq;
    m_index = index;
    int err = 0;

    err += setParameter(PARAM_NR_USER_ACCESS_LEVEL, USER_ACCESS_LEVEL_EXPERT);
    err += setParameter(PARAM_NR_USS_PKW_LENGTH, USS_PKW_LENGTH_4_WORDS);
    err += setParameter(PARAM_NR_COMMISSIONING_PARAM, QUICK_COMMISSIONING_QUICK_COMM);
    err += setParameter(PARAM_NR_POWER_SETING, quickCommData.powerSetting);
    err += setParameter(PARAM_NR_MOTOR_VOLTAGE_V, quickCommData.motorVoltage);
    err += setParameter(PARAM_NR_MOTOR_CURRENT_A, quickCommData.motorCurrent);
    err += setParameter(PARAM_NR_MOTOR_POWER_KW_HP, quickCommData.motorPower);

    if(quickCommData.powerSetting == POWER_SETTING_NORTH_AMERICA_HP)
        err += setParameter(PARAM_NR_MOTOR_EFFICIENCY_FACTOR, quickCommData.motorEff);
    else
        err += setParameter(PARAM_NR_MOTOR_COS_PHI, quickCommData.motorCosPhi);

    err += setParameter(PARAM_NR_MOTOR_FREQ_HZ, quickCommData.motorFreq);
    err += setParameter(PARAM_NR_MOTOR_SPEED_PER_MINUTE, quickCommData.motorSpeed);
    err += setParameter(PARAM_NR_MOTOR_COOLING, quickCommData.motorCooling);
    err += setParameter(PARAM_NR_MOTOR_OVERLOAD_FACTOR, quickCommData.motorOverload);
    err += setParameter(PARAM_NR_SEL_CMD_SOURCE, quickCommData.cmdSource);
    err += setParameter(PARAM_NR_SEL_FREQ_SETPOINT, quickCommData.setpointSource);
    err += setParameter(PARAM_NR_MIN_FREQ_HZ, quickCommData.minFreq);
    err += setParameter(PARAM_NR_MAX_FREQ_HZ, quickCommData.maxFreq);
    err += setParameter(PARAM_NR_RAMP_UP_TIME_S, quickCommData.rampupTime);
    err += setParameter(PARAM_NR_RAMP_DOWN_TIME_S, quickCommData.rampdownTime);
    err += setParameter(PARAM_NR_OFF3_RAMP_DOWN_TIME_S, quickCommData.OFF3rampdownTime);
    err += setParameter(PARAM_NR_CTL_MODE, quickCommData.ctlMode);
    err += setParameter(PARAM_NR_COMMISSIONING_PARAM, QUICK_COMMISSIONING_READY);
    err += setParameter(PARAM_NR_CALC_MOTOR_PARAMS, CALC_MOTOR_PARAMS_COMPLETE);

    if(!err)
        setCtlFlag(CTL_WORD_ENABLE_ENABLE | CTL_WORD_INHIBIT_RAMP_OP_COND |
                              CTL_WORD_ENABLE_RAMP_ENABLE | CTL_WORD_ENABLE_SETPOINT_ENABLE |
                              CTL_WORD_CTL_PLC_CTL_PLC);
                          
    return err;
}

void G110::setFrequency(float freq) const
{
    bool reverse = false;

    if(m_interface == nullptr)
        return;

    if(freq < 0)
    {
        reverse = true;
        freq *= -1.0f;
    }
    // f[Hz] = (f(hex) / FREQUENCY_CALC_BASE) * refFreq
    uint16_t f_hex = static_cast<uint16_t>((freq / m_refFreq) * static_cast<float>(FREQUENCY_CALC_BASE));

    if(reverse)
        setCtlFlag(CTL_WORD_REVERSE_FALG);
    else
        clearCtlFlag(CTL_WORD_REVERSE_FALG);

    m_interface->setMainsetpoint(f_hex, m_index);
    
}

void G110::setON() const
{
    setCtlFlag(CTL_WORD_ON_OFF1_ON | CTL_WORD_OFF2_OP_COND |
                          CTL_WORD_OFF3_OP_COND);
}

void G110::setOFF1() const
{
    clearCtlFlag(CTL_WORD_ON_OFF1_FLAG);
}

void G110::setCtlFlag(const uint16_t flags) const
{
    if(m_interface == nullptr)
        return;

    m_interface->setCtlFlag(flags, m_index);
}

void G110::clearCtlFlag(const uint16_t flags) const
{
    if(m_interface == nullptr)
        return;

    m_interface->clearCtlFlag(flags, m_index);
}

bool G110::running() const
{
    return checkStatusFlag(STATUS_WORD_OP_ENABLED_FLAG);
}

bool G110::getOFF2() const
{
    return !checkStatusFlag(STATUS_WORD_OFF2_FLAG);
}

bool G110::getOFF3() const
{
    return !checkStatusFlag(STATUS_WORD_OFF3_FLAG);
}

bool G110::setpointReached() const
{
    return checkStatusFlag(STATUS_WORD_SETPOINT_TOL_FLAG);
}

bool G110::reverse() const
{
    return !checkStatusFlag(STATUS_WORD_MOTOR_RUNS_RIGHT_FLAG);
}

bool G110::checkStatusFlag(const uint16_t flag) const
{
    if(m_interface == nullptr)
        return false;

    return m_interface->checkStatusFlag(flag, m_index);
}

float G110::getFrequency() const
{
    if(m_interface == nullptr)
        return -1.0;

    uint16_t f_hex = m_interface->getActualvalue(m_index);
    // f[Hz] = (f(hex) / FREQUENCY_CALC_BASE) * refFreq
    return (static_cast<float>(f_hex) / static_cast<float>(FREQUENCY_CALC_BASE)) * m_refFreq;
}

void G110::reset() const
{
    setParameter(PARAM_NR_COMMISSIONING_PARAM, QUICK_COMMISSIONING_FACTORY_SETTING);
    setParameter(PARAM_NR_FACTORY_RESET, FACTORY_RESET_PARAMETER_RESET);
    delay(10000);
}

int G110::setParameter(const uint16_t param, const uint16_t value) const
{
    if(m_interface == nullptr)
        return -1;

    return m_interface->setParameter(param, value, m_index);
}

int G110::setParameter(const uint16_t param, const uint32_t value) const
{
     if(m_interface == nullptr)
        return -1;

    return m_interface->setParameter(param, value, m_index);
}

int G110::setParameter(const uint16_t param, const float value) const
{
     if(m_interface == nullptr)
        return -1;

    return m_interface->setParameter(param, value, m_index);
}