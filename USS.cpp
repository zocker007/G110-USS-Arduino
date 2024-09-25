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
 *   @file   USS.cpp
 *   @brief  class definition for Siemens USS protocol, implements the low level
 *           functionality of communication protocol as baselayer for higher layers
 *           like inverters.
 *   @author Merlin Kr�mmel
 *   @date   22.07.2020
 */

#include "USS.h"

//extern USS uss;

template<size_t nrSlaves>
USS<nrSlaves>::USS() :
    m_slaves{0},
    m_actualSlave(0),
    m_sendBuffer{0},
    m_recvBuffer{0},
    m_mainsetpoint{0},
    m_mainactualvalue{0},
    m_ctlword{0},
    m_statusword{0},
    m_paramValue{{0}, {0}},
    m_nextSend(0),
    m_period(0),
    m_characterRuntime(0),
    m_dePin(-1)
{
    m_sendBuffer[0] = STX_BYTE_STX;
    m_sendBuffer[1] = (PKW_LENGTH_CHARACTERS * PKW_ANZ) + (PZD_LENGTH_CHARACTERS * PZD_ANZ) + 2; // 2 for ADR and BCC bytes
}

template<size_t nrSlaves>
int USS<nrSlaves>::begin(const long speed, const byte slaves[])
{
    int telegramRuntime;

    if(slaves == nullptr)
        return -1;

    memcpy(m_slaves, slaves, nrSlaves);

    m_characterRuntime = CHARACTER_RUNTIME_BASE_US / (speed / BAUDRATE_BASE);
    telegramRuntime = USS_BUFFER_LENGTH * m_characterRuntime * 1.5f / 1000;
    Serial1.begin(speed, SERIAL_8E1);
    Serial1.setTimeout(telegramRuntime + MAX_RESP_DELAY_TIME_MS);
    m_period = telegramRuntime * 2 + (START_DELAY_LENGTH_CHARACTERS * m_characterRuntime / 1000) + MAX_RESP_DELAY_TIME_MS + MASTER_COMPUTE_DELAY_MS;

    return 0;
}

template<size_t nrSlaves>
int USS<nrSlaves>::begin(const long speed, const byte slaves[], const int dePin)
{
    m_dePin = dePin;
    pinMode(m_dePin,OUTPUT);
    digitalWrite(m_dePin, HIGH);

    return begin(speed, slaves);
}

template<size_t nrSlaves>
int USS<nrSlaves>::setParameter(const uint16_t param, const uint16_t value, const int slaveIndex)
{
    int ret = 0;

    if(slaveIndex >= nrSlaves)
        return -1;

    m_paramValue[0][slaveIndex] = (param & PKE_WORD_PARAM_MASK) | PKE_WORD_AK_CHW_PWE;
    m_paramValue[1][slaveIndex] = 0;
    m_paramValue[2][slaveIndex] = 0;
    m_paramValue[3][slaveIndex] = value;

    while(m_paramValue[0][slaveIndex] != PARAM_VALUE_EMPTY)
    {
        if(send())
        {
            ret = receive();
        }
    }

    return ret;
}

template<size_t nrSlaves>
int USS<nrSlaves>::setParameter(const uint16_t param, const uint32_t value, const int slaveIndex)
{
    int ret = 0;

    if(slaveIndex >= nrSlaves)
        return -1;

    m_paramValue[0][slaveIndex] = (param & PKE_WORD_PARAM_MASK) | PKE_WORD_AK_CHD_PWE;
    m_paramValue[1][slaveIndex] = 0;
    // USS is Big-Endian
    m_paramValue[2][slaveIndex] = (value >> 16) & 0xFFFF;
    m_paramValue[3][slaveIndex] = value & 0xFFFF;

    while(m_paramValue[0][slaveIndex] != PARAM_VALUE_EMPTY)
    {
        if(send())
        {
            ret = receive();
        }
    }

    return ret;
}

template<size_t nrSlaves>
int USS<nrSlaves>::setParameter(const uint16_t param, const float value, const int slaveIndex)
{
    uint32_t u32 = 0;;

    memcpy(&u32, &value, sizeof(u32));

    return setParameter(param, u32, slaveIndex);
}

template<size_t nrSlaves>
void USS<nrSlaves>::setMainsetpoint(const uint16_t value, const int slaveIndex)
{
    if(slaveIndex >= nrSlaves)
        return;

    m_mainsetpoint[slaveIndex] = value;
}

template<size_t nrSlaves>
void USS<nrSlaves>::setCtlFlag(const uint16_t flags, const int slaveIndex)
{
    if(slaveIndex >= nrSlaves)
        return;

    m_ctlword[slaveIndex] |= flags;
}

template<size_t nrSlaves>
void USS<nrSlaves>::clearCtlFlag(const uint16_t flags, const int slaveIndex)
{
    if(slaveIndex >= nrSlaves)
        return;

    m_ctlword[slaveIndex] &= ~flags;
}

template<size_t nrSlaves>
uint16_t USS<nrSlaves>::getActualvalue(const int slaveIndex) const
{
    if(slaveIndex >= nrSlaves)
        return -1;

    uint16_t ret;

    ret = m_mainactualvalue[slaveIndex];

    return ret;
}

template<size_t nrSlaves>
bool USS<nrSlaves>::checkStatusFlag(const uint16_t flag, const int slaveIndex) const
{
    if(slaveIndex >= nrSlaves)
        return false;

    return (m_statusword[slaveIndex] & flag) != 0 ? true : false;
}

template<size_t nrSlaves>
byte USS<nrSlaves>::BCC(const byte buffer[], const int length) const
{
    byte ret = 0;

    for(int i = 0; i < length; i++)
        ret = ret ^ buffer[i];

    return ret;
}

template<size_t nrSlaves>
bool USS<nrSlaves>::send()
{
    //while(!(millis() > m_nextSend && (millis() - m_nextSend) < 10000));
    if((millis() - m_lastSend) < m_period)
    {
        return false;
    }

    m_lastSend = millis();

    if(m_actualSlave == nrSlaves)
        m_actualSlave = 0;

    m_sendBuffer[2] = m_slaves[m_actualSlave] & ADDR_BYTE_ADDR_MASK;

    if(m_paramValue[0][m_actualSlave] != PARAM_VALUE_EMPTY)
    {
        // USS is Big-Endian
        m_sendBuffer[3] = (m_paramValue[0][m_actualSlave] >> 8) & 0xFF;
        m_sendBuffer[4] = m_paramValue[0][m_actualSlave] & 0xFF;
        m_sendBuffer[5] = (m_paramValue[1][m_actualSlave] >> 8) & 0xFF;
        m_sendBuffer[6] = m_paramValue[1][m_actualSlave] & 0xFF;
        m_sendBuffer[7] = (m_paramValue[2][m_actualSlave] >> 8) & 0xFF;
        m_sendBuffer[8] = m_paramValue[2][m_actualSlave] & 0xFF;
        m_sendBuffer[9] = (m_paramValue[3][m_actualSlave] >> 8) & 0xFF;
        m_sendBuffer[10] = m_paramValue[3][m_actualSlave] & 0xFF;
    }
    else
    {
        memset(&m_sendBuffer[3], 0, 8);
    }    

    // USS is Big-Endian
    m_sendBuffer[PKW_LENGTH_CHARACTERS * PKW_ANZ + 3] = (m_ctlword[m_actualSlave] >> 8) & 0xFF;
    m_sendBuffer[PKW_LENGTH_CHARACTERS * PKW_ANZ + 4] = m_ctlword[m_actualSlave] & 0xFF;
    m_sendBuffer[PKW_LENGTH_CHARACTERS * PKW_ANZ + 5] = (m_mainsetpoint[m_actualSlave] >> 8) & 0xFF;
    m_sendBuffer[PKW_LENGTH_CHARACTERS * PKW_ANZ + 6] = m_mainsetpoint[m_actualSlave] & 0xFF;

    m_sendBuffer[USS_BUFFER_LENGTH - 1] = BCC(m_sendBuffer, USS_BUFFER_LENGTH - 1);

    Serial1.write(m_sendBuffer, USS_BUFFER_LENGTH);
    Serial1.flush();

    delayMicroseconds(START_DELAY_LENGTH_CHARACTERS * m_characterRuntime);

    if (m_dePin != -1)
        digitalWrite(m_dePin, LOW);

    return true;
}

template<size_t nrSlaves>
int USS<nrSlaves>::receive()
{
    int ret = 0;

    if(Serial1.readBytes(m_recvBuffer, USS_BUFFER_LENGTH) == USS_BUFFER_LENGTH &&
        m_recvBuffer[0] == STX_BYTE_STX && (m_recvBuffer[2] & ADDR_BYTE_ADDR_MASK) == (m_slaves[m_actualSlave] & ADDR_BYTE_ADDR_MASK) &&
        BCC(m_recvBuffer, USS_BUFFER_LENGTH - 1) == m_recvBuffer[USS_BUFFER_LENGTH - 1])
    {
        // USS is Big-Endian
        m_statusword[m_actualSlave] = (m_recvBuffer[PKW_LENGTH_CHARACTERS * PKW_ANZ + 3] << 8) & 0xFF00;
        m_statusword[m_actualSlave] |= m_recvBuffer[PKW_LENGTH_CHARACTERS * PKW_ANZ + 4] & 0xFF;
        m_mainactualvalue[m_actualSlave] = (m_recvBuffer[PKW_LENGTH_CHARACTERS * PKW_ANZ + 5] << 8) & 0xFF00;
        m_mainactualvalue[m_actualSlave] |= m_recvBuffer[PKW_LENGTH_CHARACTERS * PKW_ANZ + 6] & 0xFF;

        if(m_paramValue[0][m_actualSlave] != PARAM_VALUE_EMPTY)
        {
            if(((m_recvBuffer[3] << 8) & PKE_WORD_AK_MASK) == PKE_WORD_AK_NO_RESP)
                ret = -1;
            if(((m_recvBuffer[3] << 8) & PKE_WORD_AK_MASK) == PKE_WORD_AK_NO_RIGHTS)
                ret = -2;
            if(((m_recvBuffer[3] << 8) & PKE_WORD_AK_MASK) == PKE_WORD_AK_CANT_EXECUTE)
            {
                ret = m_recvBuffer[10] | m_recvBuffer[9] << 8;

                if(!ret)
                    ret = -3;   // 0 is error code for illegal parameter number
            }

            m_paramValue[0][m_actualSlave] = PARAM_VALUE_EMPTY;
        }
    }
    else
    {
        ret = -1;
    }

    if (m_dePin != -1)
        digitalWrite(m_dePin, HIGH);

    m_actualSlave++;
    
    return ret;
}