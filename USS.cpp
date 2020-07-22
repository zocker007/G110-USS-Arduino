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
  *   @file   USS.cpp
  *
  *   @brief  class implementation for Siemens USS protocol
  *
  *   @author Merlin Krümmel
  *
  *   @date   22.07.2020
  */

#include "USS.h"

extern USS uss;

USS::USS()
{
    paramError = 0;
    memset((void *)sendBuffer, 0, sizeof(sendBuffer));
    sendBuffer[0] = STX_BYTE_STX;
    sendBuffer[1] = (PKW_LENGTH_CHARACTERS * PKW_ANZ) + (PZD_LENGTH_CHARACTERS * PZD_ANZ) + 2; // 2 for ADR and BCC bytes
    memset((void *)recvBuffer, 0, sizeof(recvBuffer));
    memset((void *)mainsetpoint, 0, sizeof(mainsetpoint));
    memset((void *)ctlword, 0, sizeof(ctlword));
    memset((void *)mainactualvalue, 0, sizeof(mainactualvalue));
    memset((void *)statusword, 0, sizeof(statusword));
    actualSlave = 0;
    memset((void *)paramValue, 0, sizeof(paramValue));
    slaves = nullptr;
    nextSend = 0;
    period = 0;
}

void USS::begin(long speed, char *pslaves, byte pnrSlaves, int pdePin)
{
    int telegramRuntime;

    if(pslaves == nullptr)
        return;

    slaves = pslaves;
    nrSlaves = pnrSlaves;
    dePin = pdePin;
    characterRuntime = CHARACTER_RUNTIME_BASE_US / (speed / BAUDRATE_BASE);
    telegramRuntime = BUFFER_LENGTH * characterRuntime * 1.5f / 1000;
    Serial1.begin(speed, SERIAL_8E1);
    Serial1.setTimeout(telegramRuntime + MAX_RESP_DELAY_TIME_MS);
    pinMode(dePin,OUTPUT);
    digitalWrite(dePin, HIGH);
    period = telegramRuntime * 2 + (START_DELAY_LENGTH_CHARACTERS * characterRuntime / 1000) + MAX_RESP_DELAY_TIME_MS + MASTER_COMPUTE_DELAY_MS;
}

int USS::setParameter(uint16_t param, uint16_t value, byte slaveIndex)
{
    int ret = 0;

    if(slaveIndex >= nrSlaves)
        return -1;

    paramValue[0][slaveIndex] = (param & PKE_WORD_PARAM_MASK) | PKE_WORD_AK_CHW_PWE;
    paramValue[1][slaveIndex] = 0;
    paramValue[2][slaveIndex] = 0;
    paramValue[3][slaveIndex] = value;
    paramError = 0;

    while(paramValue[0][slaveIndex] != PARAM_VALUE_EMPTY)
    {
        send();
        ret = receive();
    }

    return ret;
}

int USS::setParameter(uint16_t param, uint32_t value, byte slaveIndex)
{
    int ret = 0;

    if(slaveIndex >= nrSlaves)
        return -1;

    paramValue[0][slaveIndex] = (param & PKE_WORD_PARAM_MASK) | PKE_WORD_AK_CHD_PWE;
    paramValue[1][slaveIndex] = 0;
    paramValue[2][slaveIndex] = (value >> 16) & 0xFFFF;
    paramValue[3][slaveIndex] = value & 0xFFFF;
    paramError = 0;

    while(paramValue[0][slaveIndex] != PARAM_VALUE_EMPTY)
    {
        send();
        ret = receive();
    }

    return ret;
}

int USS::setParameter(uint16_t param, float value, byte slaveIndex)
{
    parameter_t p;

    p.f32 = value;

    return setParameter(param, p.u32, slaveIndex);
}

void USS::setMainsetpoint(uint16_t value, byte slaveIndex)
{
    if(slaveIndex >= nrSlaves)
        return;

    mainsetpoint[slaveIndex] = value;
}

void USS::setCtlFlag(uint16_t flags, byte slaveIndex)
{
    if(slaveIndex >= nrSlaves)
        return;

    ctlword[slaveIndex] |= flags;
}

void USS::clearCtlFlag(uint16_t flags, byte slaveIndex)
{
    if(slaveIndex >= nrSlaves)
        return;

    ctlword[slaveIndex] &= ~flags;
}

uint16_t USS::getActualvalue(byte slaveIndex) const
{
    if(slaveIndex >= nrSlaves)
        return -1;

    uint16_t ret;

    ret = mainactualvalue[slaveIndex];

    return ret;
}

bool USS::checkStatusFlag(byte slaveIndex, uint16_t flag) const
{
    if(slaveIndex >= nrSlaves)
        return -1;

    uint16_t ret;

    ret = (statusword[slaveIndex] & flag) != 0 ? true : false;

    return ret;
}

byte USS::BCC(volatile byte buffer[], int length) const
{
    byte ret = 0;

    for(int i = 0; i < length; i++)
        ret = ret ^ buffer[i];

    return ret;
}

void USS::send()
{
    while(!(millis() > nextSend && (millis() - nextSend) < 10000));

    nextSend = millis() + period;

    if(actualSlave == nrSlaves)
        actualSlave = 0;

    sendBuffer[2] = slaves[actualSlave] & ADDR_BYTE_ADDR_MASK;

    if(paramValue[0][actualSlave] != PARAM_VALUE_EMPTY)
    {
        sendBuffer[3] = (paramValue[0][actualSlave] >> 8) & 0xFF;
        sendBuffer[4] = paramValue[0][actualSlave] & 0xFF;
        sendBuffer[5] = (paramValue[1][actualSlave] >> 8) & 0xFF;
        sendBuffer[6] = paramValue[1][actualSlave] & 0xFF;
        sendBuffer[7] = (paramValue[2][actualSlave] >> 8) & 0xFF;
        sendBuffer[8] = paramValue[2][actualSlave] & 0xFF;
        sendBuffer[9] = (paramValue[3][actualSlave] >> 8) & 0xFF;
        sendBuffer[10] = paramValue[3][actualSlave] & 0xFF;
    }
    else
    {
        sendBuffer[3] = 0;
        sendBuffer[4] = 0;
        sendBuffer[5] = 0;
        sendBuffer[6] = 0;
        sendBuffer[7] = 0;
        sendBuffer[8] = 0;
        sendBuffer[9] = 0;
        sendBuffer[10] = 0;
    }    

    sendBuffer[PKW_LENGTH_CHARACTERS * PKW_ANZ + 3] = (ctlword[actualSlave] >> 8) & 0xFF;
    sendBuffer[PKW_LENGTH_CHARACTERS * PKW_ANZ + 4] = ctlword[actualSlave] & 0xFF;
    sendBuffer[PKW_LENGTH_CHARACTERS * PKW_ANZ + 5] = (mainsetpoint[actualSlave] >> 8) & 0xFF;
    sendBuffer[PKW_LENGTH_CHARACTERS * PKW_ANZ + 6] = mainsetpoint[actualSlave] & 0xFF;

    sendBuffer[BUFFER_LENGTH - 1] = BCC(sendBuffer, BUFFER_LENGTH - 1);

   Serial1.write(sendBuffer, BUFFER_LENGTH);
   Serial1.flush();

    delayMicroseconds(START_DELAY_LENGTH_CHARACTERS * characterRuntime);
    digitalWrite(dePin, LOW);
}

int USS::receive()
{
    int ret = 0;

    if(Serial1.readBytes((byte *)recvBuffer, BUFFER_LENGTH) == BUFFER_LENGTH &&
        recvBuffer[0] == STX_BYTE_STX && (recvBuffer[2] & ADDR_BYTE_ADDR_MASK) == (slaves[actualSlave] & ADDR_BYTE_ADDR_MASK) &&
        BCC(recvBuffer, BUFFER_LENGTH - 1) == recvBuffer[BUFFER_LENGTH - 1])
    {
        statusword[actualSlave] = (recvBuffer[PKW_LENGTH_CHARACTERS * PKW_ANZ + 3] << 8) & 0xFF00;
        statusword[actualSlave] |= recvBuffer[PKW_LENGTH_CHARACTERS * PKW_ANZ + 4] & 0xFF;
        mainactualvalue[actualSlave] = (recvBuffer[PKW_LENGTH_CHARACTERS * PKW_ANZ + 5] << 8) & 0xFF00;
        mainactualvalue[actualSlave] |= recvBuffer[PKW_LENGTH_CHARACTERS * PKW_ANZ + 6] & 0xFF;

        if(paramValue[0][actualSlave] != PARAM_VALUE_EMPTY)
        {
            if(((recvBuffer[3] << 8) & PKE_WORD_AK_MASK) == PKE_WORD_AK_NO_RESP)
                ret = -1;
            if(((recvBuffer[3] << 8) & PKE_WORD_AK_MASK) == PKE_WORD_AK_NO_RIGHTS)
                ret = -2;
            if(((recvBuffer[3] << 8) & PKE_WORD_AK_MASK) == PKE_WORD_AK_CANT_EXECUTE)
            {
                ret = recvBuffer[10] | recvBuffer[9] << 8;

                if(!ret)
                    ret = -3;   // 0 is error code for illegal parameter number
            }

            paramValue[0][actualSlave] = PARAM_VALUE_EMPTY;
        }
    }
    else
    {
        ret = -1;
    }

    digitalWrite(dePin, HIGH);
    actualSlave++;
    
    return ret;
}