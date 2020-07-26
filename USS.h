/**
  * copyright (c) 2020, Merlin Kr�mmel
  * SPDX-License-Identifier: LGPL-3.0-or-later
  */

/**
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
  *
  *   @brief  class definition for Siemens USS protocol
  *
  *   @author Merlin Kr�mmel
  *
  *   @date   22.07.2020
  */

#ifndef USS_H
#define USS_H

#include "Arduino.h"

#define STX_BYTE_STX               0x02

#define PARAM_VALUE_EMPTY          0xA000

#define ADDR_BYTE_BROADCAST_FLAG   0x20
#define ADDR_BYTE_MIRROR_FLAG      0x40
#define ADDR_BYTE_SPECIAL_FLAG     0x80

#define ADDR_BYTE_ADDR_MASK        0x1F
#define PKE_WORD_PARAM_MASK        0x7FF
#define PKE_WORD_SP_FLAG           0x800
#define PKE_WORD_AK_MASK           0xF000
#define PKE_WORD_AK_NO_TASK        0x0000
#define PKE_WORD_AK_REQ_PWE        0x1000
#define PKE_WORD_AK_CHW_PWE        0x2000
#define PKE_WORD_AK_CHD_PWE        0x3000

#define PKE_WORD_AK_NO_RESP        0x0000
#define PKE_WORD_AK_TRW_PWE        0x1000
#define PKE_WORD_AK_TRD_PWE        0x2000
#define PKE_WORD_AK_NO_RIGHTS      0x8000
#define PKE_WORD_AK_CANT_EXECUTE   0x7000

#define CTL_WORD_ON_OFF1_FLAG      0x0001
#define CTL_WORD_ON_OFF1_OFF1      0x0000
#define CTL_WORD_ON_OFF1_ON        0x0001
#define CTL_WORD_OFF2_FLAG         0x0002
#define CTL_WORD_OFF2_OFF2         0x0000
#define CTL_WORD_OFF2_OP_COND      0x0002
#define CTL_WORD_OFF3_FLAG         0x0004
#define CTL_WORD_OFF3_OFF3         0x0000
#define CTL_WORD_OFF3_OP_COND      0x0004
#define CTL_WORD_ENABLE_FLAG       0x0008
#define CTL_WORD_ENABLE_INHIBIT    0x0000
#define CTL_WORD_ENABLE_ENABLE     0x0008
#define CTL_WORD_INHIBIT_RAMP_FLAG       0x0010
#define CTL_WORD_INHIBIT_RAMP_INHIBIT    0x0000
#define CTL_WORD_INHIBIT_RAMP_OP_COND    0x0010
#define CTL_WORD_ENABLE_RAMP_FLAG        0x0020
#define CTL_WORD_ENABLE_RAMP_HOLD        0x0000
#define CTL_WORD_ENABLE_RAMP_ENABLE      0x0020
#define CTL_WORD_ENABLE_SETPOINT_FLAG    0x0040
#define CTL_WORD_ENABLE_SETPOINT_INHIBIT 0x0000
#define CTL_WORD_ENABLE_SETPOINT_ENABLE  0x0040
#define CTL_WORD_ACK_FLAG          0x0080
#define CTL_WORD_CTL_PLC_FLAG      0x0400
#define CTL_WORD_CTL_PLC_NO_CTL    0x0000
#define CTL_WORD_CTL_PLC_CTL_PLC   0x0400

#define STATUS_WORD_SWITCH_READY_FLAG           0x0001
#define STATUS_WORD_SWITCH_READY                0x0001
#define STATUS_WORD_SWITCH_NOT_READY            0x0000
#define STATUS_WORD_READY_FLAG                  0x0002
#define STATUS_WORD_READY                       0x0002
#define STATUS_WORD_NOT_READY                   0x0000
#define STATUS_WORD_OP_ENABLED_FLAG             0x0004
#define STATUS_WORD_OP_ENABLED_ENABLED          0x0004
#define STATUS_WORD_OP_ENABLED_INHIBIT          0x0000
#define STATUS_WORD_FAULT_FLAG                  0x0008
#define STATUS_WORD_FAULT_FAULT                 0x0008
#define STATUS_WORD_FAULT_FAUlT_FREE            0x0000
#define STATUS_WORD_OFF2_FLAG                   0x0010
#define STATUS_WORD_OFF2_NO_OFF2                0x0010
#define STATUS_WORD_OFF2_OFF2                   0x0000
#define STATUS_WORD_OFF3_FLAG                   0x0020
#define STATUS_WORD_OFF3_NO_OFF3                0x0020
#define STATUS_WORD_OFF3_OFF3                   0x0000
#define STATUS_WORD_SWITCH_INHIBIT_FLAG         0x0040
#define STATUS_WORD_SWITCH_INHIBIT_INHIBIT      0x0040
#define STATUS_WORD_SWITCH_INHIBIT_NO_INHIBIT   0x0000
#define STATUS_WORD_ALARM_FLAG                  0x0080
#define STATUS_WORD_ALARM_ALARM                 0x0080
#define STATUS_WORD_ALARM_NO_ALARM              0x0000
#define STATUS_WORD_SETPOINT_TOL_FLAG           0x0100
#define STATUS_WORD_SETPOINT_TOL_IN_RANGE       0x0100
#define STATUS_WORD_SETPOINT_TOL_NOT_IN_RANGE   0x0000
#define STATUS_WORD_CTL_REQ_FLAG                0x0200
#define STATUS_WORD_CTL_REQ_CTL_REQ             0x0200
#define STATUS_WORD_CTL_REQ_LOCAL_OP            0x0000
#define STATUS_WORD_F_N_REACHED_FLAG            0x0400
#define STATUS_WORD_F_N_REACHED_REACHED         0x0400
#define STATUS_WORD_F_N_REACHED_FALLEN_BELOW    0x0000

#define USS_SLAVES                 2

#define PZD_ANZ                    1
#define PZD_LENGTH_CHARACTERS      4
#define PKW_ANZ                    1
#define PKW_LENGTH_CHARACTERS      8

#define CHARACTER_RUNTIME_BASE_US  1150
#define BAUDRATE_BASE              9600
#define MAX_RESP_DELAY_TIME_MS     20
#define MASTER_COMPUTE_DELAY_MS    20
#define START_DELAY_LENGTH_CHARACTERS 2
#define TELEGRAM_OVERHEAD_CHARACTERS 4

#define USS_BUFFER_LENGTH               (TELEGRAM_OVERHEAD_CHARACTERS + (PKW_LENGTH_CHARACTERS * PKW_ANZ) + (PZD_LENGTH_CHARACTERS * PZD_ANZ))

class USS
{
    public:

    USS();
    int begin(long speed, const char pslaves[], byte pnrSlaves, int dePin);
    int setParameter(uint16_t param, uint16_t value, byte slaveIndex);
    int setParameter(uint16_t param, uint32_t value, byte slaveIndex);
    int setParameter(uint16_t param, float value, byte slaveIndex);
    void setMainsetpoint(uint16_t value, byte slaveIndex);
    void setCtlFlag(uint16_t flags, byte slaveIndex);
    void clearCtlFlag(uint16_t flags, byte slaveIndex);
    uint16_t getActualvalue(byte slaveIndex) const;
    bool checkStatusFlag(byte slaveindex, uint16_t flag) const;

    void send();
    int receive();

    private:

    typedef union
    {
        uint32_t u32;
        float f32;
    } parameter_t;
    
    byte BCC(const byte buffer[], int length) const;

    char slaves[USS_SLAVES];
    byte nrSlaves;
    byte actualSlave;
    byte sendBuffer[USS_BUFFER_LENGTH];
    byte recvBuffer[USS_BUFFER_LENGTH];
    uint16_t mainsetpoint[USS_SLAVES];
    uint16_t mainactualvalue[USS_SLAVES];
    uint16_t ctlword[USS_SLAVES];
    uint16_t statusword[USS_SLAVES];
    uint16_t paramValue[PKW_LENGTH_CHARACTERS / 2][USS_SLAVES];
    unsigned long nextSend;
    unsigned long period;
    int characterRuntime;
    int dePin;
};

#endif