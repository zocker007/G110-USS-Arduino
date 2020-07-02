#include "G110.h"

G110::G110()
{
    interface = nullptr;
    refFreq = 0.0f;
    index = 0;
}

int G110::start(USS *pinterface, quick_commissioning_t quick_comm_data, byte pindex)
{
    if(pinterface == nullptr)
        return -1;

    interface = pinterface;
    refFreq = quick_comm_data.motorFreq;
    index = pindex;
    int err = 0;

    err += setParameter(PARAM_NR_USER_ACCESS_LEVEL, USER_ACCESS_LEVEL_EXPERT);
    err += setParameter(PARAM_NR_USS_PKW_LENGTH, USS_PKW_LENGTH_4_WORDS);
    err += setParameter(PARAM_NR_COMMISSIONING_PARAM, QUICK_COMMISSIONING_QUICK_COMM);
    err += setParameter(PARAM_NR_POWER_SETING, quick_comm_data.powerSetting);
    err += setParameter(PARAM_NR_MOTOR_VOLTAGE_V, quick_comm_data.motorVoltage);
    err += setParameter(PARAM_NR_MOTOR_CURRENT_A, quick_comm_data.motorCurrent);
    err += setParameter(PARAM_NR_MOTOR_POWER_KW_HP, quick_comm_data.motorPower);

    if(quick_comm_data.powerSetting == POWER_SETTING_NORTH_AMERICA_HP)
        err += setParameter(PARAM_NR_MOTOR_EFFICIENCY_FACTOR, quick_comm_data.motorEff);
    else
        err += setParameter(PARAM_NR_MOTOR_COS_PHI, quick_comm_data.motorCosPhi);

    err += setParameter(PARAM_NR_MOTOR_FREQ_HZ, quick_comm_data.motorFreq);
    err += setParameter(PARAM_NR_MOTOR_SPEED_PER_MINUTE, quick_comm_data.motorSpeed);
    err += setParameter(PARAM_NR_MOTOR_COOLING, quick_comm_data.motorCooling);
    err += setParameter(PARAM_NR_MOTOR_OVERLOAD_FACTOR, quick_comm_data.motorOverload);
    err += setParameter(PARAM_NR_SEL_CMD_SOURCE, quick_comm_data.cmdSource);
    err += setParameter(PARAM_NR_SEL_FREQ_SETPOINT, quick_comm_data.setpointSource);
    err += setParameter(PARAM_NR_MIN_FREQ_HZ, quick_comm_data.minFreq);
    err += setParameter(PARAM_NR_MAX_FREQ_HZ, quick_comm_data.maxFreq);
    err += setParameter(PARAM_NR_RAMP_UP_TIME_S, quick_comm_data.rampupTime);
    err += setParameter(PARAM_NR_RAMP_DOWN_TIME_S, quick_comm_data.rampdownTime);
    err += setParameter(PARAM_NR_OFF3_RAMP_DOWN_TIME_S, quick_comm_data.OFF3rampdownTime);
    err += setParameter(PARAM_NR_CTL_MODE, quick_comm_data.ctlMode);
    err += setParameter(PARAM_NR_CALC_MOTOR_PARAMS, CALC_MOTOR_PARAMS_COMPLETE);
    err += setParameter(PARAM_NR_COMMISSIONING_PARAM, QUICK_COMMISSIONING_READY);

    if(!err)
        setCtlFlag(CTL_WORD_ENABLE_ENABLE | CTL_WORD_INHIBIT_RAMP_OP_COND |
                              CTL_WORD_ENABLE_RAMP_ENABLE | CTL_WORD_ENABLE_SETPOINT_ENABLE |
                              CTL_WORD_CTL_PLC_CTL_PLC);
                          
    return err;
}

void G110::setFrequency(float freq)
{
    bool reverse = false;

    if(freq < 0)
    {
        reverse = true;
        freq *= -1.0f;
    }
    // f[Hz] = (f(hex) / FREQUENCY_CALC_BASE) * refFreq
    uint16_t f_hex = (freq / refFreq) * FREQUENCY_CALC_BASE;

    if(reverse)
        setCtlFlag(CTL_WORD_REVERSE_FALG);
    else
        clearCtlFlag(CTL_WORD_REVERSE_FALG);

    interface->setMainsetpoint(f_hex, index);
    
}

void G110::setON()
{
    setCtlFlag(CTL_WORD_ON_OFF1_ON | CTL_WORD_OFF2_OP_COND |
                          CTL_WORD_OFF3_OP_COND);
}

void G110::setOFF1()
{
    clearCtlFlag(CTL_WORD_ON_OFF1_FLAG);
}

void G110::setCtlFlag(uint16_t flags)
{
    interface->setCtlFlag(flags, index);
}

void G110::clearCtlFlag(uint16_t flags)
{
    interface->clearCtlFlag(flags, index);
}

bool G110::running()
{
    return checkStatusFlag(STATUS_WORD_OP_ENABLED_FLAG);
}

bool G110::getOFF2()
{
    return !checkStatusFlag(STATUS_WORD_OFF2_FLAG);
}

bool G110::getOFF3()
{
    return !checkStatusFlag(STATUS_WORD_OFF3_FLAG);
}

bool G110::setpointReached()
{
    return checkStatusFlag(STATUS_WORD_SETPOINT_TOL_FLAG);
}

bool G110::reverse()
{
    return !checkStatusFlag(STATUS_WORD_MOTOR_RUNS_RIGHT_FLAG);
}

bool G110::checkStatusFlag(uint16_t flag)
{
    return interface->checkStatusFlag(index, flag);
}

float G110::getFrequency()
{
    uint16_t f_hex = interface->getActualvalue(index);
    // f[Hz] = (f(hex) / FREQUENCY_CALC_BASE) * refFreq
    return (f_hex / FREQUENCY_CALC_BASE) * refFreq;
}

void G110::reset()
{
    setParameter(PARAM_NR_COMMISSIONING_PARAM, QUICK_COMMISSIONING_FACTORY_SETTING);
    setParameter(PARAM_NR_FACTORY_RESET, FACTORY_RESET_PARAMETER_RESET);
    delay(10000);
}

int G110::setParameter(uint16_t param, uint16_t value)
{
    return interface->setParameter(param, value, index);
}

int G110::setParameter(uint16_t param, uint32_t value)
{
    return interface->setParameter(param, value, index);
}

int G110::setParameter(uint16_t param, float value)
{
    return interface->setParameter(param, value, index);
}