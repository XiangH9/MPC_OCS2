#ifndef ENUMCLASS_H
#define ENUMCLASS_H

enum class FSMStateName
{
    // EXIT,
    INVALID,
    PASSIVE,
    FIXEDDOWN,
    FIXEDSTAND,
    FREESTAND,
    TROTTING,

    SWINGTEST,
    BALANCETEST,

    NMPC,
    OCS2,
    RL
};

enum class FSMMode
{
    NORMAL,
    CHANGE
};

enum class FrameType
{
    BODY,
    HIP,
    GLOBAL
};

enum class WaveStatus
{
    STANCE_ALL,
    SWING_ALL,
    WAVE_ALL
};

#endif //ENUMCLASS_H
