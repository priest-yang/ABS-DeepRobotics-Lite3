#ifndef CUSTOM_TYPES_H_
#define CUSTOM_TYPES_H_

#include "common_types.h"

namespace types{
    enum RobotType{
        X30 = 0,
        Lite3,
        P50
    };

    enum RobotMotionState{
        WaitingForStand = 0,
        StandingUp      = 1,
        JointDamping    = 2,

        RLControlMode   = 6,
    };

    enum StateName{
        kInvalid      = -1,
        kIdle         = 0,
        kStandUp      = 1,
        kJointDamping = 2,

        kRLControl    = 6,
    };
};

#endif