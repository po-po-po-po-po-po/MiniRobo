#pragma once

#include <Udon/Algorithm/PidController.hpp>
#include <Udon/Algorithm/SpeedPid.hpp>

namespace Pid
{
    struct Parameter
    {
        double p;
        double i;
        double d;

        operator bool()
        {
            return (p != 0) || (i != 0) || (d != 0);
        }

        operator Udon::PidController::Parameter()
        {
            return Udon::PidController::Parameter{ p, i, d };
        }

        operator Udon::SpeedPidController::Parameter()
        {
            return Udon::SpeedPidController::Parameter{ p, i, d };
        }
    };
}    // namespace Pid


struct MoveInfo
{
    double          speed;
    Udon::Direction dir;
    Pid::Parameter  parameter;

    MoveInfo(double Speed_, Udon::Direction Dir_, Pid::Parameter Parameter_ = {})
        : speed(Speed_)
        , dir(Dir_)
        , parameter(Parameter_)
    {
    }

    int dirToSign()
    {
        return Udon::DirectionToSign(dir);
    }

    double speedByDir()
    {
        return speed * dirToSign();
    }
};
