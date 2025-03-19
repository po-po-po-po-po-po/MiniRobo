#pragma once

#include <utility>
#include <cmath>
#include <memory>
#include <Udon/Driver/RoboMasterMotor.hpp>
#include <Udon/Algorithm/PidController.hpp>
#include <Udon/Algorithm/SpeedPid.hpp>
#include <Udon/Algorithm/MovingAverage.hpp>
#include "MoveInfo.hpp"

namespace Impl
{
    template <typename Pid>
    struct RoboMasterFB
    {
        static_assert(
            std::is_same_v<Pid, Udon::SpeedPidController> ||
                std::is_same_v<Pid, Udon::PidController>,
            "Template argument must be `Udon::PidController` or `Udon::SpeedPidController`.");

        template <typename U>
        RoboMasterFB(U&& DriverInstance_, Pid&& Pid_)
            : driver(std::make_unique<U>(std::forward<U>(DriverInstance_)))
            , pid(std::move(Pid_))
            , range(driver->getCurrentRange())
            , offset(0)
            , current(0)
        {
            static_assert(std::is_base_of_v<Udon::RoboMasterBase, U>, "Driver must be drived `Udon::RoboMasterBase`.");
        }

        void moveVelocity(double rpm);

        void move(double rpm) {}

        void setCurrent(int current);

        void stop();

        bool isStuck(double immediate, double average);

        void reset();

        bool setOffset(MoveInfo moveInfo, double immediate, double average);

        void show();

        double getAngle() { return driver->getAngle() - offset; }

        Udon::RoboMasterBase* operator->() const { return driver.get(); };

    private:
        std::unique_ptr<Udon::RoboMasterBase> driver;
        Pid                                   pid;
        const Udon::Range<int16_t>            range;
        double                                offset;
        Udon::MovingAverage<50>               torqueCurrentAverage;

        double current;
    };


    template <typename T>
    inline void RoboMasterFB<T>::moveVelocity(double rpm)
    {
        driver->setCurrent(current = pid(driver->getVelocity(), rpm, range.min, range.max));
    }

    template <>
    inline void RoboMasterFB<Udon::PidController>::move(double angle)
    {
        driver->setCurrent(current = pid(this->getAngle(), angle, range.min, range.max));
    }


    template <>
    inline void RoboMasterFB<Udon::SpeedPidController>::move(double rpm)
    {
        moveVelocity(rpm);
    }


    template <typename T>
    inline void RoboMasterFB<T>::setCurrent(int current)
    {
        driver->setCurrent(current);
        pid.clearPower();
    }


    template <typename T>
    inline void RoboMasterFB<T>::stop()
    {
        driver->setCurrent(0);
        pid.clearPower();
    }

    template <typename T>
    inline bool RoboMasterFB<T>::isStuck(double immediate, double average)
    {
        bool imm = std::fabs(driver->getTorqueCurrent()) > immediate;
        bool ave = torqueCurrentAverage(std::fabs(driver->getTorqueCurrent())) > average;

        return imm || ave;
    }


    template <typename T>
    inline void RoboMasterFB<T>::reset()
    {
        pid.clearPower();
        offset               = driver->getAngle();
        torqueCurrentAverage = {};
    }


    template <typename T>
    inline bool RoboMasterFB<T>::setOffset(MoveInfo moveInfo, double immediate, double average)
    {
        if ((bool)moveInfo.parameter)
        {
            pid.requestParam(moveInfo.parameter);
        }

        if (this->isStuck(immediate, average))
        {
            this->stop();
            this->reset();
            return false;
        }
        else
        {
            this->moveVelocity(moveInfo.speedByDir());
            return true;
        }
    }


    template <typename T>
    inline void RoboMasterFB<T>::show()
    {
        Udon::Printf("Angle         : %9.3f \n", driver->getAngle() - offset);
        Udon::Printf("RawAngle      : %9.3f \n", driver->getRawAngle());
        Udon::Printf("Velocity      : %6d \n", driver->getVelocity());
        Udon::Printf("TorqueCurrent : %4d \n", driver->getTorqueCurrent());
        Udon::Printf("Temperature   : %4d \n", driver->getTemperature());
        Udon::Printf("Current       : %6.1f \n", current);
    }
}    // namespace Impl

using RoboMasterFB      = Impl::RoboMasterFB<Udon::PidController>;
using RoboMasterSpeedFB = Impl::RoboMasterFB<Udon::SpeedPidController>;
