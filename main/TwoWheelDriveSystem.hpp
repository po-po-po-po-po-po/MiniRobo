#pragma once

#include <Udon.hpp>

#include <cmath>
#include <algorithm>

#include "RoboMasterFB.hpp"
#include "RelativeGyro.hpp"

struct TwoWheelDriveSystem
{
    TwoWheelDriveSystem(RoboMasterSpeedFB&& Right_, RoboMasterSpeedFB&& Left_, RelativeGyro&& Gyro_, Udon::PidController Pid_)
        : right(std::move(Right_))
        , left(std::move(Left_))
        , gyro(std::move(Gyro_))
        , pid(std::move(Pid_))
    {
    }

    void moveA(Udon::Stick moveInfo);

    void moveB(double rightStick, double leftStick);
    
    void stop();

private:
    RoboMasterSpeedFB   right;
    RoboMasterSpeedFB   left;
    RelativeGyro        gyro;
    Udon::PidController pid;
    unsigned long long  lastTime = 0;

    // コントローラーの入力範囲 -255 ~ +255 をロボマスのRPMの範囲 -20'000 ~ +20'000 に変換する倍率
    static constexpr double mapping = 20'000.0 / 255.0;
};


void TwoWheelDriveSystem::moveA(Udon::Stick moveInfo)
{
    if (moveInfo.turn)
    {
        lastTime = millis();
    }
    if (millis() - lastTime < 500)
    {
        gyro.clear();
    }

    moveInfo.turn -= pid(gyro.getTurn(), 0);

    moveInfo *= mapping;

    double rightPower = moveInfo.vector.x - moveInfo.turn;
    double leftPower  = moveInfo.vector.x + moveInfo.turn;

    double maxElement    = std::max(std::fabs(rightPower), std::fabs(leftPower));
    double magnification = 1;

    if (maxElement > 20'000)
    {
        magnification = 20'000 / maxElement;
    }

    right.move(rightPower * magnification);
    left.move(leftPower * magnification);
}


void TwoWheelDriveSystem::moveB(double rightStick, double leftStick)
{
    right.move(rightStick * mapping);
    left.move(leftStick * mapping);
}


void TwoWheelDriveSystem::stop()
{
    right.stop();
    left.stop();
    gyro.clear();
    pid.clearPower();
}
