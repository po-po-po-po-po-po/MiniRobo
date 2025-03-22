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

    void moveA(Udon::Stick moveInfo, double maxPower = 20'000);

    void moveB(double rightStick, double leftStick);

    void stop();

    void show();

private:
    RoboMasterSpeedFB   right;
    RoboMasterSpeedFB   left;
    RelativeGyro        gyro;
    Udon::PidController pid;
    unsigned long long  lastTime       = 0;
    double              lastRightPower = 0;
    double              lastLeftPower  = 0;

    // コントローラーの入力範囲 -255 ~ +255 をロボマスのRPMの範囲 -20'000 ~ +20'000 に変換する倍率
    static constexpr double mapping = 20'000.0 / 255.0;
};


inline void TwoWheelDriveSystem::moveA(Udon::Stick moveInfo, double maxPower)
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

    double rightPower = moveInfo.vector.y - moveInfo.turn;
    double leftPower  = moveInfo.vector.y + moveInfo.turn;

    double maxElement    = std::max(std::fabs(rightPower), std::fabs(leftPower));
    double magnification = 1;

    if (maxElement > maxPower)
    {
        magnification = maxPower / maxElement;
    }

    right.move(lastRightPower = rightPower * magnification);
    left.move(lastLeftPower = leftPower * magnification);
}


inline void TwoWheelDriveSystem::moveB(double rightStick, double leftStick)
{
    right.move(lastRightPower = rightStick * mapping);
    left.move(lastLeftPower = leftStick * mapping);
}


inline void TwoWheelDriveSystem::stop()
{
    right.stop();
    left.stop();
    gyro.clear();
    pid.clearPower();
}


inline void TwoWheelDriveSystem::show()
{
    Udon::Printf("lastPower  :  Left %6.1f    Right %6.1f \n", lastLeftPower, lastRightPower);
    Serial.println();
    right.show();
    Serial.println();
    left.show();
}
