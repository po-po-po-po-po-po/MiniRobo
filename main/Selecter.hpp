#pragma once

#include <Udon.hpp>

#include "MotorFB.hpp"


struct Selecter
{
    Selecter(MotorFB&& Motor_, Udon::CanReader<Udon::Message::Switch>&& SW_)
        : motor(std::move(Motor_))
        , sw(std::move(SW_))
    {
    }

    void move(bool isOpen);

    void stop();

    bool takeOffset(Udon::Direction dir, double power = 10);

    void show();

private:
    MotorFB                                motor;
    Udon::CanReader<Udon::Message::Switch> sw;
};


inline void Selecter::move(bool isOpen)
{
    constexpr double openAngle  = 10;
    constexpr double closeAngle = 60;

    motor.move(isOpen ? openAngle : closeAngle);
}


inline void Selecter::stop()
{
    motor.stop();
}


inline bool Selecter::takeOffset(Udon::Direction dir, double power)
{
    bool isContinue = !sw.getMessage().value().press;
    if (isContinue)
    {
        motor.movePower(power * Udon::DirectionToSign(dir));
    }
    else
    {
        motor.stop();
        motor.reset();
    }
    return isContinue;
}


inline void Selecter::show()
{
    motor.show();
    Udon::Printf("sw : %d", sw.getMessage().value().press);
    Serial.println();
}
