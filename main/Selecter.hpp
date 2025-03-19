#pragma once

#include <Udon.hpp>

#include "MotorFB.hpp"


struct Selecter
{
    Selecter(MotorFB&& Motor_, Udon::CanReader<Udon::Message::Switch>&& SW_)
        : motor(std::move(motor))
        , sw(std::move(SW_))
    {
    }

    void move(bool isOpen);

    void stop();

    bool takeOffset(Udon::Direction dir, double power = 10);

private:
    MotorFB                                motor;
    Udon::CanReader<Udon::Message::Switch> sw;
};


void Selecter::move(bool isOpen)
{
    constexpr double openAngle  = 10;
    constexpr double closeAngle = 60;

    motor.move(isOpen ? openAngle : closeAngle);
}


void Selecter::stop()
{
    motor.stop();
}


bool Selecter::takeOffset(Udon::Direction dir, double power)
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
