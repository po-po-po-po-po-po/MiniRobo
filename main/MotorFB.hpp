#pragma once

#include <utility>
#include <Udon/Com/Driver/Motor.hpp>
#include <Udon/Com/Driver/Encoder.hpp>
#include <Udon/Algorithm/PidController.hpp>
#include "MoveInfo.hpp"

struct MotorFB
{
    MotorFB(
        Udon::MotorBy<Udon::CanWriter>&&   Driver_,
        Udon::EncoderBy<Udon::CanReader>&& Enc_,
        Udon::PidController&&              Pid_,
        int&&                              Resolution_ = 8192)
        : driver(std::move(Driver_))
        , enc(std::move(Enc_))
        , pid(std::move(Pid_))
        , resolution(std::move(Resolution_))
    {
    }

    void move(double angle, int max = 200);

    void movePower(int power);

    void stop();

    void reset();

    double getAngle();

    void show();

    bool isMoving() { return Udon::Abs(pid.getPower()) > 15; }

private:
    Udon::MotorBy<Udon::CanWriter>   driver;
    Udon::EncoderBy<Udon::CanReader> enc;
    Udon::PidController              pid;
    const int                        resolution;
};


inline void MotorFB::move(double angle, int max)
{
    double target = (angle / Udon::TwoPi * resolution);

    enc.update();
    driver.move(pid(enc.getCount(), target, -max, max));
}


inline void MotorFB::movePower(int power)
{
    enc.update();
    pid.clearPower();
    driver.move(power);
}


inline void MotorFB::stop()
{
    enc.update();
    driver.stop();
    pid.clearPower();
}


inline void MotorFB::reset()
{
    enc.update();
    enc.setOffset();
    pid.clearPower();
}


inline double MotorFB::getAngle()
{
    enc.update();
    return enc.getCount() * Udon::TwoPi / resolution;
}


inline void MotorFB::show()
{
    Serial.println("encoder");
    enc.show();
    Serial.println();
    Serial.println("motor power");
    driver.show();
    Serial.println();
}
