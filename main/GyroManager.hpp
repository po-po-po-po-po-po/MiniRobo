#pragma once


#include <utility>
#include <Udon/Driver/BNO055.hpp>
#include <Udon/Utility/Printf.hpp>


struct GyroMnager
{
    GyroMnager(Udon::BNO055&& Gyro_)
        : gyro(std::move(Gyro_))
    {
    }

    GyroMnager(const GyroMnager&) = delete;
    GyroMnager(GyroMnager&&)      = delete;

    void clear(double newTurn = 0)
    {
        gyro.clear();

        turn = newTurn;

        auto realTurn = gyro.getQuaternion().toYaw();
        turnOffset    = turn - realTurn;
        lastTurn      = realTurn;
    }

    void begin() { gyro.begin(); }

    void update();

    double getTurn() const { return turn; }


private:
    Udon::BNO055 gyro;

    double turn       = 0;
    double turnOffset = 0;
    double lastTurn   = 0;

    double wrapTurn(double current, double last)
    {
        if (std::fabs(current - last) > Udon::Pi)
        {
            if (current > 0)
            {
                turnOffset -= Udon::TwoPi;
            }
            else if (current < 0)
            {
                turnOffset += Udon::TwoPi;
            }
        }
        return current + turnOffset;
    }
};


inline void GyroMnager::update()
{
    gyro.update();

    auto realTurn = gyro.getQuaternion().toYaw();
    turn          = wrapTurn(realTurn, lastTurn);
    lastTurn      = realTurn;
}
