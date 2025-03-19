#pragma once

#include "GyroManager.hpp"


struct RelativeGyro
{
    RelativeGyro(const GyroMnager& Gyro_)
        : gyro(Gyro_)
    {
    }

    void clear(double newTurn = 0) { offset = gyro.getTurn(); }

    double getTurn() const { return gyro.getTurn() - offset; }

private:
    const GyroMnager& gyro;
    double            offset;
};
