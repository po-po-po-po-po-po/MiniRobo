#include <Udon.hpp>

#include "RoboMasterFB.hpp"
#include "GyroManager.hpp"
#include "TwoWheelDriveSystem.hpp"
#include "Selecter.hpp"


Udon::LoopCycleController loopCtrl{ 10000 };
Udon::Led                 led{ LED_BUILTIN };


Udon::E220PadPS5 pad{ {
    .serial = Serial2,
    .m0     = 2,
    .m1     = 3,
    .aux    = 4,
} };


Udon::CanBusTeensy<CAN1> comBus;
Udon::CanBusTeensy<CAN2> motorBus;


GyroMnager gyro{ { Wire } };


TwoWheelDriveSystem tw{
    RoboMasterSpeedFB{
        Udon::RoboMasterC610{ motorBus, 1, Udon::Direction::Forward },
        Udon::SpeedPidController{ 0.032, 0.2, 0.0009, loopCtrl.cycleUs(), 20000 } },
    RoboMasterSpeedFB{
        Udon::RoboMasterC610{ motorBus, 2, Udon::Direction::Backward },
        Udon::SpeedPidController{ 0.032, 0.2, 0.0009, loopCtrl.cycleUs(), 20000 } },
    RelativeGyro{ gyro },
    Udon::PidController{ 35, 0.2, 7.8, loopCtrl.cycleUs() },
};


RoboMasterSpeedFB collector{
    Udon::RoboMasterC610{ motorBus, 3, Udon::Direction::Forward },
    Udon::SpeedPidController{ 0.01, 0.2, 0.000, loopCtrl.cycleUs(), 20000 },
};


Udon::MotorBy<Udon::CanWriter> lift{ Udon::CanWriter<Udon::Message::Motor>{ comBus, 0x001 } };


Selecter selecter{
    MotorFB{
        Udon::MotorBy<Udon::CanWriter>{ { comBus, 0x002 }, Udon::Direction::Forward },
        Udon::EncoderBy<Udon::CanReader>{ { comBus, 0x003 }, Udon::Direction::Forward },
        Udon::PidController{ 0.07, 0.0001, 0.001, loopCtrl.cycleUs(), 10 },
    },
    Udon::CanReader<Udon::Message::Switch>{ comBus, 0x004 },
};


bool isResetting;


void setup()
{
    Serial.begin(115200);
    pad.begin();
    comBus.begin();
    motorBus.begin();
    led.begin();
    gyro.begin();
    delay(200);
    isResetting = true;
}


void loop()
{
    pad.update();
    comBus.update();
    motorBus.update();
    gyro.update();

    if (pad.isOperable() && (bool)comBus && (bool)motorBus)
    {
        if (isResetting)
        {
            isResetting = selecter.takeOffset(Udon::Direction::Backward);
        }
        else
        {
            tw.moveA(pad.getMoveInfo());
            collector.move(pad.getCircle().toggle ? 15000 : 0);
            lift.move(pad.getSquare().toggle ? 10 : 0);
            selecter.move(pad.getTriangle().toggle);
        }
    }
    else
    {
        tw.stop();
        collector.stop();
        lift.stop();
        selecter.stop();
    }

    if (pad.getTouch().click)
    {
        isResetting = true;
    }

    if ((bool)comBus && (bool)motorBus && (bool)pad)
    {
        led.flush();
    }
    else
    {
        led.flush(100);
    }

    loopCtrl.update();
}
