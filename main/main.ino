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
        Udon::RoboMasterC610{ motorBus, 1, Udon::Direction::Backward },
        Udon::SpeedPidController{ 0.032, 0.2, 0.0009, loopCtrl.cycleUs(), 20000 } },
    RoboMasterSpeedFB{
        Udon::RoboMasterC610{ motorBus, 2, Udon::Direction::Forward },
        Udon::SpeedPidController{ 0.032, 0.2, 0.0009, loopCtrl.cycleUs(), 20000 } },
    RelativeGyro{ gyro },
    Udon::PidController{ 70, 0.2, 7.8, loopCtrl.cycleUs() },
};


RoboMasterSpeedFB collector{
    Udon::RoboMasterC610{ motorBus, 3, Udon::Direction::Forward },
    Udon::SpeedPidController{ 0.01, 0.2, 0.000, loopCtrl.cycleUs(), 20000 },
};


Udon::MotorBy<Udon::CanWriter> lift{ Udon::CanWriter<Udon::Message::Motor>{ comBus, 0x001 }, Udon::Direction::Backward };
Udon::MotorBy<Udon::CanWriter> selector{ Udon::CanWriter<Udon::Message::Motor>{ comBus, 0x002 }, Udon::Direction::Backward };


bool isResetting;


void setup()
{
    delay(200);
    Serial.begin(115200);
    pad.begin(Udon::DecodeDipSwitch({ 9, 10, 11, 12 }));
    delay(200);
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
        auto info = pad.getMoveInfo() / 1.5;
        info.turn /= 2;
        tw.moveA(info, 20000);

        int collectPower = pad.getR2().press ? 5000 : pad.getL2().press ? -5000
                                                                        : 0;
        collector.move(collectPower);

        int liftPower = pad.getUp().press ? 130 : pad.getDown().press ? -130
                                                                      : 0;
        lift.move(liftPower);

        int selectPower = pad.getL1().press ? 20 : pad.getR1().press ? -20
                                                                     : 0;
        selector.move(selectPower);
    }
    else
    {
        tw.stop();
        collector.stop();
        lift.stop();
        selector.stop();
    }

    if ((bool)comBus && (bool)motorBus && (bool)pad)
    {
        led.flush();
    }
    else
    {
        led.flush(100);
    }


    Serial.println("\n\n");
    Serial.println("tw-------------------------------");
    tw.show();
    Serial.println();

    Serial.println("pad------------------------------");
    Udon::Showln(pad.getMessage());
    Serial.println();

    loopCtrl.update();
}
