#include <Udon.hpp>


Udon::LoopCycleController loopCtrl{ 1000 };
Udon::Led                 led{ LED_BUILTIN };
Udon::PicoWDT             wdt;

Udon::CanBusSpi comBus;

constexpr int motorNum = 2;

using Reader = Udon::CanReader<Udon::Message::Motor>;

std::array<Reader, motorNum> readers{ Reader{ comBus, 0x001 },
                                      Reader{ comBus, 0x002 } };

std::array<Udon::Motor3, motorNum> drivers{ Udon::Motor3{ 0, 2, 1 },
                                            Udon::Motor3{ 3, 5, 4 } };

Udon::EncoderPico                       enc{0, 1};
Udon::CanWriter<Udon::Message::Encoder> encWriter{ comBus, 0x003 };

constexpr size_t                       swPin = 0;
Udon::CanWriter<Udon::Message::Switch> swWriter{ comBus, 0x004 };


void setup()
{
    Serial.begin(115200);
    led.begin();
    comBus.begin();
    pinMode(swPin, INPUT_PULLUP);
    enc.begin();

    for (auto& driver : drivers)
    {
        driver.begin();
    }

    delay(50);
}


void loop()
{
    wdt.update();
    comBus.update();

    if ((bool)comBus)
    {
        for (size_t i; i < motorNum; i++)
        {
            drivers[i].move(readers[i].getMessage().value().power);
        }
        led.flush();
    }
    else
    {
        for (auto& driver : drivers)
        {
            driver.stop();
        }
        led.flush(100);
    }

    encWriter.setMessage({ enc.read() });
    swWriter.setMessage({ !digitalRead(swPin) });

    loopCtrl.update();
}
