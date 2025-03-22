#include <Udon.hpp>


Udon::LoopCycleController loopCtrl{ 2000 };
Udon::Led                 led{ LED_BUILTIN };
Udon::PicoWDT             wdt;

Udon::CanBusSpi comBus;

constexpr int motorNum = 2;

using Reader = Udon::CanReader<Udon::Message::Motor>;

std::array<Reader, motorNum> readers{ Reader{ comBus, 0x001 },
                                      Reader{ comBus, 0x002 } };

std::array<Udon::Motor3, motorNum> drivers{ Udon::Motor3{ 8, 10, 9 },
                                            Udon::Motor3{ 15, 21, 14 } };

Udon::EncoderPico                       enc{ 0, 1 };
Udon::CanWriter<Udon::Message::Encoder> encWriter{ comBus, 0x003 };

constexpr size_t                       swPin = 28;
Udon::CanWriter<Udon::Message::Switch> swWriter{ comBus, 0x004 };


void setup()
{
    Serial.begin(115200);
    led.begin();
    comBus.begin();
    pinMode(swPin, INPUT_PULLUP);
    delay(10);
    bool is = enc.begin();
    // if (is) {
    //     Serial.println("encoder faild");
    // delay(10000);
    // }
    // pinMode(0, INPUT_PULLUP);
    // pinMode(1, INPUT_PULLUP);

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
        for (size_t i = 0; i < motorNum; i++)
        {
            auto power = 0;
            if (auto mes = readers[i].getMessage())
            {
                drivers[i].move(power = mes.value().power);
            }
            else
            {
                drivers[i].stop();
            }
            Udon::Printf("%d \t", power);
        }
        led.flush();
    }
    else
    {
        for (auto& driver : drivers)
        {
            driver.stop();
            Udon::Printf("receive error \t");
        }
        led.flush(100);
    }
    Serial.println();

    auto count = enc.read();
    encWriter.setMessage({ count });
    Udon::Printf("encoder %d \n", count);
    // enc.show();
    // Udon::Printf("pin0: %d   pin1: %d", digitalRead(0), digitalRead(1));

    auto sw = !digitalRead(swPin);
    swWriter.setMessage({ sw });
    Udon::Printf("sw %d \n", sw);

    loopCtrl.update();
}
