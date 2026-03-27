# Mti_Serial

Minimal Arduino library for Mti

Lightweight, non-blocking parser based on `Stream`, designed for embedded systems.

---

## Quick Example

```cpp
#include <Arduino.h>
#include "Mti_Serial.h"

HardwareSerial SerialMti(2);
Mti_Serial mti(SerialMti);

void setup()
{
    Serial.begin(115200);
    SerialMti.begin(115200, SERIAL_8N1, 33, 32);

    mti.begin();
    mti.set_output_config(10);
    mti.go_to_measurement();
}

void loop()
{
    mti.poll();

    if (mti.has_new_euler())
    {
        auto &e = mti.get_euler();

        Serial.print(e.roll);
        Serial.print(", ");
        Serial.print(e.pitch);
        Serial.print(", ");
        Serial.println(e.yaw);

        mti.clear_new_euler_flag();
    }
}
```

---

## Notes

* Unofficial library
* Designed for simplicity and easy extension

