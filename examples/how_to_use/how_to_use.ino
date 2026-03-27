#include <Arduino.h>
#include <Mti_Serial.h>

// ======================================================
// USER SERIAL CONFIGURATION
// ======================================================

// Example for ESP32:
// change UART number and pins as needed for your board/setup.
HardwareSerial SerialMti(2);

// Create MTi driver instance
Mti_Serial mti(SerialMti);

// Optional: print parser statistics every N milliseconds
static uint32_t last_stats_print_ms = 0;
static const uint32_t STATS_PRINT_PERIOD_MS = 5000;

// ======================================================
// HELPERS
// ======================================================

static void print_result(const char *label, Mti_Serial::Result result)
{
    Serial.print(label);
    Serial.print(": ");

    switch (result)
    {
    case Mti_Serial::RESULT_OK:
        Serial.println("OK");
        break;
    case Mti_Serial::RESULT_TIMEOUT:
        Serial.println("TIMEOUT");
        break;
    case Mti_Serial::RESULT_BAD_ACK:
        Serial.println("BAD_ACK");
        break;
    case Mti_Serial::RESULT_DEVICE_ERROR:
        Serial.println("DEVICE_ERROR");
        break;
    case Mti_Serial::RESULT_INVALID_ARG:
        Serial.println("INVALID_ARG");
        break;
    case Mti_Serial::RESULT_PROTOCOL_ERROR:
        Serial.println("PROTOCOL_ERROR");
        break;
    default:
        Serial.println("UNKNOWN");
        break;
    }
}

static bool configure_mti()
{
    Mti_Serial::Result result;

    result = mti.go_to_config();
    print_result("go_to_config", result);
    if (result != Mti_Serial::RESULT_OK)
        return false;

    // Set same output rate for euler, gyro and acceleration
    result = mti.set_output_config(10);
    print_result("set_output_config", result);
    if (result != Mti_Serial::RESULT_OK)
        return false;

    result = mti.set_filters_profile();
    print_result("set_filters_profile", result);
    if (result != Mti_Serial::RESULT_OK)
        return false;

    result = mti.go_to_measurement();
    print_result("go_to_measurement", result);
    if (result != Mti_Serial::RESULT_OK)
        return false;

    return true;
}

static void print_stats()
{
    const Mti_Serial::ParserStats &stats = mti.get_stats();

    Serial.println();
    Serial.println("===== MTi parser stats =====");
    Serial.print("bytes_rx: ");
    Serial.println(stats.bytes_rx);

    Serial.print("frames_ok: ");
    Serial.println(stats.frames_ok);

    Serial.print("frames_bad_checksum: ");
    Serial.println(stats.frames_bad_checksum);

    Serial.print("frames_bad_length: ");
    Serial.println(stats.frames_bad_length);

    Serial.print("resync_count: ");
    Serial.println(stats.resync_count);

    Serial.print("timeout_count: ");
    Serial.println(stats.timeout_count);

    Serial.print("parser_resets: ");
    Serial.println(stats.parser_resets);

    Serial.print("frame_counter: ");
    Serial.println(stats.frame_counter);

    Serial.print("last_mid: 0x");
    Serial.println(stats.last_mid, HEX);

    Serial.print("last_error_code: 0x");
    Serial.println(stats.last_error_code, HEX);
    Serial.println("============================");
    Serial.println();
}

// ======================================================
// ARDUINO SETUP / LOOP
// ======================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("Mti_Serial example - how_to_use");

    // Example UART configuration for ESP32
    // Change baud rate / RX / TX pins to match your hardware.
    SerialMti.begin(115200, SERIAL_8N1, 33, 32);

    mti.begin();

    if (!configure_mti())
    {
        Serial.println("MTi configuration failed.");
    }
    else
    {
        Serial.println("MTi configured successfully.");
    }
}

void loop()
{
    // Keep parser updated
    mti.poll();

    if (mti.has_new_euler())
    {
        const Mti_Serial::EulerAngles &e = mti.get_euler();

        Serial.print("[EULER] roll=");
        Serial.print(e.roll, 3);
        Serial.print(" pitch=");
        Serial.print(e.pitch, 3);
        Serial.print(" yaw=");
        Serial.print(e.yaw, 3);
        Serial.print(" updated_ms=");
        Serial.println(e.updated_ms);

        mti.clear_new_euler_flag();
    }

    if (mti.has_new_acceleration())
    {
        const Mti_Serial::Acceleration &a = mti.get_acceleration();

        Serial.print("[ACC] x=");
        Serial.print(a.x, 3);
        Serial.print(" y=");
        Serial.print(a.y, 3);
        Serial.print(" z=");
        Serial.print(a.z, 3);
        Serial.print(" updated_ms=");
        Serial.println(a.updated_ms);

        mti.clear_new_acceleration_flag();
    }

    if (mti.has_new_gyro())
    {
        const Mti_Serial::Gyro &g = mti.get_gyro();

        Serial.print("[GYRO] x=");
        Serial.print(g.x, 3);
        Serial.print(" y=");
        Serial.print(g.y, 3);
        Serial.print(" z=");
        Serial.print(g.z, 3);
        Serial.print(" updated_ms=");
        Serial.println(g.updated_ms);

        mti.clear_new_gyro_flag();
    }

    // Print parser stats periodically
    const uint32_t now = millis();
    if ((uint32_t)(now - last_stats_print_ms) >= STATS_PRINT_PERIOD_MS)
    {
        last_stats_print_ms = now;
        print_stats();
    }
}