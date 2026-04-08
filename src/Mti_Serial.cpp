#include "Mti_Serial.h"

#include <math.h>
#include <string.h>

Mti_Serial::Mti_Serial(Stream &serial)
    : _serial(serial),
      _parser_state(WAIT_PREAMBLE),
      _payload_index(0),
      _checksum_accumulator(0),
      _last_rx_ms(0),
      _parser_timeout_ms(MTI3_UART_DEFAULT_PARSER_TIMEOUT_MS),
      _has_new_euler(false),
      _has_new_acc(false),
      _has_new_gyro(false),
      _has_new_packet(false),
      _frame_ready(false),
      _coord_system(COORD_ENU),
      _output_hz(MTI3_UART_DEFAULT_OUTPUT_HZ),
      _acc_convention(ACC_CONVENTION_SENSOR)
{
}

void Mti_Serial::begin()
{
    _euler = Euler();
    _acc = Acc();
    _gyro = Gyro();

    _has_new_euler = false;
    _has_new_acc = false;
    _has_new_gyro = false;
    _has_new_packet = false;
    _frame_ready = false;

    _coord_system = COORD_ENU;
    _output_hz = MTI3_UART_DEFAULT_OUTPUT_HZ;
    _acc_convention = ACC_CONVENTION_SENSOR;

    _stats = ParserStats();
    _last_valid_frame = XbusFrame();

    reset_parser();
}

void Mti_Serial::reset_parser()
{
    _parser_state = WAIT_PREAMBLE;
    _building_frame = XbusFrame();
    _payload_index = 0;
    _checksum_accumulator = 0;
    _last_rx_ms = millis();
    _stats.parser_resets++;
}

void Mti_Serial::poll()
{
    const uint32_t now = millis();

    if (_parser_state != WAIT_PREAMBLE)
    {
        if ((uint32_t)(now - _last_rx_ms) > _parser_timeout_ms)
        {
            _stats.timeout_count++;
            reset_parser();
        }
    }

    while (_serial.available() > 0)
    {
        const int raw = _serial.read();
        if (raw < 0)
            break;

        _last_rx_ms = millis();
        _stats.bytes_rx++;

        process_byte((uint8_t)raw);
    }
}

bool Mti_Serial::poll_until_frame(uint32_t timeout_ms)
{
    const uint32_t start_ms = millis();
    _frame_ready = false;

    while ((uint32_t)(millis() - start_ms) < timeout_ms)
    {
        const uint32_t now = millis();

        if (_parser_state != WAIT_PREAMBLE)
        {
            if ((uint32_t)(now - _last_rx_ms) > _parser_timeout_ms)
            {
                _stats.timeout_count++;
                reset_parser();
            }
        }

        while (_serial.available() > 0)
        {
            const int raw = _serial.read();
            if (raw < 0)
                break;

            _last_rx_ms = millis();
            _stats.bytes_rx++;

            process_byte((uint8_t)raw);

            if (_frame_ready)
                return true;
        }

        delay(1);
    }

    return false;
}

void Mti_Serial::process_byte(uint8_t b)
{
    switch (_parser_state)
    {
    case WAIT_PREAMBLE:
        if (b == XBUS_PREAMBLE)
        {
            _parser_state = WAIT_BID;
        }
        break;

    case WAIT_BID:
        if (b == XBUS_BID)
        {
            _checksum_accumulator = b;
            _parser_state = WAIT_MID;
        }
        else if (b == XBUS_PREAMBLE)
        {
            _stats.resync_count++;
            _parser_state = WAIT_BID;
        }
        else
        {
            _stats.resync_count++;
            _parser_state = WAIT_PREAMBLE;
        }
        break;

    case WAIT_MID:
        _building_frame.mid = b;
        _checksum_accumulator += b;
        _parser_state = WAIT_LEN;
        break;

    case WAIT_LEN:
        _building_frame.len = b;
        _checksum_accumulator += b;
        _payload_index = 0;

        if (_building_frame.len == 0)
        {
            _parser_state = WAIT_CHECKSUM;
        }
        else
        {
            _parser_state = WAIT_PAYLOAD;
        }
        break;

    case WAIT_PAYLOAD:
        if (_payload_index >= sizeof(_building_frame.payload))
        {
            _stats.frames_bad_length++;
            reset_parser();
            return;
        }

        _building_frame.payload[_payload_index++] = b;
        _checksum_accumulator += b;

        if (_payload_index >= _building_frame.len)
        {
            _parser_state = WAIT_CHECKSUM;
        }
        break;

    case WAIT_CHECKSUM:
    {
        _building_frame.checksum = b;
        _building_frame.rx_ms = millis();

        const uint8_t expected = (uint8_t)(0x100 - (_checksum_accumulator & 0xFF));

        if (_building_frame.checksum == expected)
        {
            _building_frame.valid = true;
            _stats.frames_ok++;
            _stats.frame_counter++;
            _stats.last_mid = _building_frame.mid;

            on_frame_ready(_building_frame);
        }
        else
        {
            _stats.frames_bad_checksum++;
        }

        reset_parser();
        break;
    }

    default:
        reset_parser();
        break;
    }
}

void Mti_Serial::on_frame_ready(const XbusFrame &frame)
{
    _last_valid_frame = frame;
    _has_new_packet = true;
    _frame_ready = true;

#if MTI3_UART_ENABLE_FRAME_DEBUG
    debug_print_frame(frame);
#endif

    if (frame.mid == MID_MT_DATA2)
    {
        decode_mtdata2(frame);

#if MTI3_UART_ENABLE_FRAME_DEBUG
        debug_print_decoded_values();
#endif
    }
    else if (frame.mid == MID_ERROR)
    {
        if (frame.len >= 1)
        {
            _stats.last_error_code = frame.payload[0];
        }
    }
}

void Mti_Serial::decode_mtdata2(const XbusFrame &frame)
{
    size_t index = 0;
    const uint32_t now = millis();

    while ((index + 3U) <= frame.len)
    {
        const uint8_t data_id_hi = frame.payload[index++];
        const uint8_t data_id_lo = frame.payload[index++];
        const uint8_t data_len = frame.payload[index++];

        if ((index + data_len) > frame.len)
        {
            _stats.frames_bad_length++;
            return;
        }

        const uint16_t data_id = ((uint16_t)data_id_hi << 8) | data_id_lo;
        const uint8_t *data = &frame.payload[index];

        switch (data_id & 0xFFF0)
        {
        case XDI_EULER_ANGLES_BASE:
        {
            if (data_len == 12)
            {
                float roll = 0.0f;
                float pitch = 0.0f;
                float yaw = 0.0f;

                if (decode_float_triplet_be(data, data_len, roll, pitch, yaw))
                {
                    _euler.roll = roll;
                    _euler.pitch = pitch;
                    _euler.yaw = yaw;
                    _euler.valid = true;
                    _euler.updated_ms = now;
                    _has_new_euler = true;
                }
            }
            break;
        }

        case XDI_ACCELERATION_BASE:
        {
            if (data_len == 12)
            {
                float x = 0.0f;
                float y = 0.0f;
                float z = 0.0f;

                if (decode_float_triplet_be(data, data_len, x, y, z))
                {
                    _acc.x = x;
                    _acc.y = y;
                    _acc.z = z;
                    _acc.valid = true;
                    _acc.updated_ms = now;
                    _has_new_acc = true;
                }
            }
            break;
        }

        case XDI_RATE_OF_TURN_BASE:
        {
            if (data_len == 12)
            {
                float x = 0.0f;
                float y = 0.0f;
                float z = 0.0f;

                if (decode_float_triplet_be(data, data_len, x, y, z))
                {
                    _gyro.x = x;
                    _gyro.y = y;
                    _gyro.z = z;
                    _gyro.valid = true;
                    _gyro.updated_ms = now;
                    _has_new_gyro = true;
                }
            }
            break;
        }

        default:
            break;
        }

        index += data_len;
    }
}

bool Mti_Serial::decode_float_be(const uint8_t *src, float &out) const
{
    if (!src)
        return false;

    uint8_t tmp[4];
    tmp[0] = src[3];
    tmp[1] = src[2];
    tmp[2] = src[1];
    tmp[3] = src[0];

    memcpy(&out, tmp, sizeof(float));
    return true;
}

bool Mti_Serial::decode_float_triplet_be(const uint8_t *src, size_t len,
                                       float &a, float &b, float &c) const
{
    if (!src || len < 12)
        return false;

    return decode_float_be(src + 0, a) &&
           decode_float_be(src + 4, b) &&
           decode_float_be(src + 8, c);
}

bool Mti_Serial::validate_output_hz(uint16_t hz) const
{
    return (hz > 0U) && (hz <= MTI3_UART_MAX_OUTPUT_HZ);
}

Mti_Serial::Result Mti_Serial::send_raw_command(const uint8_t *cmd, size_t len)
{
    if (!cmd || len == 0)
        return RESULT_INVALID_ARG;

    const size_t written = _serial.write(cmd, len);
    if (written != len)
        return RESULT_PROTOCOL_ERROR;

    _serial.flush();
    return RESULT_OK;
}

Mti_Serial::Result Mti_Serial::wait_for_mid(uint8_t expected_mid, uint32_t timeout_ms, XbusFrame *out_frame)
{
    const uint32_t start_ms = millis();

    while ((uint32_t)(millis() - start_ms) < timeout_ms)
    {
        const uint32_t remaining = timeout_ms - (uint32_t)(millis() - start_ms);

        if (!poll_until_frame(remaining))
            break;

        _frame_ready = false;

        if (_last_valid_frame.mid == expected_mid)
        {
            if (out_frame)
                *out_frame = _last_valid_frame;
            return RESULT_OK;
        }

        if (_last_valid_frame.mid == MID_ERROR)
        {
            if (out_frame)
                *out_frame = _last_valid_frame;
            return RESULT_DEVICE_ERROR;
        }

        // frame valido ma non quello atteso: ignoralo e continua
    }

    _stats.timeout_count++;
    return RESULT_TIMEOUT;
}

Mti_Serial::Result Mti_Serial::send_command_and_wait_ack(const uint8_t *cmd, size_t len,
                                                     uint8_t expected_mid, uint32_t timeout_ms)
{
    discard_input();
    reset_parser();

    const Result tx = send_raw_command(cmd, len);
    if (tx != RESULT_OK)
        return tx;

    return wait_for_mid(expected_mid, timeout_ms, NULL);
}

Mti_Serial::Result Mti_Serial::go_to_config(uint32_t timeout_ms)
{
    static const uint8_t cmd[] = {
        XBUS_PREAMBLE, XBUS_BID, MID_GO_TO_CONFIG, 0x00, 0xD1};

    return send_command_and_wait_ack(cmd, sizeof(cmd), MID_GO_TO_CONFIG_ACK, timeout_ms);
}

Mti_Serial::Result Mti_Serial::go_to_measurement(uint32_t timeout_ms)
{
    static const uint8_t cmd[] = {
        XBUS_PREAMBLE, XBUS_BID, MID_GO_TO_MEASUREMENT, 0x00, 0xF1};

    return send_command_and_wait_ack(cmd, sizeof(cmd), MID_GO_TO_MEASUREMENT_ACK, timeout_ms);
}

Mti_Serial::Result Mti_Serial::set_alignment_rotation_in_config(
    uint8_t frame,
    float q0, float q1, float q2, float q3,
    uint32_t timeout_ms)
{
    if (frame > 1)
        return RESULT_INVALID_ARG;

    static const size_t CMD_LEN = 22;
    uint8_t cmd[CMD_LEN];

    cmd[0] = XBUS_PREAMBLE;
    cmd[1] = XBUS_BID;
    cmd[2] = 0xEC; // SetAlignmentRotation
    cmd[3] = 0x11; // payload len = 17

    cmd[4] = frame;

    float_to_be(q0, &cmd[5]);
    float_to_be(q1, &cmd[9]);
    float_to_be(q2, &cmd[13]);
    float_to_be(q3, &cmd[17]);

    cmd[21] = compute_checksum(XBUS_BID, cmd[2], cmd[3], &cmd[4]);

    return send_command_and_wait_ack(cmd, CMD_LEN, 0xED, timeout_ms);
}

Mti_Serial::Result Mti_Serial::build_output_configuration_command(uint16_t hz, CoordinateSystem coord,
                                                              uint8_t *out_cmd, size_t out_size, size_t &out_len) const
{
    if (!out_cmd)
        return RESULT_INVALID_ARG;

    if (!validate_output_hz(hz))
        return RESULT_INVALID_ARG;

    // 3 entry * 4 byte = 12 payload bytes
    // header(4) + payload(12) + checksum(1) = 17
    if (out_size < 17U)
        return RESULT_INVALID_ARG;

    const uint16_t euler_id = (uint16_t)(XDI_EULER_ANGLES_BASE | (uint16_t)coord);

    // Dal datasheet questi sono sensor-fixed
    const uint16_t acc_id = XDI_ACCELERATION_BASE;
    const uint16_t gyro_id = XDI_RATE_OF_TURN_BASE;

    out_cmd[0] = XBUS_PREAMBLE;
    out_cmd[1] = XBUS_BID;
    out_cmd[2] = MID_SET_OUTPUT_CONFIGURATION;
    out_cmd[3] = 0x0C;

    // Euler
    out_cmd[4] = (uint8_t)((euler_id >> 8) & 0xFF);
    out_cmd[5] = (uint8_t)(euler_id & 0xFF);
    out_cmd[6] = (uint8_t)((hz >> 8) & 0xFF);
    out_cmd[7] = (uint8_t)(hz & 0xFF);

    // Acc
    out_cmd[8] = (uint8_t)((acc_id >> 8) & 0xFF);
    out_cmd[9] = (uint8_t)(acc_id & 0xFF);
    out_cmd[10] = (uint8_t)((hz >> 8) & 0xFF);
    out_cmd[11] = (uint8_t)(hz & 0xFF);

    // Gyro
    out_cmd[12] = (uint8_t)((gyro_id >> 8) & 0xFF);
    out_cmd[13] = (uint8_t)(gyro_id & 0xFF);
    out_cmd[14] = (uint8_t)((hz >> 8) & 0xFF);
    out_cmd[15] = (uint8_t)(hz & 0xFF);

    out_cmd[16] = compute_checksum(XBUS_BID,
                                   MID_SET_OUTPUT_CONFIGURATION,
                                   0x0C,
                                   &out_cmd[4]);

    out_len = 17U;
    return RESULT_OK;
}

Mti_Serial::Result Mti_Serial::configure_output(uint16_t hz, CoordinateSystem coord, uint32_t timeout_ms)
{
    uint8_t cmd[17];
    size_t cmd_len = 0;

    Result r = go_to_config(timeout_ms);
    if (r != RESULT_OK)
        return r;

    r = build_output_configuration_command(hz, coord, cmd, sizeof(cmd), cmd_len);
    if (r != RESULT_OK)
        return r;

    r = send_command_and_wait_ack(cmd, cmd_len, MID_SET_OUTPUT_CONFIGURATION_ACK, timeout_ms);
    if (r != RESULT_OK)
        return r;

    _coord_system = coord;
    _output_hz = hz;

    return go_to_measurement(timeout_ms);
}

Mti_Serial::Result Mti_Serial::set_alignment_rotation(
    uint8_t frame,
    float q0, float q1, float q2, float q3,
    uint32_t timeout_ms)
{
    Result r = go_to_config(timeout_ms);
    if (r != RESULT_OK)
        return r;

    r = set_alignment_rotation_in_config(frame, q0, q1, q2, q3, timeout_ms);
    if (r != RESULT_OK)
        return r;

    return go_to_measurement(timeout_ms);
}

Mti_Serial::Result Mti_Serial::set_aircraft_alignment(uint32_t timeout_ms)
{
    const float q0 = 0.0f;
    const float q1 = -0.70710677f;
    const float q2 = -0.70710677f;
    const float q3 = 0.0f;

    Result r = go_to_config(timeout_ms);
    if (r != RESULT_OK)
        return r;

    r = set_alignment_rotation_in_config(1, q0, q1, q2, q3, timeout_ms); // RotSensor
    if (r != RESULT_OK)
        return r;

    r = set_alignment_rotation_in_config(0, q0, q1, q2, q3, timeout_ms); // RotLocal
    if (r != RESULT_OK)
        return r;

    return go_to_measurement(timeout_ms);
}

Mti_Serial::Result Mti_Serial::reset_alignment(uint32_t timeout_ms)
{
    Result r;

    r = set_alignment_rotation(1, 1.0f, 0.0f, 0.0f, 0.0f, timeout_ms);
    if (r != RESULT_OK)
        return r;

    r = set_alignment_rotation(0, 1.0f, 0.0f, 0.0f, 0.0f, timeout_ms);
    if (r != RESULT_OK)
        return r;

    return RESULT_OK;
}

Mti_Serial::Result Mti_Serial::set_filter_profile(FilterProfile profile, uint32_t timeout_ms)
{
    uint8_t cmd[7];

    cmd[0] = XBUS_PREAMBLE;
    cmd[1] = XBUS_BID;
    cmd[2] = MID_SET_FILTER_PROFILE;
    cmd[3] = 0x02;             // payload length
    cmd[4] = 0x00;             // MSB
    cmd[5] = (uint8_t)profile; // LSB
    cmd[6] = compute_checksum(XBUS_BID, cmd[2], cmd[3], &cmd[4]);

    Result r = go_to_config(timeout_ms);
    if (r != RESULT_OK)
        return r;

    r = send_command_and_wait_ack(cmd, sizeof(cmd), MID_SET_FILTER_PROFILE_ACK, timeout_ms);
    if (r != RESULT_OK)
        return r;

    return go_to_measurement(timeout_ms);
}

Mti_Serial::Result Mti_Serial::set_general_filter_profile(uint32_t timeout_ms)
{
    return set_filter_profile(FILTER_PROFILE_GENERAL, timeout_ms);
}

const Mti_Serial::Euler &Mti_Serial::get_euler() const { return _euler; }

Mti_Serial::Acc Mti_Serial::get_raw_acc() const { return _acc; }

Mti_Serial::Acc Mti_Serial::get_acc() const
{
    Acc out = _acc;

    if (_acc_convention == ACC_CONVENTION_INVERTED)
    {
        out.x = -out.x;
        out.y = -out.y;
        out.z = -out.z;
    }

    return out;
}

void Mti_Serial::set_acceleration_convention(AccelerationConvention convention) { _acc_convention = convention; }

Mti_Serial::AccelerationConvention Mti_Serial::get_acceleration_convention() const { return _acc_convention; }

const Mti_Serial::Gyro &Mti_Serial::get_gyro() const { return _gyro; }

bool Mti_Serial::has_new_euler() const { return _has_new_euler; }
bool Mti_Serial::has_new_acc() const { return _has_new_acc; }
bool Mti_Serial::has_new_gyro() const { return _has_new_gyro; }

bool Mti_Serial::has_new_packet() const { return _has_new_packet; }
void Mti_Serial::clear_new_euler_flag() { _has_new_euler = false; }
void Mti_Serial::clear_new_acc_flag() { _has_new_acc = false; }

void Mti_Serial::clear_new_gyro_flag() { _has_new_gyro = false; }

void Mti_Serial::clear_new_packet_flag() { _has_new_packet = false; }

Mti_Serial::CoordinateSystem Mti_Serial::get_coordinate_system() const { return _coord_system; }

uint16_t Mti_Serial::get_output_hz() const { return _output_hz; }

const Mti_Serial::XbusFrame &Mti_Serial::get_last_valid_frame() const { return _last_valid_frame; }

const Mti_Serial::ParserStats &Mti_Serial::get_stats() const { return _stats; }

void Mti_Serial::set_parser_timeout_ms(uint32_t timeout_ms) { _parser_timeout_ms = timeout_ms; }

uint32_t Mti_Serial::get_parser_timeout_ms() const { return _parser_timeout_ms; }

void Mti_Serial::discard_input()
{
    while (_serial.available() > 0)
    {
        _serial.read();
    }
}

uint8_t Mti_Serial::compute_checksum(uint8_t bid, uint8_t mid, uint8_t len, const uint8_t *payload)
{
    uint16_t sum = 0;
    sum += bid;
    sum += mid;
    sum += len;

    for (uint8_t i = 0; i < len; ++i)
    {
        sum += payload[i];
    }

    return (uint8_t)(0x100 - (sum & 0xFF));
}

#if MTI3_UART_ENABLE_FRAME_DEBUG
void Mti_Serial::debug_print_frame(const XbusFrame &frame) const
{
    Serial.print(F("[MTI] Frame OK | MID=0x"));
    if (frame.mid < 16)
        Serial.print('0');
    Serial.print(frame.mid, HEX);
    Serial.print(F(" LEN="));
    Serial.print(frame.len);
    Serial.print(F(" DATA="));

    for (uint8_t i = 0; i < frame.len; ++i)
    {
        if (frame.payload[i] < 16)
            Serial.print('0');
        Serial.print(frame.payload[i], HEX);
        Serial.print(' ');
    }

    Serial.println();
}

void Mti_Serial::debug_print_decoded_values() const
{
    if (_euler.valid)
    {
        Serial.print(F("[MTI] Euler  -> roll="));
        Serial.print(_euler.roll, 6);
        Serial.print(F(" pitch="));
        Serial.print(_euler.pitch, 6);
        Serial.print(F(" yaw="));
        Serial.println(_euler.yaw, 6);
    }

    if (_acc.valid)
    {
        Serial.print(F("[MTI] Acc    -> x=   "));
        Serial.print(_acc.x, 6);
        Serial.print(F(" y=   "));
        Serial.print(_acc.y, 6);
        Serial.print(F(" z=   "));
        Serial.println(_acc.z, 6);
    }

    if (_gyro.valid)
    {
        Serial.print(F("[MTI] Gyro   -> x=   "));
        Serial.print(_gyro.x, 6);
        Serial.print(F(" y=   "));
        Serial.print(_gyro.y, 6);
        Serial.print(F(" z=   "));
        Serial.println(_gyro.z, 6);
    }
}
#endif