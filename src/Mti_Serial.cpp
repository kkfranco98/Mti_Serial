#include "Mti_Serial.h"

#include <math.h>
#include <string.h>

Mti_Serial::Mti_Serial(Stream &serial)
    : _serial(serial),
      _parser_state(WAIT_PREAMBLE),
      _payload_index(0),
      _checksum_accumulator(0),
      _last_rx_ms(0),
      _parser_timeout_ms(MTI_DEFAULT_PARSER_TIMEOUT_MS),
      _has_new_packet(false),
      _has_new_euler(false),
      _has_new_acc(false),
      _has_new_gyro(false),
      _default_output_hz(MTI_DEFAULT_OUTPUT_HZ),
      _euler_output_hz(MTI_DEFAULT_OUTPUT_HZ),
      _gyro_output_hz(MTI_DEFAULT_OUTPUT_HZ),
      _acc_output_hz(MTI_DEFAULT_OUTPUT_HZ)
{
}

void Mti_Serial::begin()
{
    _euler = EulerAngles();
    _acc = Acceleration();
    _gyro = Gyro();

    _stats = ParserStats();

    _has_new_packet = false;
    _has_new_euler = false;
    _has_new_acc = false;
    _has_new_gyro = false;

    _default_output_hz = MTI_DEFAULT_OUTPUT_HZ;
    _euler_output_hz = _default_output_hz;
    _gyro_output_hz = _default_output_hz;
    _acc_output_hz = _default_output_hz;

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

    if (frame.mid == MID_MT_DATA2)
    {
        decode_mtdata2(frame);
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
        const uint8_t data_type = frame.payload[index++];
        const uint8_t data_format = frame.payload[index++];
        const uint8_t data_len = frame.payload[index++];

        if ((index + data_len) > frame.len)
        {
            _stats.frames_bad_length++;
            return;
        }

        const uint8_t *data = &frame.payload[index];

        switch (data_type)
        {
        case EULER_MESSAGE:
            if (data_format == EULER_FRAME && data_len == 12)
            {
                float roll = 0.0f;
                float pitch = 0.0f;
                float yaw = 0.0f;

                if (decode_float_triplet_be(data, data_len, roll, pitch, yaw))
                {
                    if (check_angle(roll) && check_angle(pitch) && check_angle(yaw))
                    {
                        _euler.roll = roll;
                        _euler.pitch = pitch;
                        _euler.yaw = yaw;
                        _euler.valid = true;
                        _euler.updated_ms = now;
                        _has_new_euler = true;
                    }
                }
            }
            break;

        case GYRO_MESSAGE:
            if (data_format == GYRO_FRAME && data_len == 12)
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

        case ACC_MESSAGE:
            if (data_format == ACC_FRAME && data_len == 12)
            {
                float x = 0.0f;
                float y = 0.0f;
                float z = 0.0f;

                if (decode_float_triplet_be(data, data_len, x, y, z))
                {
                    if (check_acceleration(x) && check_acceleration(y) && check_acceleration(z))
                    {
                        _acc.x = x;
                        _acc.y = y;
                        _acc.z = z;
                        _acc.valid = true;
                        _acc.updated_ms = now;
                        _has_new_acc = true;
                    }
                }
            }
            break;

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

bool Mti_Serial::decode_float_triplet_be(const uint8_t *src, size_t len, float &a, float &b, float &c) const
{
    if (!src || len < 12)
        return false;

    return decode_float_be(src + 0, a) &&
           decode_float_be(src + 4, b) &&
           decode_float_be(src + 8, c);
}

float Mti_Serial::extract_calibration_from_icc_ack(const XbusFrame &frame) const
{
    if (frame.mid != MID_ICC_COMMAND_ACK)
        return 0.0f;

    if (frame.len < 7)
        return 0.0f;

    float out = 0.0f;
    decode_float_be(&frame.payload[3], out);
    return out;
}

bool Mti_Serial::check_angle(float v) const
{
    return isfinite(v) && fabsf(v) <= 180.0f;
}

bool Mti_Serial::check_acceleration(float v) const
{
    return isfinite(v) && fabsf(v) <= 98.1f;
}

bool Mti_Serial::validate_output_hz(uint16_t hz) const
{
    return (hz > 0U) && (hz <= MTI_MAX_SUPPORTED_OUTPUT_HZ);
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
    const uint32_t start_frame_counter = _stats.frame_counter;

    while ((uint32_t)(millis() - start_ms) < timeout_ms)
    {
        poll();

        if (_stats.frame_counter != start_frame_counter)
        {
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
        }

        delay(1);
    }

    _stats.timeout_count++;
    return RESULT_TIMEOUT;
}

Mti_Serial::Result Mti_Serial::send_command_and_wait_ack(const uint8_t *cmd, size_t len, uint8_t expected_mid, uint32_t timeout_ms)
{
    const Result tx = send_raw_command(cmd, len);
    if (tx != RESULT_OK)
        return tx;

    return wait_for_mid(expected_mid, timeout_ms, NULL);
}

Mti_Serial::Result Mti_Serial::build_output_config_command(uint16_t euler_hz, uint16_t gyro_hz, uint16_t acc_hz,
                                                           uint8_t *out_cmd, size_t out_cmd_size, size_t &out_len) const
{
    if (!out_cmd)
        return RESULT_INVALID_ARG;

    if (out_cmd_size < 17U)
        return RESULT_INVALID_ARG;

    if (!validate_output_hz(euler_hz) || !validate_output_hz(gyro_hz) || !validate_output_hz(acc_hz))
        return RESULT_INVALID_ARG;

    out_cmd[0] = XBUS_PREAMBLE;
    out_cmd[1] = XBUS_BID;
    out_cmd[2] = MID_OUTPUT_CONFIG;
    out_cmd[3] = 0x0C;

    out_cmd[4] = EULER_MESSAGE;
    out_cmd[5] = EULER_FRAME;
    out_cmd[6] = (uint8_t)((euler_hz >> 8) & 0xFF);
    out_cmd[7] = (uint8_t)(euler_hz & 0xFF);

    out_cmd[8] = GYRO_MESSAGE;
    out_cmd[9] = GYRO_FRAME;
    out_cmd[10] = (uint8_t)((gyro_hz >> 8) & 0xFF);
    out_cmd[11] = (uint8_t)(gyro_hz & 0xFF);

    out_cmd[12] = ACC_MESSAGE;
    out_cmd[13] = ACC_FRAME;
    out_cmd[14] = (uint8_t)((acc_hz >> 8) & 0xFF);
    out_cmd[15] = (uint8_t)(acc_hz & 0xFF);

    out_cmd[16] = compute_checksum(XBUS_BID, MID_OUTPUT_CONFIG, 0x0C, &out_cmd[4]);
    out_len = 17U;

    return RESULT_OK;
}

Mti_Serial::Result Mti_Serial::go_to_config(uint32_t timeout_ms)
{
    return send_command_and_wait_ack(
        config_mode_command,
        sizeof(config_mode_command),
        MID_GO_TO_CONFIG_ACK,
        timeout_ms);
}

Mti_Serial::Result Mti_Serial::go_to_measurement(uint32_t timeout_ms)
{
    return send_command_and_wait_ack(
        go_meas_command,
        sizeof(go_meas_command),
        MID_GO_TO_MEASUREMENT_ACK,
        timeout_ms);
}

Mti_Serial::Result Mti_Serial::set_output_config(uint16_t hz, uint32_t timeout_ms)
{
    return set_output_config(hz, hz, hz, timeout_ms);
}

Mti_Serial::Result Mti_Serial::set_output_config(uint16_t euler_hz, uint16_t gyro_hz, uint16_t acc_hz,
                                                 uint32_t timeout_ms)
{
    uint8_t cmd[17];
    size_t cmd_len = 0;

    Result r = go_to_config(timeout_ms);
    if (r != RESULT_OK)
        return r;

    r = build_output_config_command(euler_hz, gyro_hz, acc_hz, cmd, sizeof(cmd), cmd_len);
    if (r != RESULT_OK)
        return r;

    r = send_command_and_wait_ack(cmd, cmd_len, MID_OUTPUT_CONFIG_ACK, timeout_ms);
    if (r != RESULT_OK)
        return r;

    _euler_output_hz = euler_hz;
    _gyro_output_hz = gyro_hz;
    _acc_output_hz = acc_hz;

    return RESULT_OK;
}

Mti_Serial::Result Mti_Serial::set_filters_profile(uint32_t timeout_ms)
{
    Result r = go_to_config(timeout_ms);
    if (r != RESULT_OK)
        return r;

    r = send_command_and_wait_ack(
        clear_option,
        sizeof(clear_option),
        MID_SET_OPTION_FLAGS_ACK,
        timeout_ms);
    if (r != RESULT_OK)
        return r;

#if (ENABLE_IN_RUN_COMPASS_CALIBRATION_SERIAL_CONFIG == true)
    r = send_command_and_wait_ack(
        in_run_compass_calibration,
        sizeof(in_run_compass_calibration),
        MID_SET_OPTION_FLAGS_ACK,
        timeout_ms);
    if (r != RESULT_OK)
        return r;
#endif

    return send_command_and_wait_ack(
        filter_profile_config,
        sizeof(filter_profile_config),
        MID_SET_FILTER_PROFILE_ACK,
        timeout_ms);
}

Mti_Serial::Result Mti_Serial::start_rep_mode(uint32_t timeout_ms)
{
    return send_command_and_wait_ack(
        start_rep_mode_command,
        sizeof(start_rep_mode_command),
        MID_ICC_COMMAND_ACK,
        timeout_ms);
}

Mti_Serial::Result Mti_Serial::stop_rep_mode(float &calibration, uint32_t timeout_ms)
{
    calibration = 0.0f;

    const Result tx = send_raw_command(stop_rep_mode_command, sizeof(stop_rep_mode_command));
    if (tx != RESULT_OK)
        return tx;

    XbusFrame ack_frame;
    const Result rx = wait_for_mid(MID_ICC_COMMAND_ACK, timeout_ms, &ack_frame);
    if (rx != RESULT_OK)
        return rx;

    calibration = extract_calibration_from_icc_ack(ack_frame);
    return RESULT_OK;
}

Mti_Serial::Result Mti_Serial::store_rep_mode(uint32_t timeout_ms)
{
    Result r = go_to_config(timeout_ms);
    if (r != RESULT_OK)
        return r;

    r = send_command_and_wait_ack(
        store_data_command,
        sizeof(store_data_command),
        MID_ICC_COMMAND_ACK,
        timeout_ms);
    if (r != RESULT_OK)
        return r;

    return go_to_measurement(timeout_ms);
}

uint16_t Mti_Serial::get_euler_output_hz() const
{
    return _euler_output_hz;
}

uint16_t Mti_Serial::get_gyro_output_hz() const
{
    return _gyro_output_hz;
}

uint16_t Mti_Serial::get_acc_output_hz() const
{
    return _acc_output_hz;
}

void Mti_Serial::set_default_output_hz(uint16_t hz)
{
    if (validate_output_hz(hz))
    {
        _default_output_hz = hz;
    }
}

uint16_t Mti_Serial::get_default_output_hz() const
{
    return _default_output_hz;
}

const Mti_Serial::EulerAngles &Mti_Serial::get_euler() const
{
    return _euler;
}

const Mti_Serial::Acceleration &Mti_Serial::get_acceleration() const
{
    return _acc;
}

const Mti_Serial::Gyro &Mti_Serial::get_gyro() const
{
    return _gyro;
}

bool Mti_Serial::has_new_packet() const
{
    return _has_new_packet;
}

void Mti_Serial::clear_new_packet_flag()
{
    _has_new_packet = false;
}

bool Mti_Serial::has_new_euler() const
{
    return _has_new_euler;
}

void Mti_Serial::clear_new_euler_flag()
{
    _has_new_euler = false;
}

bool Mti_Serial::has_new_acceleration() const
{
    return _has_new_acc;
}

void Mti_Serial::clear_new_acceleration_flag()
{
    _has_new_acc = false;
}

bool Mti_Serial::has_new_gyro() const
{
    return _has_new_gyro;
}

void Mti_Serial::clear_new_gyro_flag()
{
    _has_new_gyro = false;
}

const Mti_Serial::XbusFrame &Mti_Serial::get_last_valid_frame() const
{
    return _last_valid_frame;
}

const Mti_Serial::ParserStats &Mti_Serial::get_stats() const
{
    return _stats;
}

uint8_t Mti_Serial::get_last_error_code() const
{
    return _stats.last_error_code;
}

void Mti_Serial::set_parser_timeout_ms(uint32_t timeout_ms)
{
    _parser_timeout_ms = timeout_ms;
}

uint32_t Mti_Serial::get_parser_timeout_ms() const
{
    return _parser_timeout_ms;
}

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

    for (uint8_t i = 0; i < len; i++)
    {
        sum += payload[i];
    }

    return (uint8_t)(0x100 - (sum & 0xFF));
}