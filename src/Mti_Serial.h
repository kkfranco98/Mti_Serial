#pragma once

#include <Arduino.h>
#include "Mti_Serial_config.h"

class Mti_Serial
{
public:
    struct EulerAngles
    {
        float roll;
        float pitch;
        float yaw;
        bool valid;
        uint32_t updated_ms;

        EulerAngles() : roll(0.0f), pitch(0.0f), yaw(0.0f), valid(false), updated_ms(0) {}
    };

    struct Acceleration
    {
        float x;
        float y;
        float z;
        bool valid;
        uint32_t updated_ms;

        Acceleration() : x(0.0f), y(0.0f), z(0.0f), valid(false), updated_ms(0) {}
    };

    struct Gyro
    {
        float x;
        float y;
        float z;
        bool valid;
        uint32_t updated_ms;

        Gyro() : x(0.0f), y(0.0f), z(0.0f), valid(false), updated_ms(0) {}
    };

    struct XbusFrame
    {
        uint8_t mid;
        uint8_t len;
        uint8_t payload[255];
        uint8_t checksum;
        bool valid;
        uint32_t rx_ms;

        XbusFrame() : mid(0), len(0), checksum(0), valid(false), rx_ms(0)
        {
            memset(payload, 0, sizeof(payload));
        }
    };

    struct ParserStats
    {
        uint32_t bytes_rx;
        uint32_t frames_ok;
        uint32_t frames_bad_checksum;
        uint32_t frames_bad_length;
        uint32_t resync_count;
        uint32_t timeout_count;
        uint32_t parser_resets;
        uint32_t frame_counter;
        uint8_t last_mid;
        uint8_t last_error_code;

        ParserStats()
            : bytes_rx(0),
              frames_ok(0),
              frames_bad_checksum(0),
              frames_bad_length(0),
              resync_count(0),
              timeout_count(0),
              parser_resets(0),
              frame_counter(0),
              last_mid(0),
              last_error_code(0)
        {
        }
    };

    enum Result
    {
        RESULT_OK = 0,
        RESULT_TIMEOUT,
        RESULT_BAD_ACK,
        RESULT_DEVICE_ERROR,
        RESULT_INVALID_ARG,
        RESULT_PROTOCOL_ERROR
    };

    explicit Mti_Serial(Stream &serial);

    void begin();
    void reset_parser();
    void poll();

    Result go_to_config(uint32_t timeout_ms = MTI_DEFAULT_COMMAND_TIMEOUT_MS);
    Result go_to_measurement(uint32_t timeout_ms = MTI_DEFAULT_COMMAND_TIMEOUT_MS);

    Result set_output_config(uint16_t hz, uint32_t timeout_ms = MTI_DEFAULT_COMMAND_TIMEOUT_MS);
    Result set_output_config(uint16_t euler_hz, uint16_t gyro_hz, uint16_t acc_hz,
                             uint32_t timeout_ms = MTI_DEFAULT_COMMAND_TIMEOUT_MS);

    Result set_filters_profile(uint32_t timeout_ms = MTI_DEFAULT_COMMAND_TIMEOUT_MS);
    Result start_rep_mode(uint32_t timeout_ms = MTI_DEFAULT_COMMAND_TIMEOUT_MS);
    Result stop_rep_mode(float &calibration, uint32_t timeout_ms = 1000);
    Result store_rep_mode(uint32_t timeout_ms = 1000);

    uint16_t get_euler_output_hz() const;
    uint16_t get_gyro_output_hz() const;
    uint16_t get_acc_output_hz() const;

    void set_default_output_hz(uint16_t hz);
    uint16_t get_default_output_hz() const;

    const EulerAngles &get_euler() const;
    const Acceleration &get_acceleration() const;
    const Gyro &get_gyro() const;

    bool has_new_packet() const;
    void clear_new_packet_flag();

    bool has_new_euler() const;
    void clear_new_euler_flag();

    bool has_new_acceleration() const;
    void clear_new_acceleration_flag();

    bool has_new_gyro() const;
    void clear_new_gyro_flag();

    const XbusFrame &get_last_valid_frame() const;
    const ParserStats &get_stats() const;

    uint8_t get_last_error_code() const;

    void set_parser_timeout_ms(uint32_t timeout_ms);
    uint32_t get_parser_timeout_ms() const;

    void discard_input();

private:
    enum ParserState
    {
        WAIT_PREAMBLE = 0,
        WAIT_BID,
        WAIT_MID,
        WAIT_LEN,
        WAIT_PAYLOAD,
        WAIT_CHECKSUM
    };

    Stream &_serial;

    ParserState _parser_state;
    XbusFrame _building_frame;
    XbusFrame _last_valid_frame;

    size_t _payload_index;
    uint16_t _checksum_accumulator;
    uint32_t _last_rx_ms;
    uint32_t _parser_timeout_ms;

    EulerAngles _euler;
    Acceleration _acc;
    Gyro _gyro;

    ParserStats _stats;

    bool _has_new_packet;
    bool _has_new_euler;
    bool _has_new_acc;
    bool _has_new_gyro;

    uint16_t _default_output_hz;
    uint16_t _euler_output_hz;
    uint16_t _gyro_output_hz;
    uint16_t _acc_output_hz;

    void process_byte(uint8_t b);
    void on_frame_ready(const XbusFrame &frame);

    void decode_mtdata2(const XbusFrame &frame);
    bool decode_float_be(const uint8_t *src, float &out) const;
    bool decode_float_triplet_be(const uint8_t *src, size_t len, float &a, float &b, float &c) const;
    float extract_calibration_from_icc_ack(const XbusFrame &frame) const;

    bool check_angle(float v) const;
    bool check_acceleration(float v) const;
    bool validate_output_hz(uint16_t hz) const;

    Result send_raw_command(const uint8_t *cmd, size_t len);
    Result send_command_and_wait_ack(const uint8_t *cmd, size_t len, uint8_t expected_mid, uint32_t timeout_ms);
    Result wait_for_mid(uint8_t expected_mid, uint32_t timeout_ms, XbusFrame *out_frame);
    Result build_output_config_command(uint16_t euler_hz, uint16_t gyro_hz, uint16_t acc_hz,
                                       uint8_t *out_cmd, size_t out_cmd_size, size_t &out_len) const;

    static uint8_t compute_checksum(uint8_t bid, uint8_t mid, uint8_t len, const uint8_t *payload);
};