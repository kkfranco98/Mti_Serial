#pragma once

#include "Mti_Serial_config.h"

class Mti_Serial
{
public:
    // =====================================================
    // XBUS / MTI CONSTANTS
    // =====================================================
    static const uint8_t XBUS_PREAMBLE = 0xFA;
    static const uint8_t XBUS_BID = 0xFF;

    static const uint8_t MID_GO_TO_CONFIG = 0x30;
    static const uint8_t MID_GO_TO_CONFIG_ACK = 0x31;

    static const uint8_t MID_GO_TO_MEASUREMENT = 0x10;
    static const uint8_t MID_GO_TO_MEASUREMENT_ACK = 0x11;

    static const uint8_t MID_SET_OUTPUT_CONFIGURATION = 0xC0;
    static const uint8_t MID_SET_OUTPUT_CONFIGURATION_ACK = 0xC1;

    static const uint8_t MID_MT_DATA2 = 0x36;
    static const uint8_t MID_ERROR = 0x42;

    static const uint8_t MID_SET_FILTER_PROFILE = 0x64;
    static const uint8_t MID_SET_FILTER_PROFILE_ACK = 0x65;

    // =====================================================
    // DATA ID BASES (MTData2)
    // =====================================================
    // Dal datasheet:
    // Euler      = 0x203y
    // Acc        = 0x402y
    // RateOfTurn = 0x802y
    //
    // y = format bits:
    // precision: Float32 = 0x0
    // coord sys: ENU=0x0, NED=0x4, NWU=0x8

    static const uint16_t XDI_EULER_ANGLES_BASE = 0x2030;
    static const uint16_t XDI_ACCELERATION_BASE = 0x4020;
    static const uint16_t XDI_RATE_OF_TURN_BASE = 0x8020;

    enum CoordinateSystem
    {
        COORD_ENU = 0x0,
        COORD_NED = 0x4,
        COORD_NWU = 0x8
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

    enum FilterProfile
    {
        FILTER_PROFILE_GENERAL = 0x32,
        FILTER_PROFILE_HIGH_MAG_DEP = 0x33,
        FILTER_PROFILE_DYNAMIC = 0x34,
        FILTER_PROFILE_NORTH_REFERENCE = 0x35,
        FILTER_PROFILE_VRU_GENERAL = 0x36
    };

    struct Euler
    {
        float roll;
        float pitch;
        float yaw;
        bool valid;
        uint32_t updated_ms;

        Euler() : roll(0.0f), pitch(0.0f), yaw(0.0f), valid(false), updated_ms(0) {}
    };

    struct Acc
    {
        float x;
        float y;
        float z;
        bool valid;
        uint32_t updated_ms;

        Acc() : x(0.0f), y(0.0f), z(0.0f), valid(false), updated_ms(0) {}
    };

    enum AccelerationConvention
    {
        ACC_CONVENTION_SENSOR = 0,
        ACC_CONVENTION_INVERTED
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

    explicit Mti_Serial(Stream &serial);

    void begin();
    void poll();
    bool poll_until_frame(uint32_t timeout_ms);
    void reset_parser();
    void discard_input();

    Result configure_output(uint16_t hz, CoordinateSystem coord,
                            uint32_t timeout_ms = MTI3_UART_DEFAULT_COMMAND_TIMEOUT_MS);

    Result go_to_config(uint32_t timeout_ms = MTI3_UART_DEFAULT_COMMAND_TIMEOUT_MS);
    Result go_to_measurement(uint32_t timeout_ms = MTI3_UART_DEFAULT_COMMAND_TIMEOUT_MS);

    const Euler &get_euler() const;
    Acc get_raw_acc() const;
    Acc get_acc() const;
    const Gyro &get_gyro() const;

    void set_acceleration_convention(AccelerationConvention convention);
    AccelerationConvention get_acceleration_convention() const;

    bool has_new_euler() const;
    bool has_new_acc() const;
    bool has_new_gyro() const;
    bool has_new_packet() const;

    void clear_new_euler_flag();
    void clear_new_acc_flag();
    void clear_new_gyro_flag();
    void clear_new_packet_flag();

    CoordinateSystem get_coordinate_system() const;
    uint16_t get_output_hz() const;

    const XbusFrame &get_last_valid_frame() const;
    const ParserStats &get_stats() const;

    void set_parser_timeout_ms(uint32_t timeout_ms);
    uint32_t get_parser_timeout_ms() const;

    Result set_alignment_rotation(uint8_t frame,
                                  float q0, float q1, float q2, float q3,
                                  uint32_t timeout_ms = MTI3_UART_DEFAULT_COMMAND_TIMEOUT_MS);

    Result set_aircraft_alignment(uint32_t timeout_ms = MTI3_UART_DEFAULT_COMMAND_TIMEOUT_MS);
    Result reset_alignment(uint32_t timeout_ms = MTI3_UART_DEFAULT_COMMAND_TIMEOUT_MS);

    Result set_filter_profile(FilterProfile profile,
                              uint32_t timeout_ms = MTI3_UART_DEFAULT_COMMAND_TIMEOUT_MS);

    Result set_general_filter_profile(uint32_t timeout_ms = MTI3_UART_DEFAULT_COMMAND_TIMEOUT_MS);

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

    Euler _euler;
    Acc _acc;
    Gyro _gyro;

    bool _has_new_euler;
    bool _has_new_acc;
    bool _has_new_gyro;
    bool _has_new_packet;

    bool _frame_ready;

    AccelerationConvention _acc_convention;

    CoordinateSystem _coord_system;
    uint16_t _output_hz;

    ParserStats _stats;

    void process_byte(uint8_t b);
    void on_frame_ready(const XbusFrame &frame);
    void decode_mtdata2(const XbusFrame &frame);

    bool decode_float_be(const uint8_t *src, float &out) const;
    bool decode_float_triplet_be(const uint8_t *src, size_t len, float &a, float &b, float &c) const;

    bool validate_output_hz(uint16_t hz) const;

    Result send_raw_command(const uint8_t *cmd, size_t len);
    Result send_command_and_wait_ack(const uint8_t *cmd, size_t len,
                                     uint8_t expected_mid, uint32_t timeout_ms);
    Result wait_for_mid(uint8_t expected_mid, uint32_t timeout_ms, XbusFrame *out_frame);

    Result build_output_configuration_command(uint16_t hz, CoordinateSystem coord,
                                              uint8_t *out_cmd, size_t out_size, size_t &out_len) const;

    static uint8_t compute_checksum(uint8_t bid, uint8_t mid, uint8_t len, const uint8_t *payload);

    Result set_alignment_rotation_in_config(uint8_t frame,
                                            float q0, float q1, float q2, float q3,
                                            uint32_t timeout_ms);

#if MTI3_UART_ENABLE_FRAME_DEBUG
    void debug_print_frame(const XbusFrame &frame) const;
    void debug_print_decoded_values() const;
#endif
};