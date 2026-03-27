#pragma once

#include <Arduino.h>

// ======================================================
// USER CONFIG
// ======================================================

#define FILTER_PROFILE_GENERAL 1
#define FILTER_PROFILE_MAG 2

#ifndef ENABLE_IN_RUN_COMPASS_CALIBRATION_SERIAL_CONFIG
#define ENABLE_IN_RUN_COMPASS_CALIBRATION_SERIAL_CONFIG false
#endif

#ifndef SET_FILTER_PROFILE_SERIAL_CONFIG
#define SET_FILTER_PROFILE_SERIAL_CONFIG FILTER_PROFILE_GENERAL
#endif

#ifndef MTI_DEFAULT_PARSER_TIMEOUT_MS
#define MTI_DEFAULT_PARSER_TIMEOUT_MS 100
#endif

#ifndef MTI_DEFAULT_COMMAND_TIMEOUT_MS
#define MTI_DEFAULT_COMMAND_TIMEOUT_MS 500
#endif

#ifndef MTI_DEFAULT_OUTPUT_HZ
#define MTI_DEFAULT_OUTPUT_HZ 10
#endif

#ifndef MTI_MAX_SUPPORTED_OUTPUT_HZ
#define MTI_MAX_SUPPORTED_OUTPUT_HZ 100
#endif

// ======================================================
// XBUS CONSTANTS
// ======================================================

static const uint8_t XBUS_PREAMBLE = 0xFA;
static const uint8_t XBUS_BID = 0xFF;

// ======================================================
// MID
// ======================================================

static const uint8_t MID_GO_TO_CONFIG = 0x30;
static const uint8_t MID_GO_TO_CONFIG_ACK = 0x31;

static const uint8_t MID_GO_TO_MEASUREMENT = 0x10;
static const uint8_t MID_GO_TO_MEASUREMENT_ACK = 0x11;

static const uint8_t MID_MT_DATA2 = 0x36;

static const uint8_t MID_OUTPUT_CONFIG = 0xC0;
static const uint8_t MID_OUTPUT_CONFIG_ACK = 0xC1;

static const uint8_t MID_ERROR = 0x42;

static const uint8_t MID_SET_OPTION_FLAGS = 0x48;
static const uint8_t MID_SET_OPTION_FLAGS_ACK = 0x49;

static const uint8_t MID_SET_FILTER_PROFILE = 0x64;
static const uint8_t MID_SET_FILTER_PROFILE_ACK = 0x65;

static const uint8_t MID_ICC_COMMAND = 0x74;
static const uint8_t MID_ICC_COMMAND_ACK = 0x75;

// ======================================================
// MTDATA2 BLOCKS
// ======================================================

static const uint8_t EULER_MESSAGE = 0x20;
static const uint8_t EULER_FRAME = 0x30;

static const uint8_t ACC_MESSAGE = 0x40;
static const uint8_t ACC_FRAME = 0x20;

static const uint8_t GYRO_MESSAGE = 0x80;
static const uint8_t GYRO_FRAME = 0x20;

// ======================================================
// COMMANDS
// ======================================================

static const uint8_t config_mode_command[] = {
    0xFA, 0xFF, 0x30, 0x00, 0xD1};

static const uint8_t go_meas_command[] = {
    0xFA, 0xFF, 0x10, 0x00, 0xF1};

static const uint8_t clear_option[] = {
    0xFA, 0xFF, 0x48, 0x08,
    0x00, 0x00, 0x00, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF,
    0xB5};

#if (SET_FILTER_PROFILE_SERIAL_CONFIG == FILTER_PROFILE_MAG)
static const uint8_t filter_profile_config[] = {
    0xFA, 0xFF, 0x64, 0x02, 0x00, 0x33, 0x68};
#elif (SET_FILTER_PROFILE_SERIAL_CONFIG == FILTER_PROFILE_GENERAL)
static const uint8_t filter_profile_config[] = {
    0xFA, 0xFF, 0x64, 0x02, 0x00, 0x32, 0x69};
#endif

static const uint8_t in_run_compass_calibration[] = {
    0xFA, 0xFF, 0x48, 0x08,
    0x00, 0x00, 0x00, 0x80,
    0x00, 0x00, 0x00, 0x00,
    0x31};

static const uint8_t status_rep_mode[] = {
    0xFA, 0xFF, 0x74, 0x01, 0x03, 0x89};

static const uint8_t request_rep_mode_status[] = {
    0xFA, 0xFF, 0x30, 0x00, 0xD1};

static const uint8_t start_rep_mode_command[] = {
    0xFA, 0xFF, 0x74, 0x01, 0x00, 0x8C};

static const uint8_t stop_rep_mode_command[] = {
    0xFA, 0xFF, 0x74, 0x01, 0x01, 0x8B};

static const uint8_t store_data_command[] = {
    0xFA, 0xFF, 0x74, 0x01, 0x02, 0x8A};