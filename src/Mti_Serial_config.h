#pragma once

#include <Arduino.h>

// =====================================================
// USER CONFIG
// =====================================================

// Abilita stampa di ogni frame valido ricevuto/decodificato
#ifndef MTI3_UART_ENABLE_FRAME_DEBUG
#define MTI3_UART_ENABLE_FRAME_DEBUG 0
#endif

#ifndef MTI3_UART_DEFAULT_PARSER_TIMEOUT_MS
#define MTI3_UART_DEFAULT_PARSER_TIMEOUT_MS 100
#endif

#ifndef MTI3_UART_DEFAULT_COMMAND_TIMEOUT_MS
#define MTI3_UART_DEFAULT_COMMAND_TIMEOUT_MS 500
#endif

#ifndef MTI3_UART_DEFAULT_OUTPUT_HZ
#define MTI3_UART_DEFAULT_OUTPUT_HZ 10
#endif

#ifndef MTI3_UART_MAX_OUTPUT_HZ
#define MTI3_UART_MAX_OUTPUT_HZ 100
#endif

static void float_to_be(float v, uint8_t *out)
{
    uint8_t tmp[4];
    memcpy(tmp, &v, 4);

    out[0] = tmp[3];
    out[1] = tmp[2];
    out[2] = tmp[1];
    out[3] = tmp[0];
}