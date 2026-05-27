/**
 * @file uart_parser.c
 * @brief UART Frame Parser with Inter-Byte Timeout Simulation
 * @note Assignment for Embed Square Solutions Pvt. Ltd.
 * Compiles cleanly with: gcc -Wall -std=c99 uart_parser.c -o uart_parser
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

// ============================================================================
// Constants and Configuration
// ============================================================================
#define SOF_MARKER          0xAA
#define MAX_PAYLOAD_LEN     16

// Parser return codes
#define PARSER_FRAME_IN_PROGRESS    0
#define PARSER_FRAME_OK             1
#define PARSER_ERROR_CHECKSUM      -1
#define PARSER_ERROR_TIMEOUT       -2

// ============================================================================
// State Machine States
// ============================================================================
typedef enum {
    STATE_WAIT_SOF,
    STATE_RX_CMD,
    STATE_RX_LEN,
    STATE_RX_PAYLOAD,
    STATE_RX_CHECKSUM
} parser_state_t;

// ============================================================================
// Parser Context Structure
// ============================================================================
typedef struct {
    parser_state_t state;
    uint8_t  cmd;
    uint8_t  len;
    uint8_t  payload[MAX_PAYLOAD_LEN];
    uint8_t  payload_idx;
    uint8_t  calculated_checksum;
    uint32_t last_byte_time_ms;
    bool     is_mid_frame;
} uart_parser_t;

// ============================================================================
// Function Declarations
// ============================================================================
void parser_init(uart_parser_t *parser);
int8_t parser_feed_byte(uart_parser_t *parser, uint8_t byte, uint32_t timestamp_ms, uint32_t timeout_ms);
void run_test_sequence(const char *test_name, const uint8_t *bytes, const uint32_t *timestamps, size_t length, uint32_t timeout_ms);

// ============================================================================
// Parser Logic Implementation
// ============================================================================

/**
 * @brief Initializes or resets the parser instance context to defaults.
 */
void parser_init(uart_parser_t *parser) {
    if (parser == NULL) return;
    
    parser->state = STATE_WAIT_SOF;
    parser->cmd = 0x00;
    parser->len = 0x00;
    parser->payload_idx = 0;
    parser->calculated_checksum = 0x00;
    parser->last_byte_time_ms = 0;
    parser->is_mid_frame = false;
    memset(parser->payload, 0, sizeof(parser->payload));
}

/**
 * @brief Feeds a single byte into the parser state machine with timeout enforcement.
 */
int8_t parser_feed_byte(uart_parser_t *parser, uint8_t byte, uint32_t timestamp_ms, uint32_t timeout_ms) {
    if (parser == NULL) return PARSER_FRAME_IN_PROGRESS;

    // Part B: Check Inter-byte Timeout Before consuming the byte
    if (timeout_ms > 0 && parser->is_mid_frame) {
        uint32_t elapsed_gap = timestamp_ms - parser->last_byte_time_ms;
        if (elapsed_gap > timeout_ms) {
            parser_init(parser);
            // Save time reference for the dropped processing window boundary
            parser->last_byte_time_ms = timestamp_ms; 
            return PARSER_ERROR_TIMEOUT;
        }
    }

    // Part A: Process byte normally via State Machine
    switch (parser->state) {
        
        case STATE_WAIT_SOF:
            if (byte == SOF_MARKER) {
                parser->state = STATE_RX_CMD;
                parser->is_mid_frame = true;
                parser->calculated_checksum = 0x00;
                parser->payload_idx = 0;
                parser->last_byte_time_ms = timestamp_ms;
                return PARSER_FRAME_IN_PROGRESS;
            }
            return PARSER_FRAME_IN_PROGRESS;

        case STATE_RX_CMD:
            parser->cmd = byte;
            parser->calculated_checksum ^= byte;
            parser->state = STATE_RX_LEN;
            parser->last_byte_time_ms = timestamp_ms;
            return PARSER_FRAME_IN_PROGRESS;

        case STATE_RX_LEN:
            if (byte > MAX_PAYLOAD_LEN) {
                parser_init(parser);
                return PARSER_FRAME_IN_PROGRESS;
            }
            parser->len = byte;
            parser->calculated_checksum ^= byte;
            
            if (parser->len == 0) {
                parser->state = STATE_RX_CHECKSUM;
            } else {
                parser->state = STATE_RX_PAYLOAD;
            }
            parser->last_byte_time_ms = timestamp_ms;
            return PARSER_FRAME_IN_PROGRESS;

        case STATE_RX_PAYLOAD:
            parser->payload[parser->payload_idx] = byte;
            parser->calculated_checksum ^= byte;
            parser->payload_idx++;
            
            if (parser->payload_idx >= parser->len) {
                parser->state = STATE_RX_CHECKSUM;
            }
            parser->last_byte_time_ms = timestamp_ms;
            return PARSER_FRAME_IN_PROGRESS;

        case STATE_RX_CHECKSUM:
            parser->last_byte_time_ms = timestamp_ms;
            if (byte == parser->calculated_checksum) {
                parser->is_mid_frame = false; 
                parser->state = STATE_WAIT_SOF;
                return PARSER_FRAME_OK;
            } else {
                parser_init(parser);
                return PARSER_ERROR_CHECKSUM;
            }

        default:
            parser_init(parser);
            return PARSER_FRAME_IN_PROGRESS;
    }
}

// ============================================================================
// Part C - Feed Helper and Formatted Logging Function
// ============================================================================
void run_test_sequence(const char *test_name, const uint8_t *bytes, const uint32_t *timestamps, size_t length, uint32_t timeout_ms) {
    uart_parser_t parser;
    parser_init(&parser);
    
    printf("\n--- %s (Timeout = %d ms) ---\n", test_name, timeout_ms);

    for (size_t i = 0; i < length; i++) {
        uint8_t byte = bytes[i];
        uint32_t t_ms = timestamps[i];
        
        uint32_t initial_last_time = parser.last_byte_time_ms;
        int8_t result = parser_feed_byte(&parser, byte, t_ms, timeout_ms);

        // If a timeout occurred, log it and immediately re-feed the byte
        if (result == PARSER_ERROR_TIMEOUT) {
            uint32_t missing_gap = t_ms - initial_last_time;
            printf("t=%dms byte=0x%02X -> TIMEOUT (%dms gap > %dms) - parser reset\n", 
                   t_ms, byte, missing_gap, timeout_ms);
            
            // Re-feed step
            result = parser_feed_byte(&parser, byte, t_ms, timeout_ms);
            printf("t=%dms byte=0x%02X -> receiving... (re-fed after reset)\n", t_ms, byte);
            continue;
        }

        // Standard dynamic logging
        printf("t=%dms byte=0x%02X -> ", t_ms, byte);
        
        if (result == PARSER_FRAME_IN_PROGRESS) {
            printf("receiving...\n");
        } 
        else if (result == PARSER_FRAME_OK) {
            printf("FRAME OK $CMD=0x%02X$ LEN %d PAYLOAD [", parser.cmd, parser.len);
            for (uint8_t p = 0; p < parser.len; p++) {
                printf("%02X%s", parser.payload[p], (p == parser.len - 1) ? "" : " ");
            }
            printf("]\n");
        } 
        else if (result == PARSER_ERROR_CHECKSUM) {
            printf("checksum error\n");
        }
    }
}

// ============================================================================
// Test Case Drivers
// ============================================================================
int main(void) {
    // Test 1 -- Clean Valid Frame
    uint8_t  test1_bytes[] = {0xAA, 0x01, 0x03, 0x10, 0x20, 0x30, 0x22};
    uint32_t test1_times[] = {0,    5,    10,   15,   20,   25,   30};
    size_t   test1_len   = sizeof(test1_bytes) / sizeof(test1_bytes[0]);
    run_test_sequence("Test 1 -- Clean Valid Frame", test1_bytes, test1_times, test1_len, 50);

    // Test 2 -- Timeout Mid-Frame, Then Recovery (Checksum: 0x05 ^ 0x01 ^ 0x7F = 0x7B)
    uint8_t  test2_bytes[] = {0xAA, 0x01, 0x03, 0x10, 0x10, 0xAA, 0x05, 0x01, 0x7F, 0x7B};
    uint32_t test2_times[] = {0,    5,    10,   15,   200,  200,  205,  210,  215,  220};
    size_t   test2_len   = sizeof(test2_bytes) / sizeof(test2_bytes[0]);
    run_test_sequence("Test 2 -- Timeout Mid-Frame, Then Recovery", test2_bytes, test2_times, test2_len, 50);

    // Test 3 -- Two Valid Frames Back-to-Back (cs1 = 0x57, cs2 = 0x17)
    uint8_t  test3_bytes[] = {0xAA, 0x03, 0x01, 0x55, 0x57, 0xAA, 0x04, 0x02, 0xAA, 0xBB, 0x17};
    uint32_t test3_times[] = {0,    5,    10,   15,   20,   25,   30,   35,   40,   45,   50};
    size_t   test3_len   = sizeof(test3_bytes) / sizeof(test3_bytes[0]);
    run_test_sequence("Test 3 -- Two Valid Frames Back-to-Back", test3_bytes, test3_times, test3_len, 50);

    // Test 4 -- Timeout Disabled
    run_test_sequence("Test 4 -- Timeout Disabled", test2_bytes, test2_times, test2_len, 0);

    return 0;
}
