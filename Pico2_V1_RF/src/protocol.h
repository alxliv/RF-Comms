#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

enum {
    RF_PACKET_SIZE = 32,
    RF_RESPONSE_DATA_MAX = 27,

    RF_COMMAND_GETSTAT = 0x01,
    RF_COMMAND_PING = 0x02,
    RF_COMMAND_MOVE = 0x0A,
    RF_COMMAND_STOP = 0x0B,
    RF_RESPONSE_ACK_MASK = 0xF0,
};

typedef struct {
    uint8_t command;
    int8_t argument;
} rf_command_t;

typedef struct {
    uint8_t source_id;
    uint8_t command;
    int8_t argument;
    uint8_t data_length;
    uint8_t data[RF_RESPONSE_DATA_MAX];
} rf_response_t;

void protocol_encode_command(const rf_command_t *command,
                             uint8_t packet[RF_PACKET_SIZE]);
bool protocol_decode_command(const uint8_t packet[RF_PACKET_SIZE],
                             rf_command_t *command);
void protocol_encode_ack(uint8_t source_id, uint8_t command,
                         int8_t argument,
                         uint8_t packet[RF_PACKET_SIZE]);
bool protocol_encode_data_response(
    uint8_t source_id, uint8_t command, const uint8_t *data,
    uint8_t data_length,
    uint8_t packet[RF_PACKET_SIZE]);
bool protocol_decode_response(const uint8_t packet[RF_PACKET_SIZE],
                              rf_response_t *response);

#endif
