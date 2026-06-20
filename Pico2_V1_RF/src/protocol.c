#include "protocol.h"

#include <string.h>

enum {
    PACKET_CRC_OFFSET = RF_PACKET_SIZE - 2,
};

static uint16_t crc16_ccitt(const uint8_t *data, uint8_t length) {
    uint16_t crc = 0xFFFF;

    for (uint8_t index = 0; index < length; ++index) {
        crc ^= (uint16_t)data[index] << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000u) != 0
                      ? (uint16_t)((crc << 1) ^ 0x1021u)
                      : (uint16_t)(crc << 1);
        }
    }

    return crc;
}

static void finish_packet(uint8_t packet[RF_PACKET_SIZE]) {
    const uint16_t crc = crc16_ccitt(packet, PACKET_CRC_OFFSET);
    packet[PACKET_CRC_OFFSET] = (uint8_t)(crc & 0xFFu);
    packet[PACKET_CRC_OFFSET + 1] = (uint8_t)(crc >> 8);
}

static bool packet_crc_valid(const uint8_t packet[RF_PACKET_SIZE]) {
    const uint16_t expected =
        (uint16_t)packet[PACKET_CRC_OFFSET] |
        ((uint16_t)packet[PACKET_CRC_OFFSET + 1] << 8);
    return expected == crc16_ccitt(packet, PACKET_CRC_OFFSET);
}

static uint8_t ack_byte(uint8_t source_id) {
    return (uint8_t)(RF_RESPONSE_ACK_MASK | (source_id - 1u));
}

void protocol_encode_command(const rf_command_t *command,
                             uint8_t packet[RF_PACKET_SIZE]) {
    memset(packet, 0, RF_PACKET_SIZE);
    packet[0] = command->command;
    packet[1] = (uint8_t)command->argument;
    finish_packet(packet);
}

bool protocol_decode_command(const uint8_t packet[RF_PACKET_SIZE],
                             rf_command_t *command) {
    if (!packet_crc_valid(packet) ||
        (packet[0] & RF_RESPONSE_ACK_MASK) ==
            RF_RESPONSE_ACK_MASK) {
        return false;
    }

    command->command = packet[0];
    command->argument = (int8_t)packet[1];
    return true;
}

void protocol_encode_ack(uint8_t source_id, uint8_t command,
                         int8_t argument,
                         uint8_t packet[RF_PACKET_SIZE]) {
    memset(packet, 0, RF_PACKET_SIZE);
    packet[0] = ack_byte(source_id);
    packet[1] = command;
    packet[2] = (uint8_t)argument;
    finish_packet(packet);
}

bool protocol_encode_data_response(
    uint8_t source_id, uint8_t command, const uint8_t *data,
    uint8_t data_length,
    uint8_t packet[RF_PACKET_SIZE]) {
    if (data_length > RF_RESPONSE_DATA_MAX) {
        return false;
    }

    memset(packet, 0, RF_PACKET_SIZE);
    packet[0] = ack_byte(source_id);
    packet[1] = command;
    packet[2] = data_length;
    if (data_length != 0) {
        memcpy(&packet[3], data, data_length);
    }
    finish_packet(packet);
    return true;
}

bool protocol_decode_response(const uint8_t packet[RF_PACKET_SIZE],
                              rf_response_t *response) {
    if (!packet_crc_valid(packet) ||
        (packet[0] & RF_RESPONSE_ACK_MASK) !=
            RF_RESPONSE_ACK_MASK) {
        return false;
    }

    response->source_id = (uint8_t)((packet[0] & 0x0Fu) + 1u);
    response->command = packet[1];
    response->argument = (int8_t)packet[2];
    response->data_length = 0;

    if (response->command == RF_COMMAND_GETSTAT ||
        response->command == RF_COMMAND_GETVER) {
        response->data_length = packet[2];
        if (response->data_length > RF_RESPONSE_DATA_MAX) {
            return false;
        }
        memcpy(response->data, &packet[3], response->data_length);
    }

    return true;
}
