#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace rf_protocol {

constexpr std::size_t MAX_PAYLOAD_SIZE = 32;

enum CommandType : uint8_t {
    CMD_NOP = 0x00,
    CMD_STOP = 0x10,
    CMD_ARM = 0x11,
    CMD_MOVE = 0x12,
    CMD_GETVER = 0x20,
    CMD_SETPARAM = 0x30,
};

enum ReplyType : uint8_t {
    REPLY_LINK_LOST = 0x00,
    REPLY_TELEMETRY_V1 = 0x01,
    REPLY_VERSION = 0x02,
    REPLY_REQUEST_TIMEOUT = 0x03,
};

enum TelemetryFlag : uint8_t {
    FLAG_RUNNING = 1u << 0,
    FLAG_FAULT = 1u << 1,
    FLAG_ESTOP = 1u << 2,
    FLAG_LOW_BATTERY = 1u << 3,
};

struct __attribute__((packed)) CommandHeader {
    uint8_t type;
    uint8_t sequence;
};

struct __attribute__((packed)) MoveCommand {
    uint8_t type;
    uint8_t sequence;
    int16_t velocity_left_mm_s;
    int16_t velocity_right_mm_s;
};

struct __attribute__((packed)) SetParameterCommand {
    uint8_t type;
    uint8_t sequence;
    uint8_t parameter_id;
    int32_t value;
};

struct __attribute__((packed)) TelemetryV1 {
    uint8_t type;
    uint8_t sequence;
    uint8_t flags;
    uint16_t battery_mv;
    uint8_t battery_percent;
    int16_t duty_left;
    int16_t duty_right;
    int16_t velocity_left_mm_s;
    int16_t velocity_right_mm_s;
    int32_t encoder_left;
    int32_t encoder_right;
    uint16_t current_ma;
    uint8_t cpu_load_percent;
};

struct __attribute__((packed)) VersionReply {
    uint8_t type;
    uint8_t firmware_major;
    uint8_t firmware_minor;
};

static_assert(sizeof(CommandHeader) == 2);
static_assert(sizeof(MoveCommand) == 6);
static_assert(sizeof(SetParameterCommand) == 7);
static_assert(sizeof(TelemetryV1) == 25);
static_assert(sizeof(VersionReply) == 3);

static_assert(sizeof(CommandHeader) <= MAX_PAYLOAD_SIZE);
static_assert(sizeof(MoveCommand) <= MAX_PAYLOAD_SIZE);
static_assert(sizeof(SetParameterCommand) <= MAX_PAYLOAD_SIZE);
static_assert(sizeof(TelemetryV1) <= MAX_PAYLOAD_SIZE);
static_assert(sizeof(VersionReply) <= MAX_PAYLOAD_SIZE);

static_assert(std::is_trivially_copyable_v<CommandHeader>);
static_assert(std::is_trivially_copyable_v<MoveCommand>);
static_assert(std::is_trivially_copyable_v<SetParameterCommand>);
static_assert(std::is_trivially_copyable_v<TelemetryV1>);
static_assert(std::is_trivially_copyable_v<VersionReply>);

}  // namespace rf_protocol
