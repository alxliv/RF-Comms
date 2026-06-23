#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace rf_protocol {

constexpr std::size_t MAX_PAYLOAD_SIZE = 32;

// Base -> Wanderer. Commands set goals (MOVE/STOP/ARM) or ask a question
// (GETVER/GETSTAT). None of them block the base: a query's answer comes back
// later on the downlink stream, not as a synchronous reply.
enum CommandType : uint8_t {
    CMD_NOP = 0x00,
    CMD_STOP = 0x10,
    CMD_ARM = 0x11,
    CMD_MOVE = 0x12,
    CMD_GETVER = 0x20,
    CMD_GETSTAT = 0x21,
};

// Wanderer -> Base. Every ACK payload is one of these frames, identified by
// its leading type byte. The base routes each frame by type in a single
// dispatcher: telemetry is monitored and replies are handled. REPLY_LINK_LOST
// is the one exception: the base synthesizes it locally for the host when the
// RF link drops (the Wanderer cannot send it).
enum ReplyType : uint8_t {
    REPLY_LINK_LOST = 0x00,
    REPLY_TELEMETRY = 0x01,
    REPLY_VERSION = 0x02,
    REPLY_STAT = 0x03,
};

// FRAME_SYNC marks the start of a binary host frame on the base's USB CDC
// link. It is outside printable ASCII so it never collides with a typed text
// CLI line on the same byte stream.
constexpr uint8_t FRAME_SYNC = 0xAA;

enum TelemetryFlag : uint8_t {
    WAND_MOVING = 1u << 0,
    WAND_ARMED = 1u << 1,
};

// Every command starts with this header. Header-only commands (NOP, STOP, ARM,
// GETVER, GETSTAT) are sent as a bare CommandHeader; commands that carry
// arguments embed it as their first member.
struct __attribute__((packed)) CommandHeader {
    uint8_t type;
    uint8_t sequence;
};

struct __attribute__((packed)) MoveCommand {
    CommandHeader header;
    int16_t velocity_left_mm_s;
    int16_t velocity_right_mm_s;
};

// The continuous monitoring heartbeat. It currently carries only the sequence
// (for gap/rate stats) and the state flags. Real sensor fields (battery,
// encoders, velocity, ...) are added here when that hardware exists.
struct __attribute__((packed)) Telemetry {
    uint8_t type;
    uint8_t sequence;
    uint8_t flags;
};

struct __attribute__((packed)) VersionReply {
    uint8_t type;
    uint8_t firmware_major;
    uint8_t firmware_minor;
};

// Reply to CMD_GETSTAT: the Wanderer's current commanded state. This is the
// proper channel for "is it armed / what is it doing", as opposed to the
// telemetry heartbeat which is one-way monitoring.
struct __attribute__((packed)) StatReply {
    uint8_t type;
    uint8_t flags;
    int16_t target_left_mm_s;
    int16_t target_right_mm_s;
};

// Sent by the base to a host frame client when the RF link to the Wanderer
// is lost. The base detects this from a missing hardware ACK.
struct __attribute__((packed)) LinkLostNotice {
    uint8_t type;
};

static_assert(sizeof(CommandHeader) == 2);
static_assert(sizeof(MoveCommand) == 6);
static_assert(sizeof(Telemetry) == 3);
static_assert(sizeof(VersionReply) == 3);
static_assert(sizeof(StatReply) == 6);
static_assert(sizeof(LinkLostNotice) == 1);

static_assert(sizeof(CommandHeader) <= MAX_PAYLOAD_SIZE);
static_assert(sizeof(MoveCommand) <= MAX_PAYLOAD_SIZE);
static_assert(sizeof(Telemetry) <= MAX_PAYLOAD_SIZE);
static_assert(sizeof(VersionReply) <= MAX_PAYLOAD_SIZE);
static_assert(sizeof(StatReply) <= MAX_PAYLOAD_SIZE);
static_assert(sizeof(LinkLostNotice) <= MAX_PAYLOAD_SIZE);

static_assert(std::is_trivially_copyable_v<CommandHeader>);
static_assert(std::is_trivially_copyable_v<MoveCommand>);
static_assert(std::is_trivially_copyable_v<Telemetry>);
static_assert(std::is_trivially_copyable_v<VersionReply>);
static_assert(std::is_trivially_copyable_v<StatReply>);
static_assert(std::is_trivially_copyable_v<LinkLostNotice>);

}  // namespace rf_protocol
