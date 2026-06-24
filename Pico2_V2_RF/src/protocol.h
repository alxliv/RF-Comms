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
    CMD_GETPA = 0x22,
    CMD_SETPA = 0x23,
};

// Wanderer -> Base. Every ACK payload is one of these frames, identified by
// its leading type byte. The base routes each frame by type in a single
// dispatcher: telemetry is monitored and query replies are translated into the
// laptop text protocol. A dropped RF link is detected by the base from a
// missing hardware ACK and reported to the laptop as a `!link down` event.
enum ReplyType : uint8_t {
    REPLY_TELEMETRY = 0x01,
    REPLY_VERSION = 0x02,
    REPLY_STAT = 0x03,
    REPLY_PA = 0x04,
};

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

// CMD_SETPA: ask the Wanderer to set its nRF24 PA level (0 = MIN .. 3 = MAX).
// "Try to set" -- the Wanderer applies it and answers with a PaReply carrying
// the level actually in effect.
struct __attribute__((packed)) SetPaCommand {
    CommandHeader header;
    uint8_t pa_level;
};

enum class TacticalState : uint8_t {
    Safe     = 0,  // boot: disarmed, motors gated off
    Active   = 1,  // armed: emitting the commanded velocity
    Fallback = 2,  // no live commander: ramping commanded velocity to zero
    Fault    = 3,  // latched fault: motors gated off until explicitly cleared
};

// The continuous monitoring heartbeat. It currently carries only the sequence
// (for gap/rate stats) and the state flags. Real sensor fields (battery,
// encoders, velocity, ...) are added here when that hardware exists.
struct __attribute__((packed)) Telemetry {
    uint8_t type;
    uint8_t sequence;
    uint8_t tactical_state;
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

// Reply to CMD_GETPA: the Wanderer's own nRF24 PA level (0 = MIN .. 3 = MAX).
// PA rarely changes, so it is queried on demand rather than carried in every
// telemetry frame.
struct __attribute__((packed)) PaReply {
    uint8_t type;
    uint8_t pa_level;
};

static_assert(sizeof(CommandHeader) == 2);
static_assert(sizeof(MoveCommand) == 6);
static_assert(sizeof(SetPaCommand) == 3);
static_assert(sizeof(Telemetry) == 4);
static_assert(sizeof(VersionReply) == 3);
static_assert(sizeof(StatReply) == 6);
static_assert(sizeof(PaReply) == 2);

static_assert(sizeof(CommandHeader) <= MAX_PAYLOAD_SIZE);
static_assert(sizeof(MoveCommand) <= MAX_PAYLOAD_SIZE);
static_assert(sizeof(SetPaCommand) <= MAX_PAYLOAD_SIZE);
static_assert(sizeof(Telemetry) <= MAX_PAYLOAD_SIZE);
static_assert(sizeof(VersionReply) <= MAX_PAYLOAD_SIZE);
static_assert(sizeof(StatReply) <= MAX_PAYLOAD_SIZE);
static_assert(sizeof(PaReply) <= MAX_PAYLOAD_SIZE);

static_assert(std::is_trivially_copyable_v<CommandHeader>);
static_assert(std::is_trivially_copyable_v<MoveCommand>);
static_assert(std::is_trivially_copyable_v<SetPaCommand>);
static_assert(std::is_trivially_copyable_v<Telemetry>);
static_assert(std::is_trivially_copyable_v<VersionReply>);
static_assert(std::is_trivially_copyable_v<StatReply>);
static_assert(std::is_trivially_copyable_v<PaReply>);

}  // namespace rf_protocol
