#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "RF24.h"
#include "pico/stdlib.h"
#include "protocol.h"

namespace {

constexpr uint8_t PIN_RF_CE = 14;
constexpr uint8_t PIN_RF_CSN = 17;
constexpr uint8_t PIN_RF_MISO = 16;
constexpr uint8_t PIN_RF_SCK = 18;
constexpr uint8_t PIN_RF_MOSI = 19;
constexpr uint8_t PIN_ROLE = 20;

// Both boards run the same firmware. GP20 selects what this board does:
// high = laptop-side base, low = vehicle-side Wanderer.
constexpr uint8_t RADIO_CHANNEL = 76;
constexpr uint8_t RADIO_PIPE = 1;
constexpr uint32_t POLL_PERIOD_MS = 10;
// If the Wanderer hears no valid command for this long, it stops locally.
// The base cannot receive that fact while the RF link is down; it reports
// link loss immediately and confirms the emergency-stop state after recovery.
constexpr uint32_t LINK_TIMEOUT_MS = 200;
constexpr uint32_t RADIO_HEALTH_PERIOD_MS = 1000;
constexpr uint32_t TELEMETRY_REPORT_MS = 5000;
constexpr uint32_t REQUEST_TIMEOUT_MS = 500;
constexpr size_t COMMAND_LINE_SIZE = 64;
constexpr uint8_t RADIO_ADDRESS[5] = {'V', '2', 'R', 'F', '1'};

constexpr uint8_t FIRMWARE_MAJOR = 0;
constexpr uint8_t FIRMWARE_MINOR = 2;

enum class Role : uint8_t {
    Wanderer,
    Base,
};

struct WandererState {
    // These are currently command/telemetry state only. Real motor-control
    // outputs and sensor readings will be connected later.
    bool armed;
    bool estop;
    uint8_t telemetry_sequence;
    int16_t target_left_mm_s;
    int16_t target_right_mm_s;
    absolute_time_t last_command_ts;
};

struct BaseState {
    // Command and telemetry sequence values are 8-bit and intentionally wrap
    // from 255 back to 0.
    uint8_t command_sequence;
    bool link_up;
    uint8_t last_telemetry_sequence;
    uint32_t telemetry_received;
    uint32_t telemetry_received_at_report;
    uint32_t telemetry_gaps;
    uint32_t telemetry_gaps_at_report;
    uint8_t last_telemetry_flags;
    absolute_time_t telemetry_report_started;
    absolute_time_t next_poll;
};

RF24 radio(PIN_RF_CE, PIN_RF_CSN);
SPI radio_spi;

// What the Wanderer should stage as its next ACK payload instead of routine
// telemetry. This is radio-layer staging state, not vehicle state, so it
// lives here alongside radio/radio_spi rather than inside WandererState.
struct PendingReply {
    bool active;
    uint8_t type;
};
PendingReply pending_reply;

}  // namespace

#define PRINTF(...)                   \
    do {                              \
        if (stdio_usb_connected()) {  \
            std::printf(__VA_ARGS__); \
        }                             \
    } while (0)

namespace {

// Matches Pico2_V1_RF/src/protocol.c crc16_ccitt: init 0xFFFF, poly 0x1021,
// MSB-first. Used to checksum binary host frames on the base's USB CDC link.
uint16_t crc16_ccitt(const uint8_t *data, std::size_t length) {
    uint16_t crc = 0xFFFF;
    for (std::size_t index = 0; index < length; ++index) {
        crc ^= static_cast<uint16_t>(data[index]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000u) != 0
                      ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                      : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

// Writes [FRAME_SYNC][length][payload][crc_lo][crc_hi] to the base's USB CDC
// link. The CRC covers the length byte and payload, matching the receive
// side in poll_base_usb(). This is the binary counterpart to the text CLI,
// intended for a host program rather than a human terminal.
void send_host_frame(const void *payload, uint8_t length) {
    if (!stdio_usb_connected() ||
        length > rf_protocol::MAX_PAYLOAD_SIZE) {
        return;
    }

    uint8_t frame[2 + rf_protocol::MAX_PAYLOAD_SIZE + 2];
    frame[0] = rf_protocol::FRAME_SYNC;
    frame[1] = length;
    std::memcpy(frame + 2, payload, length);

    const uint16_t crc =
        crc16_ccitt(frame + 1, static_cast<std::size_t>(length) + 1);
    frame[2 + length] = static_cast<uint8_t>(crc & 0xFFu);
    frame[3 + length] = static_cast<uint8_t>(crc >> 8);

    std::fwrite(frame, 1, static_cast<std::size_t>(length) + 4, stdout);
    std::fflush(stdout);
}

Role read_role() {
    // The pulldown makes an unconnected role pin safely select Wanderer.
    gpio_init(PIN_ROLE);
    gpio_set_dir(PIN_ROLE, GPIO_IN);
    gpio_pull_down(PIN_ROLE);
    sleep_ms(2);
    return gpio_get(PIN_ROLE) ? Role::Base : Role::Wanderer;
}

bool configure_radio(Role role) {
    // Both roles must use identical Enhanced ShockBurst settings. Dynamic
    // payloads are required because commands and ACK replies have different
    // lengths.
    radio.setAddressWidth(sizeof(RADIO_ADDRESS));
    radio.setChannel(RADIO_CHANNEL);
    radio.setDataRate(RF24_1MBPS);
    radio.setCRCLength(RF24_CRC_16);
    radio.setPALevel(RF24_PA_MIN);
    radio.setRetries(2, 15);
    radio.setAutoAck(true);
    radio.enableDynamicPayloads();
    radio.enableAckPayload();
    radio.flush_rx();
    radio.flush_tx();

    if (role == Role::Base) {
        radio.openWritingPipe(RADIO_ADDRESS);
        radio.stopListening();
    } else {
        radio.openReadingPipe(RADIO_PIPE, RADIO_ADDRESS);
    }

    return radio.isChipConnected();
}

rf_protocol::TelemetryV1 build_telemetry(WandererState *state) {
    // A telemetry packet is a complete snapshot, not a delta. Losing one
    // packet does not make later packets impossible to decode.
    rf_protocol::TelemetryV1 telemetry{};
    telemetry.type = rf_protocol::REPLY_TELEMETRY_V1;
    telemetry.sequence = state->telemetry_sequence++;
    if (state->armed &&
        (state->target_left_mm_s != 0 ||
         state->target_right_mm_s != 0)) {
        telemetry.flags |= rf_protocol::FLAG_RUNNING;
    }
    if (state->estop) {
        telemetry.flags |= rf_protocol::FLAG_ESTOP;
    }

    // Physical telemetry sources are added with the vehicle-control code.
    return telemetry;
}

rf_protocol::VersionReply build_version_reply() {
    rf_protocol::VersionReply reply{};
    reply.type = rf_protocol::REPLY_VERSION;
    reply.firmware_major = FIRMWARE_MAJOR;
    reply.firmware_minor = FIRMWARE_MINOR;
    return reply;
}

bool stage_next_ack_payload(WandererState *state) {
    // writeAckPayload() only queues data. The nRF24 sends it automatically
    // with the ACK for the next command received on RADIO_PIPE.
    //
    // Important: the ACK for the command currently being read has already
    // gone over the air. Therefore the payload staged here is always for the
    // following base poll. This one-poll delay is normal.
    if (pending_reply.active) {
        pending_reply.active = false;

        switch (pending_reply.type) {
            case rf_protocol::REPLY_VERSION: {
                const rf_protocol::VersionReply reply =
                    build_version_reply();
                return radio.writeAckPayload(
                    RADIO_PIPE, &reply, sizeof(reply));
            }

            default:
                break;
        }
    }

    const rf_protocol::TelemetryV1 telemetry = build_telemetry(state);
    return radio.writeAckPayload(RADIO_PIPE, &telemetry,
                                 sizeof(telemetry));
}

bool handle_wanderer_command(const uint8_t *payload, uint8_t length,
                             WandererState *state) {
    // Never cast and trust arbitrary radio bytes. First verify the exact
    // length, then copy into the packed protocol structure.
    if (length < sizeof(rf_protocol::CommandHeader)) {
        return false;
    }

    rf_protocol::CommandHeader header{};
    std::memcpy(&header, payload, sizeof(header));

    switch (header.type) {
        case rf_protocol::CMD_NOP:
            return length == sizeof(rf_protocol::CommandHeader);

        case rf_protocol::CMD_STOP:
            if (length != sizeof(rf_protocol::CommandHeader)) {
                return false;
            }
            state->armed = false;
            state->target_left_mm_s = 0;
            state->target_right_mm_s = 0;
            PRINTF("Command STOP seq=%u: disarmed, targets cleared\r\n",
                   header.sequence);
            return true;

        case rf_protocol::CMD_ARM:
            if (length != sizeof(rf_protocol::CommandHeader)) {
                return false;
            }
            state->armed = true;
            state->estop = false;
            PRINTF("Command ARM seq=%u: armed, emergency stop cleared\r\n",
                   header.sequence);
            return true;

        case rf_protocol::CMD_MOVE: {
            if (length != sizeof(rf_protocol::MoveCommand)) {
                return false;
            }
            rf_protocol::MoveCommand command{};
            std::memcpy(&command, payload, sizeof(command));
            if (state->armed) {
                state->target_left_mm_s = command.velocity_left_mm_s;
                state->target_right_mm_s =
                    command.velocity_right_mm_s;
                PRINTF("Command MOVE seq=%u: left=%d right=%d mm/s\r\n",
                       command.sequence, command.velocity_left_mm_s,
                       command.velocity_right_mm_s);
            } else {
                PRINTF("Command MOVE seq=%u ignored: Wanderer is not armed\r\n",
                       command.sequence);
            }
            return true;
        }

        case rf_protocol::CMD_GETVER:
            if (length != sizeof(rf_protocol::CommandHeader)) {
                return false;
            }
            pending_reply = {
                true,
                rf_protocol::REPLY_VERSION,
            };
            PRINTF("Command GETVER seq=%u: version reply queued\r\n",
                   header.sequence);
            return true;

        case rf_protocol::CMD_SETPARAM: {
            if (length != sizeof(rf_protocol::SetParameterCommand)) {
                return false;
            }
            rf_protocol::SetParameterCommand command{};
            std::memcpy(&command, payload, sizeof(command));
            PRINTF("Command SETPARAM seq=%u id=%u value=%ld: "
                   "no parameter handler installed\r\n",
                   command.sequence, command.parameter_id,
                   static_cast<long>(command.value));
            return true;
        }

        default:
            return false;
    }
}

void stop_wanderer(WandererState *state) {
    state->armed = false;
    state->estop = true;
    state->target_left_mm_s = 0;
    state->target_right_mm_s = 0;
}

void process_wanderer_radio(WandererState *state) {
    // Drain every received command. The nRF24 hardware has already discarded
    // packets that failed its CRC before they can appear in this FIFO.
    uint8_t pipe = 0;
    while (radio.available(&pipe)) {
        const uint8_t length = radio.getDynamicPayloadSize();
        if (length == 0) {
            // getDynamicPayloadSize() already flushed RX itself: a 0 here
            // means it detected a corrupted R_RX_PL_WID read (a known nRF24
            // SPI erratum) and recovered. RX is already clean; only the
            // ACK-payload TX FIFO is still our responsibility.
            radio.flush_tx();
            stage_next_ack_payload(state);
            return;
        }

        uint8_t payload[rf_protocol::MAX_PAYLOAD_SIZE]{};
        radio.read(payload, length);

        // Only a valid, recognized command refreshes the safety watchdog.
        if (pipe == RADIO_PIPE &&
            handle_wanderer_command(payload, length, state)) {
            state->last_command_ts = get_absolute_time();
        }

        if (!stage_next_ack_payload(state)) {
            // The ACK payload uses the three-entry TX FIFO. A failed queue
            // operation means stale entries filled it; discard them and put
            // back one fresh reply.
            radio.flush_tx();
            stage_next_ack_payload(state);
        }
    }
}

void process_wanderer_failsafe(WandererState *state) {
    if (absolute_time_diff_us(state->last_command_ts,
                              get_absolute_time()) <=
        static_cast<int64_t>(LINK_TIMEOUT_MS) * 1000) {
        return;
    }

    if (!state->estop) {
        // This safety action happens entirely on the Wanderer. It does not
        // depend on a message from the base and still works if the base dies.
        stop_wanderer(state);

        // Replace the staged payload so the first ACK after link recovery
        // reports the failsafe state.
        radio.flush_tx();
        stage_next_ack_payload(state);
        PRINTF("Wanderer failsafe: command timeout\r\n");
    }
}

void update_telemetry_sequence(uint8_t sequence, bool first_telemetry,
                               BaseState *state) {
    // Unsigned 8-bit subtraction handles normal wraparound automatically.
    // Example: last=255, current=0 gives an advance of 1, not a gap.
    // The very first telemetry packet ever received has no real prior
    // sequence to compare against (last_telemetry_sequence defaults to 0,
    // which is not a meaningful baseline), so it only seeds the baseline.
    if (!first_telemetry) {
        const uint8_t advance =
            static_cast<uint8_t>(sequence -
                                 state->last_telemetry_sequence);
        if (advance > 1) {
            state->telemetry_gaps +=
                static_cast<uint32_t>(advance - 1u);
        }
    }

    state->last_telemetry_sequence = sequence;
}

void print_telemetry_health(BaseState *state) {
    // Report measured frames/second over the latest interval. The first gap
    // number is new gaps in this interval; the second is total since boot.
    const absolute_time_t now = get_absolute_time();
    const int64_t elapsed_us =
        absolute_time_diff_us(state->telemetry_report_started, now);
    const uint32_t interval_frames =
        state->telemetry_received -
        state->telemetry_received_at_report;
    const uint32_t interval_gaps =
        state->telemetry_gaps - state->telemetry_gaps_at_report;

    uint32_t rate_tenths = 0;
    if (elapsed_us > 0) {
        rate_tenths = static_cast<uint32_t>(
            (static_cast<uint64_t>(interval_frames) * 10000000u) /
            static_cast<uint64_t>(elapsed_us));
    }

    PRINTF("Telemetry OK: rate=%lu.%lu Hz total=%lu gaps=%lu/%lu\r\n",
           static_cast<unsigned long>(rate_tenths / 10u),
           static_cast<unsigned long>(rate_tenths % 10u),
           static_cast<unsigned long>(state->telemetry_received),
           static_cast<unsigned long>(interval_gaps),
           static_cast<unsigned long>(state->telemetry_gaps));

    state->telemetry_received_at_report =
        state->telemetry_received;
    state->telemetry_gaps_at_report = state->telemetry_gaps;
    state->telemetry_report_started = now;
}

void report_wanderer_state(uint8_t flags, bool first_telemetry,
                           BaseState *state) {
    // Normal state is not printed repeatedly. State transitions are more
    // useful to a person watching the terminal than five-second repetitions.
    const bool running =
        (flags & rf_protocol::FLAG_RUNNING) != 0;
    const bool estop =
        (flags & rf_protocol::FLAG_ESTOP) != 0;

    if (first_telemetry) {
        // The very first telemetry packet ever received has no real prior
        // flags to diff against (last_telemetry_flags defaults to 0, which
        // is not a meaningful baseline), so report the starting state
        // instead of a misleading transition.
        PRINTF("Wanderer state: running=%s estop=%s\r\n",
               running ? "yes" : "no",
               estop ? "active" : "clear");
        return;
    }

    const bool was_running =
        (state->last_telemetry_flags &
         rf_protocol::FLAG_RUNNING) != 0;
    const bool was_estop =
        (state->last_telemetry_flags &
         rf_protocol::FLAG_ESTOP) != 0;

    if (estop != was_estop) {
        if (estop) {
            PRINTF("ALERT: Wanderer emergency stop active\r\n");
        } else {
            PRINTF("Wanderer emergency stop cleared\r\n");
        }
    }

    if (running != was_running) {
        PRINTF("Wanderer movement state: %s\r\n",
               running ? "running" : "stopped");
    }
}

void process_base_reply(const uint8_t *payload, uint8_t length,
                        BaseState *state) {
    if (length == sizeof(rf_protocol::TelemetryV1) &&
        payload[0] == rf_protocol::REPLY_TELEMETRY_V1) {
        rf_protocol::TelemetryV1 telemetry{};
        std::memcpy(&telemetry, payload, sizeof(telemetry));
        // telemetry_received is a monotonic, never-reset count, so checking
        // it before incrementing tells us whether this is the first
        // telemetry packet ever received (no two dedicated bools needed).
        const bool first_telemetry = state->telemetry_received == 0;
        ++state->telemetry_received;
        update_telemetry_sequence(telemetry.sequence, first_telemetry, state);
        report_wanderer_state(telemetry.flags, first_telemetry, state);
        state->last_telemetry_flags = telemetry.flags;
        send_host_frame(&telemetry, sizeof(telemetry));

        if (time_reached(delayed_by_ms(state->telemetry_report_started,
                                      TELEMETRY_REPORT_MS))) {
            print_telemetry_health(state);
        }
    }
}

void update_base_link(bool delivered, BaseState *state) {
    // radio.write() returning false means no ACK arrived. It says the RF link
    // failed, not necessarily that this base radio is broken.
    if (!delivered) {
        if (state->link_up) {
            state->link_up = false;
            PRINTF("Base: link lost; Wanderer failsafe assumed active\r\n");
            const rf_protocol::LinkLostNotice notice{
                rf_protocol::REPLY_LINK_LOST,
            };
            send_host_frame(&notice, sizeof(notice));
        }
        return;
    }

    if (!state->link_up) {
        state->link_up = true;
        PRINTF("Base: link established\r\n");
    }
}

void drain_base_replies(BaseState *state) {
    while (radio.available()) {
        const uint8_t length = radio.getDynamicPayloadSize();
        if (length == 0) {
            // getDynamicPayloadSize() already flushed RX itself on a
            // corrupted R_RX_PL_WID read; nothing left to recover here.
            return;
        }

        uint8_t payload[rf_protocol::MAX_PAYLOAD_SIZE]{};
        radio.read(payload, length);
        process_base_reply(payload, length, state);
    }
}

bool transmit_base_frame(const void *frame, uint8_t length,
                         BaseState *state) {
    const bool delivered = radio.write(frame, length);
    update_base_link(delivered, state);
    if (delivered) {
        drain_base_replies(state);
    }
    return delivered;
}

void poll_wanderer(BaseState *state) {
    if (!time_reached(state->next_poll)) {
        return;
    }
    state->next_poll = make_timeout_time_ms(POLL_PERIOD_MS);

    const rf_protocol::CommandHeader command{
        // NOP has no vehicle action. Its purpose is to keep the watchdog
        // alive and clock the next telemetry ACK payload back to the base.
        rf_protocol::CMD_NOP,
        state->command_sequence++,
    };
    transmit_base_frame(&command, sizeof(command), state);
}

bool request_reply(uint8_t command_type, uint8_t expected_reply_type,
                   uint8_t expected_reply_length, void *reply,
                   uint8_t reply_capacity, uint32_t timeout_ms,
                   BaseState *state) {
    if (expected_reply_length > reply_capacity) {
        return false;
    }

    const rf_protocol::CommandHeader request{
        command_type,
        state->command_sequence++,
    };
    const absolute_time_t deadline = make_timeout_time_ms(timeout_ms);

    // Send the request once. radio.write() already uses the nRF24 hardware
    // retry mechanism. After delivery, NOP polls retrieve the reply that the
    // Wanderer stages for a later ACK payload.
    if (!transmit_base_frame(&request, sizeof(request), state)) {
        state->next_poll = make_timeout_time_ms(POLL_PERIOD_MS);
        return false;
    }

    while (!time_reached(deadline)) {
        const rf_protocol::CommandHeader nop{
            rf_protocol::CMD_NOP,
            state->command_sequence++,
        };
        const bool delivered = radio.write(&nop, sizeof(nop));
        update_base_link(delivered, state);

        if (delivered) {
            while (radio.available()) {
                const uint8_t length = radio.getDynamicPayloadSize();
                if (length == 0) {
                    // getDynamicPayloadSize() already flushed RX itself on a
                    // corrupted R_RX_PL_WID read; nothing left to recover.
                    break;
                }

                uint8_t payload[rf_protocol::MAX_PAYLOAD_SIZE]{};
                radio.read(payload, length);

                if (length == expected_reply_length &&
                    payload[0] == expected_reply_type) {
                    std::memcpy(reply, payload, length);
                    state->next_poll =
                        make_timeout_time_ms(POLL_PERIOD_MS);
                    return true;
                }

                // Telemetry remains live while a typed reply is pending.
                process_base_reply(payload, length, state);
            }
        }

        sleep_ms(POLL_PERIOD_MS);
    }

    state->next_poll = make_timeout_time_ms(POLL_PERIOD_MS);
    return false;
}

// The perform_*() helpers below hold the only code paths that build and
// transmit each command. Both the text CLI (process_base_command_line) and
// the binary host frame dispatcher (process_base_command_frame) call them, so
// the two interfaces cannot diverge in RF behavior.

bool perform_arm(BaseState *state) {
    // The nRF24 hardware ACK is the only delivery confirmation ARM gets, the
    // same as STOP and MOVE. Telemetry is a one-way status stream, not a
    // reply channel; a future GETSTAT command covers querying whether the
    // Wanderer is actually armed.
    const rf_protocol::CommandHeader command{
        rf_protocol::CMD_ARM,
        state->command_sequence++,
    };
    return transmit_base_frame(&command, sizeof(command), state);
}

bool perform_stop(BaseState *state) {
    const rf_protocol::CommandHeader command{
        rf_protocol::CMD_STOP,
        state->command_sequence++,
    };
    return transmit_base_frame(&command, sizeof(command), state);
}

bool perform_move(int16_t left_mm_s, int16_t right_mm_s, BaseState *state) {
    const rf_protocol::MoveCommand command{
        rf_protocol::CMD_MOVE,
        state->command_sequence++,
        left_mm_s,
        right_mm_s,
    };
    return transmit_base_frame(&command, sizeof(command), state);
}

bool perform_getver(rf_protocol::VersionReply *version, BaseState *state) {
    return request_reply(rf_protocol::CMD_GETVER, rf_protocol::REPLY_VERSION,
                         sizeof(*version), version, sizeof(*version),
                         REQUEST_TIMEOUT_MS, state);
}

void print_base_help() {
    PRINTF("Commands:\r\n");
    PRINTF("  help          Show commands\r\n");
    PRINTF("  arm           Clear Wanderer emergency stop\r\n");
    PRINTF("  stop          Stop and disarm Wanderer\r\n");
    PRINTF("  move L R      Set signed left/right velocity in mm/s\r\n");
    PRINTF("  getver        Request Wanderer firmware version\r\n");
}

void process_base_command_frame(const uint8_t *payload, uint8_t length,
                                BaseState *state) {
    // The binary host frame protocol carries the same command structs used
    // over RF, just delivered over USB CDC instead of air. It mirrors
    // exactly the text CLI's command set: ARM, STOP, MOVE, GETVER.
    if (length < sizeof(rf_protocol::CommandHeader)) {
        return;
    }

    rf_protocol::CommandHeader header{};
    std::memcpy(&header, payload, sizeof(header));

    switch (header.type) {
        case rf_protocol::CMD_ARM:
        case rf_protocol::CMD_STOP: {
            if (length != sizeof(header)) {
                return;
            }
            const bool delivered = header.type == rf_protocol::CMD_ARM
                                       ? perform_arm(state)
                                       : perform_stop(state);
            const rf_protocol::CommandResult result{
                rf_protocol::REPLY_COMMAND_RESULT,
                header.sequence,
                header.type,
                delivered ? static_cast<uint8_t>(1) : static_cast<uint8_t>(0),
            };
            send_host_frame(&result, sizeof(result));
            return;
        }

        case rf_protocol::CMD_MOVE: {
            if (length != sizeof(rf_protocol::MoveCommand)) {
                return;
            }
            rf_protocol::MoveCommand command{};
            std::memcpy(&command, payload, sizeof(command));
            const bool delivered = perform_move(command.velocity_left_mm_s,
                                                command.velocity_right_mm_s,
                                                state);
            const rf_protocol::CommandResult result{
                rf_protocol::REPLY_COMMAND_RESULT,
                command.sequence,
                rf_protocol::CMD_MOVE,
                delivered ? static_cast<uint8_t>(1) : static_cast<uint8_t>(0),
            };
            send_host_frame(&result, sizeof(result));
            return;
        }

        case rf_protocol::CMD_GETVER: {
            if (length != sizeof(header)) {
                return;
            }
            rf_protocol::VersionReply version{};
            if (perform_getver(&version, state)) {
                send_host_frame(&version, sizeof(version));
            } else {
                const rf_protocol::RequestTimeoutNotice notice{
                    rf_protocol::REPLY_REQUEST_TIMEOUT,
                    rf_protocol::CMD_GETVER,
                };
                send_host_frame(&notice, sizeof(notice));
            }
            return;
        }

        default:
            return;
    }
}

void process_base_command_line(char *line, BaseState *state) {
    // This text CLI remains for manual debugging over a serial terminal. The
    // binary host frame protocol (process_base_command_frame) carries the
    // same command set for host software and runs on the same USB CDC link.
    if (std::strcmp(line, "help") == 0) {
        print_base_help();
    } else if (std::strcmp(line, "arm") == 0) {
        if (perform_arm(state)) {
            PRINTF("ARM delivered\r\n");
        } else {
            PRINTF("ARM failed: no acknowledgement\r\n");
        }
    } else if (std::strcmp(line, "stop") == 0) {
        if (perform_stop(state)) {
            PRINTF("STOP delivered\r\n");
        } else {
            PRINTF("STOP failed: no acknowledgement\r\n");
        }
    } else if (std::strcmp(line, "getver") == 0) {
        rf_protocol::VersionReply version{};
        if (perform_getver(&version, state)) {
            PRINTF("GETVER version=%u.%u\r\n",
                   version.firmware_major, version.firmware_minor);
        } else {
            PRINTF("GETVER timeout after %lu ms\r\n",
                   static_cast<unsigned long>(REQUEST_TIMEOUT_MS));
        }
    } else if (std::strncmp(line, "move ", 5) == 0) {
        char *end = nullptr;
        const long left = std::strtol(line + 5, &end, 10);
        if (end == line + 5) {
            PRINTF("Usage: move L R\r\n");
            return;
        }

        while (*end == ' ') {
            ++end;
        }
        char *right_start = end;
        const long right = std::strtol(right_start, &end, 10);
        while (*end == ' ') {
            ++end;
        }
        if (end == right_start || *end != '\0' ||
            left < -32768L || left > 32767L ||
            right < -32768L || right > 32767L) {
            PRINTF("Usage: move L R; values must fit signed 16-bit\r\n");
            return;
        }

        if (perform_move(static_cast<int16_t>(left),
                         static_cast<int16_t>(right), state)) {
            PRINTF("MOVE delivered left=%ld right=%ld\r\n",
                   left, right);
        } else {
            PRINTF("MOVE failed: no acknowledgement\r\n");
        }
    } else if (line[0] != '\0') {
        PRINTF("Unknown command. Type 'help'.\r\n");
    }
}

enum class HostInputState : uint8_t {
    Text,
    FrameLength,
    FramePayload,
    FrameCrc,
};

void poll_base_usb(BaseState *state) {
    // USB CDC is a byte stream shared by the typed text CLI and binary host
    // frames. A frame always starts with FRAME_SYNC, a byte outside
    // printable ASCII that a human never types, so the two coexist on one
    // stream without a separate channel.
    static char line[COMMAND_LINE_SIZE];
    static size_t line_length;
    static HostInputState input_state = HostInputState::Text;
    static uint8_t frame_payload[rf_protocol::MAX_PAYLOAD_SIZE];
    static uint8_t frame_length;
    static uint8_t frame_received;
    static uint8_t frame_crc[2];

    int input = 0;
    while ((input = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        const uint8_t byte = static_cast<uint8_t>(input);

        if (input_state == HostInputState::Text) {
            if (byte == rf_protocol::FRAME_SYNC) {
                input_state = HostInputState::FrameLength;
            } else if (input == '\r' || input == '\n') {
                if (line_length != 0) {
                    line[line_length] = '\0';
                    process_base_command_line(line, state);
                    line_length = 0;
                }
            } else if (input == '\b' || input == 0x7F) {
                if (line_length != 0) {
                    --line_length;
                }
            } else if (line_length + 1 < sizeof(line)) {
                line[line_length++] = static_cast<char>(input);
            }
            continue;
        }

        if (input_state == HostInputState::FrameLength) {
            if (byte > rf_protocol::MAX_PAYLOAD_SIZE) {
                // Implausible length; resync on the next FRAME_SYNC byte.
                input_state = HostInputState::Text;
                continue;
            }
            frame_length = byte;
            frame_received = 0;
            input_state = frame_length == 0 ? HostInputState::FrameCrc
                                            : HostInputState::FramePayload;
            continue;
        }

        if (input_state == HostInputState::FramePayload) {
            frame_payload[frame_received++] = byte;
            if (frame_received == frame_length) {
                input_state = HostInputState::FrameCrc;
                frame_received = 0;
            }
            continue;
        }

        // HostInputState::FrameCrc
        frame_crc[frame_received++] = byte;
        if (frame_received < 2) {
            continue;
        }

        input_state = HostInputState::Text;
        uint8_t check_buffer[1 + rf_protocol::MAX_PAYLOAD_SIZE];
        check_buffer[0] = frame_length;
        std::memcpy(check_buffer + 1, frame_payload, frame_length);
        const uint16_t expected =
            static_cast<uint16_t>(frame_crc[0]) |
            (static_cast<uint16_t>(frame_crc[1]) << 8);
        if (expected ==
            crc16_ccitt(check_buffer,
                        static_cast<std::size_t>(frame_length) + 1)) {
            process_base_command_frame(frame_payload, frame_length, state);
        }
        // A CRC mismatch silently drops the frame; the next FRAME_SYNC byte
        // resynchronizes the receiver.
    }
}

}  // namespace

int main() {
    stdio_init_all();

    const Role role = read_role();
    radio_spi.begin(spi0, PIN_RF_SCK, PIN_RF_MOSI, PIN_RF_MISO);
    const bool radio_ready =
        radio.begin(&radio_spi) && configure_radio(role);

    for (uint32_t elapsed_ms = 0;
         elapsed_ms < 5000u && !stdio_usb_connected();
         elapsed_ms += 100u) {
        sleep_ms(100);
    }

    PRINTF("\r\nPico2 V2 RF ACK-payload transport\r\n");
    PRINTF("Role: %s\r\n",
           role == Role::Base ? "base" : "wanderer");
    PRINTF("RF24: %s\r\n",
           radio_ready ? "detected" : "not detected");
    if (role == Role::Base) {
        print_base_help();
    }

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, false);

    WandererState wanderer{};
    wanderer.last_command_ts = get_absolute_time();

    BaseState base{};
    base.telemetry_report_started = get_absolute_time();
    base.next_poll = get_absolute_time();

    if (radio_ready && role == Role::Wanderer) {
        // Without this preload, the first base poll would receive an empty
        // ACK. Later ACK payloads are staged after each command is read.
        stage_next_ack_payload(&wanderer);
        radio.startListening();
    }

    absolute_time_t next_led_toggle = make_timeout_time_ms(500);
    absolute_time_t next_radio_health =
        make_timeout_time_ms(RADIO_HEALTH_PERIOD_MS);
    bool radio_connected = radio_ready;

    while (true) {
        if (radio_ready) {
            if (role == Role::Base) {
                poll_base_usb(&base);
                poll_wanderer(&base);
            } else {
                process_wanderer_radio(&wanderer);
                process_wanderer_failsafe(&wanderer);
            }
        }

        if (time_reached(next_radio_health)) {
            const bool connected = radio.isChipConnected();
            if (connected != radio_connected) {
                radio_connected = connected;
                PRINTF("RF24: %s\r\n",
                       connected ? "detected" : "not detected");
            }
            if (!connected && role == Role::Wanderer) {
                stop_wanderer(&wanderer);
            }
            next_radio_health =
                make_timeout_time_ms(RADIO_HEALTH_PERIOD_MS);
        }

        if (time_reached(next_led_toggle)) {
            gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
            next_led_toggle = make_timeout_time_ms(500);
        }
        tight_loop_contents();
    }
}
