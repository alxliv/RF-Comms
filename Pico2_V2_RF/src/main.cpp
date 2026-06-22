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
constexpr uint32_t ARM_CONFIRM_TIMEOUT_MS = 500;
constexpr size_t COMMAND_LINE_SIZE = 64;
constexpr uint8_t RADIO_ADDRESS[5] = {'V', '2', 'R', 'F', '1'};

constexpr uint8_t FIRMWARE_MAJOR = 0;
constexpr uint8_t FIRMWARE_MINOR = 2;

enum class Role : uint8_t {
    Wanderer,
    Base,
};

struct PendingReply {
    bool active;
    uint8_t type;
};

struct WandererState {
    // These are currently command/telemetry state only. Real motor-control
    // outputs and sensor readings will be connected later.
    bool armed;
    bool estop;
    PendingReply pending_reply;
    uint8_t telemetry_sequence;
    int16_t target_left_mm_s;
    int16_t target_right_mm_s;
    absolute_time_t last_valid_command;
};

struct BaseState {
    // Command and telemetry sequence values are 8-bit and intentionally wrap
    // from 255 back to 0.
    uint8_t command_sequence;
    bool link_up;
    bool arm_confirmation_pending;
    bool telemetry_sequence_valid;
    bool telemetry_flags_valid;
    uint8_t last_telemetry_sequence;
    uint32_t telemetry_received;
    uint32_t telemetry_received_at_report;
    uint32_t telemetry_gaps;
    uint32_t telemetry_gaps_at_report;
    uint8_t last_telemetry_flags;
    absolute_time_t arm_confirmation_deadline;
    absolute_time_t telemetry_report_started;
    absolute_time_t next_telemetry_report;
    absolute_time_t next_poll;
};

RF24 radio(PIN_RF_CE, PIN_RF_CSN);
SPI radio_spi;

}  // namespace

#define PRINTF(...)                   \
    do {                              \
        if (stdio_usb_connected()) {  \
            std::printf(__VA_ARGS__); \
        }                             \
    } while (0)

namespace {

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
    if (state->pending_reply.active) {
        state->pending_reply.active = false;

        switch (state->pending_reply.type) {
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
            state->pending_reply = {
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
        if (length == 0 || length > rf_protocol::MAX_PAYLOAD_SIZE) {
            radio.flush_rx();
            radio.flush_tx();
            stage_next_ack_payload(state);
            return;
        }

        uint8_t payload[rf_protocol::MAX_PAYLOAD_SIZE]{};
        radio.read(payload, length);

        // Only a valid, recognized command refreshes the safety watchdog.
        if (pipe == RADIO_PIPE &&
            handle_wanderer_command(payload, length, state)) {
            state->last_valid_command = get_absolute_time();
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
    if (absolute_time_diff_us(state->last_valid_command,
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

void update_telemetry_sequence(uint8_t sequence, BaseState *state) {
    // Unsigned 8-bit subtraction handles normal wraparound automatically.
    // Example: last=255, current=0 gives an advance of 1, not a gap.
    if (state->telemetry_sequence_valid) {
        const uint8_t advance =
            static_cast<uint8_t>(sequence -
                                 state->last_telemetry_sequence);
        if (advance > 1) {
            state->telemetry_gaps +=
                static_cast<uint32_t>(advance - 1u);
        }
    } else {
        state->telemetry_sequence_valid = true;
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
    state->next_telemetry_report =
        make_timeout_time_ms(TELEMETRY_REPORT_MS);
}

void report_wanderer_state(uint8_t flags, BaseState *state) {
    // Normal state is not printed repeatedly. State transitions are more
    // useful to a person watching the terminal than five-second repetitions.
    const bool running =
        (flags & rf_protocol::FLAG_RUNNING) != 0;
    const bool estop =
        (flags & rf_protocol::FLAG_ESTOP) != 0;

    if (!state->telemetry_flags_valid) {
        state->telemetry_flags_valid = true;
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
        ++state->telemetry_received;
        update_telemetry_sequence(telemetry.sequence, state);
        report_wanderer_state(telemetry.flags, state);
        state->last_telemetry_flags = telemetry.flags;

        if (state->arm_confirmation_pending &&
            (telemetry.flags & rf_protocol::FLAG_ESTOP) == 0) {
            state->arm_confirmation_pending = false;
            PRINTF("ARM confirmed: FLAG_ESTOP cleared\r\n");
        }

        if (time_reached(state->next_telemetry_report)) {
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
        if (length == 0 || length > rf_protocol::MAX_PAYLOAD_SIZE) {
            radio.flush_rx();
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
                if (length == 0 ||
                    length > rf_protocol::MAX_PAYLOAD_SIZE) {
                    radio.flush_rx();
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

void print_base_help() {
    PRINTF("Commands:\r\n");
    PRINTF("  help          Show commands\r\n");
    PRINTF("  arm           Clear Wanderer emergency stop\r\n");
    PRINTF("  stop          Stop and disarm Wanderer\r\n");
    PRINTF("  move L R      Set signed left/right velocity in mm/s\r\n");
    PRINTF("  getver        Request Wanderer firmware version\r\n");
}

void send_simple_command(uint8_t command_type, const char *name,
                         BaseState *state) {
    // Hardware ACK proves that the Wanderer radio received this command.
    // Commands that need application-level data, such as GETVER, instead use
    // request_reply() and wait for a matching typed ACK payload.
    const rf_protocol::CommandHeader command{
        command_type,
        state->command_sequence++,
    };
    if (transmit_base_frame(&command, sizeof(command), state)) {
        PRINTF("%s delivered\r\n", name);
    } else {
        PRINTF("%s failed: no acknowledgement\r\n", name);
    }
}

void process_base_command_line(char *line, BaseState *state) {
    // This text CLI is temporary development tooling. Step 5 replaces it with
    // the framed binary laptop protocol.
    if (std::strcmp(line, "help") == 0) {
        print_base_help();
    } else if (std::strcmp(line, "arm") == 0) {
        const rf_protocol::CommandHeader command{
            rf_protocol::CMD_ARM,
            state->command_sequence++,
        };
        if (transmit_base_frame(&command, sizeof(command), state)) {
            state->arm_confirmation_pending = true;
            state->arm_confirmation_deadline =
                make_timeout_time_ms(ARM_CONFIRM_TIMEOUT_MS);
            PRINTF("ARM delivered; waiting for telemetry confirmation\r\n");
        } else {
            PRINTF("ARM failed: no acknowledgement\r\n");
        }
    } else if (std::strcmp(line, "stop") == 0) {
        state->arm_confirmation_pending = false;
        send_simple_command(rf_protocol::CMD_STOP, "STOP", state);
    } else if (std::strcmp(line, "getver") == 0) {
        rf_protocol::VersionReply version{};
        if (request_reply(rf_protocol::CMD_GETVER,
                          rf_protocol::REPLY_VERSION,
                          sizeof(version), &version, sizeof(version),
                          REQUEST_TIMEOUT_MS, state)) {
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

        const rf_protocol::MoveCommand command{
            rf_protocol::CMD_MOVE,
            state->command_sequence++,
            static_cast<int16_t>(left),
            static_cast<int16_t>(right),
        };
        if (transmit_base_frame(&command, sizeof(command), state)) {
            PRINTF("MOVE delivered left=%ld right=%ld\r\n",
                   left, right);
        } else {
            PRINTF("MOVE failed: no acknowledgement\r\n");
        }
    } else if (line[0] != '\0') {
        PRINTF("Unknown command. Type 'help'.\r\n");
    }
}

void poll_base_usb(BaseState *state) {
    // USB CDC is a byte stream. Accumulate a line without blocking the 100 Hz
    // RF polling loop.
    static char line[COMMAND_LINE_SIZE];
    static size_t length;

    int input = 0;
    while ((input = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (input == '\r' || input == '\n') {
            if (length != 0) {
                line[length] = '\0';
                process_base_command_line(line, state);
                length = 0;
            }
        } else if (input == '\b' || input == 0x7F) {
            if (length != 0) {
                --length;
            }
        } else if (length + 1 < sizeof(line)) {
            line[length++] = static_cast<char>(input);
        }
    }
}

void process_base_timeouts(BaseState *state) {
    if (state->arm_confirmation_pending &&
        time_reached(state->arm_confirmation_deadline)) {
        state->arm_confirmation_pending = false;
        PRINTF("ARM telemetry confirmation timeout\r\n");
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
    wanderer.last_valid_command = get_absolute_time();

    BaseState base{};
    base.telemetry_report_started = get_absolute_time();
    base.next_telemetry_report =
        make_timeout_time_ms(TELEMETRY_REPORT_MS);
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
                process_base_timeouts(&base);
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
