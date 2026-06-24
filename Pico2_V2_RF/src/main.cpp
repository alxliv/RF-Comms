#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "RF24.h"
#include "pico/stdlib.h"
#include "protocol.h"
#include "tactical.h"

using rf_protocol::TacticalState;

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
constexpr uint32_t RADIO_HEALTH_PERIOD_MS = 1000;
constexpr uint32_t TELEMETRY_REPORT_MS = 5000;
constexpr size_t COMMAND_LINE_SIZE = 64;
constexpr uint8_t RADIO_ADDRESS[5] = {'V', '2', 'R', 'F', '1'};

constexpr uint8_t FIRMWARE_MAJOR = 0;
constexpr uint8_t FIRMWARE_MINOR = 3;

enum class Role : uint8_t {
    Wanderer,
    Base,
};

class BaseState {
public:
    BaseState()
        : report_started_(get_absolute_time()),
          next_poll_(get_absolute_time()) {}

    // Sequence values are 8-bit and intentionally wrap from 255 back to 0.
    uint8_t next_command_sequence() { return command_sequence_++; }

    bool poll_due() const { return time_reached(next_poll_); }
    void schedule_poll() { next_poll_ = make_timeout_time_ms(POLL_PERIOD_MS); }

    // Records one received telemetry packet and updates gap statistics.
    // Returns true only for the very first packet ever received: unsigned 8-bit
    // subtraction handles wraparound (last=255, current=0 is an advance of 1,
    // not a gap), and the first packet only seeds the baseline.
    bool record_telemetry(uint8_t sequence) {
        bool first = telemetry_received_ == 0;
        ++telemetry_received_;
        if (!first) {
            const uint8_t advance =
                static_cast<uint8_t>(sequence - last_sequence_);
            if (advance > 1) {
                telemetry_gaps_ += static_cast<uint32_t>(advance - 1u);
            }
        }
        last_sequence_ = sequence;
        return first;
    }

    uint8_t last_flags() const { return last_flags_; }
    void set_last_flags(uint8_t flags) { last_flags_ = flags; }

    bool health_report_due() const {
        return time_reached(delayed_by_ms(report_started_, TELEMETRY_REPORT_MS));
    }
    void print_health();  // defined below, after the PRINTF macro

    // Build and send one command to the Wanderer, stamped with the next
    // sequence number. Each returns whether the nRF24 acknowledged delivery.
    // Query answers (GETVER/GETSTAT) arrive later on the downlink stream, not
    // here. Defined out-of-line below, where transmit_frame() is visible.
    bool tx_nop_cmd() { return tx_header_cmd(rf_protocol::CMD_NOP); }
    bool tx_arm_cmd() { return tx_header_cmd(rf_protocol::CMD_ARM); }
    bool tx_stop_cmd() { return tx_header_cmd(rf_protocol::CMD_STOP); }
    bool tx_getver_cmd() { return tx_header_cmd(rf_protocol::CMD_GETVER); }
    bool tx_getstat_cmd() { return tx_header_cmd(rf_protocol::CMD_GETSTAT); }
    bool tx_move_cmd(int16_t left_mm_s, int16_t right_mm_s);

private:
    bool tx_header_cmd(uint8_t type);

    // The single primitive for talking to the Wanderer: send one frame, track
    // the link from the hardware ACK, and dispatch whatever ACK payload came
    // back. Every command goes through here. Nothing blocks waiting for a
    // reply; replies arrive on the stream on a later poll.
    bool transmit_frame(const void *frame, uint8_t length);
    void update_link(bool delivered);  // link up/down from the ACK result
    void drain_downlink();             // read + dispatch queued ACK payloads

    uint8_t command_sequence_ = 0;
    bool link_up_ = false;
    uint8_t last_sequence_ = 0;
    uint32_t telemetry_received_ = 0;
    uint32_t telemetry_received_at_report_ = 0;
    uint32_t telemetry_gaps_ = 0;
    uint32_t telemetry_gaps_at_report_ = 0;
    uint8_t last_flags_ = 0;
    absolute_time_t report_started_;
    absolute_time_t next_poll_;
};

RF24 radio(PIN_RF_CE, PIN_RF_CSN);
SPI radio_spi;

// The Wanderer's downlink to the base is a single stream of frames carried on
// nRF24 ACK payloads. Telemetry is the default heartbeat; query replies are
// queued here and sent ahead of telemetry so they are never overwritten. This
// is radio-layer staging, not vehicle state, so it lives alongside
// radio/radio_spi rather than inside the TacticalCore.
struct rfFrame {
    uint8_t length;
    uint8_t bytes[rf_protocol::MAX_PAYLOAD_SIZE];
};

class rfQueue {
public:
    // Queues one frame. Returns false if the frame is oversized or the queue is
    // full; the caller decides what to do. The queue is short and drained on
    // every base poll (100 Hz).
    bool push(const void *data, uint8_t length) {
        if (length > rf_protocol::MAX_PAYLOAD_SIZE || count_ == DEPTH) {
            return false;
        }
        const uint8_t slot = (head_ + count_) % DEPTH;
        frames_[slot].length = length;
        std::memcpy(frames_[slot].bytes, data, length);
        ++count_;
        return true;
    }

    bool empty() const { return count_ == 0; }
    const rfFrame &front() const { return frames_[head_]; }

    // Drops the front frame. Returns false and leaves the indexes alone if the
    // queue is empty.
    bool pop() {
        if (count_ == 0) {
            return false;
        }
        head_ = (head_ + 1u) % DEPTH;
        --count_;
        return true;
    }

private:
    static constexpr uint8_t DEPTH = 4;
    rfFrame frames_[DEPTH]{};
    uint8_t head_ = 0;
    uint8_t count_ = 0;
};
rfQueue rf_queue;

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
    // payloads are required because commands and downlink frames have
    // different lengths.
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

// ---------------------------------------------------------------------------
// Wanderer: downlink staging
// ---------------------------------------------------------------------------

// Maps the FSM state to the wire flag bits shared by telemetry and GETSTAT.
// WAND_MOVING means actually moving (motors live with a non-zero target),
// distinct from WAND_ARMED (motors live, allowed to move). This is reporting
// code, not vehicle state, so it reads the TacticalCore rather than living
// inside it. The full FSM state also rides telemetry as tactical_state.
uint8_t wanderer_flags(const TacticalCore *core) {
    uint8_t flags = 0;
    if (core->motors_enabled()) {
        flags |= rf_protocol::WAND_ARMED;
    }
    if (core->motors_enabled() &&
        (core->target_left() != 0 || core->target_right() != 0)) {
        flags |= rf_protocol::WAND_MOVING;
    }
    return flags;
}

// Assembles the telemetry heartbeat from every source that contributes to it.
// Today that is just the TacticalCore's FSM state and command-state flags;
// battery, odometry, and the rest get folded in here as their hardware lands.
// It lives outside the TacticalCore precisely because telemetry aggregates
// multiple sources. The sequence counter belongs to the telemetry stream, not
// the vehicle, so it lives here too.
rf_protocol::Telemetry build_telemetry(const TacticalCore *core) {
    static uint8_t sequence = 0;
    rf_protocol::Telemetry telemetry{};
    telemetry.type = rf_protocol::REPLY_TELEMETRY;
    telemetry.sequence = sequence++;
    telemetry.flags = wanderer_flags(core);
    telemetry.tactical_state = static_cast<uint8_t>(core->state());
    return telemetry;
}

// writeAckPayload() only queues data. The nRF24 sends it automatically with
// the ACK for the next command received on RADIO_PIPE.
//
// Important: the ACK for the command currently being read has already gone
// over the air. The payload staged here is therefore always for the following
// base poll. This one-poll delay is normal. A queued reply rides ahead of the
// telemetry heartbeat and is dropped from the queue whether or not the radio
// accepts it -- we never re-transmit an ACK payload; the base re-asks if it
// cares.
bool stage_next_ack_payload(TacticalCore *core) {
    if (!rf_queue.empty()) {
        rfFrame frame = rf_queue.front();
        rf_queue.pop();
        return radio.writeAckPayload(RADIO_PIPE, frame.bytes, frame.length);
    }

    rf_protocol::Telemetry telemetry = build_telemetry(core);
    return radio.writeAckPayload(RADIO_PIPE, &telemetry, sizeof(telemetry));
}

// Discards whatever is already staged in the TX FIFO and stages a fresh
// frame. Used after a state change so the next ACK reflects current state
// rather than a payload built before the change.
void restage_ack(TacticalCore *core) {
    radio.flush_tx();
    stage_next_ack_payload(core);
}

// ---------------------------------------------------------------------------
// Wanderer: command handling
// ---------------------------------------------------------------------------

bool handle_wanderer_command(const uint8_t *payload, uint8_t length,
                             TacticalCore *core, absolute_time_t now) {
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
            core->cmd_stop(now);
            PRINTF("Command STOP seq=%u: disarmed, targets cleared\r\n",
                   header.sequence);
            return true;

        case rf_protocol::CMD_ARM:
            if (length != sizeof(rf_protocol::CommandHeader)) {
                return false;
            }
            core->cmd_arm(now);
            PRINTF("Command ARM seq=%u: armed\r\n", header.sequence);
            return true;

        case rf_protocol::CMD_MOVE: {
            if (length != sizeof(rf_protocol::MoveCommand)) {
                return false;
            }
            rf_protocol::MoveCommand command{};
            std::memcpy(&command, payload, sizeof(command));
            if (core->cmd_move(command.velocity_left_mm_s,
                               command.velocity_right_mm_s, now)) {
                PRINTF("Command MOVE seq=%u: left=%d right=%d mm/s\r\n",
                       command.header.sequence, command.velocity_left_mm_s,
                       command.velocity_right_mm_s);
            } else {
                PRINTF("Command MOVE seq=%u ignored: Wanderer is not active\r\n",
                       command.header.sequence);
            }
            return true;
        }

        case rf_protocol::CMD_GETVER:
            if (length != sizeof(rf_protocol::CommandHeader)) {
                return false;
            }
            {
                rf_protocol::VersionReply reply{
                    rf_protocol::REPLY_VERSION,
                    FIRMWARE_MAJOR,
                    FIRMWARE_MINOR,
                };
                rf_queue.push(&reply, sizeof(reply));
            }
            PRINTF("Command GETVER seq=%u: version reply queued\r\n",
                   header.sequence);
            return true;

        case rf_protocol::CMD_GETSTAT:
            if (length != sizeof(rf_protocol::CommandHeader)) {
                return false;
            }
            {
                rf_protocol::StatReply reply{
                    rf_protocol::REPLY_STAT,
                    wanderer_flags(core),
                    core->target_left(),
                    core->target_right(),
                };
                rf_queue.push(&reply, sizeof(reply));
            }
            PRINTF("Command GETSTAT seq=%u: stat reply queued\r\n",
                   header.sequence);
            return true;

        default:
            return false;
    }
}

void process_wanderer_radio(TacticalCore *core) {
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
            restage_ack(core);
            return;
        }

        uint8_t payload[rf_protocol::MAX_PAYLOAD_SIZE]{};
        radio.read(payload, length);
        const absolute_time_t now = get_absolute_time();
        core->note_commander_alive(now);
        if (pipe == RADIO_PIPE) {
            handle_wanderer_command(payload, length, core, now);
        }

        if (!stage_next_ack_payload(core)) {
            // The ACK payload uses the three-entry TX FIFO. A failed queue
            // operation means stale entries filled it; discard them and put
            // back one fresh frame.
            restage_ack(core);
        }
    }
}

// ---------------------------------------------------------------------------
// Base: link tracking and telemetry health
// ---------------------------------------------------------------------------

// Prints the measured frames/second over the latest interval and resets the
// window. The first gap number is new gaps in this interval; the second is the
// total since boot.
void BaseState::print_health() {
    const absolute_time_t now = get_absolute_time();
    const int64_t elapsed_us = absolute_time_diff_us(report_started_, now);
    const uint32_t interval_frames =
        telemetry_received_ - telemetry_received_at_report_;
    const uint32_t interval_gaps = telemetry_gaps_ - telemetry_gaps_at_report_;

    uint32_t rate_tenths = 0;
    if (elapsed_us > 0) {
        rate_tenths = static_cast<uint32_t>(
            (static_cast<uint64_t>(interval_frames) * 10000000u) /
            static_cast<uint64_t>(elapsed_us));
    }

    PRINTF("Telemetry OK: rate=%lu.%lu Hz total=%lu gaps=%lu/%lu\r\n",
           static_cast<unsigned long>(rate_tenths / 10u),
           static_cast<unsigned long>(rate_tenths % 10u),
           static_cast<unsigned long>(telemetry_received_),
           static_cast<unsigned long>(interval_gaps),
           static_cast<unsigned long>(telemetry_gaps_));

    telemetry_received_at_report_ = telemetry_received_;
    telemetry_gaps_at_report_ = telemetry_gaps_;
    report_started_ = now;
}

void report_wanderer_state(uint8_t flags, bool first_telemetry,
                           BaseState *state) {
    // Normal state is not printed repeatedly. State transitions are more
    // useful to a person watching the terminal than five-second repetitions.
    bool running = (flags & rf_protocol::WAND_MOVING) != 0;

    if (first_telemetry) {
        // The very first telemetry packet has no real prior flags to diff
        // against, so report the starting state instead of a transition.
        PRINTF("Wanderer state: running=%s\r\n", running ? "yes" : "no");
        return;
    }

    bool was_running =
        (state->last_flags() & rf_protocol::WAND_MOVING) != 0;

    if (running != was_running) {
        PRINTF("Wanderer movement state: %s\r\n",
               running ? "running" : "stopped");
    }
}

// ---------------------------------------------------------------------------
// Base: the one downlink dispatcher
// ---------------------------------------------------------------------------

// Every frame the Wanderer sends arrives here, identified by its type byte.
// Telemetry feeds monitoring; query replies are handled if known and logged if
// not. Each frame is also forwarded verbatim to the host so a laptop client
// sees the same asynchronous stream the base does.
void dispatch_downlink_frame(const uint8_t *payload, uint8_t length,
                             BaseState *state) {
    if (length == 0) {
        return;
    }

    switch (payload[0]) {
        case rf_protocol::REPLY_TELEMETRY: {
            if (length != sizeof(rf_protocol::Telemetry)) {
                return;
            }
            rf_protocol::Telemetry telemetry{};
            std::memcpy(&telemetry, payload, sizeof(telemetry));
            bool first_telemetry =
                state->record_telemetry(telemetry.sequence);
            report_wanderer_state(telemetry.flags, first_telemetry, state);
            state->set_last_flags(telemetry.flags);
            send_host_frame(&telemetry, sizeof(telemetry));

            if (state->health_report_due()) {
                state->print_health();
            }
            return;
        }

        case rf_protocol::REPLY_VERSION: {
            if (length != sizeof(rf_protocol::VersionReply)) {
                return;
            }
            rf_protocol::VersionReply version{};
            std::memcpy(&version, payload, sizeof(version));
            PRINTF("Wanderer version=%u.%u\r\n", version.firmware_major,
                   version.firmware_minor);
            send_host_frame(&version, sizeof(version));
            return;
        }

        case rf_protocol::REPLY_STAT: {
            if (length != sizeof(rf_protocol::StatReply)) {
                return;
            }
            rf_protocol::StatReply stat{};
            std::memcpy(&stat, payload, sizeof(stat));
            PRINTF("Wanderer stat: armed=%s running=%s target=%d/%d mm/s\r\n",
                   (stat.flags & rf_protocol::WAND_ARMED) ? "yes" : "no",
                   (stat.flags & rf_protocol::WAND_MOVING) ? "yes" : "no",
                   stat.target_left_mm_s, stat.target_right_mm_s);
            send_host_frame(&stat, sizeof(stat));
            return;
        }

        default:
            PRINTF("Downlink: unknown frame type=0x%02X len=%u\r\n",
                   payload[0], length);
            return;
    }
}

void BaseState::update_link(bool delivered) {
    // radio.write() returning false means no ACK arrived. It says the RF link
    // failed, not necessarily that this base radio is broken.
    if (!delivered) {
        if (link_up_) {
            link_up_ = false;
            PRINTF("Base: link lost\r\n");
            const rf_protocol::LinkLostNotice notice{
                rf_protocol::REPLY_LINK_LOST,
            };
            send_host_frame(&notice, sizeof(notice));
        }
        return;
    }

    if (!link_up_) {
        link_up_ = true;
        PRINTF("Base: link established\r\n");
    }
}

void BaseState::drain_downlink() {
    while (radio.available()) {
        const uint8_t length = radio.getDynamicPayloadSize();
        if (length == 0) {
            // getDynamicPayloadSize() already flushed RX itself on a corrupted
            // R_RX_PL_WID read; nothing left to recover here.
            return;
        }

        uint8_t payload[rf_protocol::MAX_PAYLOAD_SIZE]{};
        radio.read(payload, length);
        dispatch_downlink_frame(payload, length, this);
    }
}

// ---------------------------------------------------------------------------
// Base: commands (all fire-and-forget)
// ---------------------------------------------------------------------------

// The BaseState command methods live here, where the dispatch_downlink_frame()
// they reach through drain_downlink() is visible. The text CLI and the binary
// host frame path both call the tx_*_cmd() methods, so the two interfaces
// cannot diverge in RF behavior.

bool BaseState::transmit_frame(const void *frame, uint8_t length) {
    bool delivered = radio.write(frame, length);
    update_link(delivered);
    if (delivered) {
        drain_downlink();
    }
    return delivered;
}

bool BaseState::tx_header_cmd(uint8_t type) {
    rf_protocol::CommandHeader command{type, next_command_sequence()};
    return transmit_frame(&command, sizeof(command));
}

bool BaseState::tx_move_cmd(int16_t left_mm_s, int16_t right_mm_s) {
    rf_protocol::MoveCommand command{
        {rf_protocol::CMD_MOVE, next_command_sequence()},
        left_mm_s,
        right_mm_s,
    };
    return transmit_frame(&command, sizeof(command));
}

void poll_wanderer(BaseState *state) {
    if (!state->poll_due()) {
        return;
    }
    state->schedule_poll();

    // NOP has no vehicle action. Its purpose is to clock the next downlink
    // frame (telemetry or a queued reply) back to the base.
    state->tx_nop_cmd();
}

void print_base_help() {
    PRINTF("Commands:\r\n");
    PRINTF("  help          Show commands\r\n");
    PRINTF("  arm           Allow the Wanderer to move\r\n");
    PRINTF("  stop          Stop and disarm Wanderer\r\n");
    PRINTF("  move L R      Set signed left/right velocity in mm/s\r\n");
    PRINTF("  getver        Request firmware version (async reply)\r\n");
    PRINTF("  getstat       Request Wanderer status (async reply)\r\n");
}

void process_base_command_frame(const uint8_t *payload, uint8_t length,
                                BaseState *state) {
    // The binary host frame protocol carries the same command structs used
    // over RF, just over USB CDC. Like the text CLI, every command is
    // fire-and-forget: the host sends and watches the forwarded downlink
    // stream for telemetry, replies, and events.
    if (length < sizeof(rf_protocol::CommandHeader)) {
        return;
    }

    rf_protocol::CommandHeader header{};
    std::memcpy(&header, payload, sizeof(header));

    if (header.type == rf_protocol::CMD_MOVE) {
        if (length != sizeof(rf_protocol::MoveCommand)) {
            return;
        }
        rf_protocol::MoveCommand command{};
        std::memcpy(&command, payload, sizeof(command));
        state->tx_move_cmd(command.velocity_left_mm_s,
                           command.velocity_right_mm_s);
        return;
    }

    if (length != sizeof(header)) {
        return;
    }
    switch (header.type) {
        case rf_protocol::CMD_ARM:
            state->tx_arm_cmd();
            return;
        case rf_protocol::CMD_STOP:
            state->tx_stop_cmd();
            return;
        case rf_protocol::CMD_GETVER:
            state->tx_getver_cmd();
            return;
        case rf_protocol::CMD_GETSTAT:
            state->tx_getstat_cmd();
            return;
        default:
            return;
    }
}

void process_base_command_line(char *line, BaseState *state) {
    // This text CLI remains for manual debugging over a serial terminal. The
    // binary host frame protocol (process_base_command_frame) carries the
    // same command set for host software on the same USB CDC link.
    if (std::strcmp(line, "help") == 0) {
        print_base_help();
    } else if (std::strcmp(line, "arm") == 0) {
        PRINTF(state->tx_arm_cmd()
                   ? "ARM delivered\r\n"
                   : "ARM failed: no acknowledgement\r\n");
    } else if (std::strcmp(line, "stop") == 0) {
        PRINTF(state->tx_stop_cmd()
                   ? "STOP delivered\r\n"
                   : "STOP failed: no acknowledgement\r\n");
    } else if (std::strcmp(line, "getver") == 0) {
        PRINTF(state->tx_getver_cmd()
                   ? "GETVER sent; version will follow\r\n"
                   : "GETVER failed: no acknowledgement\r\n");
    } else if (std::strcmp(line, "getstat") == 0) {
        PRINTF(state->tx_getstat_cmd()
                   ? "GETSTAT sent; status will follow\r\n"
                   : "GETSTAT failed: no acknowledgement\r\n");
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

        PRINTF(state->tx_move_cmd(static_cast<int16_t>(left),
                                  static_cast<int16_t>(right))
                   ? "MOVE delivered\r\n"
                   : "MOVE failed: no acknowledgement\r\n");
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
    // frames. A frame always starts with FRAME_SYNC, a byte outside printable
    // ASCII that a human never types, so the two coexist on one stream.
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
    bool radio_ready =
        radio.begin(&radio_spi) && configure_radio(role);

    for (uint32_t elapsed_ms = 0;
         elapsed_ms < 5000u && !stdio_usb_connected();
         elapsed_ms += 100u) {
        sleep_ms(100);
    }

    PRINTF("\r\nPico2 V2 RF ACK-payload transport\r\n");
    PRINTF("Role: %s\r\n", role == Role::Base ? "base" : "wanderer");
    PRINTF("RF24: %s\r\n", radio_ready ? "detected" : "not detected");
    if (role == Role::Base) {
        print_base_help();
    }

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, false);

    TacticalCore wanderer;
    BaseState base;

    if (radio_ready && role == Role::Wanderer) {
        // Without this preload, the first base poll would receive an empty
        // ACK. Later frames are staged after each command is read.
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
                TacticalState before = wanderer.state();
                process_wanderer_radio(&wanderer);
                wanderer.tick(get_absolute_time());
                if (wanderer.state() != before) {
                    PRINTF("TacticalState changed. From %u to %u\r\n",
                           static_cast<unsigned>(before),
                           static_cast<unsigned>(wanderer.state()));
                }

            }
        }

        if (time_reached(next_radio_health)) {
            bool connected = radio.isChipConnected();
            if (connected != radio_connected) {
                radio_connected = connected;
                PRINTF("RF24: %s\r\n",
                       connected ? "detected" : "not detected");
            }
            next_radio_health = make_timeout_time_ms(RADIO_HEALTH_PERIOD_MS);
        }

        if (time_reached(next_led_toggle)) {
            gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
            next_led_toggle = make_timeout_time_ms(500);
        }
        tight_loop_contents();
    }
}
