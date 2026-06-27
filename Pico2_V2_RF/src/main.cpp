#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "RF24.h"
#include "pico/stdlib.h"
#include "protocol.h"
#include "tactical.h"

using rf_protocol::TacticalState;

namespace {

// Pins are chosen to clear the Wanderer's occupied GPIOs (motors GP16/17/19/20,
// encoders GP10-13, I2C GP4-7, ToF GP8, UART GP0/1). SPI0's MISO/RX function
// only reaches GP0/4/16/20 -- all taken on the Wanderer -- so the radio runs on
// spi1 (see SPI_INSTANCE below). SCK/MOSI/MISO must be valid spi1 pins; CE, CSN,
// and ROLE are plain GPIO and can be any free pin.
constexpr uint8_t PIN_RF_CE = 21;
constexpr uint8_t PIN_RF_CSN = 9;     // spi1 CSn (driven in software by RF24)
constexpr uint8_t PIN_RF_MISO = 28;   // spi1 RX (only free spi1 RX pin)
constexpr uint8_t PIN_RF_SCK = 14;    // spi1 SCK
constexpr uint8_t PIN_RF_MOSI = 15;   // spi1 TX
constexpr uint8_t PIN_ROLE = 22;
constexpr uint8_t PIN_LINK_LED = 2;  // Wanderer-only: RF link-good indicator

// Both boards run the same firmware. PIN_ROLE selects what this board does:
// high = laptop-side base, low = vehicle-side Wanderer.
constexpr uint8_t RADIO_CHANNEL = 76;
constexpr uint32_t LINK_LED_PERIOD_MS = 250;  // ~2 blinks/sec while link is up
constexpr uint8_t RADIO_PIPE = 1;
constexpr uint32_t POLL_PERIOD_MS = 10;
constexpr uint32_t RADIO_HEALTH_PERIOD_MS = 1000;
constexpr uint32_t RF_REPORT_PERIOD_MS = 1000;  // `rf on` auto-repeat cadence
constexpr size_t COMMAND_LINE_SIZE = 64;
constexpr uint8_t RADIO_ADDRESS[5] = {'V', '2', 'R', 'F', '1'};

constexpr uint8_t FIRMWARE_MAJOR = 0;
constexpr uint8_t FIRMWARE_MINOR = 7;

enum class Role : uint8_t {
    Wanderer,
    Base,
};

class BaseState {
public:
    BaseState() : next_poll_(get_absolute_time()) {}

    // Sequence values are 8-bit and intentionally wrap from 255 back to 0.
    uint8_t next_command_sequence() { return command_sequence_++; }

    bool poll_due() const { return time_reached(next_poll_); }
    void schedule_poll() { next_poll_ = make_timeout_time_ms(POLL_PERIOD_MS); }

    // Telemetry forwarding control: the `tlm` command. The base always polls
    // the Wanderer at POLL_PERIOD_MS regardless; this only decimates what is
    // forwarded to the laptop as `#` lines. Default off, so a human attaching a
    // terminal gets a clean console instead of the full-rate firehose.
    void telemetry_off() { forward_on_ = false; }
    void telemetry_every_frame() {
        forward_on_ = true;
        forward_interval_ms_ = 0;
    }
    void telemetry_rate(uint32_t hz) {
        forward_on_ = true;
        forward_interval_ms_ = hz != 0 ? 1000u / hz : 0;
        next_forward_ = get_absolute_time();
    }
    // True when the `#` line for the frame just received should be forwarded.
    bool forward_telemetry_due() {
        if (!forward_on_) return false;
        if (forward_interval_ms_ == 0) return true;
        if (!time_reached(next_forward_)) return false;
        next_forward_ = make_timeout_time_ms(forward_interval_ms_);
        return true;
    }

    // `!state` is derived by watching tactical_state change across the
    // telemetry the base receives internally, so it fires even while `#`
    // forwarding is off. Defined out-of-line below, where the PRINTF macro and
    // the state-name helper are visible.
    void observe_telemetry_state(uint8_t state);

    // nRF24 link statistics (the `rf` command). Every frame the base sends is
    // counted by delivery so the ACK success ratio reflects link quality --
    // the primary metric for a range test. `rf on` repeats the report once a
    // second; `report_rf_stats()` emits one `>rf` line and resets the window.
    void rf_auto_enable() {
        rf_auto_ = true;
        next_rf_report_ = make_timeout_time_ms(RF_REPORT_PERIOD_MS);
    }
    void rf_auto_disable() { rf_auto_ = false; }
    bool rf_report_due() {
        if (!rf_auto_ || !time_reached(next_rf_report_)) return false;
        next_rf_report_ = make_timeout_time_ms(RF_REPORT_PERIOD_MS);
        return true;
    }
    void report_rf_stats();

    // Build and send one command to the Wanderer, stamped with the next
    // sequence number. Each returns whether the nRF24 acknowledged delivery,
    // but the text `=ok` ack does not depend on that -- it is emitted before
    // the frame goes over the air. Query answers (GETVER/GETSTAT) arrive later
    // on the downlink stream. Defined out-of-line below, where transmit_frame()
    // is visible.
    bool tx_nop_cmd() { return tx_header_cmd(rf_protocol::CMD_NOP); }
    bool tx_arm_cmd() { return tx_header_cmd(rf_protocol::CMD_ARM); }
    bool tx_stop_cmd() { return tx_header_cmd(rf_protocol::CMD_STOP); }
    bool tx_getver_cmd() { return tx_header_cmd(rf_protocol::CMD_GETVER); }
    bool tx_getstat_cmd() { return tx_header_cmd(rf_protocol::CMD_GETSTAT); }
    bool tx_getpa_cmd() { return tx_header_cmd(rf_protocol::CMD_GETPA); }
    bool tx_move_cmd(int16_t left_mm_s, int16_t right_mm_s);
    bool tx_setpa_cmd(uint8_t pa_level);

private:
    bool tx_header_cmd(uint8_t type);

    // The single primitive for talking to the Wanderer: send one frame, track
    // the link from the hardware ACK, and dispatch whatever ACK payload came
    // back. Every command goes through here. Nothing blocks waiting for a
    // reply; replies arrive on the stream on a later poll.
    bool transmit_frame(const void *frame, uint8_t length);
    void update_link(bool delivered);  // link up/down -> `!link` events
    void drain_downlink();             // read + dispatch queued ACK payloads

    uint8_t command_sequence_ = 0;
    bool link_up_ = false;
    bool forward_on_ = false;           // `#` off until `tlm on|<hz>`
    uint32_t forward_interval_ms_ = 0;  // 0 = forward every frame
    bool have_state_ = false;           // seen at least one telemetry frame
    uint8_t last_state_ = 0;            // last tactical_state, for `!state`
    // RF link counters: lifetime totals and the snapshot taken at the last
    // `>rf` report, so each report covers only the interval since the previous.
    uint32_t tx_total_ = 0;
    uint32_t tx_ok_ = 0;
    uint32_t tx_total_at_report_ = 0;
    uint32_t tx_ok_at_report_ = 0;
    bool rf_auto_ = false;
    absolute_time_t next_rf_report_ = {};
    absolute_time_t next_forward_ = {};
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

        case rf_protocol::CMD_GETPA:
            if (length != sizeof(rf_protocol::CommandHeader)) {
                return false;
            }
            {
                rf_protocol::PaReply reply{
                    rf_protocol::REPLY_PA,
                    radio.getPALevel(),
                };
                rf_queue.push(&reply, sizeof(reply));
            }
            PRINTF("Command GETPA seq=%u: pa reply queued\r\n",
                   header.sequence);
            return true;

        case rf_protocol::CMD_SETPA: {
            if (length != sizeof(rf_protocol::SetPaCommand)) {
                return false;
            }
            rf_protocol::SetPaCommand command{};
            std::memcpy(&command, payload, sizeof(command));
            if (command.pa_level <= 3) {
                radio.setPALevel(command.pa_level);
            }
            // Answer with the level actually in effect, so "try to set" is
            // confirmed by read-back rather than assumed. An out-of-range value
            // leaves the PA untouched and the reply shows it stayed put.
            rf_protocol::PaReply reply{
                rf_protocol::REPLY_PA,
                radio.getPALevel(),
            };
            rf_queue.push(&reply, sizeof(reply));
            PRINTF("Command SETPA seq=%u: pa=%u applied, reply queued\r\n",
                   command.header.sequence, radio.getPALevel());
            return true;
        }

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
// Base: link tracking and telemetry-derived events
// ---------------------------------------------------------------------------

// The FSM state name carried in `#` telemetry and `!state` events. Names, not
// raw enum numbers, per PROTOCOL.md -- readability is the point on this link.
const char *tactical_state_name(uint8_t state) {
    switch (static_cast<TacticalState>(state)) {
        case TacticalState::Safe:     return "SAFE";
        case TacticalState::Active:   return "ACTIVE";
        case TacticalState::Fallback: return "FALLBACK";
        case TacticalState::Fault:    return "FAULT";
    }
    return "?";
}

// Emits `!state from=.. to=..` when the Wanderer's FSM state changes. Driven by
// every telemetry frame the base receives internally, so the event fires even
// when `#` forwarding is off. The first frame only seeds the baseline.
void BaseState::observe_telemetry_state(uint8_t state) {
    if (have_state_ && state != last_state_) {
        PRINTF("!state from=%s to=%s\r\n", tactical_state_name(last_state_),
               tactical_state_name(state));
    }
    have_state_ = true;
    last_state_ = state;
}

// ---------------------------------------------------------------------------
// Base: the one downlink dispatcher
// ---------------------------------------------------------------------------

// Every frame the Wanderer sends arrives here, identified by its type byte, and
// is translated into one text line for the laptop: telemetry to `#` (subject to
// `tlm` forwarding control and feeding `!state` events), query answers to `>`.
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
            // Event derivation runs on every frame, independent of forwarding.
            state->observe_telemetry_state(telemetry.tactical_state);
            if (state->forward_telemetry_due()) {
                PRINTF("#seq=%u state=%s armed=%d moving=%d\r\n",
                       telemetry.sequence,
                       tactical_state_name(telemetry.tactical_state),
                       (telemetry.flags & rf_protocol::WAND_ARMED) ? 1 : 0,
                       (telemetry.flags & rf_protocol::WAND_MOVING) ? 1 : 0);
            }
            return;
        }

        case rf_protocol::REPLY_VERSION: {
            if (length != sizeof(rf_protocol::VersionReply)) {
                return;
            }
            rf_protocol::VersionReply version{};
            std::memcpy(&version, payload, sizeof(version));
            PRINTF(">ver fw=%u.%u\r\n", version.firmware_major,
                   version.firmware_minor);
            return;
        }

        case rf_protocol::REPLY_STAT: {
            if (length != sizeof(rf_protocol::StatReply)) {
                return;
            }
            rf_protocol::StatReply stat{};
            std::memcpy(&stat, payload, sizeof(stat));
            PRINTF(">stat armed=%d moving=%d vL=%d vR=%d\r\n",
                   (stat.flags & rf_protocol::WAND_ARMED) ? 1 : 0,
                   (stat.flags & rf_protocol::WAND_MOVING) ? 1 : 0,
                   stat.target_left_mm_s, stat.target_right_mm_s);
            return;
        }

        case rf_protocol::REPLY_PA: {
            if (length != sizeof(rf_protocol::PaReply)) {
                return;
            }
            rf_protocol::PaReply pa{};
            std::memcpy(&pa, payload, sizeof(pa));
            PRINTF(">pa=%u\r\n", pa.pa_level);
            return;
        }

        default:
            PRINTF("*downlink: unknown frame type=0x%02X len=%u\r\n",
                   payload[0], length);
            return;
    }
}

void BaseState::update_link(bool delivered) {
    // Every transmit is one sample of link quality: counted here so the `rf`
    // report's ACK ratio reflects the NOP poll stream plus any commands.
    ++tx_total_;
    if (delivered) {
        ++tx_ok_;
    }

    // radio.write() returning false means no ACK arrived. It says the RF link
    // failed, not necessarily that this base radio is broken.
    if (!delivered) {
        if (link_up_) {
            link_up_ = false;
            PRINTF("!link down\r\n");
        }
        return;
    }

    if (!link_up_) {
        link_up_ = true;
        PRINTF("!link up\r\n");
    }
}

// Emits one `>rf` line and resets the measurement window. sent/ok/lost cover
// only the interval since the previous report, so an `rf on` stream shows the
// link's instantaneous behavior. arc is the last packet's auto-retransmit count
// (0-15, a rough signal-margin proxy) and rpd is the received-power detector
// (1 = last received signal >= -64 dBm) -- both read live from the radio, along
// with pa_base, the base's own PA level (0..3). The Wanderer's PA is on the
// remote board, so it is queried over RF and arrives separately as `>pa=`.
void BaseState::report_rf_stats() {
    const uint32_t sent = tx_total_ - tx_total_at_report_;
    const uint32_t ok = tx_ok_ - tx_ok_at_report_;
    tx_total_at_report_ = tx_total_;
    tx_ok_at_report_ = tx_ok_;

    PRINTF(">rf link=%s sent=%lu ok=%lu lost=%lu arc=%u rpd=%d chan=%u "
           "pa_base=%u\r\n",
           link_up_ ? "up" : "down", static_cast<unsigned long>(sent),
           static_cast<unsigned long>(ok),
           static_cast<unsigned long>(sent - ok), radio.getARC(),
           radio.testRPD() ? 1 : 0, radio.getChannel(), radio.getPALevel());

    // Ask the Wanderer for its PA; the `>pa=` reply follows on a later poll.
    tx_getpa_cmd();
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
// they reach through drain_downlink() is visible. The text protocol command
// handler calls these tx_*_cmd() methods.

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

bool BaseState::tx_setpa_cmd(uint8_t pa_level) {
    rf_protocol::SetPaCommand command{
        {rf_protocol::CMD_SETPA, next_command_sequence()},
        pa_level,
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
    // Help text is `*` log: human-facing, ignored by a machine parser.
    PRINTF("*commands: arm | stop | move <vL> <vR> | ver | stat | "
           "tlm on|off|<hz> | rf on|off | setbpa <0-3> | setwpa <0-3> | "
           "ping | help\r\n");
    PRINTF("*rf fields: arc=auto-retransmit count of last packet (0-15, "
           "lower is better link margin); rpd=received power detector "
           "(1 = last signal stronger than ~-64 dBm)\r\n");
}

void print_wanderer_help() {
    // The Wanderer's local debug console: a subset that acts directly on this
    // board, with no base or RF link involved. Read commands report local state;
    // arm/stop/move drive the FSM as if a commander issued them; setpa changes
    // this radio's own PA level.
    PRINTF("*wanderer debug console (local, USB only)\r\n");
    PRINTF("*commands: arm | stop | move <vL> <vR> | ver | stat | rf | "
           "setpa <0-3> | help\r\n");
}

// Case-insensitive compare of a parsed token against a lowercase literal.
bool token_is(const char *token, const char *lower) {
    while (*token != '\0' && *lower != '\0') {
        char c = *token;
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<char>(c - 'A' + 'a');
        }
        if (c != *lower) {
            return false;
        }
        ++token;
        ++lower;
    }
    return *token == '\0' && *lower == '\0';
}

// Parses a token as a velocity and reports whether it is a clean signed 16-bit
// integer. The whole token must be consumed -- "12x" is rejected, not truncated.
bool parse_velocity(const char *token, int16_t *out) {
    char *end = nullptr;
    const long value = std::strtol(token, &end, 10);
    if (end == token || *end != '\0' || value < -32768L || value > 32767L) {
        return false;
    }
    *out = static_cast<int16_t>(value);
    return true;
}

// Parses a token as an nRF24 PA level (0 = MIN .. 3 = MAX). The whole token
// must be consumed and the value must be in range.
bool parse_pa(const char *token, uint8_t *out) {
    char *end = nullptr;
    const long value = std::strtol(token, &end, 10);
    if (end == token || *end != '\0' || value < 0 || value > 3) {
        return false;
    }
    *out = static_cast<uint8_t>(value);
    return true;
}

// Splits `line` in place into at most `max` whitespace-separated tokens and
// returns the count. Anything past the limit stays attached to the last token,
// which is enough to reject over-long arg lists. Shared by both command parsers.
int tokenize(char *line, char **token, int max) {
    int count = 0;
    for (char *cursor = line; *cursor != '\0' && count < max;) {
        while (*cursor == ' ' || *cursor == '\t') {
            ++cursor;
        }
        if (*cursor == '\0') {
            break;
        }
        token[count++] = cursor;
        while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t') {
            ++cursor;
        }
        if (*cursor != '\0') {
            *cursor++ = '\0';
        }
    }
    return count;
}

// Handles one received command line (the verb and its arguments). Per
// PROTOCOL.md the `=` ack is syntactic and emitted here, locally, before any
// frame goes over the air; query answers and execution outcomes show up later
// on the `>`/`#`/`!` channels.
void process_base_command_line(char *line, BaseState *state) {
    char *token[4];
    int count = tokenize(line, token, 4);

    // Blank lines and `*` comments are ignored on input, so a command script
    // may carry spacing and `*` annotations.
    if (count == 0 || token[0][0] == '*') {
        return;
    }
    const char *verb = token[0];

    if (token_is(verb, "arm")) {
        PRINTF("=ok arm\r\n");
        state->tx_arm_cmd();
    } else if (token_is(verb, "stop")) {
        PRINTF("=ok stop\r\n");
        state->tx_stop_cmd();
    } else if (token_is(verb, "ver")) {
        PRINTF("=ok ver\r\n");
        state->tx_getver_cmd();
    } else if (token_is(verb, "stat")) {
        PRINTF("=ok stat\r\n");
        state->tx_getstat_cmd();
    } else if (token_is(verb, "move")) {
        int16_t left = 0;
        int16_t right = 0;
        if (count != 3 || !parse_velocity(token[1], &left) ||
            !parse_velocity(token[2], &right)) {
            PRINTF("=err move: expected 2 integers\r\n");
            return;
        }
        PRINTF("=ok move vL=%d vR=%d\r\n", left, right);
        state->tx_move_cmd(left, right);
    } else if (token_is(verb, "tlm")) {
        if (count != 2) {
            PRINTF("=err tlm: expected on|off|<hz>\r\n");
        } else if (token_is(token[1], "on")) {
            state->telemetry_every_frame();
            PRINTF("=ok tlm on=1\r\n");
        } else if (token_is(token[1], "off")) {
            state->telemetry_off();
            PRINTF("=ok tlm on=0\r\n");
        } else {
            char *end = nullptr;
            const long hz = std::strtol(token[1], &end, 10);
            if (end == token[1] || *end != '\0' || hz <= 0 || hz > 1000) {
                PRINTF("=err tlm: expected on|off|<hz>\r\n");
                return;
            }
            state->telemetry_rate(static_cast<uint32_t>(hz));
            PRINTF("=ok tlm rate=%ld on=1\r\n", hz);
        }
    } else if (token_is(verb, "rf")) {
        // Base-local nRF24 link stats; `rf on|off` toggles a 1 Hz auto-repeat.
        if (count == 1) {
            PRINTF("=ok rf\r\n");
            state->report_rf_stats();
        } else if (count == 2 && token_is(token[1], "on")) {
            state->rf_auto_enable();
            PRINTF("=ok rf on=1\r\n");
        } else if (count == 2 && token_is(token[1], "off")) {
            state->rf_auto_disable();
            PRINTF("=ok rf on=0\r\n");
        } else {
            PRINTF("=err rf: expected on|off\r\n");
        }
    } else if (token_is(verb, "setbpa")) {
        // Base-local: set this radio's PA and echo the level read back.
        uint8_t pa = 0;
        if (count != 2 || !parse_pa(token[1], &pa)) {
            PRINTF("=err setbpa: expected 0-3\r\n");
        } else {
            radio.setPALevel(pa);
            PRINTF("=ok setbpa pa=%u\r\n", radio.getPALevel());
        }
    } else if (token_is(verb, "setwpa")) {
        // Forwarded: the Wanderer applies it and answers `>pa=` with the level
        // actually in effect a poll later.
        uint8_t pa = 0;
        if (count != 2 || !parse_pa(token[1], &pa)) {
            PRINTF("=err setwpa: expected 0-3\r\n");
        } else {
            PRINTF("=ok setwpa pa=%u\r\n", pa);
            state->tx_setpa_cmd(pa);
        }
    } else if (token_is(verb, "ping")) {
        // Base-local liveness: answers even when the RF link is down.
        PRINTF("=ok ping\r\n");
    } else if (token_is(verb, "help")) {
        print_base_help();
        PRINTF("=ok help\r\n");
    } else {
        PRINTF("=err unknown command: %s\r\n", verb);
    }
}

// The Wanderer's local debug console, the counterpart to
// process_base_command_line. Everything here acts directly on this board: read
// commands report local state, arm/stop/move drive the FSM as if a commander
// issued them, and setpa changes this radio's PA. No frame goes over the air.
void process_wanderer_command_line(char *line, TacticalCore *core) {
    char *token[4];
    int count = tokenize(line, token, 4);
    if (count == 0 || token[0][0] == '*') {
        return;
    }
    const char *verb = token[0];
    const absolute_time_t now = get_absolute_time();

    if (token_is(verb, "arm")) {
        // A local command makes this console the live commander, so the FSM does
        // not immediately fall back for want of a remote one.
        core->note_commander_alive(now);
        core->cmd_arm(now);
        PRINTF("=ok arm\r\n");
    } else if (token_is(verb, "stop")) {
        core->cmd_stop(now);
        PRINTF("=ok stop\r\n");
    } else if (token_is(verb, "move")) {
        int16_t left = 0;
        int16_t right = 0;
        if (count != 3 || !parse_velocity(token[1], &left) ||
            !parse_velocity(token[2], &right)) {
            PRINTF("=err move: expected 2 integers\r\n");
            return;
        }
        core->note_commander_alive(now);
        if (core->cmd_move(left, right, now)) {
            PRINTF("=ok move vL=%d vR=%d\r\n", left, right);
        } else {
            PRINTF("=err move: not active (arm first)\r\n");
        }
    } else if (token_is(verb, "ver")) {
        PRINTF(">ver fw=%u.%u\r\n", FIRMWARE_MAJOR, FIRMWARE_MINOR);
        PRINTF("=ok ver\r\n");
    } else if (token_is(verb, "stat")) {
        const uint8_t flags = wanderer_flags(core);
        PRINTF(">stat state=%s armed=%d moving=%d vL=%d vR=%d\r\n",
               tactical_state_name(static_cast<uint8_t>(core->state())),
               (flags & rf_protocol::WAND_ARMED) ? 1 : 0,
               (flags & rf_protocol::WAND_MOVING) ? 1 : 0,
               core->target_left(), core->target_right());
        PRINTF("=ok stat\r\n");
    } else if (token_is(verb, "rf")) {
        // Local radio view. The Wanderer is a receiver, so it has no TX/ACK
        // counters like the base -- it reports its PA, channel, the last RPD,
        // and whether a commander has been heard within the liveness window.
        PRINTF(">rf link=%s pa=%u chan=%u rpd=%d\r\n",
               core->commander_alive(now) ? "up" : "down", radio.getPALevel(),
               radio.getChannel(), radio.testRPD() ? 1 : 0);
        PRINTF("=ok rf\r\n");
    } else if (token_is(verb, "setpa")) {
        uint8_t pa = 0;
        if (count != 2 || !parse_pa(token[1], &pa)) {
            PRINTF("=err setpa: expected 0-3\r\n");
        } else {
            radio.setPALevel(pa);
            PRINTF("=ok setpa pa=%u\r\n", radio.getPALevel());
        }
    } else if (token_is(verb, "help")) {
        print_wanderer_help();
        PRINTF("=ok help\r\n");
    } else {
        PRINTF("=err unknown command: %s\r\n", verb);
    }
}

// USB CDC carries one encoding: text, one command per line. A line is collected
// until newline, then handed to `handle`; nothing else shares the stream. Both
// roles share this reader -- the base for its laptop CLI, the Wanderer for its
// local debug console -- so the per-line handler is a parameter. The line buffer
// is static, which is safe because only one role's poller runs on a given board.
void poll_usb_lines(void (*handle)(char *line, void *context), void *context) {
    static char line[COMMAND_LINE_SIZE];
    static size_t line_length = 0;
    static bool overflow = false;

    int input = 0;
    while ((input = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        // A leading '\r' is tolerated so '\r\n' terminators work: the '\r'
        // ends the (here empty) line and the following '\n' is a no-op.
        if (input == '\n' || input == '\r') {
            if (overflow) {
                PRINTF("=err line too long\r\n");
            } else if (line_length != 0) {
                line[line_length] = '\0';
                handle(line, context);
            }
            line_length = 0;
            overflow = false;
        } else if (overflow) {
            continue;  // discard an over-length line to the next newline
        } else if (line_length + 1 < sizeof(line)) {
            line[line_length++] = static_cast<char>(input);
        } else {
            overflow = true;
        }
    }
}

void base_line_handler(char *line, void *context) {
    process_base_command_line(line, static_cast<BaseState *>(context));
}

void wanderer_line_handler(char *line, void *context) {
    process_wanderer_command_line(line, static_cast<TacticalCore *>(context));
}

}  // namespace

int main() {
    stdio_init_all();

    const Role role = read_role();
    radio_spi.begin(spi1, PIN_RF_SCK, PIN_RF_MOSI, PIN_RF_MISO);
    bool radio_ready =
        radio.begin(&radio_spi) && configure_radio(role);

    for (uint32_t elapsed_ms = 0;
         elapsed_ms < 5000u && !stdio_usb_connected();
         elapsed_ms += 100u) {
        sleep_ms(100);
    }

    if (role == Role::Base) {
        // The base speaks only the laptop text protocol, so its banner is a
        // `*` log line a machine client skips and a human can read.
        PRINTF("*Pico2 V2 RF base ready (RF24 %s)\r\n",
               radio_ready ? "detected" : "not detected");
        print_base_help();
    } else {
        PRINTF("\r\nPico2 V2 RF wanderer (RF24 %s) -- type 'help' for the "
               "local debug console\r\n",
               radio_ready ? "detected" : "not detected");
    }

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, false);

    if (role == Role::Wanderer) {
        // External link-good indicator on GP2: blinks while the base is heard,
        // off when the link drops. Wanderer-only -- the base leaves GP2 alone.
        gpio_init(PIN_LINK_LED);
        gpio_set_dir(PIN_LINK_LED, GPIO_OUT);
        gpio_put(PIN_LINK_LED, false);
    }

    TacticalCore wanderer;
    BaseState base;

    if (radio_ready && role == Role::Wanderer) {
        // Without this preload, the first base poll would receive an empty
        // ACK. Later frames are staged after each command is read.
        stage_next_ack_payload(&wanderer);
        radio.startListening();
    }

    absolute_time_t next_led_toggle = make_timeout_time_ms(500);
    absolute_time_t next_link_led_toggle = make_timeout_time_ms(LINK_LED_PERIOD_MS);
    absolute_time_t next_radio_health =
        make_timeout_time_ms(RADIO_HEALTH_PERIOD_MS);
    bool radio_connected = radio_ready;

    while (true) {
        if (role == Role::Base) {
            // The USB console is the base's lifeline to the laptop and must run
            // every loop regardless of radio state -- a missing or unpowered
            // nRF24 must never silence command handling. Only the actions that
            // actually touch the radio are gated on radio_ready.
            poll_usb_lines(base_line_handler, &base);
            if (radio_ready) {
                poll_wanderer(&base);
                if (base.rf_report_due()) {
                    base.report_rf_stats();
                }
            }
        } else {
            // Wanderer. The local USB debug console runs whenever a terminal is
            // attached, independent of the radio, so the board can be inspected
            // and driven even with the nRF24 absent. Only radio command handling
            // is gated on radio_ready; the FSM ticks regardless so locally
            // issued arm/move actually advance state and fall back on silence.
            if (stdio_usb_connected()) {
                poll_usb_lines(wanderer_line_handler, &wanderer);
            }
            TacticalState before = wanderer.state();
            if (radio_ready) {
                process_wanderer_radio(&wanderer);
            }
            wanderer.tick(get_absolute_time());
            if (wanderer.state() != before) {
                PRINTF("TacticalState changed. From %u to %u\r\n",
                       static_cast<unsigned>(before),
                       static_cast<unsigned>(wanderer.state()));
            }
        }

        if (time_reached(next_radio_health)) {
            bool connected = radio.isChipConnected();
            if (connected != radio_connected) {
                radio_connected = connected;
                // Re-detect across power cycles: when the nRF24 comes back, the
                // chip is at power-on defaults, so rerun begin()+configure()
                // before declaring it ready again. This lets the RF link resume
                // after the radio is repowered without rebooting the base.
                radio_ready =
                    connected && radio.begin(&radio_spi) && configure_radio(role);
                if (radio_ready && role == Role::Wanderer) {
                    stage_next_ack_payload(&wanderer);
                    radio.startListening();
                }
                PRINTF("*RF24: %s\r\n",
                       radio_ready ? "detected" : "not detected");
            }
            next_radio_health = make_timeout_time_ms(RADIO_HEALTH_PERIOD_MS);
        }

        if (time_reached(next_led_toggle)) {
            gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
            next_led_toggle = make_timeout_time_ms(500);
        }

        // GP2 link-good LED: blink while the base is heard, hold off otherwise.
        // Runs every loop (not just when radio_ready) so a dropped radio forces
        // the LED dark instead of freezing it mid-blink.
        if (role == Role::Wanderer) {
            bool link_up =
                radio_ready && wanderer.commander_alive(get_absolute_time());
            if (!link_up) {
                gpio_put(PIN_LINK_LED, false);
                next_link_led_toggle = make_timeout_time_ms(LINK_LED_PERIOD_MS);
            } else if (time_reached(next_link_led_toggle)) {
                gpio_xor_mask(1u << PIN_LINK_LED);
                next_link_led_toggle = make_timeout_time_ms(LINK_LED_PERIOD_MS);
            }
        }
        tight_loop_contents();
    }
}
