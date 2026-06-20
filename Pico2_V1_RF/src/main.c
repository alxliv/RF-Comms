#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "nrf24.h"
#include "pico/stdlib.h"
#include "protocol.h"

enum {
    PIN_ID_BIT_0 = 10,
    PIN_ID_BIT_1 = 11,
    PIN_ID_BIT_2 = 12,
    PIN_ID_BIT_3 = 13,
    PIN_RF_CE = 14,
    PIN_RF_IRQ = 15,
    PIN_RF_MISO = 16,
    PIN_RF_CSN = 17,
    PIN_RF_SCK = 18,
    PIN_RF_MOSI = 19,
    PIN_ROLE = 20,

    RF_CHANNEL = 76,
    BASE_STATION_ID = 0,
    MIN_REMOTE_ID = 1,
    MAX_REMOTE_ID = 16,
    REMOTE_CONTEXT_COUNT = 16,
    DEFAULT_REMOTE_ID = 1,
    COMMAND_TIMEOUT_MS = 150,
    AUTO_ACK_GUARD_US = 500,
    GETSTAT_DATA_SIZE = 11,
    DEFAULT_POLL_PERIOD_MS = 500,
    MIN_POLL_PERIOD_MS = 100,
    MAX_POLL_PERIOD_MS = 60000,
    RF_CHANNEL_COUNT = 126,
    RF_SCAN_SAMPLES = 16,
};

typedef enum {
    ROLE_REMOTE = 0,
    ROLE_BASE = 1,
} station_role_t;

typedef struct {
    bool waiting;
    rf_command_t command;
    absolute_time_t started;
    absolute_time_t deadline;
} pending_command_t;

typedef struct {
    pending_command_t pending;
    bool connected;
    bool seen;

    int8_t move_value;
    uint8_t last_command;
    bool status_valid;
    uint32_t reported_uptime_s;
    uint32_t reported_commands_received;
    bool reported_radio_ok;

    uint32_t hardware_tx_ok;
    uint32_t hardware_tx_failed;
    uint32_t hardware_retransmits;
    uint32_t commands_sent;
    uint32_t commands_acked;
    uint32_t command_timeouts;
    uint32_t commands_received;
    uint32_t response_tx_failed;
    uint32_t unexpected_responses;
    uint64_t total_response_us;
    uint32_t maximum_response_us;
} remote_context_t;

typedef struct {
    uint32_t rx_valid;
    uint32_t rx_invalid;
    uint32_t invalid_source;
} station_stats_t;

static nrf24_t radio;
static station_stats_t station_stats;
static remote_context_t remote_contexts[REMOTE_CONTEXT_COUNT];
static station_role_t role;
static uint8_t station_id;
static uint8_t selected_remote_id = DEFAULT_REMOTE_ID;
static bool periodic_poll_enabled;
static uint32_t periodic_poll_ms = DEFAULT_POLL_PERIOD_MS;
static absolute_time_t next_periodic_ping;

static remote_context_t *remote_context(uint8_t remote_id) {
    if (remote_id < MIN_REMOTE_ID || remote_id > MAX_REMOTE_ID) {
        return NULL;
    }
    return &remote_contexts[remote_id - 1u];
}

static station_role_t read_role(void) {
    gpio_init(PIN_ROLE);
    gpio_set_dir(PIN_ROLE, GPIO_IN);
    gpio_pull_down(PIN_ROLE);
    sleep_ms(2);
    return gpio_get(PIN_ROLE) ? ROLE_BASE : ROLE_REMOTE;
}

static uint8_t read_remote_id(void) {
    const uint8_t id_pins[] = {
        PIN_ID_BIT_0,
        PIN_ID_BIT_1,
        PIN_ID_BIT_2,
        PIN_ID_BIT_3,
    };
    uint8_t id_bits = 0;

    for (uint8_t bit = 0; bit < 4; ++bit) {
        gpio_init(id_pins[bit]);
        gpio_set_dir(id_pins[bit], GPIO_IN);
        gpio_pull_down(id_pins[bit]);
    }
    sleep_ms(2);

    for (uint8_t bit = 0; bit < 4; ++bit) {
        if (gpio_get(id_pins[bit])) {
            id_bits |= (uint8_t)(1u << bit);
        }
    }
    return (uint8_t)(id_bits + 1u);
}

static void make_address(uint8_t id,
                         uint8_t address[NRF24_ADDRESS_SIZE]) {
    address[0] = 0xD7;
    address[1] = 0x31;
    address[2] = 0x56;
    address[3] = 0x52;
    address[4] = id;
}

static void start_station_receiver(void) {
    uint8_t address[NRF24_ADDRESS_SIZE];
    make_address(station_id, address);
    nrf24_set_channel(&radio, RF_CHANNEL);
    nrf24_start_listening(&radio, address);
}

static nrf24_tx_report_t transmit(
    uint8_t destination, remote_context_t *context,
    const uint8_t packet[RF_PACKET_SIZE]) {
    uint8_t address[NRF24_ADDRESS_SIZE];
    make_address(destination, address);

    const nrf24_tx_report_t report =
        nrf24_send(&radio, address, packet);
    context->hardware_retransmits += report.retransmit_count;
    if (report.result == NRF24_SEND_OK) {
        ++context->hardware_tx_ok;
    } else {
        ++context->hardware_tx_failed;
    }
    return report;
}

static const char *send_result_text(nrf24_send_result_t result) {
    switch (result) {
        case NRF24_SEND_OK:
            return "sent";
        case NRF24_SEND_MAX_RETRIES:
            return "no-ack";
        case NRF24_SEND_TIMEOUT:
        default:
            return "timeout";
    }
}
static bool send_command(uint8_t remote_id, uint8_t command,
                         int8_t argument) {

    static int send_cmd_nums;

    remote_context_t *context = remote_context(remote_id);
    if (context == NULL) {
        return false;
    }
    if (context->pending.waiting) {
        printf("Remote %u already has a pending command\r\n", remote_id);
        return false;
    }
    send_cmd_nums++;

    context->pending.command.command = command;
    context->pending.command.argument = argument;

    uint8_t packet[RF_PACKET_SIZE];
    protocol_encode_command(&context->pending.command, packet);
    ++context->commands_sent;
    context->seen = true;
    context->pending.waiting = true;
    context->pending.started = get_absolute_time();
    context->pending.deadline =
        make_timeout_time_ms(COMMAND_TIMEOUT_MS);

    const nrf24_tx_report_t report =
        transmit(remote_id, context, packet);
    start_station_receiver();

    printf("[%d] COMMAND remote=%u id=0x%02X arg=%d radio=%s hw_retry=%u\r\n", send_cmd_nums,
           remote_id, command, argument,
           send_result_text(report.result), report.retransmit_count);
    return true;
}

static uint32_t read_u32_le(const uint8_t *data) {
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static void write_u32_le(uint8_t *data, uint32_t value) {
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)((value >> 8) & 0xFFu);
    data[2] = (uint8_t)((value >> 16) & 0xFFu);
    data[3] = (uint8_t)(value >> 24);
}

static void update_getstat_context(remote_context_t *context,
                                   const rf_response_t *response) {
    if (response->data_length != GETSTAT_DATA_SIZE) {
        printf("GETSTAT remote=%u invalid data length=%u\r\n",
               response->source_id, response->data_length);
        return;
    }

    context->move_value = (int8_t)response->data[0];
    context->reported_uptime_s = read_u32_le(&response->data[1]);
    context->reported_commands_received =
        read_u32_le(&response->data[5]);
    context->last_command = response->data[9];
    context->reported_radio_ok = response->data[10] != 0;
    context->status_valid = true;

    printf("GETSTAT remote=%u move=%d uptime_s=%lu "
           "commands_received=%lu last_command=0x%02X radio=%s\r\n",
           response->source_id, context->move_value,
           (unsigned long)context->reported_uptime_s,
           (unsigned long)context->reported_commands_received,
           context->last_command,
           context->reported_radio_ok ? "ok" : "fault");
}

static void process_base_responses(void) {
    uint8_t packet[RF_PACKET_SIZE];

    while (nrf24_receive(&radio, packet)) {
        rf_response_t response;
        if (!protocol_decode_response(packet, &response)) {
            ++station_stats.rx_invalid;
            continue;
        }
        ++station_stats.rx_valid;

        remote_context_t *context =
            remote_context(response.source_id);
        if (context == NULL) {
            ++station_stats.invalid_source;
            continue;
        }
        context->seen = true;

        if (!context->pending.waiting ||
            response.command != context->pending.command.command ||
            (response.command != RF_COMMAND_GETSTAT &&
             response.argument !=
                 context->pending.command.argument)) {
            ++context->unexpected_responses;
            continue;
        }

        const uint32_t response_us = (uint32_t)absolute_time_diff_us(
            context->pending.started, get_absolute_time());
        context->total_response_us += response_us;
        if (response_us > context->maximum_response_us) {
            context->maximum_response_us = response_us;
        }
        ++context->commands_acked;
        context->connected = true;
        context->pending.waiting = false;

        if (response.command == RF_COMMAND_GETSTAT) {
            update_getstat_context(context, &response);
        } else {
            context->last_command = response.command;
            if (response.command == RF_COMMAND_MOVE) {
                context->move_value = response.argument;
            } else if (response.command == RF_COMMAND_STOP) {
                context->move_value = 0;
            }
            if (response.command == RF_COMMAND_PING) {
                printf("PING remote=%u latency_us=%lu\r\n",
                       response.source_id,
                       (unsigned long)response_us);
            } else {
                printf("ACK remote=%u command=0x%02X arg=%d "
                       "response_us=%lu\r\n",
                       response.source_id, response.command,
                       response.argument, (unsigned long)response_us);
            }
        }
    }
}

static void process_base_timeouts(void) {
    for (uint8_t index = 0; index < REMOTE_CONTEXT_COUNT; ++index) {
        remote_context_t *context = &remote_contexts[index];
        if (!context->pending.waiting ||
            !time_reached(context->pending.deadline)) {
            continue;
        }

        ++context->command_timeouts;
        context->connected = false;
        context->pending.waiting = false;
        printf("CONNECTION LOST remote=%u command=0x%02X "
               "timeout_ms=%u\r\n",
               (unsigned int)(index + 1u),
               context->pending.command.command,
               COMMAND_TIMEOUT_MS);
    }
}

static void build_getstat_data(const remote_context_t *context,
                               uint8_t data[GETSTAT_DATA_SIZE]) {
    data[0] = (uint8_t)context->move_value;
    write_u32_le(&data[1],
                 (uint32_t)(to_ms_since_boot(get_absolute_time()) /
                            1000u));
    write_u32_le(&data[5], context->commands_received);
    data[9] = context->last_command;
    data[10] = nrf24_check_config(&radio, RF_CHANNEL) ? 1 : 0;
}

static int recv_cmd_nums = 0;

static void process_remote_commands(void) {
    remote_context_t *context = remote_context(station_id);
    uint8_t packet[RF_PACKET_SIZE];

    while (nrf24_receive(&radio, packet)) {
        recv_cmd_nums++;
        rf_command_t command;
        if (!protocol_decode_command(packet, &command)) {
            ++station_stats.rx_invalid;
            continue;
        }
        ++station_stats.rx_valid;
        ++context->commands_received;
        context->seen = true;
        context->last_command = command.command;

        uint8_t response_packet[RF_PACKET_SIZE];
        bool response_ready = true;

        switch (command.command) {
            case RF_COMMAND_GETSTAT: {
                uint8_t data[GETSTAT_DATA_SIZE];
                build_getstat_data(context, data);
                response_ready = protocol_encode_data_response(
                    station_id, command.command, data, sizeof(data),
                    response_packet);
                break;
            }
            case RF_COMMAND_PING:
                protocol_encode_ack(station_id, command.command,
                                    command.argument, response_packet);
                break;
            case RF_COMMAND_MOVE:
                context->move_value = command.argument;
                protocol_encode_ack(station_id, command.command,
                                    command.argument, response_packet);
                break;
            case RF_COMMAND_STOP:
                context->move_value = 0;
                protocol_encode_ack(station_id, command.command,
                                    command.argument, response_packet);
                break;
            default:
                response_ready = false;
                break;
        }

        if (!response_ready) {
            printf("[%d] UNKNOWN command=0x%02X arg=%d\r\n", recv_cmd_nums,
                   command.command, command.argument);
            continue;
        }

        sleep_us(AUTO_ACK_GUARD_US);
        const nrf24_tx_report_t report =
            transmit(BASE_STATION_ID, context, response_packet);
        start_station_receiver();
        if (report.result == NRF24_SEND_OK) {
            context->connected = true;
        } else {
            context->connected = false;
            ++context->response_tx_failed;
        }

        printf("[%d] RECEIVED command=0x%02X arg=%d response=%s "
               "hw_retry=%u\r\n", recv_cmd_nums,
               command.command, command.argument,
               send_result_text(report.result),
               report.retransmit_count);
    }
}

static void print_remote_context(uint8_t remote_id,
                                 const remote_context_t *context) {
    const uint32_t average_response_us =
        context->commands_acked == 0
            ? 0
            : (uint32_t)(context->total_response_us /
                         context->commands_acked);

    printf("Remote=%u connected=%s waiting=%s move=%d "
           "last_command=0x%02X\r\n",
           remote_id, context->connected ? "yes" : "no",
           context->pending.waiting ? "yes" : "no",
           context->move_value, context->last_command);
    printf("  HW tx_ok=%lu failed=%lu retransmits=%lu "
           "commands sent=%lu acked=%lu timeouts=%lu received=%lu\r\n",
           (unsigned long)context->hardware_tx_ok,
           (unsigned long)context->hardware_tx_failed,
           (unsigned long)context->hardware_retransmits,
           (unsigned long)context->commands_sent,
           (unsigned long)context->commands_acked,
           (unsigned long)context->command_timeouts,
           (unsigned long)context->commands_received);
    printf("  response_tx_failed=%lu unexpected=%lu "
           "response average_us=%lu maximum_us=%lu\r\n",
           (unsigned long)context->response_tx_failed,
           (unsigned long)context->unexpected_responses,
           (unsigned long)average_response_us,
           (unsigned long)context->maximum_response_us);
    if (context->status_valid) {
        printf("  reported uptime_s=%lu commands_received=%lu "
               "radio=%s\r\n",
               (unsigned long)context->reported_uptime_s,
               (unsigned long)context->reported_commands_received,
               context->reported_radio_ok ? "ok" : "fault");
    }
}

static void print_help(void) {
    printf("Commands:\r\n");
    printf("  help         Show commands\r\n");
    printf("  status       Show selected remote and radio status\r\n");
    printf("  status all   Show all used remote contexts\r\n");
    printf("  stats reset  Clear communication statistics\r\n");
    printf("  scan         Measure local RF energy on channels 0..125\r\n");
    printf("  power N      TX power 0..3: 0=min (default), 3=max\r\n");
    if (role == ROLE_BASE) {
        printf("  select N     Select remote ID 1..16\r\n");
        printf("  ping         Ping selected remote once\r\n");
        printf("  poll on|off  Enable or disable periodic ping\r\n");
        printf("  period MS    Set periodic ping interval 100..60000\r\n");
        printf("  getstat      Request selected remote status\r\n");
        printf("  move N       Set signed movement value -127..127\r\n");
        printf("  stop         Stop selected remote movement\r\n");
    }
}

static void print_status(bool all_contexts) {
    printf("Role=%s station_id=%u radio_config=%s channel=%u "
           "power=%u RF_SETUP=0x%02X STATUS=0x%02X "
           "RX valid=%lu invalid=%lu "
           "invalid_source=%lu\r\n",
           role == ROLE_BASE ? "base" : "remote", station_id,
           nrf24_check_config(&radio, RF_CHANNEL) ? "ok" : "fault",
           nrf24_read_channel(&radio), nrf24_get_tx_power(&radio),
           nrf24_read_rf_setup(&radio),
           nrf24_read_status(&radio),
           (unsigned long)station_stats.rx_valid,
           (unsigned long)station_stats.rx_invalid,
           (unsigned long)station_stats.invalid_source);

    if (role == ROLE_REMOTE) {
        print_remote_context(
            station_id, remote_context(station_id));
        return;
    }

    if (!all_contexts) {
        printf("Selected remote=%u poll=%s period_ms=%lu\r\n",
               selected_remote_id,
               periodic_poll_enabled ? "on" : "off",
               (unsigned long)periodic_poll_ms);
        print_remote_context(
            selected_remote_id,
            remote_context(selected_remote_id));
        return;
    }

    for (uint8_t remote_id = MIN_REMOTE_ID;
         remote_id <= MAX_REMOTE_ID; ++remote_id) {
        const remote_context_t *context =
            remote_context(remote_id);
        if (context->seen || context->pending.waiting) {
            print_remote_context(remote_id, context);
        }
    }
}

static void reset_statistics(void) {
    memset(&station_stats, 0, sizeof(station_stats));
    for (uint8_t index = 0; index < REMOTE_CONTEXT_COUNT; ++index) {
        remote_context_t *context = &remote_contexts[index];
        context->hardware_tx_ok = 0;
        context->hardware_tx_failed = 0;
        context->hardware_retransmits = 0;
        context->commands_sent = 0;
        context->commands_acked = 0;
        context->command_timeouts = 0;
        context->commands_received = 0;
        context->response_tx_failed = 0;
        context->unexpected_responses = 0;
        context->total_response_us = 0;
        context->maximum_response_us = 0;
    }
}

static bool any_command_pending(void) {
    for (uint8_t index = 0; index < REMOTE_CONTEXT_COUNT; ++index) {
        if (remote_contexts[index].pending.waiting) {
            return true;
        }
    }
    return false;
}

static void scan_rf_channels(void) {
    uint8_t detected[RF_CHANNEL_COUNT];
    uint8_t occupied_channels = 0;
    uint8_t peak_channel = 0;
    uint8_t peak_hits = 0;

    if (role == ROLE_BASE && any_command_pending()) {
        printf("Cannot scan while commands are pending\r\n");
        return;
    }

    printf("RF scan start: %u samples/channel, 10 ms/sample, "
           "RPD threshold approximately -64 dBm\r\n",
           RF_SCAN_SAMPLES);

    for (uint8_t channel = 0; channel < RF_CHANNEL_COUNT; ++channel) {
        detected[channel] = nrf24_measure_channel_noise(
            &radio, channel, RF_SCAN_SAMPLES);
        if (detected[channel] != 0) {
            ++occupied_channels;
        }
        if (detected[channel] > peak_hits) {
            peak_hits = detected[channel];
            peak_channel = channel;
        }
    }

    start_station_receiver();

    for (uint8_t channel = 0; channel < RF_CHANNEL_COUNT; ++channel) {
        const uint32_t noise_percent =
            ((uint32_t)detected[channel] * 100u +
             RF_SCAN_SAMPLES / 2u) /
            RF_SCAN_SAMPLES;
        printf("channel=%3u frequency=%4uMHz noise=%3lu%% "
               "hits=%u/%u\r\n",
               channel, 2400u + channel,
               (unsigned long)noise_percent, detected[channel],
               RF_SCAN_SAMPLES);
    }
    if (occupied_channels == 0) {
        printf("RF scan summary: no energy above the RPD threshold "
               "was detected\r\n");
    } else {
        const uint32_t peak_percent =
            ((uint32_t)peak_hits * 100u + RF_SCAN_SAMPLES / 2u) /
            RF_SCAN_SAMPLES;
        printf("RF scan summary: occupied_channels=%u "
               "peak_channel=%u peak_noise=%lu%%\r\n",
               occupied_channels, peak_channel,
               (unsigned long)peak_percent);
    }
    printf("RF scan complete; restored channel %u\r\n", RF_CHANNEL);
}

static void service_periodic_ping(void) {
    if (role != ROLE_BASE || !periodic_poll_enabled ||
        !time_reached(next_periodic_ping)) {
        return;
    }

    remote_context_t *context =
        remote_context(selected_remote_id);
    if (!context->pending.waiting) {
        send_command(selected_remote_id, RF_COMMAND_PING, 0);
    }
    next_periodic_ping =
        make_timeout_time_ms(periodic_poll_ms);
}

static void process_command_line(char *line) {
    if (strcmp(line, "help") == 0) {
        print_help();
    } else if (strcmp(line, "status") == 0) {
        print_status(false);
    } else if (strcmp(line, "status all") == 0) {
        print_status(true);
    } else if (strcmp(line, "stats reset") == 0) {
        reset_statistics();
        printf("Statistics reset\r\n");
    } else if (strcmp(line, "scan") == 0) {
        scan_rf_channels();
    } else if (strncmp(line, "power ", 6) == 0) {
        unsigned int value = 0;
        char extra = '\0';
        if (sscanf(line + 6, "%u %c", &value, &extra) == 1 &&
            value <= 3) {
            nrf24_set_tx_power(&radio, (uint8_t)value);
            printf("TX power=%u (%s)\r\n", value,
                   value == 0 ? "minimum"
                              : value == 3 ? "maximum"
                                           : "intermediate");
        } else {
            printf("Power must be 0..3\r\n");
        }
    } else if (role == ROLE_BASE && strcmp(line, "ping") == 0) {
        send_command(selected_remote_id, RF_COMMAND_PING, 0);
    } else if (role == ROLE_BASE && strcmp(line, "poll on") == 0) {
        periodic_poll_enabled = true;
        next_periodic_ping = get_absolute_time();
        printf("Periodic ping enabled: remote=%u period_ms=%lu\r\n",
               selected_remote_id,
               (unsigned long)periodic_poll_ms);
    } else if (role == ROLE_BASE && strcmp(line, "poll off") == 0) {
        periodic_poll_enabled = false;
        printf("Periodic ping disabled\r\n");
    } else if (role == ROLE_BASE &&
               strncmp(line, "period ", 7) == 0) {
        unsigned int value = 0;
        char extra = '\0';
        if (sscanf(line + 7, "%u %c", &value, &extra) == 1 &&
            value >= MIN_POLL_PERIOD_MS &&
            value <= MAX_POLL_PERIOD_MS) {
            periodic_poll_ms = value;
            next_periodic_ping = make_timeout_time_ms(value);
            printf("Periodic ping period=%u ms\r\n", value);
        } else {
            printf("Period must be 100..60000 ms\r\n");
        }
    } else if (role == ROLE_BASE && strcmp(line, "getstat") == 0) {
        send_command(selected_remote_id, RF_COMMAND_GETSTAT, 0);
    } else if (role == ROLE_BASE && strcmp(line, "stop") == 0) {
        send_command(selected_remote_id, RF_COMMAND_STOP, 0);
    } else if (role == ROLE_BASE &&
               strncmp(line, "move ", 5) == 0) {
        int value = 0;
        char extra = '\0';
        if (sscanf(line + 5, "%d %c", &value, &extra) == 1 &&
            value >= -127 && value <= 127) {
            send_command(selected_remote_id, RF_COMMAND_MOVE,
                         (int8_t)value);
        } else {
            printf("Move value must be -127..127\r\n");
        }
    } else if (role == ROLE_BASE &&
               strncmp(line, "select ", 7) == 0) {
        unsigned int value = 0;
        char extra = '\0';
        if (sscanf(line + 7, "%u %c", &value, &extra) == 1 &&
            value >= MIN_REMOTE_ID && value <= MAX_REMOTE_ID) {
            selected_remote_id = (uint8_t)value;
            printf("Selected remote=%u\r\n", selected_remote_id);
        } else {
            printf("Remote ID must be 1..16\r\n");
        }
    } else if (line[0] != '\0') {
        printf("Unknown command. Type 'help'.\r\n");
    }
}

static void poll_usb_commands(void) {
    static char line[64];
    static size_t length;

    int input;
    while ((input = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (input == '\r' || input == '\n') {
            if (length != 0) {
                line[length] = '\0';
                process_command_line(line);
                length = 0;
            }
        } else if ((input == '\b' || input == 0x7F) &&
                   length != 0) {
            --length;
        } else if (input >= 0x20 && input <= 0x7E &&
                   length < sizeof(line) - 1) {
            line[length++] = (char)input;
        }
    }
}

int main(void) {
    stdio_init_all();
    role = read_role();
    station_id = role == ROLE_BASE ? BASE_STATION_ID : read_remote_id();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, false);

    gpio_init(PIN_RF_IRQ);
    gpio_set_dir(PIN_RF_IRQ, GPIO_IN);
    gpio_pull_up(PIN_RF_IRQ);

    nrf24_init_pins(&radio, spi0, PIN_RF_CSN, PIN_RF_CE, PIN_RF_SCK,
                    PIN_RF_MOSI, PIN_RF_MISO);
    const bool radio_found = nrf24_init(&radio, RF_CHANNEL);
    if (radio_found) {
        start_station_receiver();
    }

    if (role == ROLE_BASE) {
        for (uint32_t elapsed_ms = 0;
             elapsed_ms < 3000u && !stdio_usb_connected();
             elapsed_ms += 100u) {
            sleep_ms(100);
        }
    }

    nrf24_set_tx_power(&radio, 2);

    printf("\r\nPico2 V1 RF command protocol\r\n");
    printf("Role: %s\r\n", role == ROLE_BASE ? "base" : "remote");
    printf("Station ID: %u\r\n", station_id);
    printf("Radio: %s\r\n", radio_found ? "detected" : "not detected");
    printf("TX_Power: %u\r\n", nrf24_get_tx_power(&radio));

    if (!radio_found) {
        printf("Check E01 power, ground, and SPI wiring.\r\n");
        while (true) {
            gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
            sleep_ms(100);
            poll_usb_commands();
        }
    }

    absolute_time_t next_led_toggle =
        make_timeout_time_ms(role == ROLE_BASE ? 250 : 500);
    print_help();

    while (true) {
        poll_usb_commands();
        if (role == ROLE_BASE) {
            process_base_responses();
            process_base_timeouts();
            service_periodic_ping();
        } else {
            process_remote_commands();
        }

        if (time_reached(next_led_toggle)) {
            gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
            next_led_toggle =
                make_timeout_time_ms(role == ROLE_BASE ? 250 : 500);
        }
        tight_loop_contents();
    }
}
