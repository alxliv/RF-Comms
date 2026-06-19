#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "nrf24.h"
#include "pico/stdlib.h"

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
    DEFAULT_REMOTE_ID = 1,
    AUTO_PING_PERIOD_MS = 1000,
    REPLY_TIMEOUT_MS = 100,
    AUTO_ACK_GUARD_US = 500,

    PACKET_MAGIC = 0xA5,
    PACKET_VERSION = 1,
    PACKET_TYPE_PING = 1,
    PACKET_TYPE_REPLY = 2,
};

typedef enum {
    ROLE_REMOTE = 0,
    ROLE_BASE = 1,
} station_role_t;

typedef struct {
    uint32_t tx_ok;
    uint32_t tx_failed;
    uint32_t rx_valid;
    uint32_t rx_invalid;
    uint32_t replies_received;
} radio_stats_t;

static nrf24_t radio;
static radio_stats_t stats;
static station_role_t role;
static uint8_t station_id;
static uint8_t selected_remote_id = DEFAULT_REMOTE_ID;
static uint16_t next_sequence = 1;

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
    address[0] = 0xC3;
    address[1] = 0x4D;
    address[2] = 0x4F;
    address[3] = 0x43;
    address[4] = id;
}

static uint8_t packet_checksum(const uint8_t packet[NRF24_PAYLOAD_SIZE]) {
    uint8_t checksum = 0;

    for (size_t index = 0; index < NRF24_PAYLOAD_SIZE - 1; ++index) {
        checksum ^= packet[index];
    }

    return checksum;
}

static void make_packet(uint8_t type, uint8_t source, uint8_t destination,
                        uint16_t sequence,
                        uint8_t packet[NRF24_PAYLOAD_SIZE]) {
    packet[0] = PACKET_MAGIC;
    packet[1] = PACKET_VERSION;
    packet[2] = type;
    packet[3] = source;
    packet[4] = destination;
    packet[5] = (uint8_t)(sequence & 0xFFu);
    packet[6] = (uint8_t)(sequence >> 8);
    packet[7] = packet_checksum(packet);
}

static bool packet_is_valid(const uint8_t packet[NRF24_PAYLOAD_SIZE]) {
    return packet[0] == PACKET_MAGIC &&
           packet[1] == PACKET_VERSION &&
           packet[7] == packet_checksum(packet);
}

static uint16_t packet_sequence(
    const uint8_t packet[NRF24_PAYLOAD_SIZE]) {
    return (uint16_t)packet[5] | ((uint16_t)packet[6] << 8);
}

static void start_station_receiver(void) {
    uint8_t address[NRF24_ADDRESS_SIZE];

    make_address(station_id, address);
    nrf24_start_listening(&radio, address);
}

static nrf24_send_result_t send_packet(
    uint8_t destination,
    const uint8_t packet[NRF24_PAYLOAD_SIZE]) {
    uint8_t address[NRF24_ADDRESS_SIZE];
    make_address(destination, address);

    const nrf24_send_result_t result =
        nrf24_send(&radio, address, packet);

    if (result == NRF24_SEND_OK) {
        ++stats.tx_ok;
    } else {
        ++stats.tx_failed;
    }

    return result;
}

static const char *send_error_text(nrf24_send_result_t result) {
    return result == NRF24_SEND_MAX_RETRIES ? "no acknowledgement"
                                            : "timeout";
}

static void send_ping(uint8_t remote_id) {
    uint8_t packet[NRF24_PAYLOAD_SIZE];
    const uint16_t sequence = next_sequence++;

    make_packet(PACKET_TYPE_PING, BASE_STATION_ID, remote_id, sequence,
                packet);
    printf("PING remote=%u sequence=%u\r\n", remote_id, sequence);

    const nrf24_send_result_t result = send_packet(remote_id, packet);
    start_station_receiver();

    if (result != NRF24_SEND_OK) {
        printf("RF transmit failed: %s\r\n", send_error_text(result));
        return;
    }

    const absolute_time_t deadline = make_timeout_time_ms(REPLY_TIMEOUT_MS);
    while (!time_reached(deadline)) {
        uint8_t reply[NRF24_PAYLOAD_SIZE];
        if (!nrf24_receive(&radio, reply)) {
            tight_loop_contents();
            continue;
        }

        if (!packet_is_valid(reply)) {
            ++stats.rx_invalid;
            continue;
        }

        ++stats.rx_valid;
        if (reply[2] == PACKET_TYPE_REPLY &&
            reply[3] == remote_id &&
            reply[4] == BASE_STATION_ID &&
            packet_sequence(reply) == sequence) {
            ++stats.replies_received;
            printf("REPLY remote=%u sequence=%u\r\n", remote_id,
                   sequence);
            return;
        }
    }

    printf("Reply timeout: remote=%u sequence=%u\r\n", remote_id,
           sequence);
}

static void process_remote_packets(void) {
    uint8_t packet[NRF24_PAYLOAD_SIZE];

    while (nrf24_receive(&radio, packet)) {
        if (!packet_is_valid(packet)) {
            ++stats.rx_invalid;
            continue;
        }

        ++stats.rx_valid;
        if (packet[2] != PACKET_TYPE_PING ||
            packet[3] != BASE_STATION_ID ||
            packet[4] != station_id) {
            continue;
        }

        const uint16_t sequence = packet_sequence(packet);
        uint8_t reply[NRF24_PAYLOAD_SIZE];
        make_packet(PACKET_TYPE_REPLY, station_id, BASE_STATION_ID,
                    sequence, reply);

        /*
         * Keep CE high long enough for the nRF24 to finish the automatic ACK
         * before changing from PRX to PTX for the application-level reply.
         */
        sleep_us(AUTO_ACK_GUARD_US);
        const nrf24_send_result_t result =
            send_packet(BASE_STATION_ID, reply);
        start_station_receiver();

        printf("PING received: sequence=%u; reply=%s\r\n", sequence,
               result == NRF24_SEND_OK ? "sent"
                                       : send_error_text(result));
    }
}

static void print_help(void) {
    printf("Commands:\r\n");
    printf("  help         Show commands\r\n");
    printf("  status       Show role, radio, and counters\r\n");
    if (role == ROLE_BASE) {
        printf("  select N     Select remote ID 1..16\r\n");
        printf("  ping         Ping the selected remote\r\n");
    }
}

static void print_status(void) {
    printf("Role=%s station_id=%u channel=%u RF_SETUP=0x%02X "
           "STATUS=0x%02X\r\n",
           role == ROLE_BASE ? "base" : "remote", station_id,
           nrf24_read_channel(&radio), nrf24_read_rf_setup(&radio),
           nrf24_read_status(&radio));
    if (role == ROLE_BASE) {
        printf("Selected remote=%u\r\n", selected_remote_id);
    }
    printf("TX ok=%lu failed=%lu RX valid=%lu invalid=%lu replies=%lu\r\n",
           (unsigned long)stats.tx_ok, (unsigned long)stats.tx_failed,
           (unsigned long)stats.rx_valid,
           (unsigned long)stats.rx_invalid,
           (unsigned long)stats.replies_received);
}

static void process_command(char *line) {
    if (strcmp(line, "help") == 0) {
        print_help();
        return;
    }
    if (strcmp(line, "status") == 0) {
        print_status();
        return;
    }
    if (role == ROLE_BASE && strcmp(line, "ping") == 0) {
        send_ping(selected_remote_id);
        return;
    }
    if (role == ROLE_BASE && strncmp(line, "select ", 7) == 0) {
        unsigned int requested_id = 0;
        char extra = '\0';
        if (sscanf(line + 7, "%u %c", &requested_id, &extra) == 1 &&
            requested_id >= MIN_REMOTE_ID &&
            requested_id <= MAX_REMOTE_ID) {
            selected_remote_id = (uint8_t)requested_id;
            printf("Selected remote=%u\r\n", selected_remote_id);
        } else {
            printf("Remote ID must be 1..16\r\n");
        }
        return;
    }
    if (line[0] != '\0') {
        printf("Unknown command. Type 'help'.\r\n");
    }
}

static void poll_usb_commands(void) {
    static char line[64];
    static size_t length = 0;

    int input;
    while ((input = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (input == '\r' || input == '\n') {
            if (length != 0) {
                line[length] = '\0';
                process_command(line);
                length = 0;
            }
            continue;
        }
        if ((input == '\b' || input == 0x7F) && length != 0) {
            --length;
            continue;
        }
        if (input >= 0x20 && input <= 0x7E &&
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

    printf("\r\nPico2 Basic RF\r\n");
    printf("Role: %s\r\n", role == ROLE_BASE ? "base" : "remote");
    printf("Station ID: %u\r\n", station_id);
    printf("Radio: %s\r\n", radio_found ? "detected" : "not detected");

    if (!radio_found) {
        printf("Check E01 power, ground, and SPI wiring.\r\n");
        while (true) {
            gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
            sleep_ms(100);
            poll_usb_commands();
        }
    }

    print_help();

    absolute_time_t next_ping = make_timeout_time_ms(AUTO_PING_PERIOD_MS);
    absolute_time_t next_led_toggle =
        make_timeout_time_ms(role == ROLE_BASE ? 250 : 500);

    while (true) {
        poll_usb_commands();

        if (role == ROLE_BASE && time_reached(next_ping)) {
            send_ping(selected_remote_id);
            next_ping = make_timeout_time_ms(AUTO_PING_PERIOD_MS);
        } else if (role == ROLE_REMOTE) {
            process_remote_packets();
        }

        if (time_reached(next_led_toggle)) {
            gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
            next_led_toggle =
                make_timeout_time_ms(role == ROLE_BASE ? 250 : 500);
        }

        tight_loop_contents();
    }
}
