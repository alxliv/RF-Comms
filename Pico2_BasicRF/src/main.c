#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"

enum {
    PIN_ID_BIT_0 = 10,
    PIN_ID_BIT_1 = 11,
    PIN_ID_BIT_2 = 12,
    PIN_ID_BIT_3 = 13,
    PIN_ROLE = 20,
};

typedef enum {
    ROLE_REMOTE = 0,
    ROLE_BASE = 1,
} station_role_t;

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

int main(void) {
    stdio_init_all();

    const station_role_t role = read_role();
    const uint8_t station_id = role == ROLE_BASE ? 0 : read_remote_id();
    const uint32_t blink_period_ms = role == ROLE_BASE ? 500u : 1000u;

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, false);

    for (uint32_t elapsed_ms = 0; elapsed_ms < 10000u; elapsed_ms += 100u) {
        if (stdio_usb_connected()) {
            break;
        }
        sleep_ms(100);
    }

    printf("\r\nPico2 Basic RF hello world\r\n");
    printf("Role: %s\r\n", role == ROLE_BASE ? "base" : "remote");
    printf("Station ID: %u\r\n", station_id);
    printf("LED blink period: %lu ms\r\n", (unsigned long)blink_period_ms);

    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(blink_period_ms / 2u);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(blink_period_ms / 2u);
    }
}
