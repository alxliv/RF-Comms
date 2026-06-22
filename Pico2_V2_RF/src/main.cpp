#include <cstdio>

#include "RF24.h"
#include "pico/stdlib.h"

namespace {

constexpr uint8_t PIN_RF_CE = 14;
constexpr uint8_t PIN_RF_CSN = 17;
constexpr uint8_t PIN_RF_MISO = 16;
constexpr uint8_t PIN_RF_SCK = 18;
constexpr uint8_t PIN_RF_MOSI = 19;

RF24 radio(PIN_RF_CE, PIN_RF_CSN);
SPI radio_spi;

}  // namespace

#define PRINTF(...)             \
    do {                        \
        if (!usb_off) {         \
            std::printf(__VA_ARGS__); \
        }                       \
    } while (0)

int main() {
    stdio_init_all();

    radio_spi.begin(spi0, PIN_RF_SCK, PIN_RF_MOSI, PIN_RF_MISO);
    const bool radio_ready = radio.begin(&radio_spi);
    bool usb_off = true;

    for (uint32_t elapsed_ms = 0;
         elapsed_ms < 5000u && !stdio_usb_connected();
         elapsed_ms += 100u) {
        sleep_ms(100);
    }
    usb_off = !stdio_usb_connected();

    PRINTF("\r\nPico2 V2 RF scaffold\r\n");
    PRINTF("RF24: %s\r\n",
                radio_ready ? "detected" : "not detected");

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, false);

    absolute_time_t next_led_toggle = make_timeout_time_ms(500);

    while (true)
    {
        if (time_reached(next_led_toggle))
        {
            gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
            next_led_toggle =
                make_timeout_time_ms(500);
        }
        tight_loop_contents();
    }
}
