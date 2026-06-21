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

int main() {
    stdio_init_all();

    radio_spi.begin(spi0, PIN_RF_SCK, PIN_RF_MOSI, PIN_RF_MISO);
    const bool radio_ready = radio.begin(&radio_spi);

    sleep_ms(2000);
    std::printf("\r\nPico2 V2 RF scaffold\r\n");
    std::printf("RF24: %s\r\n",
                radio_ready ? "detected" : "not detected");

    while (true) {
        sleep_ms(1000);
    }
}
