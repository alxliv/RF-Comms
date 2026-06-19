#ifndef NRF24_H
#define NRF24_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/spi.h"
#include "protocol.h"

enum {
    NRF24_ADDRESS_SIZE = 5,
    NRF24_PAYLOAD_SIZE = RF_PACKET_SIZE,
};

typedef enum {
    NRF24_SEND_OK = 0,
    NRF24_SEND_MAX_RETRIES,
    NRF24_SEND_TIMEOUT,
} nrf24_send_result_t;

typedef struct {
    nrf24_send_result_t result;
    uint8_t retransmit_count;
    uint8_t lost_packet_count;
} nrf24_tx_report_t;

typedef struct {
    spi_inst_t *spi;
    uint8_t pin_csn;
    uint8_t pin_ce;
} nrf24_t;

void nrf24_init_pins(nrf24_t *radio, spi_inst_t *spi, uint8_t pin_csn,
                     uint8_t pin_ce, uint8_t pin_sck, uint8_t pin_mosi,
                     uint8_t pin_miso);
bool nrf24_init(nrf24_t *radio, uint8_t channel);
bool nrf24_check_config(nrf24_t *radio, uint8_t channel);
void nrf24_set_channel(nrf24_t *radio, uint8_t channel);
void nrf24_set_tx_power(nrf24_t *radio, uint8_t level);
uint8_t nrf24_get_tx_power(nrf24_t *radio);
void nrf24_start_listening(nrf24_t *radio,
                           const uint8_t address[NRF24_ADDRESS_SIZE]);
nrf24_tx_report_t nrf24_send(
    nrf24_t *radio, const uint8_t address[NRF24_ADDRESS_SIZE],
    const uint8_t payload[NRF24_PAYLOAD_SIZE]);
bool nrf24_receive(nrf24_t *radio,
                   uint8_t payload[NRF24_PAYLOAD_SIZE]);
uint8_t nrf24_read_status(nrf24_t *radio);
uint8_t nrf24_read_channel(nrf24_t *radio);
uint8_t nrf24_read_rf_setup(nrf24_t *radio);
uint8_t nrf24_measure_channel_noise(nrf24_t *radio, uint8_t channel,
                                    uint8_t samples);

#endif
