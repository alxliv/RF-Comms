#include "nrf24.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"

enum {
    NRF24_CMD_R_REGISTER = 0x00,
    NRF24_CMD_W_REGISTER = 0x20,
    NRF24_CMD_R_RX_PAYLOAD = 0x61,
    NRF24_CMD_W_TX_PAYLOAD = 0xA0,
    NRF24_CMD_FLUSH_TX = 0xE1,
    NRF24_CMD_FLUSH_RX = 0xE2,
    NRF24_CMD_NOP = 0xFF,

    NRF24_REG_CONFIG = 0x00,
    NRF24_REG_EN_AA = 0x01,
    NRF24_REG_EN_RXADDR = 0x02,
    NRF24_REG_SETUP_AW = 0x03,
    NRF24_REG_SETUP_RETR = 0x04,
    NRF24_REG_RF_CH = 0x05,
    NRF24_REG_RF_SETUP = 0x06,
    NRF24_REG_STATUS = 0x07,
    NRF24_REG_OBSERVE_TX = 0x08,
    NRF24_REG_RX_ADDR_P0 = 0x0A,
    NRF24_REG_TX_ADDR = 0x10,
    NRF24_REG_RX_PW_P0 = 0x11,
    NRF24_REG_FIFO_STATUS = 0x17,
    NRF24_REG_DYNPD = 0x1C,
    NRF24_REG_FEATURE = 0x1D,

    NRF24_CONFIG_PRIM_RX = 1u << 0,
    NRF24_CONFIG_PWR_UP = 1u << 1,
    NRF24_CONFIG_CRCO = 1u << 2,
    NRF24_CONFIG_EN_CRC = 1u << 3,

    NRF24_STATUS_MAX_RT = 1u << 4,
    NRF24_STATUS_TX_DS = 1u << 5,
    NRF24_STATUS_RX_DR = 1u << 6,
    NRF24_STATUS_IRQ_MASK = NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS |
                            NRF24_STATUS_MAX_RT,

    NRF24_FIFO_RX_EMPTY = 1u << 0,

    NRF24_CONFIG_CRC2_POWERED_UP =
        NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO | NRF24_CONFIG_PWR_UP,
    NRF24_SETUP_RETR_VALUE = 0x2F,
    NRF24_RF_SETUP_VALUE = 0x00,
    NRF24_TX_TIMEOUT_MS = 30,
    NRF24_MODE_SETTLE_US = 150,
};

static void csn_low(const nrf24_t *radio) {
    gpio_put(radio->pin_csn, false);
}

static void csn_high(const nrf24_t *radio) {
    gpio_put(radio->pin_csn, true);
}

static uint8_t command(nrf24_t *radio, uint8_t value) {
    uint8_t status = 0;

    csn_low(radio);
    spi_write_read_blocking(radio->spi, &value, &status, 1);
    csn_high(radio);
    return status;
}

static uint8_t read_register(nrf24_t *radio, uint8_t reg) {
    const uint8_t tx[2] = {
        (uint8_t)(NRF24_CMD_R_REGISTER | (reg & 0x1Fu)),
        NRF24_CMD_NOP,
    };
    uint8_t rx[2] = {0};

    csn_low(radio);
    spi_write_read_blocking(radio->spi, tx, rx, 2);
    csn_high(radio);
    return rx[1];
}

static void write_register(nrf24_t *radio, uint8_t reg, uint8_t value) {
    const uint8_t tx[2] = {
        (uint8_t)(NRF24_CMD_W_REGISTER | (reg & 0x1Fu)),
        value,
    };

    csn_low(radio);
    spi_write_blocking(radio->spi, tx, 2);
    csn_high(radio);
}

static void write_register_buffer(nrf24_t *radio, uint8_t reg,
                                  const uint8_t *data, size_t length) {
    const uint8_t command_byte =
        (uint8_t)(NRF24_CMD_W_REGISTER | (reg & 0x1Fu));

    csn_low(radio);
    spi_write_blocking(radio->spi, &command_byte, 1);
    spi_write_blocking(radio->spi, data, length);
    csn_high(radio);
}

static void write_payload(nrf24_t *radio,
                          const uint8_t payload[NRF24_PAYLOAD_SIZE]) {
    const uint8_t command_byte = NRF24_CMD_W_TX_PAYLOAD;

    csn_low(radio);
    spi_write_blocking(radio->spi, &command_byte, 1);
    spi_write_blocking(radio->spi, payload, NRF24_PAYLOAD_SIZE);
    csn_high(radio);
}

static void read_payload(nrf24_t *radio,
                         uint8_t payload[NRF24_PAYLOAD_SIZE]) {
    const uint8_t command_byte = NRF24_CMD_R_RX_PAYLOAD;

    csn_low(radio);
    spi_write_blocking(radio->spi, &command_byte, 1);
    spi_read_blocking(radio->spi, NRF24_CMD_NOP, payload,
                      NRF24_PAYLOAD_SIZE);
    csn_high(radio);
}

void nrf24_init_pins(nrf24_t *radio, spi_inst_t *spi, uint8_t pin_csn,
                     uint8_t pin_ce, uint8_t pin_sck, uint8_t pin_mosi,
                     uint8_t pin_miso) {
    radio->spi = spi;
    radio->pin_csn = pin_csn;
    radio->pin_ce = pin_ce;

    spi_init(spi, 4u * 1000u * 1000u);
    gpio_set_function(pin_sck, GPIO_FUNC_SPI);
    gpio_set_function(pin_mosi, GPIO_FUNC_SPI);
    gpio_set_function(pin_miso, GPIO_FUNC_SPI);

    gpio_init(pin_csn);
    gpio_set_dir(pin_csn, GPIO_OUT);
    gpio_put(pin_csn, true);

    gpio_init(pin_ce);
    gpio_set_dir(pin_ce, GPIO_OUT);
    gpio_put(pin_ce, false);
}

bool nrf24_init(nrf24_t *radio, uint8_t channel) {
    sleep_ms(100);
    gpio_put(radio->pin_ce, false);

    write_register(radio, NRF24_REG_CONFIG,
                   NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO);
    write_register(radio, NRF24_REG_EN_AA, 0x01);
    write_register(radio, NRF24_REG_EN_RXADDR, 0x01);
    write_register(radio, NRF24_REG_SETUP_AW, 0x03);
    write_register(radio, NRF24_REG_SETUP_RETR,
                   NRF24_SETUP_RETR_VALUE);
    write_register(radio, NRF24_REG_RF_CH, (uint8_t)(channel & 0x7Fu));
    write_register(radio, NRF24_REG_RF_SETUP, NRF24_RF_SETUP_VALUE);
    write_register(radio, NRF24_REG_RX_PW_P0, NRF24_PAYLOAD_SIZE);
    write_register(radio, NRF24_REG_DYNPD, 0x00);
    write_register(radio, NRF24_REG_FEATURE, 0x00);
    write_register(radio, NRF24_REG_STATUS, NRF24_STATUS_IRQ_MASK);
    command(radio, NRF24_CMD_FLUSH_RX);
    command(radio, NRF24_CMD_FLUSH_TX);

    write_register(radio, NRF24_REG_CONFIG,
                   NRF24_CONFIG_CRC2_POWERED_UP);
    sleep_ms(5);

    return nrf24_check_config(radio, channel);
}

bool nrf24_check_config(nrf24_t *radio, uint8_t channel) {
    return read_register(radio, NRF24_REG_EN_AA) == 0x01 &&
           read_register(radio, NRF24_REG_EN_RXADDR) == 0x01 &&
           read_register(radio, NRF24_REG_SETUP_AW) == 0x03 &&
           read_register(radio, NRF24_REG_SETUP_RETR) ==
               NRF24_SETUP_RETR_VALUE &&
           read_register(radio, NRF24_REG_RF_CH) ==
               (uint8_t)(channel & 0x7Fu) &&
           read_register(radio, NRF24_REG_RF_SETUP) ==
               NRF24_RF_SETUP_VALUE &&
           read_register(radio, NRF24_REG_RX_PW_P0) ==
               NRF24_PAYLOAD_SIZE;
}

void nrf24_start_listening(nrf24_t *radio,
                           const uint8_t address[NRF24_ADDRESS_SIZE]) {
    gpio_put(radio->pin_ce, false);
    write_register_buffer(radio, NRF24_REG_RX_ADDR_P0, address,
                          NRF24_ADDRESS_SIZE);
    write_register(radio, NRF24_REG_STATUS, NRF24_STATUS_IRQ_MASK);
    write_register(radio, NRF24_REG_CONFIG,
                   NRF24_CONFIG_CRC2_POWERED_UP |
                       NRF24_CONFIG_PRIM_RX);
    gpio_put(radio->pin_ce, true);
    sleep_us(NRF24_MODE_SETTLE_US);
}

nrf24_tx_report_t nrf24_send(
    nrf24_t *radio, const uint8_t address[NRF24_ADDRESS_SIZE],
    const uint8_t payload[NRF24_PAYLOAD_SIZE]) {
    nrf24_tx_report_t report = {
        .result = NRF24_SEND_TIMEOUT,
        .retransmit_count = 0,
        .lost_packet_count = 0,
    };

    gpio_put(radio->pin_ce, false);
    write_register(radio, NRF24_REG_CONFIG,
                   NRF24_CONFIG_CRC2_POWERED_UP);
    write_register_buffer(radio, NRF24_REG_TX_ADDR, address,
                          NRF24_ADDRESS_SIZE);
    write_register_buffer(radio, NRF24_REG_RX_ADDR_P0, address,
                          NRF24_ADDRESS_SIZE);
    write_register(radio, NRF24_REG_STATUS, NRF24_STATUS_IRQ_MASK);
    command(radio, NRF24_CMD_FLUSH_TX);
    write_payload(radio, payload);

    sleep_us(NRF24_MODE_SETTLE_US);
    gpio_put(radio->pin_ce, true);
    sleep_us(15);
    gpio_put(radio->pin_ce, false);

    const absolute_time_t deadline =
        make_timeout_time_ms(NRF24_TX_TIMEOUT_MS);
    while (!time_reached(deadline)) {
        const uint8_t status = nrf24_read_status(radio);

        if ((status & NRF24_STATUS_TX_DS) != 0) {
            report.result = NRF24_SEND_OK;
            break;
        }
        if ((status & NRF24_STATUS_MAX_RT) != 0) {
            report.result = NRF24_SEND_MAX_RETRIES;
            break;
        }
        tight_loop_contents();
    }

    const uint8_t observe_tx =
        read_register(radio, NRF24_REG_OBSERVE_TX);
    report.retransmit_count = observe_tx & 0x0Fu;
    report.lost_packet_count = observe_tx >> 4;

    write_register(radio, NRF24_REG_STATUS, NRF24_STATUS_IRQ_MASK);
    if (report.result != NRF24_SEND_OK) {
        command(radio, NRF24_CMD_FLUSH_TX);
    }

    return report;
}

bool nrf24_receive(nrf24_t *radio,
                   uint8_t payload[NRF24_PAYLOAD_SIZE]) {
    if ((read_register(radio, NRF24_REG_FIFO_STATUS) &
         NRF24_FIFO_RX_EMPTY) != 0) {
        return false;
    }

    read_payload(radio, payload);
    write_register(radio, NRF24_REG_STATUS, NRF24_STATUS_RX_DR);
    return true;
}

uint8_t nrf24_read_status(nrf24_t *radio) {
    return command(radio, NRF24_CMD_NOP);
}

uint8_t nrf24_read_channel(nrf24_t *radio) {
    return read_register(radio, NRF24_REG_RF_CH);
}

uint8_t nrf24_read_rf_setup(nrf24_t *radio) {
    return read_register(radio, NRF24_REG_RF_SETUP);
}
