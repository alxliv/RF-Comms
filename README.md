# RF-COMMS Playground

## Development and test of RF communications



1. ### First project - Pico2 basic TX/RX



Hardware: **Raspberry PI Pico 2** connected to **EBYTE E01 2G4M27D** (nRF24 based) rf module

Connections are:

*   GP16 = MISO     GP17 = CSN     GP18 = SCK     GP19 = MOSI
*   GP14 = CE       GP15 = IRQ (reserved, unused)
*   GP20 = role strap: high = base, low = remote
*   GP10..GP13 = remote/device ID bits, internal pulldowns, id = bits + 1

Read document: **E01-2G4M27D\_Usermanual\_EN\_v1.3.pdf**
[E01-2G4M27D\_Usermanual\_EN\_v1.3.pdf](./E01-2G4M27D_Usermanual_EN_v1.3.pdf)

There are two Pico modules. Each with its own E01-2G4M27D. One is connected to PC computer and its role is base station (or master), another module is called Remote. Our goal is to communicate by RF between the two. Base station will send commands and Remote replies. We will add more remote (slave) stations later. Each slave has ID from 1 to 16. Base (Master) station has ID 0. For setting ID we use GP10 to GP13 and wire it high according to station ID.

Start by writing a simple C/C++ Pico2 firmware in Pico2\_BasicRF folder. 

1. Write simple "hello world" Pico2 code - Just to test if build works OK and firmware upload works on real Pico.
   Something like this (non-working example):

```cpp

/*
 *
 * Wire-up:
 *   GP16 = MISO     GP17 = CSN     GP18 = SCK     GP19 = MOSI
 *   GP14 = CE       GP15 = IRQ (reserved, unused)
 *   GP20 = role strap: high = base, low = remote
 *   GP10..GP13 = remote/device ID bits, internal pulldowns, id = bits + 1
 */
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

static uint8_t read_role(void) {
  gpio_init(PIN_ROLE);
  gpio_set_dir(PIN_ROLE, GPIO_IN);
  gpio_pull_down(PIN_ROLE);
  sleep_ms(2);
  return gpio_get(PIN_ROLE) ? ROLE_BASE : ROLE_REMOTE;
}

`int main(void) {`
`stdio_init_all();`

`gpio_init(LED_PIN);`
`gpio_set_dir(LED_PIN, GPIO_OUT);`
`for (int i = 0; i < 100; i++) {`
    `if (stdio_usb_connected()) {`
      `break;`
    `}`
    `sleep_ms(100);`
  `}`

printf("starting!");

while (true) {

  if (absolute_time_diff_us(get_absolute_time(), next_led) <= 0) {

   led_state = !led_state;

   gpio_put(LED_PIN, led_state);

   next_led = make_timeout_time_ms(node.role == ROLE_BASE ? 500 : 1000);

}

tight_loop_contents();

}
```

## Building Pico2 Basic RF

Run these commands from the repository root in PowerShell:

```powershell
cmake -S Pico2_BasicRF `
      -B Pico2_BasicRF/build-release `
      -G Ninja `
      -DPICO_BOARD=pico2 `
      -DCMAKE_BUILD_TYPE=Release `
      "-Dpicotool_DIR=$HOME\.pico-sdk\picotool\2.1.1\picotool"

cmake --build Pico2_BasicRF/build-release --target pico2_basic_rf -j
```

The generated UF2 is:

```text
Pico2_BasicRF/build-release/pico2_basic_rf.uf2
```
