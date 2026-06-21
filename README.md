# RF-COMMS Playground

## Development and test of RF communications



1. ### First project - Pico2 basic TX/RX



Hardware: **Raspberry PI Pico 2** connected to **EBYTE E01 2G4M27D** (nRF24 based) rf module

Connections are:

*   GP16 = MISO     GP17 = CSN     GP18 = SCK     GP19 = MOSI
*   GP14 = CE       GP15 = IRQ (reserved, unused)
*   GP20 = role strap: high = base, low = remote
*   GP10..GP13 = remote/device ID bits, internal pulldowns, id = bits + 1
*   GP2 = remote range-test LED output, active high, use a series resistor

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

## Basic RF ping/reply test

Flash the same UF2 onto both Pico 2 boards. Set the hardware straps before
powering or resetting each board:

- Base: GP20 high. Its station ID is always 0.
- Remote 1: GP20 low and GP10..GP13 low.
- Other remotes: GP20 low; GP10..GP13 contain the binary value `ID - 1`.

The Pico and the E01 external power supply must share a common ground. Attach
the antenna before transmitting.

Current radio settings:

- Channel 76
- 1 Mbps
- Two-byte CRC
- Hardware auto-acknowledgement and retransmission
- Lowest nRF24 transmit-power setting
- Fixed eight-byte packets
- GP15 IRQ reserved but not used; firmware polls the radio

After startup, the remote listens on its station address. The base selects
remote 1 by default and sends a ping every second. Expected base USB CDC output:

```text
Pico2 Basic RF
Role: base
Station ID: 0
Radio: detected
PING remote=1 sequence=1
REPLY remote=1 sequence=1
```

If the remote does not acknowledge the packet, the base reports:

```text
RF transmit failed: no acknowledgement
```

Base USB commands:

```text
help
status
select 1
ping
```

`select N` accepts remote IDs from 1 through 16. `ping` immediately pings the
selected remote; automatic pings continue once per second.

### Important nRF24 reply timing

After receiving a packet with hardware auto-acknowledgement enabled, do not
immediately lower CE and switch the radio from PRX to PTX for an
application-level reply. The nRF24 still needs time to transmit its automatic
ACK.

This firmware waits 500 microseconds after reading a valid ping before starting
the remote's explicit reply transmission. Without this guard interval, the
remote can interrupt its automatic ACK. The observed failure is:

- The remote receives every ping.
- The base reports `RF transmit failed: no acknowledgement`.
- The remote reports `reply=timeout`.

Also allow at least 150 microseconds after changing the radio from PRX to PTX
before pulsing CE to start transmission.

## Pico2 V1 RF

`Pico2_V1_RF` is the reliability-focused version. `Pico2_BasicRF` remains
unchanged as the known-good hardware and wiring test.

V1 retains the proven hardware configuration and nRF24 timing rules, and adds:

- A separate RF address prefix, preventing accidental BasicRF/V1 interaction.
- Fixed 32-byte command and response packets.
- An application-level CRC16 in addition to the nRF24 hardware CRC.
- Separate command and link context for each of 16 remote IDs.
- Up to one outstanding command per remote.
- An explicit response containing the received command ID.
- The source remote ID encoded in the low four bits of every ACK byte, so the
  base routes it to the correct context.
- A 150-millisecond command-response timeout.
- No application-level command retransmission. A response timeout is reported
  as a lost connection to the selected remote.
- Hardware retransmission counts from the nRF24 `OBSERVE_TX` register.
- Link statistics and command response-time measurements.

The nRF24 remains configured for channel 76, 1 Mbps, two-byte hardware CRC,
hardware auto-acknowledgement, 15 hardware retransmissions, and the lowest
transmit-power setting. These radio-level retransmissions are retained to
improve delivery reliability; V1 does not resend a command at the application
level. The remote still waits 500 microseconds before changing from receive
mode to transmit mode for its explicit response.

### V1 packet format

The nRF24 maximum payload size is 32 bytes, so V1 uses constant 32-byte
packets.

Command packet:

```text
Byte 0      Command ID
Byte 1      Signed command argument
Bytes 2-29 Reserved, currently zero
Bytes 30-31 CRC16-CCITT, little-endian
```

Normal ACK response for `move` and `stop`:

```text
Byte 0      0xF0 | (remote ID - 1)
Byte 1      Received command ID
Byte 2      Received command argument
Bytes 3-29 Reserved, currently zero
Bytes 30-31 CRC16-CCITT, little-endian
```

For example, `move 1` sends command bytes `0x0A 0x01`. The remote response
from remote ID 3 starts with `0xF2 0x0A 0x01`.

Data response for `getstat`:

```text
Byte 0      0xF0 | (remote ID - 1)
Byte 1      0x01 (GETSTAT)
Byte 2      Response data length
Bytes 3-29 Response data, maximum 27 bytes
Bytes 30-31 CRC16-CCITT, little-endian
```

The current `getstat` data is 11 bytes: signed movement value, 32-bit uptime in
seconds, 32-bit received-command count, last command ID, and remote TX power
level. The source remote ID is decoded from byte 0.

`getver N` uses the same data-response format with command ID `0x03`. Its
two-byte data contains the major and minor firmware version of remote `N`.

The base allocates 16 independent remote contexts. Each context stores its
pending command, timeout, connected state, movement/status data, hardware retry
counters, command counters, and response timing. `select N` only changes which
context subsequent CLI commands target; it does not replace or clear other
remote contexts.

The hardware ID strap uses GP10..GP13, so remote IDs are 1 through 16. The ACK
byte values therefore range from `0xF0` for remote 1 through `0xFF` for remote
16.

Command IDs:

```text
0x01  GETSTAT
0x02  PING
0x03  GETVER
0x0A  MOVE
0x0B  STOP
0xF0..0xFF  ACK marker and encoded source remote ID
```

### Building V1

Run from the repository root in PowerShell:

```powershell
cmake -S Pico2_V1_RF `
      -B Pico2_V1_RF/build-release `
      -G Ninja `
      -DPICO_BOARD=pico2 `
      -DCMAKE_BUILD_TYPE=Release `
      "-Dpicotool_DIR=$HOME\.pico-sdk\picotool\2.1.1\picotool"

cmake --build Pico2_V1_RF/build-release --target pico2_v1_rf -j
```

The generated firmware is:

```text
Pico2_V1_RF/build-release/pico2_v1_rf.uf2
```

Flash the V1 UF2 onto both boards. The role and remote ID straps are unchanged.
Both roles wait up to five seconds at startup for a USB CDC terminal. A remote
continues silently if none connects. A base without a terminal stops and
signals the error with two quick onboard-LED flashes each second.

Base USB commands:

```text
help
version
status
status all
stats reset
select 1
ping
poll on
poll off
period 500
power 0
getstat
getver 1
move 1
move -20
stop
scan
```

The remote prints every accepted command and whether its response was
transmitted. The base accepts an ACK only when its command ID and argument
match the outstanding command. `getstat` instead returns the data block
described above. If no matching response arrives within 150 milliseconds, the
base prints `CONNECTION LOST`.

`ping` sends command ID `0x02` to the selected remote with that remote's
8-bit ping sequence number. Each sequence starts at zero when the base starts
and advances only after the base receives a matching response:

```text
PING remote=1 sequence=0 latency_us=1340
```

If a remote receives the same sequence again, it knows the previous
application response was not received by the base. The remote raises its local
TX power by one level, up to level 3, before sending the repeated response.
On a new sequence, the remote pulses the GP2 range-test LED for 100
milliseconds. On a repeated sequence, it holds the LED on until a new sequence
arrives. The LED is off after remote startup until the first ping.

Periodic ping is disabled after power-up. `poll on` enables periodic ping to
the currently selected remote, and `poll off` disables it. The default period
is 500 milliseconds. `period MS` changes it within the range 100 through
60000 milliseconds. A periodic cycle is skipped if that remote already has a
pending command.

`power N` changes the local station's nRF24 transmit power at runtime:

```text
power 0   Minimum transmit power; startup default
power 1   Intermediate level
power 2   Intermediate level
power 3   Maximum transmit power
```

The setting affects only transmissions from the station where the command is
entered; no reboot is required. The E01 module includes an external power
amplifier, so higher levels increase supply-current demand significantly.
`status` prints the current logical level and raw `RF_SETUP` register.

`status` reports the selected remote context. `status all` reports every
remote context that has been used. Statistics include hardware retransmissions,
invalid packets, sent and acknowledged commands, command timeouts, response
transmission failures, and average/maximum response time.

### RF channel scan

The `scan` command measures local RF activity on nRF24 channels 0 through 125
(2400 through 2525 MHz). It takes 16 ten-millisecond RPD samples per channel
and prints:

```text
channel= 76 frequency=2476MHz noise= 25% hits=8/32
```

The nRF24 RPD detector is binary. A hit means received energy exceeded
approximately -64 dBm during that sample. Therefore `noise` is the percentage
of samples with detected RF energy; it is not a calibrated signal-strength or
dBm measurement. Firmware ends each receive window by lowering CE before
reading RPD, because the nRF24 latches detection for the completed receive
period at that transition.

Scanning temporarily stops normal communication on that station for roughly
20 seconds. The base refuses to start a scan while any remote command is
pending. After scanning, firmware restores the configured channel, station
receive address, and receive mode.

An all-zero result means no signal crossed the approximately -64 dBm threshold
during the observation windows. It does not prove that the band is free of
weaker or very infrequent interference.

## Pico2 V2 RF

`Pico2_V2_RF` is the next firmware version. It targets a single Wanderer
vehicle and will use nRF24 Enhanced ShockBurst ACK payloads for telemetry from
the Wanderer to the base station.

The current Step 1 scaffold:

- Builds for Raspberry Pi Pico 2 (`RP2350`, ARM).
- Uses C++17 and Raspberry Pi Pico SDK 2.2.0.
- Includes RF24 v1.6.1 as a Git submodule under
  `Pico2_V2_RF/lib/RF24`.
- Initializes the RF24 library and reports whether the radio responds over
  SPI.
- Sends startup diagnostics through USB CDC.

Current radio connections:

```text
GP14 = CE
GP17 = CSN
GP16 = MISO
GP18 = SCK
GP19 = MOSI
```

### Building Pico2 V2 RF

Set `PICO_SDK_PATH` to the installed Pico SDK 2.2.0 location if it is not
already defined. Run the following commands from the repository root in
PowerShell:

```powershell
git submodule update --init --recursive

$env:PICO_SDK_PATH = "$HOME\.pico-sdk\sdk\2.2.0"

cmake -S Pico2_V2_RF `
      -B Pico2_V2_RF/build-release `
      -G Ninja `
      -DPICO_BOARD=pico2 `
      -DCMAKE_BUILD_TYPE=Release `
      "-Dpicotool_DIR=$HOME\.pico-sdk\picotool\2.1.1\picotool"

cmake --build Pico2_V2_RF/build-release --target pico2_v2_rf -j
```

The generated UF2 is:

```text
Pico2_V2_RF/build-release/pico2_v2_rf.uf2
```

The current firmware is only an RF24/Pico 2 compatibility smoke test. The
ACK-payload protocol, telemetry frames, request/reply matching, and binary USB
host protocol are not implemented yet.
