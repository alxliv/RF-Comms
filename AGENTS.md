# AGENTS.md

Instructions for coding agents working on Raspberry Pi Pico 2 firmware.

This file is intentionally independent of any repository name, parent folder,
application, peripheral, or protocol. Use the conventions below as the default
template for future Pico 2 C/CMake projects.

## Engineering rules

1. Read the relevant source, `CMakeLists.txt`, and nearby documentation before
   changing code.
2. Do not guess when a missing requirement would materially change the result.
3. Make the smallest change that completes the requested work.
4. Do not perform unrelated refactors, cleanup, formatting, or renaming.
5. Do not add unrequested abstractions, options, dependencies, or features.
6. Match the existing code and CMake style.
7. State important assumptions briefly.
8. Define how success will be verified before editing.
9. For bug fixes, reproduce the issue and verify the fix when practical.
10. Preserve user changes. Do not reset, overwrite, or clean a dirty worktree.

## Pico 2 defaults

Unless the project explicitly says otherwise, use:

- Board: Raspberry Pi Pico 2
- MCU family: RP2350
- CMake board identifier: `pico2`
- Architecture: RP2350 ARM default selected by the Pico SDK
- C standard: C11
- C++ standard: C++17
- Build system: CMake with Ninja
- Release build type for deployable firmware
- A separate Debug build directory for debugging and SDK assertions
- UF2 as the normal BOOTSEL flashing artifact

Do not silently substitute `pico`, `pico_w`, `pico2_w`, or an RISC-V RP2350
board configuration. Use another board only when the hardware or user request
requires it.

## Toolchain prerequisites

The machine must provide:

- CMake
- Ninja
- Raspberry Pi Pico SDK
- The ARM embedded toolchain required by that SDK
- `picotool` when needed for inspection or command-line flashing

The SDK location should be available through `PICO_SDK_PATH`.

On Windows PowerShell, inspect the environment with:

```powershell
$env:PICO_SDK_PATH
cmake --version
ninja --version
```

If the SDK is installed under the standard Pico VS Code extension location,
set it for the current terminal session with a command such as:

```powershell
$env:PICO_SDK_PATH = "$HOME\.pico-sdk\sdk\2.2.0"
```

Treat the version above as an example, not a permanent requirement. Prefer the
SDK version already selected by the project. Do not silently download, upgrade,
or replace the SDK or compiler.

## Recommended project layouts

Choose the smallest layout appropriate for the repository.

### Single firmware image

```text
firmware/
|-- CMakeLists.txt
|-- pico_sdk_import.cmake
|-- src/
|   `-- main.c
`-- include/
    `-- project_config.h
```

### Multiple firmware images

Use one parent CMake project with one directory per independently flashable
target:

```text
firmware/
|-- CMakeLists.txt
|-- pico_sdk_import.cmake
|-- first_target/
|   |-- CMakeLists.txt
|   `-- first_target.c
`-- second_target/
    |-- CMakeLists.txt
    `-- second_target.c
```

The directory may be named `firmware`, `pico`, `embedded`, or something
application-specific. Discover it from the repository; do not assume a fixed
path.

Never copy `build/`, `.cache/`, generated SDK files, object files, or existing
UF2 files when using an older project as a template.

## SDK import file

Place `pico_sdk_import.cmake` beside the top-level firmware
`CMakeLists.txt`. Obtain it from:

```text
<PICO_SDK_PATH>/external/pico_sdk_import.cmake
```

Treat it as an upstream helper. Do not manually modify its contents.

If the project uses the Raspberry Pi Pico VS Code extension, preserve the
extension-managed version block at the top of generated `CMakeLists.txt` files.

## Single-target CMake template

Replace `project_name`, `target_name`, and source paths with names appropriate
for the new project.

```cmake
cmake_minimum_required(VERSION 3.13)

# Must be included before project().
include(pico_sdk_import.cmake)

project(project_name C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2)
endif()

pico_sdk_init()

add_executable(target_name
    src/main.c
)

target_include_directories(target_name PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(target_name
    pico_stdlib
)

pico_enable_stdio_usb(target_name 1)
pico_enable_stdio_uart(target_name 0)

pico_add_extra_outputs(target_name)
```

Remove `target_include_directories()` when no separate include directory exists.

## Multi-target parent CMake template

The parent owns SDK initialization:

```cmake
cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(firmware C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2)
endif()

pico_sdk_init()

add_subdirectory(first_target)
add_subdirectory(second_target)
```

Each child supports both the parent build and an optional standalone build:

```cmake
cmake_minimum_required(VERSION 3.13)

if(NOT TARGET pico_stdlib)
    include(${CMAKE_CURRENT_LIST_DIR}/../pico_sdk_import.cmake)
endif()

project(target_name C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2)
endif()

if(NOT TARGET pico_stdlib)
    pico_sdk_init()
endif()

add_executable(target_name
    target_name.c
)

target_link_libraries(target_name
    pico_stdlib
)

pico_enable_stdio_usb(target_name 1)
pico_enable_stdio_uart(target_name 0)
pico_add_extra_outputs(target_name)
```

The `if(NOT TARGET pico_stdlib)` guards prevent a child included by the parent
from importing and initializing the SDK twice.

## Hardware libraries

Link only libraries actually used by the target. Common examples:

```cmake
target_link_libraries(target_name
    pico_stdlib
    hardware_adc
    hardware_dma
    hardware_gpio
    hardware_i2c
    hardware_pio
    hardware_pwm
    hardware_spi
    hardware_timer
    hardware_uart
)
```

Do not add this whole list by default. Unused libraries obscure the target's
real dependencies.

For a `.pio` source:

```cmake
pico_generate_pio_header(target_name
    ${CMAKE_CURRENT_LIST_DIR}/src/example.pio
)

target_link_libraries(target_name
    pico_stdlib
    hardware_pio
)
```

## Building

First locate the directory containing the firmware's top-level
`CMakeLists.txt`. In the commands below:

- `<source-dir>` is that directory.
- `<build-dir>` is a new build directory.
- `<target-name>` is the CMake executable target.

### Release

```powershell
cmake -S <source-dir> `
      -B <build-dir>-release `
      -G Ninja `
      -DPICO_BOARD=pico2 `
      -DCMAKE_BUILD_TYPE=Release

cmake --build <build-dir>-release --target <target-name> -j
```

### Debug

```powershell
cmake -S <source-dir> `
      -B <build-dir>-debug `
      -G Ninja `
      -DPICO_BOARD=pico2 `
      -DCMAKE_BUILD_TYPE=Debug

cmake --build <build-dir>-debug --target <target-name> -j
```

For a single-target project, omitting `--target` builds the default target:

```powershell
cmake --build <build-dir>-release -j
```

Ninja is a single-configuration generator. Never reuse one build directory for
different:

- boards
- CPU architectures
- SDK paths
- Release/Debug configurations

If any of those change, use a new build directory or deliberately remove and
reconfigure the old one.

## Finding output binaries

`pico_add_extra_outputs()` normally creates:

- `.uf2`
- `.elf`
- `.bin`
- `.hex`
- `.map`
- disassembly/listing output, depending on SDK configuration

Do not assume an exact output subdirectory without checking the generated
build tree. Locate the UF2 with PowerShell:

```powershell
Get-ChildItem <build-dir>-release -Recurse -Filter *.uf2
```

Report the exact resulting path after every requested build.

## USB and serial selection

Choose one USB owner per target.

For normal `printf` output or a USB serial CLI:

```cmake
target_link_libraries(target_name
    pico_stdlib
)

pico_enable_stdio_usb(target_name 1)
pico_enable_stdio_uart(target_name 0)
```

Call `stdio_init_all()` in firmware before using standard I/O.

For UART output instead:

```cmake
pico_enable_stdio_usb(target_name 0)
pico_enable_stdio_uart(target_name 1)
```

For direct TinyUSB ownership:

- Link the required TinyUSB libraries.
- Disable `pico_stdio_usb`.
- Initialize and service TinyUSB explicitly.
- Do not initialize both direct TinyUSB and `pico_stdio_usb` for the same
  target.

USB CDC `printf` can block when the host is not draining output. Do not place
blocking diagnostics on time-critical control, interrupt, radio, or protocol
response paths. Capture state first and print later.

## Source-code defaults

- Prefer fixed-width integer types for hardware registers and wire protocols.
- Keep GPIO assignments and tunable hardware constants centralized.
- Document voltage levels and whether GPIO inputs are 3.3 V tolerant only.
- Initialize outputs to a safe state before enabling external power stages.
- Do not assume peripheral power is present merely because the Pico is running.
- Use timeouts for hardware waits; avoid permanent blocking unless intentional.
- Keep interrupt handlers short and non-blocking.
- Do not call `printf`, allocate memory, or sleep inside a time-critical ISR.
- Use structured SDK APIs rather than direct register access unless direct
  access is required and documented.

### nRF24 auto-ACK and reply timing

When using nRF24 hardware auto-acknowledgement, do not immediately lower CE and
switch from PRX to PTX after reading a received packet. Allow the radio to
finish transmitting its automatic ACK before starting an application-level
reply. Use a guard interval of at least 500 microseconds unless the specific
radio datasheet and measured timing justify another value.

After changing from PRX to PTX, wait at least 150 microseconds before pulsing CE
to start transmission.

Interrupting the automatic ACK can produce this misleading pattern:

- The receiver successfully reads every packet.
- The original transmitter reports no acknowledgement.
- The receiver's explicit reply transmission times out.

## Flashing

For BOOTSEL flashing:

1. Hold BOOTSEL while connecting or resetting the Pico 2.
2. Confirm the RP2350 mass-storage volume appears.
3. Copy the generated `.uf2` onto that volume.
4. Wait for the board to reboot.

Command-line flashing with `picotool` may be used when already configured and
requested. Do not assume a connected board may be reset or overwritten without
the user's intent.

## Verification checklist

For every firmware change:

1. Inspect Git status and preserve unrelated user changes.
2. Configure explicitly for `PICO_BOARD=pico2`.
3. Build the exact affected target.
4. Check compiler and linker output for warnings and errors.
5. Confirm the expected UF2 exists and report its exact path.
6. Build sibling targets only when shared CMake or shared source changed.
7. Run host tests when shared algorithms, parsers, protocols, or math changed.
8. State clearly when hardware testing was not performed.

A successful build verifies compilation and linkage only. It does not verify:

- flashing
- USB enumeration
- GPIO wiring
- voltage compatibility
- peripheral communication
- timing under load
- power integrity
- physical actuator behavior

Do not claim those properties were tested without real hardware evidence.

## Generated and local files

Do not edit generated files under a build directory. Reconfigure from source.

Do not commit unless explicitly requested:

- `build/` or `build-*`
- `.cache/` or clangd indexes
- CMake cache and generated Ninja files
- object files
- generated SDK headers
- ELF, BIN, HEX, MAP, or UF2 artifacts

When inspecting Git status, ignore unrelated user changes and never revert them.
