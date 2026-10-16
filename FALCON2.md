# FluidNC on the Creality Falcon2 laser engraver

This branch is FluidNC v4.1.1 adapted to the **stock mainboard** of the Creality
Falcon2 (board marked `v3.2.S3.402`: ESP32-S3-WROOM-1 N8R8, two TMC2208 drivers).
No hardware change is needed. It is used on a 22 W Falcon2 (`CV50-LASER-22W` head).

Everything specific to this board is compiled only with `-DFALCON2_BOARD`, in the
PlatformIO environment `wifi_s3_falcon2` (see `platformio_override.ini`).

> **Before anything else, back up the stock firmware.** It is the only way back:
> `esptool.py --chip esp32s3 read_flash 0 0x800000 falcon2_backup.bin`
> (the board has native USB only; set its BOOT switch to DOWN and power-cycle it to
> enter the bootloader).

## What this branch adds

| Feature | Files | Why |
|---|---|---|
| **Laser head handshake** (module `falcon_head`) | `FalconHead.*`, `FalconProtocol.h` | The head has its own processor and expects a serial dialogue on UART1 (9600 8N1, TX GPIO17, RX GPIO18): a start-up sequence, then an echo of each of its frames. Without it the head LED blinks yellow. Protocol from [Falcon2Universal](https://github.com/drewgreenwell/Falcon2Universal) (MIT). `$Falcon/Status` shows the link state and the head model. |
| **Air pump** (module `falcon_pump`) | `FalconPump.cpp` | The pump is an H-bridge on GPIO43/44, driven like the stock firmware: one 58 Hz, 13-bit LEDC timer, second output inverted, both pins low at rest. Follows `M7`/`M8`/`M9`, and runs whenever the laser is on (`M3`/`M4`) plus a few seconds after. `$Falcon/Pump=<0-100\|auto>`. |
| **Console off the pump pins** | `esp32/falcon2_early.c`, `Main.cpp` | GPIO43/44 are also UART0, where FluidNC prints its console. Its idle-high TX line left DC on the pump coil, which overheats it. The pins are taken back right after the console starts. GPIO45/46 (laser) are held low from the very start. |
| **Play key without Wi-Fi** (module `falcon_play`) | `FalconPlay.cpp`, `Control.cpp` | When the machine is idle and not connected to a Wi-Fi network, the PLAY key starts the newest G-code file of the SD card. Otherwise it keeps its usual meaning (resume). `$Falcon/Play` tells which file it would start. |
| **Wi-Fi fallback in 30 s** | `WebUI/WifiConfig.cpp` | With `$WiFi/Mode=STA>AP`, the access point (with captive portal) opens after 30 s instead of 40. |
| **`M7`/`M8`/`M9` accepted** | `GCode.cpp` | Stock FluidNC silently ignores them when no coolant pin is configured; the pump has none. |
| **SD card robustness** | `esp32/sdspi.cpp`, `FluidPath.cpp`, `WebUI/*`, `FileStream.cpp` | Card identified at 400 kHz, card kept mounted, web server no longer aborts on a filesystem error, driver errors routed to the FluidNC log, `$Falcon/SdTest` read test. |
| **Crash backtrace** | `esp32/backtrace.c` | The board has no serial console: the last panic backtrace is kept in RTC memory and shown by `$Backtrace/Show`. |
| **Octal PSRAM** | `platformio_override.ini` | The N8R8 module has 8 MB of octal PSRAM. Without it, FluidNC ran with a few KB of free heap and uploads failed. |

## Building and flashing

```
pio run -e wifi_s3_falcon2
```

Flash `firmware.bin` at 0x10000 (with the usual bootloader, partitions and
`boot_app0.bin`) the first time over USB; later updates can go through the
FluidNC web UI (`/updatefw`).

## Configuration

A configuration for stock FluidNC is proposed to
[fluidnc-config-files](https://github.com/bdring/fluidnc-config-files)
(`contributed/Creality_Falcon2`). With this branch, add:

```yaml
falcon_head:
  uart_num: 1
  handshake: true

falcon_pump:
  m8_level: 100
  m7_level: 50
  laser_level: 100
  run_on_ms: 5000

falcon_play:
  always: false
```

## Pinout (found by testing the machine and reading the stock firmware)

| GPIO | Use |
|---|---|
| 39 / 38 (`:low`) | X step / direction |
| 41 / 42 | Y step / direction |
| 47 | stepper enable (low = enabled) |
| 40 / 48 | X / Y limit switches (both switches of an axis on one pin, high when pressed); home is front-left |
| 46 / 45 | laser PWM (1 kHz) / head power enable; both high = beam |
| 17 / 18 | UART1 TX / RX to the laser head, 9600 8N1 |
| 43 / 44 | air pump H-bridge (also UART0!) |
| 4 / 6 / 5 / 7 | SD card MISO / MOSI / SCK / CS (SPI2) |
| 21 | buzzer |
| 11, 9, 3, 12 | keypad UP, DOWN, LEFT, RIGHT (low when pressed, need pull-ups) |
| 10, 13, 14 | keypad HOME, FRAME, PLAY/PAUSE |

## Known limits

- The head's alarms (flame, temperature, air flow) are received but not acted on:
  do not leave a job unattended.
- `$GPIO/Dump` crashes on ESP32-S3 in v4.1.1 (a fix is proposed upstream).
- Some SD cards (a 4 GB Intenso brand here) time out on reads in SPI mode; other
  cards work at 8 MHz without errors.

This work was done with the help of an AI assistant (Claude, by Anthropic), and
tested on the machine by its owner.
