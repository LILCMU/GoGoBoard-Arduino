# GoGoBoard Arduino Library

Arduino library for the **co-MCU** built into GoGoBoard. Lets a student-written sketch run on the co-MCU side-by-side with Logo / blocks programs on the main board, and call into GoGo hardware (motors, servos, sensors, beep, GMessage, IoT) via a fixed serial protocol.

## Supported boards

| Board  | Co-MCU            | Arduino core                          |
|--------|-------------------|---------------------------------------|
| GoGo Board 6  | STM32F103         | [Arduino-Maple](https://github.com/rogerclarkmelbourne/Arduino_STM32) **or** [stm32duino](https://github.com/stm32duino/Arduino_Core_STM32) |
| GoGo Board 7  | ESP32-C3          | [Arduino-ESP32](https://github.com/espressif/arduino-esp32) ≥ 3.x |

The library auto-detects the platform via `__STM32F1__` / `ARDUINO_ARCH_STM32` / `ARDUINO_ARCH_ESP32` macros — same source, no per-board sketch changes.

## Install

**Arduino IDE.** *Sketch → Include Library → Manage Libraries…* → search "GoGoBoard Arduino Library" → Install.

**PlatformIO.** Add to `platformio.ini`:
```ini
lib_deps =
    https://github.com/LILCMU/GoGoBoard-Arduino
```

## Usage

```cpp
#include <GoGoBoardArduino.h>

void setup()
{
    GoGoBoard.begin();
}

void loop()
{
    // Your code here. Packet handling runs in the background:
    //   - GoGo Board 6: 1 ms hardware-timer ISR
    //   - GoGo Board 7: dedicated FreeRTOS task at 1 ms cadence
    // so blocking calls in loop() don't stall the link.
}
```

### Compatibility note

`GoGoBoard.begin()` blocks for ~2 s after boot to let the main board come up; that's the same on both targets. Everything else is non-blocking.

## API

|              | Function                                          | Parameter(s)                   | Return                       |
| ------------ | ------------------------------------------------- | ------------------------------ | ---------------------------- |
| **Sensor**   | `readInput(port)`                                 | port number (1–4)              | sensor value (0–1023)        |
|              |                                                                                                                                |
| **Servo**    | `talkToServo("ports")`                            | port string e.g. `"13"`        | —                            |
|              | `talkToServo(p1, p2, p3, p4)`                     | port numbers (1–4)             | —                            |
|              | `setServoHead(angle)`                             | 0–180                          | —                            |
|              | `turnServoThisWay(angle)`                         | 0–180                          | —                            |
|              | `turnServoThatWay(angle)`                         | 0–180                          | —                            |
|              | `setServoPower(power)`                            | 0–100                          | —                            |
|              |                                                                                                                                |
| **Output**   | `talkToOutput("ports")`                           | port string e.g. `"24"`        | —                            |
|              | `talkToOutput(p1, p2, p3, p4)`                    | port numbers (1–4)             | —                            |
|              | `setOutputPower(power)`                           | 0–100                          | —                            |
|              | `turnOutputON()` / `turnOutputOFF()`              | —                              | —                            |
|              | `turnOutputONOFF(state)`                          | 1=on, 0=off                    | —                            |
|              | `turnOutputThisWay()` / `turnOutputThatWay()`     | —                              | —                            |
|              | `toggleOutputWay()`                               | —                              | —                            |
|              | `turnOutputDirection(dir)`                        | 1=CW, 0=CCW                    | —                            |
|              |                                                                                                                                |
| **Sound**    | `beep()`                                          | —                              | —                            |
|              |                                                                                                                                |
| **Gmessage** | `sendGmessage(key, number)`                       | key, float                     | —                            |
|              | `sendGmessage(key, string)`                       | key, String                    | —                            |
|              | `isGmessageAvailable(key)`                        | key                            | bool — new message?          |
|              | `Gmessage(key, defaultValue)`                     | key, fallback                  | String value                 |
|              |                                                                                                                                |
| **IoT**      | `connectToWifi(ssid, password)`                   | strings                        | —                            |
|              | `setBroadcastChannel(channel)`                    | uint32                         | —                            |
|              | `setBroadcastPassword(password)`                  | string                         | —                            |
|              | `sendBroadcast(topic)`                            | string                         | —                            |
|              | `receiveBroadcast(topic)`                         | string                         | bool — new message?          |
|              | `sendCloudMessage(topic, value)`                  | topic, float / String          | —                            |
|              | `isCloudMessageAvailable(topic)`                  | topic                          | bool — new message?          |
|              | `Cloudmessage(topic, defaultValue)`               | topic, fallback                | String value                 |

## Pin macros

The library exports a board-aware set of pin aliases. Use them in your sketch instead of hard-coded GPIO numbers — your code stays portable across GoGo Board 6 and GoGo Board 7.

### GoGo Board 6 (STM32F103)

| Alias              | STM32 pin |
| ------------------ | --------- |
| `GOGO_BOOT_BUTTON` | PB2       |
| `GOGO_LED_PIN`     | PB12      |
| `GOGO_SPECIAL_SCL` | PB6       |
| `GOGO_SPECIAL_SDA` | PB7       |
| `GOGO_SPECIAL_D21` | PB8       |
| `GOGO_SPECIAL_D22` | PB9       |
| `GOGO_SPECIAL_D31` | PB10      |
| `GOGO_SPECIAL_D32` | PB11      |
| `GOGO_GPIO_RX2`    | PA3       |
| `GOGO_GPIO_TX2`    | PA2       |
| `GOGO_GPIO_MOSI`   | PA7       |
| `GOGO_GPIO_MISO`   | PA6       |
| `GOGO_GPIO_SCK`    | PA5       |
| `GOGO_GPIO_NSS`    | PA4       |
| `GOGO_GPIO_SCL`    | PB6       |
| `GOGO_GPIO_SDA`    | PB7       |

### GoGo Board 7 (ESP32-C3)

GoGo Board 7's expansion-port pinout differs from GoGo Board 6 — the symbol names below are board-specific. GoGo Board 6 sketches that referenced `GOGO_SPECIAL_*` / `GOGO_GPIO_RX2/TX2/MOSI/...` aliases need to be ported to the GoGo Board 7 names when targeting the new hardware.

| Alias                        | ESP32-C3 pin | Notes |
| ---------------------------- | ------------ | ----- |
| `GOGO_BOOT_BUTTON`           | 9            | C3 boot button |
| `GOGO_MULTI_SCL`             | 2            | I²C clock (multi-purpose port) |
| `GOGO_MULTI_SDA`             | 8            | I²C data (multi-purpose port) |
| `GOGO_CONFIGURABLE_1_YELLOW` | 0            | Configurable port 1, yellow lead |
| `GOGO_CONFIGURABLE_1_WHITE`  | 1            | Configurable port 1, white lead |
| `GOGO_CONFIGURABLE_2_YELLOW` | 3            | Configurable port 2, yellow lead |
| `GOGO_CONFIGURABLE_2_WHITE`  | 4            | Configurable port 2, white lead |
| `GOGO_GPIO_DI`               | 5            | SPI MISO equivalent (data in) |
| `GOGO_GPIO_DO`               | 7            | SPI MOSI equivalent (data out) |
| `GOGO_GPIO_CLK`              | 6            | SPI clock |
| `GOGO_GPIO_CS`               | 10           | SPI chip-select |
| `GOGO7_RX_PIN`               | 20           | Internal UART RX from main board (UART1) |
| `GOGO7_TX_PIN`               | 21           | Internal UART TX to main board (UART1) |

`GOGO7_RX_PIN` / `GOGO7_TX_PIN` are used internally by `GoGoBoard.begin()` — sketches typically don't need to reference them.

## Examples

`examples/` ships sketches for `beep`, `input`, `output`, `gmessage`, `broadcast`, `cloud-message`. Each compiles unchanged on both GoGo Board 6 and GoGo Board 7.

## How it works

The library speaks a length-framed packet protocol over UART to the GoGo main board. Packets carry one of:
- `ARDUINO_CMD` (motor / servo / beep / WiFi commands)
- `ARDUINO_REQUEST` (read sensor → response)
- `ARDUINO_GMESSAGE` (key/value bridge with Logo)
- `ARDUINO_IOT` (broadcast + cloud-message)

Frames begin with `0x54 0xFE`, then `[type][length][payload][xor checksum]`. Background packet handling runs in:
- **GoGo Board 6** — `Timer1` / `HardwareTimer TIM1` ISR fires every 1 ms, calls `gogoSerialEvent()` + `processPacket()`.
- **GoGo Board 7** — dedicated FreeRTOS task `gogoSerialTask`, same body, same 1 ms cadence, runs in task context (Serial-safe on ESP32).

Either way the user's `loop()` is free to do whatever it wants without starving the link.
