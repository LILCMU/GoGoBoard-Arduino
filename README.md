# GoGoBoard Library for GoGoBoard 6.x Arduino Core

## Compatibility with the GoGoBoard library

This library is fully compatible with the STM32 arduino core based on STM32duino.

## GoGoBoard library description

GoGoBoard Arduino Library is an extension library for built-in STM32 chip aimed to written in Arduino environment to control and communicate with the used of logo language / blocks programming on GoGoBoard itself.

## Usage

```cpp
#include <GoGoBoardArduino.h>

void setup()
{
    GoGoBoard.begin();
}

void loop() {}
```

### List of commands

|              | Functions name                                  | Parameter(s)                   | Return value               |
| ------------ | ----------------------------------------------- | ------------------------------ | -------------------------- |
| **Sensor**   | `readInput(`**param**`)`                        | port number (1,2,3,4)          | sensor value (0-1023)      |
|              |
| **Servo**    | `talkToServo(`**"param"**`)`                    | port name (1,2,3,4)            | -                          |
|              | `talkToServo(`**param**`)`                      | port number (1,2,3,4)          | -                          |
|              | `setServoHead(`**param**`)`                     | servo angle (0-180)            | -                          |
|              | `turnServoThisWay(`**param**`)`                 | servo angle (0-180)            | -                          |
|              | `turnServoThatWay(`**param**`)`                 | servo angle (0-180)            | -                          |
|              | `setServoPower(`**param**`)`                    | servo power (0-100)            | -                          |
|              |
| **Output**   | `talkToOutput(`**"param"**`)`                   | port name (1,2,3,4)            | -                          |
|              | `talkToOutput(`**param**`)`                     | port number (1,2,3,4)          | -                          |
|              | `setOutputPower(`**param**`)`                   | power value (0-100)            | -                          |
|              | `turnOutputON()`                                | -                              | -                          |
|              | `turnOutputOFF()`                               | -                              | -                          |
|              | `turnOutputONOFF(`**param**`)`                  | on-off status (1=on, 0=off)    | -                          |
|              | `turnOutputThisWay()`                           | -                              | -                          |
|              | `turnOutputThatWay()`                           | -                              | -                          |
|              | `toggleOutputWay()`                             | -                              | -                          |
|              | `turnOutputDirection(`**param**`)`              | direction number (1=CW, 0=CCW) | -                          |
|              |
| **Sound**    | `beep()`                                        | -                              | -                          |
|              |
| **Gmessage** | `sendGmessage(`**"param"**`, `**param**`)`      | key, number value              | -                          |
|              | `sendGmessage(`**"param"**`, `**"param"**`)`    | key, string value              | -                          |
|              | `isGmessageAvailable(`**"param"**`)`            | key                            | new message ? (true/false) |
|              | `Gmessage(`**"param"**`, `**"param"**`)`        | key, default string value      | string message             |
|              |
| **IoT**      | `connectToWifi(`**"param"**`, `**"param"**`)`   | Wi-Fi name, Wi-Fi password     | -                          |
|              | `setBroadcastChannel(`**param**`)`              | channel number                 | -                          |
|              | `setBroadcastPassword(`**"param**"`)`           | password string                | -                          |
|              | `sendBroadcast(`**"param"**`)`                  | broadcast name                 | -                          |
|              | `receiveBroadcast(`**"param"**`)`               | broadcast name                 | new message ? (true/false) |
|              | `sendCloudMessage(`**"param"**`, `**"param"**`)`| topic, string value            | -                          |
|              | `isCloudMessageAvailable(`**"param"**`)`        | topic                          | new message ? (true/false) |
|              | `Cloudmessage(`**"param"**`, `**"param"**`)`    | topic, default string value    | string message             |
---

### List of pins definition (Alias names)

| Alias names        | Arduino pins |
| ------------------ | ------------ |
| `GOGO_SPECIAL_SCL` | PB6          |
| `GOGO_SPECIAL_SDA` | PB7          |
| `GOGO_SPECIAL_D21` | PB8          |
| `GOGO_SPECIAL_D22` | PB9          |
| `GOGO_SPECIAL_D31` | PB10         |
| `GOGO_SPECIAL_D32` | PB11         |
|                    |              |
| `GOGO_GPIO_RX2`    | PA3          |
| `GOGO_GPIO_TX2`    | PA2          |
| `GOGO_GPIO_MOSI`   | PA7          |
| `GOGO_GPIO_MISO`   | PA6          |
| `GOGO_GPIO_SCK`    | PA5          |
| `GOGO_GPIO_NSS`    | PA4          |
| `GOGO_GPIO_SCL`    | PB6          |
| `GOGO_GPIO_SDA`    | PB7          |

---
