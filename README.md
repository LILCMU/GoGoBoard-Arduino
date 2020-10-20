# GoGoBoard Library for GoGoBoard 6.x Arduino Core

## Compatibility with the GoGoBoard library

This library is fully compatible with the STM32 arduino core based on STM32duino.

## GoGoBoard library description

GoGoBoard Arduino Library is an extension library for built-in STM32 chip aimed to written in Arduino environment to control and communicate with the used of logo language / blocks programming on GoGoBoard itself.

## Usage

```cpp
#include <GoGoBoardArduino.h>
GoGoBoardArduino gogoIO;

void setup()
{
    gogoIO.begin();
}

void loop() {}
```

### List of commands

|              | Functions name                               | Parameter(s)              | Return value               |
| ------------ | -------------------------------------------- | ------------------------- | -------------------------- |
| **Sensor**   | `readInput(`**param**`)`                     | port number (1,2,3,4)     | sensor value (0-1023)      |
|              |
| **Servo**    | `talkToServo(`**"param"**`)`                 | port name (1,2,3,4)       | -                          |
|              | `setServoHead(`**param**`)`                  | servo angle (0-180)       | -                          |
|              | `setServoThisWay(`**param**`)`               | servo angle (0-180)       | -                          |
|              | `setServoThatWay(`**param**`)`               | servo angle (0-180)       | -                          |
|              |
| **Output**   | `talkToOutput(`**"param"**`)`                | port name (1,2,3,4)       | -                          |
|              | `setOutputPower(`**param**`)`                | power value (0-100)       | -                          |
|              | `turnOutputON()`                             | -                         | -                          |
|              | `turnOutputOFF()`                            | -                         | -                          |
|              | `turnOutputThisWay()`                        | -                         | -                          |
|              | `turnOutputThatWay()`                        | -                         | -                          |
|              | `toggleOutputWay()`                          | -                         | -                          |
|              |
| **Sound**    | `beep()`                                     | -                         | -                          |
|              |
| **Gmessage** | `sendGmessage(`**"param"**`, `**param**`)`   | key, number value         | -                          |
|              | `sendGmessage(`**"param"**`, `**"param"**`)` | key, string value         | -                          |
|              | `isGmessageAvailable(`**"param"**`)`         | key                       | new message ? (true/false) |
|              | `Gmessage(`**"param"**`, `**"param"**`)`     | key, default string value | string message             |
