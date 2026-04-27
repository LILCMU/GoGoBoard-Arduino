#include "GoGoBoardArduino.h"

#include <numeric>

#if defined(__STM32F1__)
//? this for stm32 arduino maple (Roger Clark's Arduino_STM32)
#include <HardwareTimer.h>
#include <usb_serial.h>
extern USBSerial Serial;
#define SerialUSB Serial
//? GoGo6 co-MCU uses USART1 (PA9 TX, PA10 RX) — alias the pre-defined
//? Serial1 instance from the core instead of constructing HardwareSerial
//? directly (Roger Clark's core expects a usart_dev*, not pin numbers).
HardwareSerial& gogoSerial = Serial1;

#elif defined(ARDUINO_ARCH_STM32)
//? this for stm32duino core
#include <HardwareTimer.h>
#include <USBSerial.h>
extern USBSerial SerialUSB;
HardwareSerial gogoSerial(PA10, PA9);

#elif defined(ARDUINO_ARCH_ESP32)
//? GoGo7 co-MCU. Native USB-CDC is `Serial`; the GoGo-facing UART is
//? UART1 with explicit RX/TX pins set in begin() from the GOGO7_*
//? macros in the header.
#define SerialUSB Serial
HardwareSerial gogoSerial(1);

#else
#error "GoGoBoardArduino: unsupported target. Need STM32F1, stm32duino, or ESP32-C3."
#endif

#if defined(ARDUINO_ARCH_ESP32)
static SemaphoreHandle_t gogoSerialSemaphore = NULL;
static SemaphoreHandle_t gogoStateMutex = NULL;
#endif

GoGoBoardArduino GoGoBoard;

uint8_t GoGoBoardArduino::gblExtSerialState = SER_WAITING_FOR_1ST_HEADER;
uint8_t GoGoBoardArduino::gblExtSerialPacketType = 0;
uint8_t GoGoBoardArduino::gblExtSerialCmdChecksum = 0;
uint8_t GoGoBoardArduino::inExtLength = 0;
uint8_t GoGoBoardArduino::gblExtSerialCmdCounter = 0;
bool GoGoBoardArduino::gblUseFirstExtCmdBuffer = false;
bool GoGoBoardArduino::gblNewExtCmdReady = false;
bool GoGoBoardArduino::gblRequestResponseAvailable = false;
bool GoGoBoardArduino::gblResponseArduinoInit = false;
uint8_t GoGoBoardArduino::gbl1stExtCMDBuffer[GOGO_DEFAULT_BUFFER_SIZE] = {0};
uint8_t GoGoBoardArduino::gbl2ndExtCMDBuffer[GOGO_DEFAULT_BUFFER_SIZE] = {0};
uint8_t *GoGoBoardArduino::gblActiveBuffer = NULL;

String GoGoBoardArduino::_key = String();
_gmessage GoGoBoardArduino::_gmessage_list;

String GoGoBoardArduino::_topic = String();
_broadcast GoGoBoardArduino::_broadcast_list;
_cloudmessage GoGoBoardArduino::_cloudmessage_list;

GoGoBoardArduino::GoGoBoardArduino(void) {}

GoGoBoardArduino::~GoGoBoardArduino(void) {}

void GoGoBoardArduino::gogoSerialEvent()
{
    if (gogoSerial.available())
    {
        uint8_t inByte = gogoSerial.read();

        if (inByte == SERIAL_1ST_HEADER && gblExtSerialState == SER_WAITING_FOR_1ST_HEADER)
        {
            gblExtSerialState = SER_WAITING_FOR_2ND_HEADER;
        }
        else if (inByte == SERIAL_2ND_HEADER && gblExtSerialState == SER_WAITING_FOR_2ND_HEADER)
        {
            gblExtSerialState = SER_CHECKING_PACKET_TYPE;
        }
        else
        {
            if (gblExtSerialState == SER_CHECKING_PACKET_TYPE)
            {
                gblExtSerialCmdChecksum = 0;
                gblExtSerialState = SER_WAITING_FOR_LENGTH;
                gblExtSerialPacketType = inByte;
            }
            else if (gblExtSerialState == SER_WAITING_FOR_LENGTH)
            {
                inExtLength = inByte;
                gblExtSerialCmdCounter = 0;
                gblExtSerialState = SER_WAITING_FOR_CMD;
            }
            else if (gblExtSerialState == SER_WAITING_FOR_CMD)
            {
                // ? reach cmd end -> do checksum
                if (gblExtSerialCmdCounter == inExtLength - 1)
                {
                    if (gblExtSerialCmdChecksum == inByte)
                    {
                        gblNewExtCmdReady = true;
                    }
                    gblUseFirstExtCmdBuffer = !gblUseFirstExtCmdBuffer;
                    gblExtSerialState = SER_WAITING_FOR_1ST_HEADER;
                }
                else // else store the cmd in the buffer
                {
                    if (gblUseFirstExtCmdBuffer)
                    {
                        gbl1stExtCMDBuffer[gblExtSerialCmdCounter++] = inByte;
                    }
                    else
                    {
                        gbl2ndExtCMDBuffer[gblExtSerialCmdCounter++] = inByte;
                    }
                    gblExtSerialCmdChecksum += inByte;
                }
            }
        }
    }
}

void GoGoBoardArduino::processPacket()
{
    if (gblNewExtCmdReady)
    {
        //? using first buffer, its inverted value
        gblActiveBuffer = (!gblUseFirstExtCmdBuffer) ? gbl1stExtCMDBuffer : gbl2ndExtCMDBuffer;
        gblNewExtCmdReady = false;

#if defined(ARDUINO_ARCH_ESP32)
        //? Acquire the shared-state mutex before touching
        //? gblRequestResponseAvailable, gblResponseArduinoInit,
        //? _gmessage_list, _broadcast_list, or _cloudmessage_list.
        //? These are all read by the user's loop() task.
        if (xSemaphoreTake(gogoStateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
#endif
            switch (gblExtSerialPacketType)
            {
            case ARDUINO_REQUEST_PACKET_TYPE: //? response request packet type from gogoboard
                switch ((gblActiveBuffer[0]))
                {
                case REQ_READ_INPUT:
                    gblRequestResponseAvailable = true;
                    break;

                case CMD_ARDUINO_INIT:
                    gblResponseArduinoInit = true;
                    break;

                default:
                    break;
                }
                break;

            case ARDUINO_GMESSAGE_PACKET_TYPE:
            {
                gblActiveBuffer[gblActiveBuffer[1] + 2] = '\0'; //? add null terminator

                char *p = (char *)gblActiveBuffer + 2;
                _key = String(strtok_r(p, ",", &p));

                _gmessage_list[_key].stringValue = String(strtok_r(p, ",", &p));
                _gmessage_list[_key].isNewValue = true;
                break;
            }

            case ARDUINO_IOT_PACKET_TYPE:
            {
                gblActiveBuffer[gblActiveBuffer[1] + 2] = '\0'; //? add null terminator

                char *p = (char *)gblActiveBuffer + 2;
                _topic = String(strtok_r(p, ",", &p));

                if (gblActiveBuffer[0] == IOT_BROADCAST_PROCESS_ID)
                {
                    _broadcast_list[_topic] = true;
                }
                else if (gblActiveBuffer[0] == IOT_CLOUD_MESSAGE_PROCESS_ID)
                {
                    _cloudmessage_list[_topic].stringValue = String(strtok_r(p, ",", &p));
                    _cloudmessage_list[_topic].isNewValue = true;
                }
                break;
            }
            }
#if defined(ARDUINO_ARCH_ESP32)
            xSemaphoreGive(gogoStateMutex);
        }
#endif
    }
}

#if defined(__STM32F1__) || defined(ARDUINO_ARCH_STM32)
//? STM32 timer-ISR path. Heartbeats the LED + drives serial state machine.
//? Drains ALL available UART bytes per ISR fire — a single read() was too
//? slow (one byte per ms meant ~100 ms to parse a 100-byte packet).
void GoGoBoardArduino::irqCallback(void)
{
    static int HBCounter = 0;
    static int toggle = 0;
    if (HBCounter++ > 1000)
    {
        toggle ^= 1;
        digitalWrite(GOGO_LED_PIN, toggle);
        HBCounter = 0;
    }

    while (gogoSerial.available())
    {
        gogoSerialEvent();
    }
    processPacket();
}
#endif

#if defined(ARDUINO_ARCH_ESP32)
//? ESP32 event-driven packet handler. A binary semaphore blocks the
//? task until the UART ISR signals data arrival. When woken, the task
//? drains ALL available bytes in one burst then dispatches. No wasted
//? polling cycles, no fixed-delay guesswork. 100 ms timeout prevents
//? the task from blocking forever if the semaphore is never given.
//?
//? The onReceive callback is IRAM_ATTR and ISR-safe: it only gives a
//? semaphore. All byte processing and String/map work stays in task
//? context, matching the ESP-IDF pattern used on the main board.

static void IRAM_ATTR onGogoSerialRx(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(gogoSerialSemaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void gogoSerialTask(void *)
{
    for (;;)
    {
        if (xSemaphoreTake(gogoSerialSemaphore, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // Drain all buffered bytes before processing any packet.
            while (gogoSerial.available())
            {
                GoGoBoardArduino::gogoSerialEvent();
            }
            GoGoBoardArduino::processPacket();
        }

#if CONFIG_FREERTOS_USE_TRACE_FACILITY
        // Stack watermark check — log if below 512 bytes free.
        static uint32_t watermarkTick = 0;
        if (++watermarkTick > 1000) {
            watermarkTick = 0;
            if (uxTaskGetStackHighWaterMark(NULL) < 512) {
                SerialUSB.println("WARN: gogoSerial task stack low");
            }
        }
#endif
    }
}
#endif

void GoGoBoardArduino::begin(void)
{
    SerialUSB.begin(115200);
#if defined(ARDUINO_ARCH_ESP32)
    //? ESP32-C3 needs explicit RX/TX pins for non-default UART1 routing.
    gogoSerial.begin(GOGO_DEFAULT_BAUDRATE, SERIAL_8N1, GOGO7_RX_PIN, GOGO7_TX_PIN);
#else
    gogoSerial.begin(GOGO_DEFAULT_BAUDRATE);
#endif

#if defined(__STM32F1__) || defined(ARDUINO_ARCH_STM32)
    //? GoGo Board 6 has a dedicated heartbeat LED. GoGo Board 7 omits it.
    pinMode(GOGO_LED_PIN, OUTPUT);
#endif

#if defined(__STM32F1__)
    Timer1.pause();
    Timer1.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);
    Timer1.setPeriod(1000);
    Timer1.setCompare(TIMER_CH1, 1);
    Timer1.attachInterrupt(TIMER_CH1, irqCallback);
    Timer1.refresh();
    Timer1.resume();

#elif defined(ARDUINO_ARCH_STM32)
    HardwareTimer *gogoTimer = new HardwareTimer(TIM1);
    gogoTimer->setOverflow(1000, MICROSEC_FORMAT);
    gogoTimer->attachInterrupt(irqCallback);
    gogoTimer->resume();

#elif defined(ARDUINO_ARCH_ESP32)
    //? Create synchronisation primitives and register the UART Rx
    //? callback BEFORE spawning the task so the semaphore exists when
    //? the first byte fires the ISR.
    gogoSerialSemaphore = xSemaphoreCreateBinary();
    gogoStateMutex = xSemaphoreCreateMutex();

    gogoSerial.onReceive(onGogoSerialRx);

    //? Spawn the packet-handler task at priority 2 (above loopTask at
    //? priority 1) so serial processing is never starved by a
    //? compute-bound user sketch.  4 KB stack is comfortable.
    BaseType_t taskRc = xTaskCreate(gogoSerialTask, "gogoSerial",
                                    4096, nullptr, 2, nullptr);
    if (taskRc != pdPASS) {
        SerialUSB.println("FATAL: failed to create gogoSerial task. "
                          "GoGoBoard will not communicate.");
        // Prevent the sketch from proceeding with a dead link.
        for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
#endif

    delay(2000); //? waiting for gogo to boot up

    //? Identify ourselves to the main board first, then run the legacy
    //? init handshake. Order matters: future auto-detect logic on the
    //? main board may use the HELLO frame to switch its parser into
    //? Arduino-bridge mode before any other packet arrives.
    sendHello();
    sendCmdPacket((uint8_t)CMD_PACKET, (uint8_t)CMD_ARDUINO_INIT, 0, 0, false);
}

void GoGoBoardArduino::sendHello(void)
{
    //? HELLO payload layout (after the standard 0x54 0xFE TYPE LEN frame):
    //?   [endpoint=0][CMD_PACKET][CMD_HELLO][firmware_id]
    //?   [proto_version][version_major][version_minor][version_patch]
    //? Old gogo-firmware that doesn't recognise CMD_HELLO ignores the
    //? whole packet (or logs unknown-cmd) — additive, no break.
    uint8_t hello[8] = {
        0,                                  // BYTE_PACKET_ENDPOINT
        CMD_PACKET,                         // BYTE_CATEGORY_ID
        CMD_HELLO,                          // BYTE_CMD_ID
        GOGOBOARD_FIRMWARE_ID_ARDUINO,      // firmware identity
        CMD_HELLO_PROTO_VERSION,            // payload schema version
        GOGOBOARD_LIB_VERSION_MAJOR,
        GOGOBOARD_LIB_VERSION_MINOR,
        GOGOBOARD_LIB_VERSION_PATCH,
    };
    sendCmdPacket(hello, sizeof(hello));
}

int GoGoBoardArduino::readInput(uint8_t port)
{
    if (port < 1 || port > 4)
        return 0;

    sendCmdPacket(CMD_PACKET, REQ_READ_INPUT, (port - 1), 0, false);

    delay(10); //? waiting for response
    int result = 0;
#if defined(ARDUINO_ARCH_ESP32)
    if (xSemaphoreTake(gogoStateMutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        if (gblRequestResponseAvailable)
        {
            gblRequestResponseAvailable = false;
            result = (int)gblActiveBuffer[1] << 8 | gblActiveBuffer[2];
        }
        xSemaphoreGive(gogoStateMutex);
    }
#else
    //? STM32 path: single-threaded (ISR can interrupt).  Reading a
    //? 2-byte value + bool flag from the buffer is atomic enough in
    //? practice on Cortex-M3 with the current simple sketches.  A
    //? proper mutex would require portENTER_CRITICAL here.
    if (gblRequestResponseAvailable)
    {
        gblRequestResponseAvailable = false;
        result = (int)gblActiveBuffer[1] << 8 | gblActiveBuffer[2];
    }
#endif
    return result;
}

void GoGoBoardArduino::talkToServo(String servo_port)
{
    if (servo_port.length() < 1 || servo_port.length() > 4)
        return;

    sendCmdPacket(CMD_PACKET, CMD_SERVO_ACTIVE, portsToBits(servo_port));
}

void GoGoBoardArduino::talkToServo(int param1, int param2, int param3, int param4)
{
    String servoStr = String(param1) + String(param2) + String(param3) + String(param4);

    if (servoStr.length() < 1 || servoStr.length() > 4)
        return;

    sendCmdPacket(CMD_PACKET, CMD_SERVO_ACTIVE, portsToBits(servoStr));
}

void GoGoBoardArduino::setServoHead(int head_angle)
{
    if (head_angle < 0 || head_angle > 180)
        return;

    sendCmdPacket(CMD_PACKET, CMD_SERVO_SET_ANGLE, 0, head_angle);
}

void GoGoBoardArduino::turnServoThisWay(int cw_angle)
{
    if (cw_angle < 0 || cw_angle > 180)
        return;

    sendCmdPacket(CMD_PACKET, CMD_SERVO_THISWAY, 0, cw_angle);
}

void GoGoBoardArduino::turnServoThatWay(int ccw_angle)
{
    if (ccw_angle < 0 || ccw_angle > 180)
        return;

    sendCmdPacket(CMD_PACKET, CMD_SERVO_THATWAY, 0, ccw_angle);
}

void GoGoBoardArduino::setServoPower(int power)
{
    if (power < 0 || power > 100)
        return;

    sendCmdPacket(CMD_PACKET, CMD_SERVO_POWER, 0, power);
}

void GoGoBoardArduino::talkToOutput(String output_port)
{
    if (output_port.length() < 1 || output_port.length() > 4)
        return;

    sendCmdPacket(CMD_PACKET, CMD_MOTOR_SET_ACTIVE, portsToBits(output_port));
}

void GoGoBoardArduino::talkToOutput(int param1, int param2, int param3, int param4)
{
    String outputStr = String(param1) + String(param2) + String(param3) + String(param4);

    if (outputStr.length() < 1 || outputStr.length() > 4)
        return;

    sendCmdPacket(CMD_PACKET, CMD_MOTOR_SET_ACTIVE, portsToBits(outputStr));
}

void GoGoBoardArduino::setOutputPower(int power)
{
    if (power < 0 || power > 100)
        return;

    sendCmdPacket(CMD_PACKET, CMD_MOTOR_SET_POWER, 0, power);
}

void GoGoBoardArduino::turnOutputONOFF(int state)
{
    state &= 1;

    uint8_t tmp[5] = {0, CMD_PACKET, CMD_MOTOR_ON_OFF, 0, state};
    sendCmdPacket(tmp, 5);
}

void GoGoBoardArduino::turnOutputON(void)
{
    turnOutputONOFF(1);
}

void GoGoBoardArduino::turnOutputOFF(void)
{
    turnOutputONOFF(0);
}

void GoGoBoardArduino::turnOutputDirection(int dir)
{
    dir &= 1; //* 1=CW, 0=CCW

    uint8_t tmp[5] = {0, CMD_PACKET, CMD_MOTOR_DIRECTION, 0, dir};
    sendCmdPacket(tmp, 5);
}

void GoGoBoardArduino::turnOutputThisWay(void)
{
    turnOutputDirection(1);
}

void GoGoBoardArduino::turnOutputThatWay(void)
{
    turnOutputDirection(0);
}

void GoGoBoardArduino::toggleOutputWay(void)
{
    sendCmdPacket((uint8_t)CMD_PACKET, (uint8_t)CMD_MOTOR_RD);
}

void GoGoBoardArduino::beep(void)
{
    sendCmdPacket((uint8_t)CMD_PACKET, (uint8_t)CMD_BEEP);
}

void GoGoBoardArduino::connectToWifi(const String &ssid, const String &password)
{
    _dataStr = ssid + "," + password;

    sendIoTPacket(DATADRIVEN_CMD_PACKET, CMD_WIFI_CONNECT, (uint8_t *)_dataStr.c_str(), _dataStr.length(), true);
}

void GoGoBoardArduino::sendGmessage(const String &key, const float value)
{
    _dataStr = key + "," + String(value);

    *(dataPkt) = TYPE_NUMBER;
    *(dataPkt + 1) = _dataStr.length();
    memcpy(dataPkt + 2, _dataStr.c_str(), _dataStr.length());

    sendReportPacket(dataPkt, _dataStr.length() + 2);
}

void GoGoBoardArduino::sendGmessage(const String &key, const String &value)
{
    _dataStr = key + "," + value;

    *(dataPkt) = TYPE_STRING;
    *(dataPkt + 1) = _dataStr.length();
    memcpy(dataPkt + 2, _dataStr.c_str(), _dataStr.length());

    sendReportPacket(dataPkt, _dataStr.length() + 2);
}

bool GoGoBoardArduino::isGmessageAvailable(const String &key)
{
#if defined(ARDUINO_ARCH_ESP32)
    if (xSemaphoreTake(gogoStateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        auto gmessage = _gmessage_list.find(key);
        bool avail = (gmessage != _gmessage_list.end()) ? gmessage->second.isNewValue : false;
        xSemaphoreGive(gogoStateMutex);
        return avail;
    }
    return false;
#else
    auto gmessage = _gmessage_list.find(key);
    if (gmessage != _gmessage_list.end())
    {
        return gmessage->second.isNewValue;
    }
    return false;
#endif
}

String GoGoBoardArduino::Gmessage(const String &key, const String &defaultValue)
{
#if defined(ARDUINO_ARCH_ESP32)
    if (xSemaphoreTake(gogoStateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        auto gmessage = _gmessage_list.find(key);
        if (gmessage != _gmessage_list.end())
        {
            gmessage->second.isNewValue = false;
            String val = gmessage->second.stringValue;
            xSemaphoreGive(gogoStateMutex);
            return val;
        }
        xSemaphoreGive(gogoStateMutex);
    }
    return defaultValue;
#else
    auto gmessage = _gmessage_list.find(key);
    if (gmessage != _gmessage_list.end())
    {
        gmessage->second.isNewValue = false;
        return gmessage->second.stringValue;
    }
    return defaultValue;
#endif
}

void GoGoBoardArduino::setBroadcastChannel(uint32_t channel)
{
    _dataStr = String(channel);

    sendIoTPacket(CATEGORY_IOT_BROADCAST, IOT_BROADCAST_SET_CHANNEL, (uint8_t *)_dataStr.c_str(), _dataStr.length());
}

void GoGoBoardArduino::setBroadcastPassword(const String &password)
{
    sendIoTPacket(CATEGORY_IOT_BROADCAST, IOT_BROADCAST_SET_CHANNEL, (uint8_t *)password.c_str(), password.length());
}

void GoGoBoardArduino::sendBroadcast(const String &topic)
{
    if (gblResponseArduinoInit)
        sendIoTPacket(CATEGORY_IOT_BROADCAST, IOT_BROADCAST_SEND, (uint8_t *)topic.c_str(), topic.length());
}

bool GoGoBoardArduino::receiveBroadcast(const String &topic)
{
#if defined(ARDUINO_ARCH_ESP32)
    bool found = false;
    bool status = false;
    if (xSemaphoreTake(gogoStateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        auto it = _broadcast_list.find(topic);
        if (it != _broadcast_list.end())
        {
            found = true;
            status = it->second;
            if (status)
                it->second = false;
        }
        xSemaphoreGive(gogoStateMutex);
    }
    if (found)
        return status;

    //? not yet subscribed — send subscribe request (outside mutex)
    if (gblResponseArduinoInit)
        sendIoTPacket(CATEGORY_IOT_BROADCAST, IOT_BROADCAST_RECEIVE,
                      (uint8_t *)topic.c_str(), topic.length());
    return false;
#else
    auto broadcast = _broadcast_list.find(topic);
    if (broadcast != _broadcast_list.end())
    {
        bool status = broadcast->second;
        if (status)
            broadcast->second = false;
        return status;
    }
    else //? may not subscribe broadcast topic yet
    {
        if (gblResponseArduinoInit)
            sendIoTPacket(CATEGORY_IOT_BROADCAST, IOT_BROADCAST_RECEIVE,
                          (uint8_t *)topic.c_str(), topic.length());
    }
    return false;
#endif
}

void GoGoBoardArduino::sendCloudMessage(const String &topic, const float payload)
{
    _dataStr = topic + "," + String(payload);

    if (gblResponseArduinoInit)
        sendIoTPacket(CATEGORY_IOT_CLOUD_MESSAGE, IOT_CLOUD_MESSAGE_PUBLISH, (uint8_t *)_dataStr.c_str(), _dataStr.length());
}

void GoGoBoardArduino::sendCloudMessage(const String &topic, const String &payload)
{
    _dataStr = topic + "," + payload;

    if (gblResponseArduinoInit)
        sendIoTPacket(CATEGORY_IOT_CLOUD_MESSAGE, IOT_CLOUD_MESSAGE_PUBLISH, (uint8_t *)_dataStr.c_str(), _dataStr.length());
}

bool GoGoBoardArduino::isCloudMessageAvailable(const String &topic)
{
#if defined(ARDUINO_ARCH_ESP32)
    bool found = false;
    bool avail = false;
    if (xSemaphoreTake(gogoStateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        auto it = _cloudmessage_list.find(topic);
        if (it != _cloudmessage_list.end())
        {
            found = true;
            avail = it->second.isNewValue;
        }
        xSemaphoreGive(gogoStateMutex);
    }
    if (found)
        return avail;

    //? not yet subscribed — send subscribe request (outside mutex)
    if (gblResponseArduinoInit)
        sendIoTPacket(CATEGORY_IOT_CLOUD_MESSAGE, IOT_CLOUD_MESSAGE_SUBSCRIBE,
                      (uint8_t *)topic.c_str(), topic.length());
    return false;
#else
    auto cloudmessage = _cloudmessage_list.find(topic);
    if (cloudmessage != _cloudmessage_list.end())
    {
        return cloudmessage->second.isNewValue;
    }
    else //? may not subscribe cloudmessage topic yet
    {
        if (gblResponseArduinoInit)
            sendIoTPacket(CATEGORY_IOT_CLOUD_MESSAGE, IOT_CLOUD_MESSAGE_SUBSCRIBE,
                          (uint8_t *)topic.c_str(), topic.length());
    }
    return false;
#endif
}

String GoGoBoardArduino::Cloudmessage(const String &topic, const String &defaultValue)
{
#if defined(ARDUINO_ARCH_ESP32)
    if (xSemaphoreTake(gogoStateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        auto it = _cloudmessage_list.find(topic);
        if (it != _cloudmessage_list.end())
        {
            it->second.isNewValue = false;
            String val = it->second.stringValue;
            xSemaphoreGive(gogoStateMutex);
            return val;
        }
        xSemaphoreGive(gogoStateMutex);
    }
    return defaultValue;
#else
    auto iotmessage = _cloudmessage_list.find(topic);
    if (iotmessage != _cloudmessage_list.end())
    {
        iotmessage->second.isNewValue = false;
        return iotmessage->second.stringValue;
    }
    return defaultValue;
#endif
}

uint8_t GoGoBoardArduino::portsToBits(const String &ports)
{
    uint8_t bits = 0;

    for (int i = 0; i < ports.length(); i++)
    {
        if (ports[i] == '1')
        {
            bits |= 1;
        }
        else if (ports[i] == '2')
        {
            bits |= 2;
        }
        else if (ports[i] == '3')
        {
            bits |= 4;
        }
        else if (ports[i] == '4')
        {
            bits |= 8;
        }
    }
    return bits;
}

void GoGoBoardArduino::sendCmdPacket(uint8_t categoryID, uint8_t cmdID, uint8_t targetVal, int value, bool isCmd)
{
    if (isCmd)
        cmdDynamicPkt[BYTE_PACKET_TYPE] = ARDUINO_CMD_PACKET_TYPE;
    else
        cmdDynamicPkt[BYTE_PACKET_TYPE] = ARDUINO_REQUEST_PACKET_TYPE;

    cmdDynamicPkt[BYTE_PACKET_LENGTH] = 7;
    cmdDynamicPkt[BYTE_PACKET_ENDPOINT] = 0;
    cmdDynamicPkt[BYTE_CATEGORY_ID] = categoryID;
    cmdDynamicPkt[BYTE_CMD_ID] = cmdID;
    cmdDynamicPkt[BYTE_TARGET] = targetVal;
    cmdDynamicPkt[BYTE_DATA] = value >> 8;
    cmdDynamicPkt[BYTE_DATA + 1] = value & 0xFF;
    cmdDynamicPkt[BYTE_CHECKSUM] = categoryID + cmdID + targetVal + value;

    delay(10);
    gogoSerial.write(cmdDynamicPkt, 11);
}

void GoGoBoardArduino::sendCmdPacket(uint8_t *data, uint8_t length, bool isCmd)
{
    if (isCmd)
        cmdDynamicPkt[BYTE_PACKET_TYPE] = ARDUINO_CMD_PACKET_TYPE;
    else
        cmdDynamicPkt[BYTE_PACKET_TYPE] = ARDUINO_REQUEST_PACKET_TYPE;

    cmdDynamicPkt[BYTE_PACKET_LENGTH] = length + 1; //? plus checksum byte
    memcpy(cmdDynamicPkt + BYTE_HEADER_OFFSET, data, length);
    cmdDynamicPkt[length + BYTE_HEADER_OFFSET] = std::accumulate(cmdDynamicPkt + BYTE_HEADER_OFFSET, cmdDynamicPkt + BYTE_HEADER_OFFSET + length, 0);

    delay(10);
    gogoSerial.write(cmdDynamicPkt, length + BYTE_HEADER_OFFSET + 1); //? plus checksum byte
}

void GoGoBoardArduino::sendReportPacket(uint8_t *data, uint8_t length)
{
    reportPkt[BYTE_PACKET_LENGTH] = length + 1; //? plus checksum byte

    memcpy(reportPkt + BYTE_HEADER_OFFSET, data, length);
    reportPkt[length + BYTE_HEADER_OFFSET] = std::accumulate(reportPkt + BYTE_HEADER_OFFSET, reportPkt + BYTE_HEADER_OFFSET + length, 0);

    delay(10);
    gogoSerial.write(reportPkt, length + BYTE_HEADER_OFFSET + 1);
}

void GoGoBoardArduino::sendIoTPacket(uint8_t categoryID, uint8_t cmdID, uint8_t *data, uint8_t length, bool isCmd)
{
    if (isCmd)
        cmdDynamicPkt[BYTE_PACKET_TYPE] = ARDUINO_CMD_PACKET_TYPE;
    else
        cmdDynamicPkt[BYTE_PACKET_TYPE] = ARDUINO_IOT_PACKET_TYPE;

    cmdDynamicPkt[BYTE_PACKET_LENGTH] = length + 5; //? plus 3 IDs, data len and checksum
    cmdDynamicPkt[BYTE_PACKET_ENDPOINT] = 0;
    cmdDynamicPkt[BYTE_CATEGORY_ID] = categoryID;
    cmdDynamicPkt[BYTE_CMD_ID] = cmdID;
    cmdDynamicPkt[BYTE_TARGET] = length;

    memcpy(cmdDynamicPkt + BYTE_HEADER_OFFSET + 4, data, length);
    cmdDynamicPkt[length + BYTE_HEADER_OFFSET + 4] = std::accumulate(cmdDynamicPkt + BYTE_HEADER_OFFSET, cmdDynamicPkt + BYTE_HEADER_OFFSET + length + 4, 0);

    delay(10);
    gogoSerial.write(cmdDynamicPkt, length + BYTE_HEADER_OFFSET + 5);
}
