#include "GoGoBoardArduino.h"

#include <HardwareTimer.h>
#include <numeric>

#if defined(__STM32F1__)
//? this for stm32 arduino maple
#include <usb_serial.h>
extern USBSerial Serial;
#define SerialUSB Serial
#else
//? this for stm32duino core
#include <USBSerial.h>
extern USBSerial SerialUSB;
#endif

#define gogoSerial Serial1

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
        gblNewExtCmdReady = false;
    }
}

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

    gogoSerialEvent();
    processPacket();
}

void GoGoBoardArduino::begin(void)
{
    SerialUSB.begin();
    gogoSerial.begin(GOGO_DEFAULT_BAUDRATE);
    pinMode(GOGO_LED_PIN, OUTPUT);

#if defined(__STM32F1__)
    Timer1.pause();
    Timer1.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);
    Timer1.setPeriod(1000);
    Timer1.setCompare(TIMER_CH1, 1);
    Timer1.attachInterrupt(TIMER_CH1, irqCallback);
    Timer1.refresh();
    Timer1.resume();

#else
    HardwareTimer *gogoTimer = new HardwareTimer(TIM1);
    gogoTimer->setOverflow(1000, MICROSEC_FORMAT);
    gogoTimer->attachInterrupt(irqCallback);
    gogoTimer->resume();
#endif

    delay(2000); //? waiting for gogo to boot up
    sendCmdPacket((uint8_t)CMD_PACKET, (uint8_t)CMD_ARDUINO_INIT, 0, 0, false);
}

int GoGoBoardArduino::readInput(uint8_t port)
{
    if (port < 1 || port > 4)
        return 0;

    sendCmdPacket(CMD_PACKET, REQ_READ_INPUT, (port - 1), 0, false);

    delay(10); //? waiting for response
    if (gblRequestResponseAvailable)
    {
        gblRequestResponseAvailable = false;

        return (int)gblActiveBuffer[1] << 8 | gblActiveBuffer[2];
    }
    else
    {
        return 0;
    }
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
    auto gmessage = _gmessage_list.find(key);
    if (gmessage != _gmessage_list.end())
    {
        return gmessage->second.isNewValue;
    }
    return false;
}

String GoGoBoardArduino::Gmessage(const String &key, const String &defaultValue)
{
    auto gmessage = _gmessage_list.find(key);
    if (gmessage != _gmessage_list.end())
    {
        gmessage->second.isNewValue = false;
        return gmessage->second.stringValue;
    }
    return defaultValue;
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
            sendIoTPacket(CATEGORY_IOT_BROADCAST, IOT_BROADCAST_RECEIVE, (uint8_t *)topic.c_str(), topic.length());
    }
    return false;
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
    auto cloudmessage = _cloudmessage_list.find(topic);
    if (cloudmessage != _cloudmessage_list.end())
    {
        return cloudmessage->second.isNewValue;
    }
    else //? may not subscribe cloudmessage topic yet
    {
        if (gblResponseArduinoInit)
            sendIoTPacket(CATEGORY_IOT_CLOUD_MESSAGE, IOT_CLOUD_MESSAGE_SUBSCRIBE, (uint8_t *)topic.c_str(), topic.length());
    }
    return false;
}

String GoGoBoardArduino::Cloudmessage(const String &topic, const String &defaultValue)
{
    auto iotmessage = _cloudmessage_list.find(topic);
    if (iotmessage != _cloudmessage_list.end())
    {
        iotmessage->second.isNewValue = false;
        return iotmessage->second.stringValue;
    }
    return defaultValue;
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
