#include "GoGoBoard_Arduino.h"

#include <HardwareTimer.h>
#include <numeric>

// #if defined(__STM32F1__)
// //? this for stm32 arduino maple
// #include <usb_serial.h>
// extern USBSerial Serial;
// #define SerialUSB Serial
// #else
// //? this for stm32duino core
// #include <USBSerial.h>
// extern USBSerial SerialUSB;
// #endif

#define gogoSerial Serial1

// * //////////////////////////////////////////////////////////////
// *  Serial handler Definitions
#define SER_WAITING_FOR_1ST_HEADER      1
#define SER_WAITING_FOR_2ND_HEADER      2
#define SER_WAITING_FOR_LENGTH          3
#define SER_WAITING_FOR_CMD             4
#define SER_CHECKING_PACKET_TYPE        5

#define SERIAL_1ST_HEADER               0x54
#define SERIAL_2ND_HEADER               0xFE

#define ARDUINO_REPORT_PACKET_TYPE      31

uint8_t GoGoBoard::gblExtSerialState = SER_WAITING_FOR_1ST_HEADER;
uint8_t GoGoBoard::gblExtSerialCmdChecksum = 0;
uint8_t GoGoBoard::inExtLength = 0;
uint8_t GoGoBoard::gblExtSerialCmdCounter = 0;
bool GoGoBoard::gblUseFirstExtCmdBuffer = false;
bool GoGoBoard::gblNewExtCmdReady = false;
uint8_t GoGoBoard::gbl1stExtCMDBuffer[64] = {0};
uint8_t GoGoBoard::gbl2ndExtCMDBuffer[64] = {0};

String GoGoBoard::_key = String();
gmessage GoGoBoard::gmessage_list;
volatile bool GoGoBoard::isSerialAvailable = false;

GoGoBoard::GoGoBoard(void) {}

GoGoBoard::~GoGoBoard(void) {}

void GoGoBoard::gogoSerialEvent()
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
            if (inByte == ARDUINO_REPORT_PACKET_TYPE && gblExtSerialState == SER_CHECKING_PACKET_TYPE)
            {
                gblExtSerialCmdChecksum = 0;
                gblExtSerialState = SER_WAITING_FOR_LENGTH;
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

void GoGoBoard::processGmessage()
{
    if (gblNewExtCmdReady)
    {
        //? using first buffer, its inverted value
        uint8_t *buf = (!gblUseFirstExtCmdBuffer) ? gbl1stExtCMDBuffer : gbl2ndExtCMDBuffer;
        buf[buf[1] + 2] = '\0'; //? add null terminator

        char *p = (char *)buf + 2;
        _key = String(strtok_r(p, ",", &p));

        gmessage_list[_key].stringValue = String(strtok_r(p, ",", &p));
        gmessage_list[_key].isNewValue = true;

        gblNewExtCmdReady = false;
    }
}

bool GoGoBoard::isGmessageAvailable(const String &key)
{
    auto gmessage = gmessage_list.find(key);
    if (gmessage != gmessage_list.end())
    {
        return gmessage->second.isNewValue;
    }
    return false;
}

String GoGoBoard::Gmessage(const String &key, const String &defaultValue)
{
    auto gmessage = gmessage_list.find(key);
    if (gmessage != gmessage_list.end())
    {
        gmessage->second.isNewValue = false;
        return gmessage->second.stringValue;
    }
    return defaultValue;
}

void GoGoBoard::irqCallback(void)
{
    static int HBCounter = 0;
    static int toggle = 0;
    if (HBCounter++ > 1000)
    {
        toggle ^= 1;
        digitalWrite(GOGO_LED_PIN, toggle);
        HBCounter = 0;
    }

    //? emulate delay 1ms but non-blocking process
    if (!isSerialAvailable)
        isSerialAvailable = true;

    gogoSerialEvent();
    processGmessage();
}

void GoGoBoard::begin(void)
{
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

    // pinMode(GOGO_RESET_BUTTON, INPUT);
    // attachInterrupt(GOGO_RESET_BUTTON, resetCallback, HIGH);
}

int GoGoBoard::readInput(uint8_t port)
{
    if (port < 1 || port > 4)
        return 0;
}

void GoGoBoard::talkToServo(String servo_port)
{
    uint8_t servoBits = 0;

    if (servo_port.length() < 1 || servo_port.length() > 4)
        return;

    for (int i = 0; i < 4; i++)
    {
        if (servo_port[i] == '1')
        {
            servoBits |= 1;
        }
        else if (servo_port[i] == '2')
        {
            servoBits |= 2;
        }
        else if (servo_port[i] == '3')
        {
            servoBits |= 4;
        }
        else if (servo_port[i] == '4')
        {
            servoBits |= 8;
        }
    }
    sendCmdPacket(CMD_PACKET, CMD_SERVO_ACTIVE, servoBits, 0);
}

void GoGoBoard::setServoHead(int head_angle)
{
    if (head_angle < 0 || head_angle > 180)
        return;

    sendCmdPacket(CMD_PACKET, CMD_SERVO_SET_ANGLE, 0, head_angle);
}

void GoGoBoard::turnServoThisWay(int cw_angle)
{
    if (cw_angle < 0 || cw_angle > 180)
        return;

    sendCmdPacket(CMD_PACKET, CMD_SERVO_THISWAY, 0, cw_angle);
}

void GoGoBoard::turnServoThatWay(int ccw_angle)
{
    if (ccw_angle < 0 || ccw_angle > 180)
        return;

    sendCmdPacket(CMD_PACKET, CMD_SERVO_THATWAY, 0, ccw_angle);
}

void GoGoBoard::talkToOutput(String output_port)
{
    uint8_t motorBits = 0;
    output_port.toLowerCase();

    if (output_port.length() < 1 || output_port.length() > 4)
        return;

    for (int i = 0; i < 4; i++)
    {
        if (output_port[i] == 'a')
        {
            motorBits |= 1;
        }
        else if (output_port[i] == 'b')
        {
            motorBits |= 2;
        }
        else if (output_port[i] == 'c')
        {
            motorBits |= 4;
        }
        else if (output_port[i] == 'd')
        {
            motorBits |= 8;
        }
    }
    sendCmdPacket(CMD_PACKET, CMD_MOTOR_SET_ACTIVE, motorBits, 0);
}

void GoGoBoard::setOutputPower(int power)
{
    if (power < 0 || power > 100)
        return;

    sendCmdPacket(CMD_PACKET, CMD_MOTOR_SET_POWER, 0, power);
}

void GoGoBoard::turnOutputONOFF(int state)
{
    state &= 1;

    uint8_t tmp[5] = {0, CMD_PACKET, CMD_MOTOR_ON_OFF, 0, state};
    sendCmdPacket(tmp, 5);
}

void GoGoBoard::turnOutputON(void)
{
    turnOutputONOFF(1);
}

void GoGoBoard::turnOutputOFF(void)
{
    turnOutputONOFF(0);
}

void GoGoBoard::turnOutputDirection(int dir)
{
    dir &= 1; //* 1=CW, 0=CCW

    uint8_t tmp[5] = {0, CMD_PACKET, CMD_MOTOR_DIRECTION, 0, dir};
    sendCmdPacket(tmp, 5);
}

void GoGoBoard::turnOutputThisWay(void)
{
    turnOutputDirection(1);
}

void GoGoBoard::turnOutputThatWay(void)
{
    turnOutputDirection(0);
}

void GoGoBoard::toggleOutputWay(void)
{
    sendCmdPacket(CMD_PACKET, CMD_MOTOR_RD, 0, 0);
}

void GoGoBoard::beep(void)
{
    sendCmdPacket(CMD_PACKET, CMD_BEEP, 0, 0);
}

void GoGoBoard::sendGmessage(const String &key, const float value)
{
    _dataStr = key + "," + String(value);

    *(dataPkt) = TYPE_NUMBER;
    *(dataPkt + 1) = _dataStr.length();
    memcpy(dataPkt + 2, _dataStr.c_str(), _dataStr.length());

    sendReportPacket(dataPkt, _dataStr.length() + 2);
}

void GoGoBoard::sendGmessage(const String &key, const String &value)
{
    _dataStr = key + "," + value;

    *(dataPkt) = TYPE_STRING;
    *(dataPkt + 1) = _dataStr.length();
    memcpy(dataPkt + 2, _dataStr.c_str(), _dataStr.length());

    sendReportPacket(dataPkt, _dataStr.length() + 2);
}

void GoGoBoard::sendCmdPacket(uint8_t categoryID, uint8_t cmdID, uint8_t targetVal, int value)
{
    cmdPkt[BYTE_CATEGORY_ID] = categoryID;
    cmdPkt[BYTE_CMD_ID] = cmdID;
    cmdPkt[BYTE_TARGET] = targetVal;
    cmdPkt[BYTE_DATA] = value >> 8;
    cmdPkt[BYTE_DATA + 1] = value & 0xFF;
    cmdPkt[BYTE_CHECKSUM] = categoryID + cmdID + targetVal + value;

    if (isSerialAvailable)
    {
        gogoSerial.write(cmdPkt, 11);
        isSerialAvailable = false;
    }
}

void GoGoBoard::sendCmdPacket(uint8_t *data, uint8_t length)
{
    cmdDynamicPkt[BYTE_PACKET_LENGTH] = length + 1; //? plus checksum byte
    memcpy(cmdDynamicPkt + BYTE_HEADER_OFFSET, data, length);
    cmdDynamicPkt[length + BYTE_HEADER_OFFSET] = std::accumulate(cmdDynamicPkt + BYTE_HEADER_OFFSET, cmdDynamicPkt + BYTE_HEADER_OFFSET + length, 0);

    if (isSerialAvailable)
    {
        gogoSerial.write(cmdDynamicPkt, length + BYTE_HEADER_OFFSET + 1); //? plus checksum byte
        isSerialAvailable = false;
    }
}

void GoGoBoard::sendReportPacket(uint8_t *data, uint8_t length)
{
    reportPkt[BYTE_PACKET_LENGTH] = length + 1; //? plus checksum byte

    memcpy(reportPkt + BYTE_HEADER_OFFSET, data, length);
    reportPkt[length + BYTE_HEADER_OFFSET] = std::accumulate(reportPkt + BYTE_HEADER_OFFSET, reportPkt + BYTE_HEADER_OFFSET + length, 0);

    if (isSerialAvailable)
    {
        gogoSerial.write(reportPkt, length + BYTE_HEADER_OFFSET + 1); //? plus checksum byte
        isSerialAvailable = false;
    }
}
