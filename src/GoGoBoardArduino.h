#ifndef __GOGOBOARD_ARDUINO_H__
#define __GOGOBOARD_ARDUINO_H__

#include <Arduino.h>
#include <map>

// * //////////////////////////////////////////////////////////////
// *  Serial handler Definitions
#define SER_WAITING_FOR_1ST_HEADER          1
#define SER_WAITING_FOR_2ND_HEADER          2
#define SER_WAITING_FOR_LENGTH              3
#define SER_WAITING_FOR_CMD                 4
#define SER_CHECKING_PACKET_TYPE            5

#define SERIAL_1ST_HEADER                   0x54
#define SERIAL_2ND_HEADER                   0xFE

#define GOGO_DEFAULT_BAUDRATE               921600
#define GOGO_DEFAULT_BUFFER_SIZE            256

#define GOGO_BOOT_BUTTON                    PB2
#define GOGO_LED_PIN                        PB12

#define GOGO_SPECIAL_SCL                    PB6
#define GOGO_SPECIAL_SDA                    PB7
#define GOGO_SPECIAL_D21                    PB8
#define GOGO_SPECIAL_D22                    PB9
#define GOGO_SPECIAL_D31                    PB10
#define GOGO_SPECIAL_D32                    PB11

#define GOGO_GPIO_RX2                       PA3
#define GOGO_GPIO_TX2                       PA2
#define GOGO_GPIO_MOSI                      PA7
#define GOGO_GPIO_MISO                      PA6
#define GOGO_GPIO_SCK                       PA5
#define GOGO_GPIO_NSS                       PA4
#define GOGO_GPIO_SCL                       PB6
#define GOGO_GPIO_SDA                       PB7

#define ARDUINO_INIT_DEFAULT                0
#define ARDUINO_INIT_IOT                    1

#define ARDUINO_CMD_PACKET_TYPE             30
#define ARDUINO_GMESSAGE_PACKET_TYPE        31
#define ARDUINO_REQUEST_PACKET_TYPE         32
#define ARDUINO_IOT_PACKET_TYPE             33

#define IOT_BROADCAST_PROCESS_ID            100
#define IOT_CLOUD_MESSAGE_PROCESS_ID        200

// * //////////////////////////////////////////////////////////////
// *  Category Definitions
#define CMD_PACKET                          0
#define FLASH_MEMORY_OPERATION_PACKET       1
#define RASPBERRY_PI_CMD_PACKET             2
#define GOGO_ARDUINO_CMD_PACKET             3
#define DATADRIVEN_CMD_PACKET               4
#define HOPHER_CMD_PACKET                   5
#define EVENT_REQUEST_PACKET                20

#define CATEGORY_IOT_BROADCAST              0
#define CATEGORY_IOT_CLOUD_MESSAGE          1

// * //////////////////////////////////////////////////////////////
// *  Command Definitions
#define CMD_PING                            1
#define CMD_MOTOR_ON_OFF                    2
#define CMD_MOTOR_DIRECTION                 3
#define CMD_MOTOR_RD                        4
#define CMD_MOTOR_SET_POWER                 6
#define CMD_MOTOR_SET_ACTIVE                7
#define CMD_MOTOR_TOGGLE_ACTIVE             8

#define CMD_LED_CONTROL                     10
#define CMD_BEEP                            11
#define CMD_AUTORUN_STATE                   12
#define CMD_LOGO_CONTROL                    13

//* servo
#define CMD_SERVO_SETH                      9
#define CMD_SERVO_ACTIVE                    14
#define CMD_SERVO_THISWAY                   15
#define CMD_SERVO_THATWAY                   16
#define CMD_SERVO_SET_ANGLE                 17
#define CMD_SERVO_TOGGLE_ACTIVE             18
#define CMD_SERVO_POWER                     19

#define CMD_SYNC_RTC                        50
#define CMD_READ_RTC                        51
#define CMD_SHOW_SHORT_TEXT                 60
#define CMD_SHOW_LONG_TEXT                  61
#define CMD_CLEAR_SCREEN                    62

#define CMD_VOICE_PLAY_PAUSE                70
#define CMD_VOICE_NEXT_TRACK                71
#define CMD_VOICE_PREV_TRACK                72
#define CMD_VOICE_GOTO_TRACK                73
#define CMD_VOICE_ERASE_ALL_TRACKS          74

#define CMD_KEYBOARD_SEND                   81
#define CMD_IR_SEND                         91

#define CMD_REBOOT                          100
#define CMD_UPDATE_COMPLETED                101
#define CMD_UPDATE_FAILED                   102

#define CMD_UPDATE_FW_OTA                   200
#define CMD_UPDATE_FW_SERIAL                201

#define CMD_ARDUINO_INIT                    210

#define LOGO_SET_MEMORY_POINTER             1
#define FLASH_SET_MEMORY_POINTER            2
#define MEM_WRITE_BYTES                     3
#define MEM_READ_BYTES                      4

#define CMD_WIFI_CONNECT                    1
#define CMD_WIFI_DISCONNECT                 2
#define CMD_UID_DATALOG                     3

#define RCMD_GOGOID                         1

#define REQ_READ_INPUT                      1
#define REQ_READ_FILTERED_INPUT             2

#define IOT_BROADCAST_SET_CHANNEL           1
#define IOT_BROADCAST_SET_PASSWORD          2
#define IOT_BROADCAST_SEND                  3
#define IOT_BROADCAST_RECEIVE               4

#define IOT_CLOUD_MESSAGE_PUBLISH           1
#define IOT_CLOUD_MESSAGE_SUBSCRIBE         2

#define BYTE_PACKET_TYPE                    2
#define BYTE_PACKET_LENGTH                  3
#define BYTE_PACKET_ENDPOINT                4
#define BYTE_CATEGORY_ID                    5
#define BYTE_CMD_ID                         6
#define BYTE_TARGET                         7
#define BYTE_DATA                           8
#define BYTE_CHECKSUM                       10

#define BYTE_HEADER_OFFSET                  4       //? 54 fe type len (report pkt)

typedef enum {
    TYPE_NUMBER,
    TYPE_STRING
} logo_data_type;

typedef struct {
    bool isNewValue;
    String stringValue;
} message_element;

typedef std::map<const String, message_element> _gmessage;
typedef std::map<const String, message_element> _cloudmessage;
typedef std::map<const String, bool> _broadcast;

class GoGoBoardArduino
{
public:
    GoGoBoardArduino(void);
    ~GoGoBoardArduino(void);

    void begin(void);

    //* Input port functions
    //? get sensors value from input port 1-4
    int readInput(uint8_t port);

    //* Servo functions
    //? set servos to interact with ..
    void talkToServo(String servo_port);
    void talkToServo(int param1, int param2 = 0, int param3 = 0, int param4 = 0);
    //? set servos head to input head_angle
    void setServoHead(int head_angle);
    //? turn servos clockwise by input angle
    void turnServoThisWay(int cw_angle);
    //? turn servos counter-clockwise by input angle
    void turnServoThatWay(int ccw_angle);
    //? set servos power (output servo ports with raw pwm duty)
    void setServoPower(int power);

    //* Motor functions
    //? set output to interact with ..
    void talkToOutput(String output_port);
    void talkToOutput(int param1, int param2 = 0, int param3 = 0, int param4 = 0);
    //? set output power
    void setOutputPower(int power);
    //? turn outputs on or off
    void turnOutputON(void);
    void turnOutputOFF(void);
    void turnOutputONOFF(int state);
    //? turn outputs direction
    void turnOutputThisWay(void);
    void turnOutputThatWay(void);
    void turnOutputDirection(int dir);
    void toggleOutputWay(void);

    void beep(void);
    // void playNote();
    // void setNoteTempo();

    void connectToWifi(const String &ssid, const String &password);

    void sendGmessage(const String &key, const float value);
    void sendGmessage(const String &key, const String &value);
    bool isGmessageAvailable(const String &key);
    String Gmessage(const String &key, const String &defaultValue = String());

    void setBroadcastChannel(uint32_t channel);
    void setBroadcastPassword(const String &password);
    void sendBroadcast(const String &topic);
    bool receiveBroadcast(const String &topic);

    void sendCloudMessage(const String &topic, const float payload);
    void sendCloudMessage(const String &topic, const String &payload);
    bool isCloudMessageAvailable(const String &topic);
    String Cloudmessage(const String &topic, const String &defaultValue = String());

private:
    // static void resetCallback(void);
    static void irqCallback(void);
    static void gogoSerialEvent(void);
    static void processPacket(void);


    static uint8_t gblExtSerialState;
    static uint8_t gblExtSerialPacketType;
    static uint8_t gblExtSerialCmdChecksum;
    static uint8_t inExtLength;
    static uint8_t gblExtSerialCmdCounter;
    static bool gblUseFirstExtCmdBuffer;
    static bool gblNewExtCmdReady;
    static bool gblRequestResponseAvailable;
    static bool gblResponseArduinoInit;
    static uint8_t gbl1stExtCMDBuffer[];
    static uint8_t gbl2ndExtCMDBuffer[];
    static uint8_t *gblActiveBuffer;

    static String _key;
    static _gmessage _gmessage_list;

    static String _topic;
    static _broadcast _broadcast_list;
    static _cloudmessage _cloudmessage_list;

    uint8_t portsToBits(const String &ports);

    void sendCmdPacket(uint8_t categoryID, uint8_t cmdID, uint8_t targetVal = 0, int value = 0, bool isCmd = true);
    void sendCmdPacket(uint8_t *data, uint8_t length, bool isCmd = true);
    void sendIoTPacket(uint8_t categoryID, uint8_t cmdID, uint8_t *data, uint8_t length, bool isCmd = false);

    void sendReportPacket(uint8_t *data, uint8_t length);

    String _dataStr;

    uint8_t dataPkt[GOGO_DEFAULT_BUFFER_SIZE] = {0};
    uint8_t cmdDynamicPkt[GOGO_DEFAULT_BUFFER_SIZE] = {0x54, 0xfe, ARDUINO_CMD_PACKET_TYPE};
    uint8_t reportPkt[GOGO_DEFAULT_BUFFER_SIZE] = {0x54, 0xfe, ARDUINO_GMESSAGE_PACKET_TYPE};
};

extern GoGoBoardArduino GoGoBoard;

#endif
