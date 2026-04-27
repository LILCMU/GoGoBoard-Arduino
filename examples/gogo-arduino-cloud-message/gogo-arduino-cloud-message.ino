#include <GoGoBoardArduino.h>

void setup()
{
    GoGoBoard.begin();
    GoGoBoard.connectToWifi("<wifi name>", "<wifi password>");
}

void loop()
{
    //? retrieve light sensor value from other gogoboard via topic `gogo-light-sensor`
    if (GoGoBoard.isCloudMessageAvailable("gogo-light-sensor"))
    {
        Serial.println(GoGoBoard.Cloudmessage("gogo-light-sensor"));
    }
    delay(100);
}
