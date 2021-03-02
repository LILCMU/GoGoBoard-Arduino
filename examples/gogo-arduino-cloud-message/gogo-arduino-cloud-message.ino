#include <GoGoBoardArduino.h>

void setup()
{
    GoGoBoard.begin();
    GoGoBoard.connectToWifi("<wifi name>", "<wifi password>");
}

void loop()
{
    //? retrieve light sensor value from other gogoboard via topic `gogo-light-sensor`
    if (gogoIO.isCloudMessageAvailable("gogo-light-sensor"))
    {
        Serial.println(gogoIO.Cloudmessage("gogo-light-sensor"));
    }
    delay(100);
}
