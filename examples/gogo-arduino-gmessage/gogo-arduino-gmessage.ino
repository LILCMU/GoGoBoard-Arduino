#include <GoGoBoardArduino.h>

GoGoBoardArduino gogoIO;

void setup()
{
    gogoIO.begin();
    Serial.begin(115200);
}

void loop()
{
    //? echo Gmessage from GoGoBoard on `greeting` key and show the message on serial monitor
    if (gogoIO.isGmessageAvailable("greeting"))
    {
        gogoIO.sendGmessage("greeting", gogoIO.Gmessage("greeting"));

        Serial.print("message: ");
        Serial.println(gogoIO.Gmessage("greeting"));
    }
}
