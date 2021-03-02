#include <GoGoBoardArduino.h>

void setup()
{
    GoGoBoard.begin();
    Serial.begin(115200);
}

void loop()
{
    //? echo Gmessage from GoGoBoard on `greeting` key and show the message on serial monitor
    if (GoGoBoard.isGmessageAvailable("greeting"))
    {
        GoGoBoard.sendGmessage("greeting", GoGoBoard.Gmessage("greeting"));

        Serial.print("message: ");
        Serial.println(GoGoBoard.Gmessage("greeting"));
    }
}
