#include <GoGoBoardArduino.h>

void setup()
{
    GoGoBoard.begin();
    GoGoBoard.connectToWifi("<wifi name>", "<wifi password>");
}

void loop()
{
    //? checking other gogoboard to broadcast `gogo-beep`, when received sound the beeper
    if (GoGoBoard.receiveBroadcast("gogo-beep"))
    {
        GoGoBoard.beep();
    }
    delay(100);
}
