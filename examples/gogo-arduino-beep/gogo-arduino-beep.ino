#include <GoGoBoardArduino.h>

GoGoBoardArduino gogoIO;

void setup()
{
    gogoIO.begin();
}

void loop()
{
    gogoIO.beep();
    delay(1000);
}
