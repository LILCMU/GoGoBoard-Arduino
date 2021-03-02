#include <GoGoBoardArduino.h>

void setup()
{
    GoGoBoard.begin();
}

void loop()
{
    GoGoBoard.beep();
    delay(1000);
}
