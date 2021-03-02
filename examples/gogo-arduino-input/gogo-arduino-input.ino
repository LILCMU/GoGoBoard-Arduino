#include <GoGoBoardArduino.h>

void setup()
{
    GoGoBoard.begin();
    Serial.begin(115200);
}

void loop()
{
    //? read input port 1 and show value on serial monitor every 1 second
    Serial.println(GoGoBoard.readInput(1));
    delay(1000);
}
