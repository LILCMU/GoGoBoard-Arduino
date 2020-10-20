#include <GoGoBoardArduino.h>

GoGoBoardArduino gogoIO;

void setup()
{
    gogoIO.begin();
    Serial.begin(115200);
}

void loop()
{
    //? read input port 1 and show value on serial monitor every 1 second
    Serial.println(gogoIO.readInput(1));
    delay(1000);
}
