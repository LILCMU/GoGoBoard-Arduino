#include <GoGoBoardArduino.h>

GoGoBoardArduino gogoIO;

void setup()
{
    gogoIO.begin();

    //? turn on output port 1 and 2
    gogoIO.talkToOutput("12");
    gogoIO.turnOutputON();
}

void loop()
{
    //? toggle motor direction every 5 seconds
    gogoIO.toggleOutputWay();
    delay(5000);
}
