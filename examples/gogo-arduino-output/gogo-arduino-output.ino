#include <GoGoBoardArduino.h>

void setup()
{
    GoGoBoard.begin();

    //? turn on output port 1 and 2
    GoGoBoard.talkToOutput(1,2);
    GoGoBoard.turnOutputON();
}

void loop()
{
    //? toggle motor direction every 5 seconds
    GoGoBoard.toggleOutputWay();
    delay(5000);
}
