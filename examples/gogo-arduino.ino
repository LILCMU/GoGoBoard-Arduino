#include <GoGoBoard_Arduino.h>

GoGoBoard gogoIO;

void setup()
{
    gogoIO.begin();
    Serial.begin(115200);
}

void loop()
{
    //  gogoIO.sendGmessage("test", "10");
    //  gogoIO.talkToOutput("abcd");
    gogoIO.turnOutputON();
    gogoIO.toggleOutputWay();
    delay(1000);
    //

    if (gogoIO.isGmessageAvailable("hello"))
    {
        Serial.print("value: ");
        Serial.println(gogoIO.Gmessage("hello"));

        gogoIO.sendGmessage("hello", "from arduino");
    }
    //    rainbow(20);

    //  if (gogoIO.isGmessageAvailable("tutorial_neopixel"))
    //  {
    //    Serial.println("received message");
    //    Serial.println(gogoIO.Gmessage("tutorial_neopixel").toInt());
    //    for (int i = 0; i < strip.numPixels(); i++)
    //      strip.setPixelColor(i, 0);
    //    //    strip.show();
    //
    //    switch (gogoIO.Gmessage("tutorial_neopixel").toInt())
    //    {
    //      case 0:
    //        strip.setPixelColor(15, 252, 0, 185); //? pink
    //        strip.setPixelColor(16, 252, 0, 185); //? pink
    //        break;
    //
    //      case 1:
    //        strip.setPixelColor(3, 252, 118, 0); //? orange
    //        strip.setPixelColor(4, 252, 118, 0); //? orange
    //        break;
    //
    //      case 2:
    //        strip.setPixelColor(0, 255, 0, 0); //? red
    //        strip.setPixelColor(23, 255, 0, 0); //? red
    //        break;
    //
    //      case 80: //? clear pixels
    //        break;
    //    }
    //    strip.show();
    //  }

    //  if (Serial1.available())
    //  {
    //    Serial.write(Serial1.read());
    //  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
//uint32_t Wheel(byte WheelPos) {
//  if (WheelPos < 85) {
//    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
//  } else {
//    if (WheelPos < 170) {
//      WheelPos -= 85;
//      return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
//    } else {
//      WheelPos -= 170;
//      return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
//    }
//  }
//}

//void rainbow(uint8_t wait) {
//  uint16_t i, j;
//
//  for (j = 0; j < 256; j++) {
//    for (i = 0; i < strip.numPixels(); i++) {
//      strip.setPixelColor(i, Wheel((i + j) & 255));
//    }
//    strip.show();
//    delay(wait);
//  }
//}