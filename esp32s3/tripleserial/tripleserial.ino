// Need this for the lower level access to set them up.
#include <HardwareSerial.h>


//Define two Serial devices mapped to the two internal UARTs
HardwareSerial MySerial0(0);
HardwareSerial MySerial1(1);
HardwareSerial MySerial2(2);

void setup()
{
    // For the USB, just use Serial as normal:
    Serial.begin(115200);

    // Configure MySerial0 on pins RX=D7, TX=D6 (-1, -1 means use the default)
    MySerial0.begin(115200, SERIAL_8N1, -1, -1);
    MySerial0.print("MySerial0");

    // And configure MySerial1 on pins RX=D9, TX=D10
    MySerial1.begin(115200, SERIAL_8N1, 8, 9);
    MySerial1.print("MySerial1");

    MySerial2.begin(115200, SERIAL_8N1, 4, 5); // RX=D3, TX=D4

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
      Serial.println("A");
      MySerial0.println("BB");
      MySerial1.println("CCC");
      MySerial2.println("DDDD");

      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);



}
