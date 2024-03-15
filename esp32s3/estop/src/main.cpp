#include <Arduino.h>

// put function declarations here:

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(D7, OUTPUT);
  pinMode(D8, INPUT_PULLDOWN);

  digitalWrite(D7, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("estop: ");
  Serial.print(digitalRead(D8));
  Serial.print("\t\n");
  delay(1);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1);
  digitalWrite(LED_BUILTIN, HIGH);

}
