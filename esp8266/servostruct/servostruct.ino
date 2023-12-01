#include <Servo.h>



typedef struct servo_struct {
  Servo servo;
  uint8_t pos; //0-180
} servo_struct;
servo_struct servos[6];

uint8_t servo_pins[6] = {0, 16, 14, 12, 13, 15}; //D3, D0, D5, D6, D7, D8

void setup() {
  // put your setup code here, to run once:
  for(int i=0; i<6; i++){
    servos[i].servo.attach(servo_pins[i], 500, 2400); //don't know why pwm range is 500-2400us
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i=0; i<6; i++){
    servos[i].servo.write(0);
  }
  delay(100);
  for(int i=0; i<6; i++){
    servos[i].servo.write(180);
  }
  delay(100);
}
