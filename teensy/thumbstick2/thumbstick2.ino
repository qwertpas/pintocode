/*
 * For Teensy 4.0
 * pins 0,1,2,3 are the digital input for the buttons in joysticks 3,2,1,0, pullups enabled
 * pins 7,8,9 are RX, TX, and DE/~RE the RS485 transceiver UART
 * pins 14-17 (A0-A3) are the analog inputs for the X and Y axes in joysticks 0,1
 * pins 18,19 are SDA0 and SCL0 for transmitting joystick data to ESP8266
 * pins 20-23 (A6-A9) are the analog inputs for the X and Y axes in joysticks 2,3
 */


#include "HX711.h"
#include "comdef.h"
#include "util.h"

#include <elapsedMillis.h>
#include <PWMServo.h>
//#include <Wire.h>

#define RS485_DE (9)
#define MIN_INT8 (0x80) //most negative int8

#define ang_per_ct (90/4096.0)


int stick_pins[4][3] = {
  {A2, A3, 3}, //stick 0 (x_pin, y_pin, button_pin)
  {A4, A5, 2},
  {A6, A7, 1},
  {A8, A9, 0},
};

typedef struct stick_struct {
  int16_t x;
  int16_t y;
  int16_t x_init;
  int16_t y_init;
  uint8_t b;
} stick_struct; //10 bytes each
stick_struct sticks[4] = {0}; //40 bytes

uint8_t uart2_RX[10] = {0}; //response Ø32 from RS485


PWMServo servo;  // create servo object to control a servo

void setup() {
  Serial.begin(115200); //UART for printing
  Serial2.begin(115200); // UART for RS485 output (RX2=pin7, TX2=pin8)
  Serial2.setTimeout(1);
  Serial3.begin(115200); //UART for joystick data

//  pinMode(18, INPUT_PULLUP);
//  pinMode(19, INPUT_PULLUP);
//  Wire.begin(8); //join i2c bus with address #8
//  Wire.onRequest(requestEvent); //register event
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);

  servo.attach(12);
  servo.write(0);

  for(int i=0; i<4; i++){
    pinMode(stick_pins[i][0], INPUT);
    pinMode(stick_pins[i][1], INPUT);
    pinMode(stick_pins[i][2], INPUT_PULLUP);

    sticks[i].x_init = -analogRead(stick_pins[i][0]);
    sticks[i].y_init = -analogRead(stick_pins[i][1]);
  }
  
  digitalWrite(LED_BUILTIN, LOW);
}

elapsedMillis stick_timer;
elapsedMillis motor_timer;

void loop() {


  if(stick_timer > 10){
    int16_t max_axis = 470;
    int16_t deadband_axis = 20;
  
    String stick_str = "";
  
    for(int i=0; i<4; i++){
      sticks[i].x = deadband(clip(-analogRead(stick_pins[i][0]) - sticks[i].x_init, -max_axis, max_axis), deadband_axis);
      sticks[i].x = map(sticks[i].x, -max_axis+deadband_axis, max_axis-deadband_axis, -511, 511);
      sticks[i].y = deadband(clip(-analogRead(stick_pins[i][1]) - sticks[i].y_init, -max_axis, max_axis), deadband_axis);
      sticks[i].y = map(sticks[i].y, -max_axis+deadband_axis, max_axis-deadband_axis, -511, 511);
      sticks[i].b = !digitalRead(stick_pins[i][2]);
  
      stick_str += String(i) + ":" + String(sticks[i].x) + "," + String(sticks[i].y) + "," + String(sticks[i].b) + "\n";
    }    
    Serial.print(stick_str);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial3.print(stick_str);
    digitalWrite(LED_BUILTIN, LOW);

    stick_timer = 0;
  }

  if(motor_timer > 2){
    motor_cmd(3, CMD_SET_VOLTAGE, twoscomplement14(sticks[0].x/4), uart2_RX);
//    motor_cmd(3, CMD_GET_POSITION, 0, uart2_RX);
    motor_timer = 0;
  }
//  if(motor_timer > 15){
//    motor_cmd(8, CMD_SET_VOLTAGE, twoscomplement14(sticks[0].y), uart2_RX);
//    motor_timer = 0;
//  }


//  motor_cmd(5, CMD_SET_VOLTAGE, twoscomplement14(stick_x1), uart2_RX);
//  motor_cmd(5, CMD_SET_CURRENT, 100, uart2_RX);
//  int16_t ang5 = pad14(uart2_RX[0], uart2_RX[1]);
//  int16_t cur5 = pad14(uart2_RX[2], uart2_RX[3]);
//  Serial.print("ang5: ");
//  Serial.println(ang5);
//  Serial.print("cur5: ");
//  Serial.println(cur5);
//  delayMicroseconds(2000);
  
//  motor_cmd(6, CMD_SET_VOLTAGE, twoscomplement14(sticks[0].x), uart2_RX);
//  motor_cmd(6, CMD_SET_CURRENT, twoscomplement14(200), uart2_RX);
//  int16_t ang6 = pad14(uart2_RX[0], uart2_RX[1]);
//  int16_t cur6 = pad14(uart2_RX[2], uart2_RX[3]);
//  Serial.print("ang6: ");
//  Serial.println(ang6);
//  Serial.print("cur6: ");
//  Serial.println(cur6);
//  Serial.print("\t\n");

  
//  delayMicroseconds(1000);

  

//  for(int i = 0; i< sizeof(uart2_RX); i++){
//    Serial.println(uart2_RX[i]);
//  }
//  Serial.print("\t\n");
}


//void requestEvent() {
//  digitalWrite(LED_BUILTIN, HIGH);  // briefly flash the LED
//  Wire.write("hello ");     // respond with message of 6 bytes
//  digitalWrite(LED_BUILTIN, LOW);
//}



uint8_t motor_cmd(uint8_t addr, uint8_t CMD_TYPE, uint16_t data, uint8_t *rx){
  while(Serial2.available()) Serial2.read(); //clear rx buffer

  uint8_t uart2_TX[3] = {0}; //command RS485 to Ø32
  uart2_TX[0] = CMD_TYPE | addr;
  uart2_TX[1] = (data >> 7) & 0b01111111;
  uart2_TX[2] = (data)      & 0b01111111;

  digitalWrite(RS485_DE, HIGH);
  Serial2.write(uart2_TX, 3);
  Serial2.flush();
  digitalWrite(RS485_DE, LOW);
  int numread = Serial2.readBytesUntil(MIN_INT8, rx, 10);
  return numread; 
}
