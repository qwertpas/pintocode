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
#define UART_SIZE 10

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

typedef struct cmd_struct {
  uint8_t servos[6];
  int16_t motors[2];
} cmd_struct;
cmd_struct cmd = {0};

typedef struct motor_struct {
  int16_t I_phase;
  int16_t duty;
  int16_t duty_offsetted;
} motor_struct;
motor_struct motor_data[2] = {0};

PWMServo servo;  // create servo object to control a servo

uint8_t motor_cmd(uint8_t addr, uint8_t CMD_TYPE, uint16_t data, motor_struct *motor_response){
  while(Serial2.available()) Serial2.read(); //clear rx buffer

  uint8_t uart2_TX[3] = {0}; //command RS485 to Ø32
  uart2_TX[0] = CMD_TYPE | addr;
  uart2_TX[1] = (data >> 7) & 0b01111111;
  uart2_TX[2] = (data)      & 0b01111111;

  digitalWrite(RS485_DE, HIGH);
  Serial2.write(uart2_TX, 3);
  Serial2.flush();
  digitalWrite(RS485_DE, LOW);
  uint8_t temp_rx[UART_SIZE];
  int numread = Serial2.readBytesUntil(MIN_INT8, temp_rx, UART_SIZE);
  if(numread == 0) return 0;
  uint8_t checksum = 0;
  for(int i=0; i<numread-1; i++){
      checksum += temp_rx[i];
  }
  checksum &= 0b01111111;
  if(checksum != temp_rx[numread-1]) return 0;

  if(pad14(temp_rx[2], temp_rx[3]) > 1200) return 0;
  
//  memcpy(rx, temp_rx, numread);
  motor_response->I_phase = pad14(temp_rx[0], temp_rx[1]);
  motor_response->duty = pad14(temp_rx[2], temp_rx[3]);
  motor_response->duty_offsetted = pad14(temp_rx[4], temp_rx[5]);
  return 1; //success
}

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
elapsedMillis print_timer;

int motor_period = 5;


void loop() {


  if(stick_timer > 10){
    int16_t max_axis = 470;
    int16_t deadband_axis = 20;
    
    for(int i=0; i<4; i++){
      sticks[i].x = deadband(clip(-analogRead(stick_pins[i][0]) - sticks[i].x_init, -max_axis, max_axis), deadband_axis);
      sticks[i].x = map(sticks[i].x, -max_axis+deadband_axis, max_axis-deadband_axis, -511, 511);
      sticks[i].y = deadband(clip(-analogRead(stick_pins[i][1]) - sticks[i].y_init, -max_axis, max_axis), deadband_axis);
      sticks[i].y = map(sticks[i].y, -max_axis+deadband_axis, max_axis-deadband_axis, -511, 511);
      sticks[i].b = !digitalRead(stick_pins[i][2]);
//      stick_str += String(i) + ":" + String(sticks[i].x) + "," + String(sticks[i].y) + "," + String(sticks[i].b) + "\n";
    }  

    cmd.motors[0] = sticks[0].x / 4;
    cmd.motors[1] = sticks[0].y / 4;
    
    cmd.servos[0] = sticks[1].x / 2;
    cmd.servos[1] = sticks[1].y / 2;

    String cmd_str = "";
    for(int i=0; i<6; i++){
      cmd_str += "s" + String(i) + ":" + cmd.servos[i] + "\n";
    }
    for(int i=0; i<2; i++){
      cmd_str += "m" + String(i) + ":" + cmd.motors[i] + "\n";
    }

    digitalWrite(LED_BUILTIN, HIGH);
    Serial3.print(cmd_str);
    Serial.print(cmd_str);
    Serial.print("\t\n");
    digitalWrite(LED_BUILTIN, LOW);

    stick_timer = 0;
  }

  if(motor_timer > motor_period && motor_timer < 100){
    motor_cmd(7, CMD_SET_VOLTAGE, twoscomplement14(sticks[0].x), &motor_data[0]);
    motor_timer = 100;
    
  }else if(motor_timer > 100+motor_period){
    motor_cmd(8, CMD_SET_VOLTAGE, twoscomplement14(sticks[0].y), &motor_data[1]);
    motor_timer = 0;
  }

//  if(print_timer > 10){
//    for(int i=0; i < 1; i++){
//      Serial.print("motor");
//      Serial.print(i);
//      Serial.print("Iphase: ");
//      Serial.println(motor_data[i].I_phase);
//      Serial.print("motor");
//      Serial.print(i);
//      Serial.print("duty: ");
//      Serial.println(motor_data[i].duty);
//      Serial.print("motor");
//      Serial.print(i);
//      Serial.print("duty_offset: ");
//      Serial.println(motor_data[i].duty_offsetted);
//    }
//    Serial.println("\t\n");
//
//    print_timer = 0;
//  }

}
