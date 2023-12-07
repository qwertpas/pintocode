/*
 * For Teensy 4.0
 * pins 0,1,2,3 are the digital input for the buttons in joysticks 3,2,1,0, pullups enabled
 * pins 7,8,9 are RX, TX, and DE/~RE the RS485 transceiver UART
 * pins 14-17 (A0-A3) are the analog inputs for the X and Y axes in joysticks 0,1
 * pins 18,19 are SDA0 and SCL0 for transmitting joystick data to ESP8266
 * pins 20-23 (A6-A9) are the analog inputs for the X and Y axes in joysticks 2,3
 */


#include "comdef.h"
#include "util.h"

#include <elapsedMillis.h>

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
  float servos[6];
  int16_t motors[2];
} cmd_struct;
cmd_struct cmd = {0};

int16_t max_axis = 470;
int16_t deadband_axis = 20;
float servo_gain = 0.005;

elapsedMillis stick_timer;
int stick_period = 10;

#define ELBOWJOINT 'e'
#define SHOULDERJOINT 's'
#define HIPJOINT 'h'
#define RIGHTSIDE 'r'
#define LEFTSIDE 'l'
void move_servo(char side, char joint, float pos_change){
  //convention is positive position change is forward or out
  if(side == RIGHTSIDE){
    if(joint == ELBOWJOINT){
      cmd.servos[1] = clipf(cmd.servos[1] - pos_change, 0, 126);
    }else if(joint == SHOULDERJOINT){
      cmd.servos[4] = clipf(cmd.servos[4] + pos_change, 40, 180);
    }else if(joint == HIPJOINT){
      cmd.servos[0] = clip(cmd.servos[0] + pos_change, 88, 152);
    }
  }else if(side == LEFTSIDE){
    if(joint == ELBOWJOINT){
      cmd.servos[2] = clip(cmd.servos[2] + pos_change, 40, 180);
    }else if(joint == SHOULDERJOINT){
      cmd.servos[3] = clip(cmd.servos[3] + pos_change, 0, 140);
    }else if(joint == HIPJOINT){
      cmd.servos[5] = clip(cmd.servos[5] - pos_change, 23, 95);
    }
  }
}


void setup() {
  Serial.begin(115200); //UART for printing
  Serial3.begin(115200); //UART send joystick data to ESP8266
  
  for(int i=0; i<4; i++){
    pinMode(stick_pins[i][0], INPUT);
    pinMode(stick_pins[i][1], INPUT);
    pinMode(stick_pins[i][2], INPUT_PULLUP);

    sticks[i].x_init = -analogRead(stick_pins[i][0]);
    sticks[i].y_init = -analogRead(stick_pins[i][1]);
  }

  for(int i=0; i<6; i++) cmd.servos[i] = 90;
  for(int i=0; i<2; i++) cmd.motors[i] = 0;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}


void loop() {

  if(stick_timer > stick_period){

    for(int i=0; i<4; i++){
      sticks[i].x = deadband(clip(-analogRead(stick_pins[i][0]) - sticks[i].x_init, -max_axis, max_axis), deadband_axis);
      sticks[i].x = map(sticks[i].x, -max_axis+deadband_axis, max_axis-deadband_axis, -511, 511);
      sticks[i].y = deadband(clip(-analogRead(stick_pins[i][1]) - sticks[i].y_init, -max_axis, max_axis), deadband_axis);
      sticks[i].y = map(sticks[i].y, -max_axis+deadband_axis, max_axis-deadband_axis, -511, 511);
      sticks[i].b = !digitalRead(stick_pins[i][2]);
    }  


//    cmd.servos[0] = clip( (int16_t)(cmd.servos[0] + sticks[0].x * servo_gain), 0, 180); //right hip, swing forward (links up) 
//    cmd.servos[1] = clip( (int16_t)(cmd.servos[1] + sticks[0].y * servo_gain), 0, 180); //right elbow, swing back
//    cmd.servos[2] = clip( (int16_t)(cmd.servos[2] + sticks[1].x * servo_gain), 0, 180); //left elbow, swing forward
//    cmd.servos[3] = clip( (int16_t)(cmd.servos[3] + sticks[1].y * servo_gain), 0, 180); //left shoulder, swing out
//    cmd.servos[4] = clip( (int16_t)(cmd.servos[4] + sticks[2].x * servo_gain), 0, 180); //right shoulder, swing in
//    cmd.servos[5] = clip( (int16_t)(cmd.servos[5] + sticks[2].y * servo_gain), 0, 180); //left hip, swing back (links up)

    move_servo(LEFTSIDE, ELBOWJOINT, sticks[0].x * servo_gain);
    move_servo(RIGHTSIDE, ELBOWJOINT, sticks[1].x * servo_gain);
    move_servo(LEFTSIDE, SHOULDERJOINT, sticks[0].y * servo_gain);
    move_servo(RIGHTSIDE, SHOULDERJOINT, sticks[1].y * servo_gain);
    move_servo(LEFTSIDE, HIPJOINT, sticks[2].x * servo_gain);
    move_servo(RIGHTSIDE, HIPJOINT, sticks[3].x * servo_gain);
    

    cmd.motors[0] = sticks[3].y; // right string
    cmd.motors[1] = sticks[2].y; //left string

    String cmd_str = "";
    for(int i=0; i<6; i++) cmd_str += "s" + String(i) + ":" + (uint8_t)(cmd.servos[i]) + "\n";
    for(int i=0; i<2; i++)cmd_str += "m" + String(i) + ":" + cmd.motors[i] + "\n";
    Serial3.print(cmd_str);
    Serial.print(cmd_str);
    Serial.print("\t\n");

    stick_timer = 0;
  }

}
