#include "HX711.h"
#include "comdef.h"
#include "util.h"
#include <elapsedMillis.h>
#include <String.h>

#define RS485_DE (9)
#define MIN_INT8 (0x80) //most negative int8
#define STICK_X0 A0
#define STICK_Y0 A1
#define STICK_X1 A5
#define STICK_Y1 A6
#define N_STICKS 1

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

int16_t max_axis = 470;
int16_t deadband_axis = 20;

uint8_t uart2_RX[10] = {0}; //response Ø32 from RS485


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
  delay(1);
//  int numread = Serial2.readBytesUntil(MIN_INT8, rx, 10);
  int numread = Serial2.readBytes(rx, 10);
  return numread; 
}


void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
 
  Serial.begin(115200); //UART for printing
  Serial2.begin(115200); // UART for RS485 output (RX2=pin7, TX2=pin8)
  Serial2.setTimeout(1);

  for(int i=0; i<N_STICKS; i++){
    pinMode(stick_pins[i][0], INPUT);
    pinMode(stick_pins[i][1], INPUT);
    pinMode(stick_pins[i][2], INPUT_PULLUP);

    sticks[i].x_init = -analogRead(stick_pins[i][0]);
    sticks[i].y_init = -analogRead(stick_pins[i][1]);
  }

  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);
  
  digitalWrite(LED_BUILTIN, HIGH);
}

elapsedMillis stick_timer;
elapsedMillis print_timer;
int stick_period = 15;
int print_period = 500;

int max_duty = 799;

void loop() {

  
  if(stick_timer > stick_period){
 
    for(int i=0; i<N_STICKS; i++){
      sticks[i].x = deadband(clip(-analogRead(stick_pins[i][0]) - sticks[i].x_init, -max_axis, max_axis), deadband_axis);
      sticks[i].x = map(sticks[i].x, -max_axis+deadband_axis, max_axis-deadband_axis, -max_duty, max_duty);
      sticks[i].y = deadband(clip(-analogRead(stick_pins[i][1]) - sticks[i].y_init, -max_axis, max_axis), deadband_axis);
      sticks[i].y = map(sticks[i].y, -max_axis+deadband_axis, max_axis-deadband_axis, -max_duty, max_duty);
      sticks[i].b = !digitalRead(stick_pins[i][2]);
    }  

  
    int respond_num = motor_cmd(7, CMD_SET_VOLTAGE, twoscomplement14(sticks[0].x), uart2_RX);
//    delay(1);
//    motor_cmd(6, CMD_SET_VOLTAGE, twoscomplement14(sticks[0].y), uart2_RX);
//    int16_t padded_rx[4];
//    padded_rx[0] = pad14(uart2_RX[0], uart2_RX[1]);
//    padded_rx[1] = pad14(uart2_RX[2], uart2_RX[3]);
//    Serial.print("rx0: ");
//    Serial.println(padded_rx[0]);
//    Serial.print("rs1: ");
//    Serial.println(padded_rx[1]);

    int I_q = pad14(uart2_RX[0], uart2_RX[1]);
    int I_d = pad14(uart2_RX[2], uart2_RX[3]);
    int temp_raw = pad14(uart2_RX[4], uart2_RX[5]);
    float temp = (1.43 - (3.3 / 4096.0 * temp_raw)) / 0.0043 + 25.0;
//    int D_u = uart2_RX[6];
//    int D_v = uart2_RX[7];
//    int D_w = uart2_RX[8];

//    Serial.println("I_q: " + String(I_q));
//    Serial.println("I_d: " + String(I_d));
//    Serial.println("temp: " + String(temp_raw));
//    Serial.println("D_u: " + String(D_u));
//    Serial.println("D_v: " + String(D_v));
//    Serial.println("D_w: " + String(D_w));
//    Serial.println("respond: " + String(respond_num));

//    Serial.println("V_q: " + String(V_q));
    for(int i = 0; i< sizeof(uart2_RX); i++){
      Serial.println(uart2_RX[i]);
    }
    Serial.print("\t\n");
    
    stick_timer = 0;
  }




  
}
