#include <elapsedMillis.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "util.h"
#include "comdef.h"

#define RS485_DE 5
#define RS485_RE 4

//my address is C8:C9:A3:56:98:6F

typedef struct stick_struct {
  int16_t x;
  int16_t y;
  uint8_t b;
} stick_struct;
stick_struct sticks[4] = {0};

typedef struct servo_struct {
  Servo servo;
  uint8_t pos; //0-180
} servo_struct;
servo_struct servos[6];

uint8_t servo_pins[6] = {0, 16, 14, 12, 13, 15}; //D3, D0, D5, D6, D7, D8

uint8_t uart_RX[10] = {0}; //response Ø32 from RS485

void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&sticks, incomingData, sizeof(sticks));

//  for(int i=0; i<4; i++){
//      Serial.print(i);
//      Serial.print(": (");
//      Serial.print(sticks[i].x); 
//      Serial.print(", ");
//      Serial.print(sticks[i].y); 
//      Serial.print(", ");
//      Serial.print(sticks[i].b); 
//      Serial.println(")");
//  }
//  Serial.println("\t\n");
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() == 0) {
    esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
    esp_now_register_recv_cb(OnDataRecv);
  }else{
//    Serial.println("Error initializing ESP-NOW");
  }

  Serial.begin(115200); // UART for RS485 output (RX2=pin7, TX2=pin8)
  Serial.setTimeout(1);

  for(int i=0; i<6; i++){
    servos[i].servo.attach(servo_pins[i], 500, 2400); //don't know why pwm range is 500-2400us
  }

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RS485_DE, OUTPUT);
  pinMode(RS485_RE, OUTPUT);

}

elapsedMillis led_timer;
elapsedMillis servo_timer;
elapsedMillis motor_timer;


void loop() {
  if (led_timer > 1000) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle the LED state
    led_timer = 0;
  }

  if(motor_timer > 2){
    motor_cmd(3, CMD_SET_VOLTAGE, twoscomplement14(sticks[0].x/4), uart_RX);
    motor_timer = 0;
  }

//  if(servo_timer > 1){
//    servos[1].pos = (uint8_t) clip( (int16_t) (servos[1].pos + sticks[1].x/20.0), 0, 180);
//    for(int i=0; i<6; i++){
//        Serial.print(i);
//        Serial.print(": ");
//        Serial.println(servos[i].pos); 
//    }
//    Serial.println("\t\n");
//  
//    for(int i=0; i<2; i++){
//      servos[i].servo.write(servos[i].pos);
//    }
//    servo_timer = 0;
//  }

}


uint8_t motor_cmd(uint8_t addr, uint8_t CMD_TYPE, uint16_t data, uint8_t *rx){
  while(Serial.available()) Serial.read(); //clear rx buffer

  uint8_t uart2_TX[3] = {0}; //command RS485 to Ø32
  uart2_TX[0] = CMD_TYPE | addr;
  uart2_TX[1] = (data >> 7) & 0b01111111;
  uart2_TX[2] = (data)      & 0b01111111;

  digitalWrite(RS485_DE, HIGH);
  digitalWrite(RS485_RE, HIGH);
  Serial.write(uart2_TX, 3);
  Serial.flush();
  digitalWrite(RS485_DE, LOW);
  digitalWrite(RS485_RE, LOW);
  int numread = Serial.readBytesUntil(MIN_INT8, rx, 10);
  return numread; 
}
