#include <elapsedMillis.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "util.h"

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
    Serial.println("Error initializing ESP-NOW");
  }

  for(int i=0; i<6; i++){
    servos[i].servo.attach(servo_pins[i], 500, 2400); //don't know why pwm range is 500-2400us
  }

  pinMode(LED_BUILTIN, OUTPUT);
}

elapsedMillis led_timer;
elapsedMillis servo_timer;

void loop() {
  if (led_timer > 1000) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle the LED state
    led_timer = 0;
  }

  if(servo_timer > 2){
    servos[1].pos = (uint8_t) clip( (int16_t) (servos[1].pos + sticks[1].x/20.0), 0, 180);
    for(int i=0; i<6; i++){
        Serial.print(i);
        Serial.print(": ");
        Serial.println(servos[i].pos); 
    }
    Serial.println("\t\n");
  
    for(int i=0; i<6; i++){
      servos[i].servo.write(servos[i].pos);
    }
    servo_timer = 0;
  }

}
