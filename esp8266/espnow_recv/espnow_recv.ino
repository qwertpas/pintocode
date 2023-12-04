#include <elapsedMillis.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "util.h"
#include "comdef.h"


#define RS485_DE 5
#define RS485_RE 4

#define HW_TIMER_INTERVAL_US      20L
#define USING_MICROS_RESOLUTION       true    //false
#define USING_TIM_DIV1                true              // for shortest and most accurate timer
#include "ESP8266_PWM.h"


//my address is C8:C9:A3:56:98:6F

typedef struct cmd_struct {
  uint8_t servos[6];
  int16_t motors[2];
} cmd_struct;
cmd_struct cmd = {0};

uint8_t servo_pos[6] = {90, 90, 90, 90, 90, 90};
uint8_t servo_pins[6] = {0, 16, 14, 12, 13, 15}; //D3, D0, D5, D6, D7, D8
uint8_t servo_channels[6];

ESP8266Timer ITimer;
ESP8266_PWM ISR_PWM;

void IRAM_ATTR TimerHandler(){
  ISR_PWM.run();
}
 
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); //RS485
  Serial1.setTimeout(1);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() == 0) {
    esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
    esp_now_register_recv_cb(OnDataRecv);
  }

  for(int i=0; i<6; i++){
    pinMode(servo_pins[i], OUTPUT);
//    servos[i].servo.attach(servo_pins[i], 500, 2400); //don't know why pwm range is 500-2400us
      servo_channels[i] = ISR_PWM.setPWM(servo_pins[i], 50, 0); 
  }
  analogWriteRange(1023);
  analogWriteFreq(100);

//  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RS485_DE, OUTPUT);
  pinMode(RS485_RE, OUTPUT);

  pinMode(3, OUTPUT); //for debug since Serial TX0 is being used for RS485 


  ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_US, TimerHandler);


}

elapsedMillis print_timer;
elapsedMillis motor_timer;
elapsedMillis servo_timer;

int motor_period = 5;


float count = 0;

void loop() {

  

  if(print_timer > 100){
//    for(int i=0; i<6; i++) Serial.println(cmd.servos[i]);
//    for(int i=0; i<2; i++) Serial.println(cmd.motors[i]);
    Serial.print("\t\n");

    count++;
    if(count > 100){
      count = 0;
    }
    print_timer = 0;
  }

  if(motor_timer > motor_period && motor_timer < 100){
    send_motor_cmd(7, CMD_SET_VOLTAGE, twoscomplement14(cmd.motors[0]));
    motor_timer = 100;
    
  }else if(motor_timer > 100+motor_period){
    send_motor_cmd(8, CMD_SET_VOLTAGE, twoscomplement14(cmd.motors[1]));
    motor_timer = 0;
  }

  if(servo_timer > 5){
    for(int i=0; i<6; i++){
      digitalWrite(3, HIGH);
      ISR_PWM.modifyPWMChannel(servo_channels[i], servo_pins[i], 50, count);
      digitalWrite(3, LOW);
//      delayMicroseconds(1000);
    }
    servo_timer = 0;
  }


}


uint8_t send_motor_cmd(uint8_t addr, uint8_t CMD_TYPE, uint16_t data){
//  while(Serial1.available()) Serial1.read(); //clear rx buffer

  uint8_t uart2_TX[3] = {0}; //command RS485 to Ø32
  uart2_TX[0] = CMD_TYPE | addr;
  uart2_TX[1] = (data >> 7) & 0b01111111;
  uart2_TX[2] = (data)      & 0b01111111;

  digitalWrite(RS485_DE, HIGH);
  digitalWrite(RS485_RE, HIGH);
  Serial1.write(uart2_TX, 3);
  Serial1.flush();
  digitalWrite(RS485_DE, LOW);
  digitalWrite(RS485_RE, LOW);
//  int numread = Serial1.readBytesUntil(MIN_INT8, rx, 10);
//  return numread; 
  return 1;
}


void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  String receivedString = String((char *)incomingData);

  char motor_type;
  int index, val;
  if (sscanf(receivedString.c_str(), "%c%d:%d", &motor_type, &index, &val) == 3) {
    if(motor_type == 's' && index < 6){
      cmd.servos[index] = val;
    }
    if(motor_type == 'm' && index < 2){
      cmd.motors[index] = val;
    }
  }

}
