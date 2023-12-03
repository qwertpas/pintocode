#include <elapsedMillis.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <SoftwareSerial.h>

//typedef struct stick_struct {
//  int16_t x;
//  int16_t y;
//  uint8_t b;
//} stick_struct;
//stick_struct sticks[4] = {0};

typedef struct cmd_struct {
  uint8_t servos[6];
  int16_t motors[2];
} cmd_struct;
cmd_struct cmd = {0};

uint8_t recv_addr[] = {0xC8, 0xC9, 0xA3, 0x56, 0x98, 0x6F}; //other side

elapsedMillis led_timer;
elapsedMillis printaddr_timer;
elapsedMillis send_timer;
elapsedMillis stick_timer;


void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}

SoftwareSerial mySerial(4, 5); //RX, TX


void setup() {
  Serial.begin(115200);

//  Wire.begin(4, 5, 0x42);  // join i2c bus (address optional for master)
  mySerial.begin(115200);

  WiFi.mode(WIFI_STA);

  if(esp_now_init() == 0){
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_register_send_cb(OnDataSent);
    esp_now_add_peer(recv_addr, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  }else{
    Serial.println("Error initializing ESP-NOW");
  }

  pinMode(LED_BUILTIN, OUTPUT);
}



void loop() {
  if (led_timer > 200) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle the LED state
    led_timer = 0;
//    Serial.println("blink!");
  }
  
//  if (printaddr_timer > 1000) {
//    Serial.println(WiFi.macAddress());
//    printaddr_timer = 0;
//  }

  if (send_timer > 10) {
    esp_now_send(recv_addr, (uint8_t *) &cmd, sizeof(cmd));
    send_timer = 0;

    for(int i = 0; i < 6; i++){
      Serial.println(cmd.servos[0]);
    }
    for(int i = 0; i < 2; i++){
      Serial.println(cmd.motors[1]);
    }
  }

  if (mySerial.available()) {
    ESP.wdtDisable();
    String receivedString = mySerial.readStringUntil('\n'); // Read a line terminated by '\n'
    char motor_type;
    int index, val;
    if (sscanf(receivedString.c_str(), "%c%d:%d", &motor_type, &index, &val) == 3) {
      if(motor_type == 's' && index < 6){
        cmd.servos[index] = val;
      }
      if(motor_type == 'm' && index < 2){
        cmd.motors[index] = val;
      }
      
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle the LED state
    }
    ESP.wdtEnable(1);
  }


  
}
