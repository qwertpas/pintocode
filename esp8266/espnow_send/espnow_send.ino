#include <elapsedMillis.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

typedef struct stick_struct {
  int16_t x;
  int16_t y;
  uint8_t b;
} stick_struct;
stick_struct sticks[4] = {0};

uint8_t recv_addr[] = {0xC8, 0xC9, 0xA3, 0x56, 0x98, 0x6F};

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


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

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
    esp_now_send(recv_addr, (uint8_t *) &sticks, sizeof(sticks));
    send_timer = 0;
  }

  if (Serial1.available()) {
    String receivedString = Serial1.readStringUntil('\n'); // Read a line terminated by '\n'
    int index, x, y, b;
    if (sscanf(receivedString.c_str(), "%d:%d,%d,%d", &index, &x, &y, &b) == 4) {
      if (index >= 0 && index < 4) { // Ensure the index is within range
        sticks[index].x = x;
        sticks[index].y = y;
        sticks[index].b = b;
        
        
//        for(int i=0; i<4; i++){
//            Serial.print(i);
//            Serial.print(": (");
//            Serial.print(sticks[i].x); 
//            Serial.print(", ");
//            Serial.print(sticks[i].y); 
//            Serial.print(", ");
//            Serial.print(sticks[i].b); 
//            Serial.println(")");
//        }
//        Serial.println("\t\n");
      }
    }
  }
}
