#include <elapsedMillis.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

//my address is C8:C9:A3:56:98:6F

typedef struct struct_message {
  int b;
  float c;
} struct_message;
struct_message data;

Servo servo16;

elapsedMillis led_timer;
elapsedMillis printaddr_timer;
elapsedMillis servo_timer;

void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&data, incomingData, sizeof(data));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("b: ");
  Serial.println(data.b);
  Serial.print("c: ");
  Serial.println(data.c);
  
  Serial.println();
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

  servo16.attach(16);

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (led_timer > 2000) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle the LED state
    led_timer = 0;
    Serial.println("blink!");
  }
  
  if (printaddr_timer > 1000) {
    Serial.println(WiFi.macAddress());
    printaddr_timer = 0;
  }

  if (servo_timer > 1800) {
    Serial.println("servo move");
    servo16.write(servo_timer/10);
    servo_timer = 0;
  }
}
