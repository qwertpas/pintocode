#include <Arduino.h>

#include <HardwareSerial.h>
#include <WiFi.h>
#include <elapsedMillis.h>
#include <esp_now.h>

#define LED_ON 0
#define LED_OFF 1
#define estop digitalRead(D8) // 1 is on, 0 is estopped

typedef struct __attribute__((packed)) {
    char serialData[64]; // Change the size according to your data needs
} MyData;

uint8_t WHICH_ADDR = 1; //<-- multiple receivers makes it unclear whether transmit succeeds, pick one address
uint8_t recv_addrs[][6] = {
    {0xC8, 0xC9, 0xA3, 0x56, 0x98, 0x6F}, // jank brain (esp8266)
    {0x30, 0x30, 0xF9, 0x34, 0x57, 0x28}, // squirrelbrain 1 (esp32s3)
    {0x30, 0x30, 0xF9, 0x34, 0x5A, 0x44}, // squirrelbrain 2 (esp32s3)
    // use FF,FF,... to broadcast all
};
uint8_t num_recv = sizeof(recv_addrs) / sizeof(recv_addrs[0]);

elapsedMillis led_timer;
elapsedMillis send_timer;

uint32_t blink_period = 100;
uint32_t send_period = 10; //100Hz

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == 0) {
        blink_period = 0; // solid light indicates transmit success
    } else {
        blink_period = 200; // fast blink indicates transmit fail
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LED_OFF);

    WiFi.mode(WIFI_STA);
    delay(1000);
    digitalWrite(LED_BUILTIN, LED_ON);


    Serial.println("Starting Estop");
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_send_cb(OnDataSent);

    memcpy(peerInfo.peer_addr, recv_addrs[1], 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    } 

    Serial.println("Finished setup");
}

void loop() {

    if (blink_period == 0) {
        led_timer = 0; //solid light
        digitalWrite(LED_BUILTIN, LED_ON);
    } else if (led_timer > blink_period) {
        led_timer = 0;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }



    if(send_timer > send_period){
        send_timer = 0;
        
        if (Serial.available()) {
            String serialInput = Serial.readStringUntil('\t'); // Read a line terminated by '\t'

            MyData myData;
            memset(&myData, 0, sizeof(myData));
            serialInput.toCharArray(myData.serialData, sizeof(myData.serialData));

            esp_now_send(0, (uint8_t *)&myData, sizeof(myData));


            // uint8_t data[] = {2,3,4,5};

            // esp_now_send(0, data, 3);

        } else {
            blink_period = 1000; // long blink for no input data
        }
    }
}
