#include <ESP8266WiFi.h>
#include <elapsedMillis.h>
#include <espnow.h>


typedef struct __attribute__((packed)) {
    char serialData[64]; // Change the size according to your data needs
} MyData;


uint8_t WHICH_ADDR = 1; //<-- multiple receivers makes it crash, change this to choose which
uint8_t recv_addrs[][6] = {
    {0xC8, 0xC9, 0xA3, 0x56, 0x98, 0x6F}, // jank brain (esp8266)
    {0x30, 0x30, 0xF9, 0x34, 0x57, 0x28}, // squirrelbrain 1 (esp32s3)
    {0x30, 0x30, 0xF9, 0x34, 0x5A, 0x44}, // squirrelbrain 2 (esp32s3)
    // use FF,FF,... to broadcast all
};
uint8_t num_recv = sizeof(recv_addrs) / sizeof(recv_addrs[0]);

elapsedMillis led_timer;
elapsedMillis printaddr_timer;
elapsedMillis send_timer;
elapsedMillis stick_timer;

uint32_t blink_period = 100;

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
    if (sendStatus == 0) {
        blink_period = 0; // solid light indicates transmit success
    } else {
        blink_period = 200; // fast blink indicates transmit fail
    }
}

void setup() {
    Serial.begin(115200);
    Serial.swap(); // RX0 becomes GPIO13, labeled D7

    WiFi.mode(WIFI_STA);

    esp_now_init();
    esp_now_register_send_cb(OnDataSent);

    delay(2000);

    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);

    // for (uint8_t i = 0; i < num_recv; i++) {
    //     esp_now_add_peer(recv_addrs[i], ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
    // }
    esp_now_add_peer(recv_addrs[WHICH_ADDR], ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
    
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {

    if (blink_period == 0) {
        digitalWrite(LED_BUILTIN, LOW); // LED is active low
        led_timer = 0;
    } else if (led_timer > blink_period) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        led_timer = 0;
    }

    if (Serial.available()) {
        String serialInput = Serial.readStringUntil('\n'); // Read a line terminated by '\n'

        MyData myData;
        memset(&myData, 0, sizeof(myData));
        serialInput.toCharArray(myData.serialData, sizeof(myData.serialData));

        // esp_now_send(recv_addr, (uint8_t *)&myData, sizeof(myData));
        // for (uint8_t i = 0; i < num_recv; i++) {
        esp_now_send(0, (uint8_t *)&myData, sizeof(myData));
        // esp_now_send(recv_addrs[2], (uint8_t *)&myData, sizeof(myData));
        // }

    } else {
        blink_period = 1000; // long blink for no input data
    }
}
