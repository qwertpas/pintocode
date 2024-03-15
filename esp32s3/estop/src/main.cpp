#include <Arduino.h>

#include <HardwareSerial.h>
#include <WiFi.h>
#include <elapsedMillis.h>
#include <esp_now.h>

#define LED_ON 0
#define LED_OFF 1
#define ESTOPPED (!digitalRead(D8)) // 1 is on, 0 is estopped

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

elapsedMillis send_timer;
elapsedMillis recv_timer; //watchdog for serial recv
elapsedMillis print_timer;

hw_timer_t *blink_timer = NULL;


uint32_t blink_period = 100;
uint32_t send_period = 100; // 100Hz

esp_now_peer_info_t peerInfo;

uint8_t packet[250] = {0};
uint8_t packet_index = 0;
uint8_t packet_ready_flag = false;

uint8_t espnow_tx[250] = {0};
uint8_t espnow_tx_size = 0;

uint8_t led_solid = false;


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_OK) {
        led_solid = true;
    } else {
        timerAlarmWrite(blink_timer, 200*1000, true); //microseconds, fast blink indicates transmit fail
        led_solid = false;
    }

    // switch(status){
    //     case ESP_OK:
    //         Serial.println("OK");
    //         timerAlarmWrite(blink_timer, 1*1000, true);
    //         led_solid = true;
    //         break;
    //     case ESP_ERR_ESPNOW_NOT_INIT:
    //         Serial.println("NOT_INIT");
    //         break;
    //     case ESP_ERR_ESPNOW_ARG:
    //         Serial.println("ARG");
    //         break;
    //     case ESP_ERR_ESPNOW_INTERNAL:
    //         Serial.println("INTERNAL");
    //         break;
    //     case ESP_ERR_ESPNOW_NO_MEM:
    //         Serial.println("NO_MEM");
    //         break;
    //     case ESP_ERR_ESPNOW_NOT_FOUND:
    //         Serial.println("NOT_FOUND");
    //         break;
    //     case ESP_ERR_ESPNOW_IF:
    //         Serial.println("IF");
    //         break;
    //     default:
    //         break;
    // }
}

void IRAM_ATTR blink_timer_ISR() { // 1kHz
    if(led_solid){
        digitalWrite(LED_BUILTIN, LED_ON);
    }else{
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}


void setup() {
    Serial.begin(115200);
    Serial.setTimeout(1);
    Serial.setTxTimeoutMs(1);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LED_OFF);

    pinMode(D8, INPUT_PULLDOWN);
    pinMode(D7, OUTPUT);
    digitalWrite(D7, HIGH);

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
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    blink_timer = timerBegin(0, 80, true);                    // timer index 0, 80MHz clock, 80div=1us precision, true=count up
    timerAttachInterrupt(blink_timer, &blink_timer_ISR, true); // true=edgetriggered
    timerAlarmWrite(blink_timer, 1000000, true);                 // microseconds, true=autoreload
    timerAlarmEnable(blink_timer);                            // start counting


    Serial.println("Finished setup");
}

void loop() {

    packet_ready_flag = false;
    if(Serial.available()){
        uint8_t newchar = Serial.read();
        // Serial.print((char) newchar);
        packet[packet_index] = newchar;
        packet_index++;
        if(newchar == '\t'){
            packet_ready_flag = true;

            // memcpy(espnow_tx, packet, packet_index); //save packet into tx buffer
            // espnow_tx_size = packet_index;

            // memset(packet, 0, sizeof(packet)); //clear packet for next serial receive
            // packet_index = 0;

            
        }

    }
    // }

    if(packet_ready_flag){
        packet_ready_flag = false;

        memset(espnow_tx, 0, sizeof(espnow_tx)); //clear espnow_tx
        memcpy(espnow_tx, packet, packet_index);
        espnow_tx_size = packet_index;

        memset(packet, 0, packet_index);
        packet_index = 0;

        if(Serial.availableForWrite()){
            if(ESTOPPED) Serial.println("~~~~ESTOPPED~~~~~");
            Serial.println("espnow_tx:");
            Serial.println((char *)espnow_tx);
            Serial.println(espnow_tx_size);
        }

        recv_timer = 0;
    }else if (recv_timer > 100){ //if it's been more than 100ms since last recv, long blink 
        recv_timer = 101;
        timerAlarmWrite(blink_timer, 1000*1000, true); //microseconds
        led_solid = false;

        if(print_timer > 1000){
            print_timer = 0;
            Serial.println("not receiving station");
            Serial.print("Estopped: ");
            Serial.println(ESTOPPED);
        }
        return;
    }

    if(send_timer > 10){
        send_timer = 0;
        if(ESTOPPED){
            timerAlarmWrite(blink_timer, 100*1000, true); //microseconds
            led_solid = false;
            return;
        }
        if(espnow_tx_size > 0){
            esp_now_send(recv_addrs[1], espnow_tx, espnow_tx_size); //don't read return message here, often no_mem errror
        }
    }
}
