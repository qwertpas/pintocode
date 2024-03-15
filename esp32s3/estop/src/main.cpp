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

hw_timer_t *blink_timer = NULL;


uint32_t blink_period = 100;
uint32_t send_period = 10; // 100Hz

esp_now_peer_info_t peerInfo;

uint8_t packet[250] = {0};
uint8_t packet_index = 0;
uint8_t packet_ready_flag = false;


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == 0) {
        // blink_period = 0; // solid light indicates transmit success
        timerAlarmWrite(blink_timer, 10*1000, true);
        
    } else {
        // blink_period = 200; // fast blink indicates transmit fail
        timerAlarmWrite(blink_timer, 200*1000, true); //microseconds
    }
}

void IRAM_ATTR blink_timer_ISR() { // 1kHz
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
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

    // if (blink_period == 0) {
    //     led_timer = 0; // solid light
    //     digitalWrite(LED_BUILTIN, LED_ON);
    // } else if (led_timer > blink_period) {
    //     led_timer = 0;
    //     digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // }



    // if (send_timer > send_period) {
    if (true) {
        send_timer = 0;

        // Serial.println("send loop");

        // Serial.println(Serial.readStringUntil('@'));
        packet_ready_flag = false;
        while(send_timer < send_period){
            if(Serial.available()){
                uint8_t newchar = Serial.read();
                // Serial.print((char) newchar);
                packet[packet_index] = newchar;
                packet_index++;
                if(newchar == '#'){
                    packet_ready_flag = true;
                    // Serial.println("");
                    Serial.println("packet ready:");
                    // for(int i = 0; i < 6; i++){
                    //     Serial.print(packet[i]);
                    // }
                    String packet_str = String((char *)packet);
                    Serial.println(packet_str);
                    break;
                }
            }
        }

        if(packet_ready_flag){
            
            packet_ready_flag = false;

            esp_now_send(0, packet, packet_index);
            Serial.println("sent");

            memset(packet, 0, packet_index);
            packet_index = 0;

            

            // MyData myData;
            // memset(&myData, 0, sizeof(myData));
            // packet.toCharArray(myData.serialData, sizeof(myData.serialData));
        }else{
            timerAlarmWrite(blink_timer, 1000000, true); //microseconds
        }


        // if (Serial.available()) {

        //     Serial.println("sdsd");
        //     /**
        //      * 1. read all of serial, put into buffer
        //      * 2. find index of '\t'
        //      * 3. if found '\t', send (lastbuf + newbuf[0:index]). else read serial again
        //      * 4. lastbuf = newbuf[index:-1]
        //      */

        //     uint8_t endindex = -1;
        //     packet_ready_flag = false;
        //     while(true){ //make sure while loop doesn't keep going
        //         Serial.println("reading");
        //         String newbuf = Serial.readString();
        //         Serial.println("done read");

        //         endindex = newbuf.indexOf('\t');
        //         if(endindex == -1){
        //             packet += newbuf; //end of packet not found
        //             Serial.println("still finding");
        //         }else{
        //             packet += newbuf.substring(0, endindex);
        //             packet_ready_flag = true;
        //             Serial.println("end of packet");
        //             break;
        //         }
        //     }

        //     Serial.println("packet");                          // try see if this is the whole packet
        //     Serial.println(packet);                       // try see if this is the whole packet
        //     Serial.println(packet_ready_flag);                       // try see if this is the whole packet


        //     MyData myData;
        //     memset(&myData, 0, sizeof(myData));
        //     packet.toCharArray(myData.serialData, sizeof(myData.serialData));

        //     esp_now_send(0, (uint8_t *)&myData, sizeof(myData));

        //     // uint8_t data[] = {2,3,4,5};

        //     // esp_now_send(0, data, 3);

        // } else {
        //     blink_period = 1000; // long blink for no input data
        // }



    }
}
