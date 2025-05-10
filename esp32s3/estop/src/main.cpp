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

uint8_t WHICH_ADDR = 6; //<-- multiple receivers makes it unclear whether transmit succeeds, pick one address
uint8_t send_to_addrs[][7] = {
    {0xC8, 0xC9, 0xA3, 0x56, 0x98, 0x6F}, // jank brain (esp8266)
    {0x30, 0x30, 0xF9, 0x34, 0x57, 0x28}, // squirrelbrain_v1 1 (esp32s3) (Recom 2A)
    {0x30, 0x30, 0xF9, 0x34, 0x5A, 0x44}, // squirrelbrain_v1 2 (esp32s3)
    {0x30, 0x30, 0xF9, 0x33, 0xDE, 0x4C}, // squirrelbrain_v2 3 (esp32s3) (Recom 6A)
    {0x74, 0x4D, 0xBD, 0x81, 0x09, 0x50}, // new
    {0x24, 0x58, 0x7C, 0xE4, 0x04, 0x00}, // rock1
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}  // use FF,FF,... to broadcast all
};

elapsedMillis send_timer;
elapsedMillis recv_timer; // watchdog for serial recv
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

char espnow_rx[250];

char serial_tx[1024] = {0};

uint8_t led_solid = false;

uint8_t station_ok = false;
uint8_t espnow_ok = false;

String receivedString = "";

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    espnow_ok = (status == ESP_OK);

    if (!espnow_ok) {
        // Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
        switch (status) {
        case ESP_ERR_ESPNOW_NOT_INIT:
            Serial.println("NOT_INIT");
            break;
        case ESP_ERR_ESPNOW_ARG:
            Serial.println("ARG");
            break;
        case ESP_ERR_ESPNOW_INTERNAL:
            Serial.println("INTERNAL");
            break;
        case ESP_ERR_ESPNOW_NO_MEM:
            Serial.println("NO_MEM");
            break;
        case ESP_ERR_ESPNOW_NOT_FOUND:
            Serial.println("NOT_FOUND");
            break;
        case ESP_ERR_ESPNOW_IF:
            Serial.println("IF");
            break;
        default:
            // Serial.println("some other error");
            // Serial.println((int)status);
            break;
        }
    }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    // espnow_rx = String((char *)incomingData).c_str();
    memcpy(espnow_rx, incomingData, len);
    // espnow_rx =

    // memcpy(espnow_rx, incomingData, sizeof(espnow_rx));
    // espnow_rx[0] = (uint8_t) (timerRead(blink_timer));
    // espnow_rx_size = len;
}

void IRAM_ATTR blink_timer_ISR() { // 1kHz
    if (led_solid) {
        digitalWrite(LED_BUILTIN, LED_ON);
    } else {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(0);
    Serial.setTxTimeoutMs(0);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LED_OFF);

    pinMode(D8, INPUT_PULLDOWN);
    pinMode(D7, OUTPUT);
    digitalWrite(D7, HIGH);

    WiFi.mode(WIFI_STA);
    delay(1000);
    Serial.println("My MAC address:");
    Serial.println(WiFi.macAddress());

    digitalWrite(LED_BUILTIN, LED_ON);

    Serial.println("Starting Estop/passthrough");
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    memcpy(peerInfo.peer_addr, send_to_addrs[WHICH_ADDR], 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    // delay(1000);

    blink_timer = timerBegin(0, 80, true);                     // timer index 0, 80MHz clock, 80div=1us precision, true=count up
    timerAttachInterrupt(blink_timer, &blink_timer_ISR, true); // true=edgetriggered
    timerAlarmWrite(blink_timer, 50 * 1000, true);             // microseconds, true=autoreload
    timerAlarmEnable(blink_timer);                             // start counting

    Serial.println("Finished setup");

    while(Serial.available()) Serial.read();
}

void loop() {

    if (Serial.availableForWrite() && print_timer > 10) {
        print_timer = 0;
        memset(serial_tx, 0, sizeof(serial_tx));
        size_t ser_len = sprintf(
            serial_tx,

            "station_ok:%d\n"
            "espnow_ok:%d\n"
            "estopped:%d\n"
            "%s\n"
            // "%d\n"
            // "%d\n"
            "\t\n",

            station_ok,
            espnow_ok,
            ESTOPPED,
            espnow_rx
            // espnow_rx[1],
            // espnow_rx[2]
        );
        Serial.write(serial_tx, ser_len);
    }

    packet_ready_flag = false;
    if (Serial.available()) {
        uint8_t newchar = Serial.read();
        // Serial.print((char) newchar);
        packet[packet_index] = newchar;
        packet_index++;
        if (newchar == '\t') {
            packet_ready_flag = true;

            // memcpy(espnow_tx, packet, packet_index); //save packet into tx buffer
            // espnow_tx_size = packet_index;

            // memset(packet, 0, sizeof(packet)); //clear packet for next serial receive
            // packet_index = 0;
        }
    }

    if (packet_ready_flag) {
        packet_ready_flag = false;

        memset(espnow_tx, 0, sizeof(espnow_tx)); // clear espnow_tx
        memcpy(espnow_tx, packet, packet_index);
        espnow_tx_size = packet_index;

        memset(packet, 0, packet_index);
        packet_index = 0;

        // if (Serial.availableForWrite() && print_timer > 100) {
        //     print_timer = 0;
        //     if (ESTOPPED)
        //         Serial.println("~~~~ESTOPPED~~~~~");
        //     Serial.println("espnow_tx:");
        //     Serial.println((char *)espnow_tx);
        //     Serial.println(espnow_tx_size);
        // }

        recv_timer = 0;
        station_ok = true;
    } else if (recv_timer > 100) { // if it's been more than 100ms since last recv, long blink
        recv_timer = 101;
        station_ok = false;
    }

    if (send_timer > 10) {
        send_timer = 0;

        if(espnow_tx_size > 0 && espnow_tx_size < 250 && station_ok && !ESTOPPED){
            esp_now_send(send_to_addrs[WHICH_ADDR], espnow_tx, espnow_tx_size); // don't read return message here, often no_mem errror
            espnow_tx_size = 0;
        }else{
            uint8_t buf[10] = {1,2,3,4,5,6,7,8,9,10};            
            esp_now_send(send_to_addrs[WHICH_ADDR], buf, sizeof(buf)); // don't read return message here, often no_mem errror
        }
        
    }

    // if (send_timer > 10 && station_ok && !ESTOPPED) {
    //     send_timer = 0;
    //     uint8_t buf[10] = {1,2,3,4,5,6,7,8,9,10};            
    //     esp_now_send(send_to_addrs[send_to_addrs], buf, sizeof(buf)); // don't read return message here, often no_mem errror
    //     // espnow_tx_size = 0;
    //     // }
    // }

    if (!espnow_ok) {
        timerAlarmWrite(blink_timer, 100 * 1000, true); // extra fast blink indicates ESP-NOW error
        led_solid = false;
    }else if(!station_ok) {
        timerAlarmWrite(blink_timer, 200 * 1000, true); // fast blink indicates station disconnected
        led_solid = false;
    } else if(ESTOPPED) {
        timerAlarmWrite(blink_timer, 500 * 1000, true); // slow blink is Estop pressed
        led_solid = false;
    } else { // normal operation
        led_solid = true;
    }
}

// void setup() {}
// void loop() {}