#include "main.h"

#define MOTOR_ON 1
#define GPIO_D1 2 // breakout GPIO
#define UART1_DE 3
#define UART1_TX 4
#define UART1_RX 5
#define UART0_DE 6
#define UART0_TX 43
#define UART0_RX 44
#define UART2_DE 7
#define UART2_TX 8
#define UART2_RX 9
#define LED_LIT LOW
#define LED_UNLIT HIGH

HardwareSerial rs485_0(0);    // RS485 bus 0
HardwareSerial rs485_1(1);    // RS485 bus 1
HardwareSerial dxl_serial(2); // dynamixel TTL bus

#define DXL_BAUD 1000000 
#define DXL_TX_BUFFER_LENGTH 1024
unsigned char tx_buffer[DXL_TX_BUFFER_LENGTH];

hw_timer_t *timer0 = NULL;
hw_timer_t *timer1 = NULL;

elapsedMillis print_timer;

typedef struct cmd_struct {
    uint8_t servos[6];
    int16_t motors[2];
} cmd_struct;
cmd_struct cmd = {0};

// My MAC address: 30:30:F9:34:57:28

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    //   memcpy(&myData, incomingData, sizeof(myData));
    String receivedString = String((char *)incomingData);
    char motor_type;
    int index, val;
    if (sscanf(receivedString.c_str(), "%c%d:%d", &motor_type, &index, &val) == 3) {
        if (motor_type == 's' && index < 6) {
            cmd.servos[index] = val;
        }
        if (motor_type == 'm' && index < 2) {
            cmd.motors[index] = val;
        }
    }
}

void IRAM_ATTR timer0_ISR() { // update servo position at 100Hz?
    // dxl_write(116, 1);
    Serial.println("timer0");
}

void IRAM_ATTR timer1_ISR() { // update Ø32controllers
    // send_O32_cmd(rs485_0, 0xA, CMD_SET_VOLTAGE, 50);
    Serial.println("timer1");
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LED_UNLIT);

    pinMode(MOTOR_ON, OUTPUT);
    digitalWrite(MOTOR_ON, LOW);

    // USB printout
    Serial.begin(115200);
    Serial.setTimeout(1);

    rs485_0.begin(1000000, SERIAL_8N1);                 // baudrate, parity
    rs485_0.setPins(UART0_RX, UART0_TX, GPIO_D1, UART0_DE); // RX, TX, CTS, DE
    rs485_0.setMode(UART_MODE_RS485_HALF_DUPLEX);
    rs485_0.setTimeout(1);

    rs485_1.begin(115200, SERIAL_8N1);
    rs485_1.setPins(UART1_RX, UART1_TX, -1, UART1_DE);
    rs485_1.setMode(UART_MODE_RS485_HALF_DUPLEX);
    rs485_1.setTimeout(1);

    dxl_serial.begin(DXL_BAUD, SERIAL_8N1, UART2_RX, UART2_TX);
    dxl_serial.setPins(UART2_RX, UART2_TX, GPIO_D1, UART2_DE); // CTS pin should be an unused GPIO, otherwise USB serial disappears
    dxl_serial.setMode(UART_MODE_RS485_HALF_DUPLEX);
    dxl_serial.setTimeout(1);

    WiFi.mode(WIFI_MODE_STA);
    Serial.println(WiFi.macAddress());
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        delay(3000);
        ESP.restart();
    }
    esp_now_register_recv_cb(OnDataRecv);

    

    delay(2000);

    digitalWrite(LED_BUILTIN, LED_LIT);
    digitalWrite(MOTOR_ON, HIGH);

    delay(500);

    dxl_write(2, 64, 1); // torque on

    // timer0 = timerBegin(0, 80, true);                // 80MHz clock, 80div=1us precision, true=count up
    // timerAttachInterrupt(timer0, &timer0_ISR, true); // true=edgetriggered
    // timerAlarmWrite(timer0, 1000000, true);          // microseconds, true=autoreload
    // timerAlarmEnable(timer0);                        // start counting

    // timer1 = timerBegin(1, 80, true);                // 80MHz clock, 80div=1us precision, true=count up
    // timerAttachInterrupt(timer1, &timer1_ISR, true); // true=edgetriggered
    // timerAlarmWrite(timer1, 1000000, true);          // microseconds, true=autoreload
    // timerAlarmEnable(timer1);                        // start counting
}

int count = 0;
uint8_t motor2_response[12] = {0};
uint8_t motor3_response[12] = {0};
uint8_t recv_bytes = 0;

void loop() {

    // if (print_timer > 100) {
    //     // for (int i = 0; i < 6; i++)
    //     //     Serial.println(cmd.servos[i]);
    //     // for (int i = 0; i < 2; i++)
    //     //     Serial.println(cmd.motors[i]);
    //     // Serial.print("\t\n");


    //     for (int i = 0; i < recv_bytes; i++) {
    //         Serial.print(i);
    //         Serial.print(": ");
    //         Serial.println(rs485_0_rx[i]);
    //     }

    //     print_timer = 0;
    // }
    recv_bytes = send_O32_cmd(0xA, CMD_SET_VOLTAGE, twoscomplement14(cmd.motors[0]), motor2_response);
    delay(10);

    if(Serial.availableForWrite()){
        Serial.println("mot2: ");
        if(recv_bytes == 5){
            int16_t pos = pad14(motor2_response[0], motor2_response[1]);
            int16_t cur = pad14(motor2_response[2], motor2_response[3]);
            int16_t temp = pad14(0x0, motor2_response[4]);
            Serial.print("pos: ");
            Serial.println(pos);
            Serial.print("cur: ");
            Serial.println(cur);
            Serial.print("temp: ");
            Serial.println(temp);
        }else{
            Serial.println(recv_bytes);
            for(int i = 0; i < recv_bytes; i++){
                Serial.println(motor2_response[i]);
            }
        }
        
        Serial.println('\t');
    }
    
    recv_bytes = send_O32_cmd(0xB, CMD_SET_VOLTAGE, twoscomplement14(cmd.motors[1]), motor3_response);
    delay(10);

    // if(Serial.availableForWrite()){
    //     Serial.print("3numread: ");
    //     Serial.println(recv_bytes);
    //     for(int i = 0; i < recv_bytes; i++){
    //         Serial.println(motor3_response[i]);
    //     }
    //     Serial.println('\t');
    // }

    if (count < 10) {
        dxl_write(2, 116, 1);
        digitalWrite(LED_BUILTIN, LED_LIT);
    } else {
        dxl_write(2, 116, 2048);
        digitalWrite(LED_BUILTIN, LED_UNLIT);
    }
    count = (count + 1) % 20;
}

uint8_t send_O32_cmd(uint8_t addr, uint8_t CMD_TYPE, uint16_t data, uint8_t *rx) {
    while (rs485_0.available()) rs485_0.read(); // clear rx buffer

    uint8_t uart2_TX[3] = {0}; // command RS485 to Ø32
    uart2_TX[0] = CMD_TYPE | addr;
    uart2_TX[1] = (data >> 7) & 0b01111111;
    uart2_TX[2] = (data) & 0b01111111;

    rs485_0.write(uart2_TX, 3);
    rs485_0.flush();
    delayMicroseconds(200);
    uint8_t numread = rs485_0.readBytesUntil(MIN_INT8, rx, 10);
    return numread;
}

void dxl_write(uint8_t id, uint32_t reg_addr, uint32_t value) {
    tx_buffer[0] = 0xFF; // header
    tx_buffer[1] = 0xFF; // header
    tx_buffer[2] = 0xFD; // header
    tx_buffer[3] = 0x00; // reserved
    tx_buffer[4] = id;   // device ID
    tx_buffer[5] = 0x09; // length L (length of instruction, parameters, and crc)
    tx_buffer[6] = 0x00; // length H

    tx_buffer[7] = 0x03; // instruction (write)

    tx_buffer[8] = (unsigned char)(reg_addr & 0x00FF);        // register address L
    tx_buffer[9] = (unsigned char)((reg_addr >> 8) & 0x00FF); // register address H

    tx_buffer[10] = (unsigned char)(value & 0x00FF);         // data 0 LSB
    tx_buffer[11] = (unsigned char)((value >> 8) & 0x00FF);  // data 1
    tx_buffer[12] = (unsigned char)((value >> 16) & 0x00FF); // data 2
    tx_buffer[13] = (unsigned char)((value >> 24) & 0x00FF); // data 3 MSB

    short CRC = update_crc(0, tx_buffer, 14);            // 12 = 5 + Packet Length(7)
    unsigned char CRC_L = (unsigned char)(CRC & 0x00FF); // Little-endian
    unsigned char CRC_H = (unsigned char)((CRC >> 8) & 0x00FF);

    tx_buffer[14] = CRC_L;
    tx_buffer[15] = CRC_H;

    dxl_serial.write(tx_buffer, 16);
    // dxl_serial.flush();
}

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size) {
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};

    for (j = 0; j < data_blk_size; j++) {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}
