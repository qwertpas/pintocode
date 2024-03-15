#include "main.h"

#define DO_CALIB false

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

#define GPIO33_UNUSED 33
#define GPIO34_UNUSED 34
#define GPIO35_UNUSED 35

HardwareSerial rs485_0(0);    // RS485 bus 0
HardwareSerial rs485_1(1);    // RS485 bus 1
HardwareSerial dxl_serial(2); // dynamixel TTL bus

uint8_t dxl_ids[] = {
    3, // right back
    4, // right front
    5, // left front
    6, // left back
    7  // roll
};

//communication buffers
uint8_t dxl_tx[100];
uint8_t dxl_rx[100];
char print_buf[1024];

hw_timer_t *timer_1khz = NULL;    // watchdog
hw_timer_t *timer_elapsed = NULL; // keeping track of elapsed time

typedef struct cmd_struct {
    uint16_t servos[6];
    int16_t motors[2];
} cmd_struct;
cmd_struct cmd = {0};

typedef struct state_struct {
    uint8_t dxl_on;
    uint16_t dxl_pos[5];
} state_struct;
state_struct state = {0};

uint32_t recv_watchdog = 0;

elapsedMillis print_timer;
uint16_t print_period = 10;

uint8_t enable_motors_flag = 0;
uint8_t disable_motors_flag = 0;

String receivedString = "";
String recv_vars = "";


//function declarations
uint8_t send_O32_cmd(uint8_t addr, uint8_t CMD_TYPE, uint16_t data, uint8_t *rx);
void dxl_enable();
void dxl_disable();
void dxl_write(uint8_t id, uint32_t reg_addr, uint32_t value);
void dxl_syncread(uint32_t reg_addr, uint32_t value, uint8_t *rx);
uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);


const String servolabels[] = {"s0", "s1", "s2"}; 
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) { //len should not be longer than 250bytes
    // uint8_t lineend = (uint8_t) (strchr((const char*)incomingData, '\n') - (const char*)incomingData); //index of first '\n'
    // char line[250] = "";

    // mem

    // uint8_t linenum = 0;
    // for(uint8_t i=0; i<len; i++){
    //     incomingData[i];
    //     sscanf((char*)incomingData[i], "%c%d:%d\n", &motor_type, &index, &val) == 3
    // }

    // char str[250] = {0};

    // char* str = ;
    // memcpy(str, recv_str, len);


    receivedString = String((char *)incomingData);

    //stupid way to read recieved string but oh well gotta go fast
    cmd.servos[0] = receivedString.substring(receivedString.indexOf("s0") + 3, receivedString.indexOf("s0") + 8).toInt();
    cmd.servos[1] = receivedString.substring(receivedString.indexOf("s1") + 3, receivedString.indexOf("s1") + 8).toInt();
    cmd.servos[2] = receivedString.substring(receivedString.indexOf("s2") + 3, receivedString.indexOf("s2") + 8).toInt();
    cmd.servos[3] = receivedString.substring(receivedString.indexOf("s3") + 3, receivedString.indexOf("s3") + 8).toInt();
    cmd.servos[4] = receivedString.substring(receivedString.indexOf("s4") + 3, receivedString.indexOf("s4") + 8).toInt();
    cmd.motors[0] = receivedString.substring(receivedString.indexOf("m0") + 3, receivedString.indexOf("m0") + 8).toInt();
    cmd.motors[1] = receivedString.substring(receivedString.indexOf("m1") + 3, receivedString.indexOf("m1") + 8).toInt();

    if(receivedString.indexOf("s0") != -1){ //data exists
        if (recv_watchdog > 100) {
            enable_motors_flag = 1;
        }
        recv_watchdog = 0;
    }

}

void IRAM_ATTR timer_1khz_ISR() { // 1kHz
    recv_watchdog++;
    // Serial.println("timer0");
}



void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LED_UNLIT);

    pinMode(MOTOR_ON, OUTPUT);
    digitalWrite(MOTOR_ON, LOW);

    pinMode(GPIO_D1, INPUT);

    // USB printout
    Serial.begin(115200);
    Serial.setTimeout(1);

    rs485_0.begin(1000000, SERIAL_8N1);                // baudrate, parity
    rs485_0.setPins(UART0_RX, UART0_TX, -1, UART0_DE); // RX, TX, CTS, DE
    rs485_0.setMode(UART_MODE_RS485_HALF_DUPLEX);
    rs485_0.setTimeout(1);

    rs485_1.begin(115200, SERIAL_8N1);
    rs485_1.setPins(UART1_RX, UART1_TX, -1, UART1_DE);
    rs485_1.setMode(UART_MODE_RS485_HALF_DUPLEX);
    rs485_1.setTimeout(1);

    dxl_serial.begin(1000000, SERIAL_8N1, UART2_RX, UART2_TX);
    dxl_serial.setPins(UART2_RX, UART2_TX, -1, UART2_DE); // CTS pin should be an unused GPIO, otherwise USB serial disappears
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

    delay(1000);

    Serial.println("My MAC address:");
    Serial.println(WiFi.macAddress());

    Serial.println("Turning on motors...");
    digitalWrite(LED_BUILTIN, LED_LIT);
    digitalWrite(MOTOR_ON, HIGH);

    delay(2000); // wait for motors to initialize
    dxl_enable();

    timer_1khz = timerBegin(0, 80, true);                    // timer index 0, 80MHz clock, 80div=1us precision, true=count up
    timerAttachInterrupt(timer_1khz, &timer_1khz_ISR, true); // true=edgetriggered
    timerAlarmWrite(timer_1khz, 1000, true);                 // microseconds, true=autoreload
    timerAlarmEnable(timer_1khz);                            // start counting

    timer_elapsed = timerBegin(1, 80, true); // also 1khz, should be enough because 64bit timer
    timerStart(timer_elapsed);
}

float cur_tot = 0.0f;
float alpha_cur_tot = 0.5;

uint8_t motA_rx[12] = {0};
uint8_t motA_nbytes = 0;
uint8_t motB_rx[12] = {0};
uint8_t motB_nbytes = 0;

uint8_t calib_state = DO_CALIB ? 1 : 0; // 1:moving, 0:done
uint16_t calib_cmd = 100;               // motor power (out of 1599) to calibrate with
uint16_t calib_cur_thres = 800;         // milliamps at stall
int32_t calib_pos_A = 0;                // position that motor stalls
int32_t calib_pos_B = 0;

int32_t pos_A = 0;
int32_t pos_B = 0;

void loop() {
    uint32_t elapsed = timerReadMilis(timer_elapsed);

    uint32_t sns_adc = analogRead(GPIO_D1); // read current no matter what
    if (sns_adc == 0) {
        cur_tot = 0;
    } else {
        cur_tot = alpha_cur_tot * (sns_adc * 11.224f + 222.865f) + (1 - alpha_cur_tot) * cur_tot;
    }

    if (recv_watchdog > 100) { // didn't receive anything from joystick in a while
        cmd = {0};
        motA_nbytes = send_O32_cmd(0xA, CMD_SET_VOLTAGE, 0, motA_rx);
        motB_nbytes = send_O32_cmd(0xB, CMD_SET_VOLTAGE, 0, motB_rx);
        dxl_disable();
        if (Serial.availableForWrite() && print_timer > 100){
            print_timer = 0;
            Serial.println("not receiving joystick \t");
        }
        return; // skip what comes after
    }
    
    if (enable_motors_flag) {
        enable_motors_flag = 0;
        delay(100);
        dxl_enable();
    }

    if (calib_state && elapsed < 10000) { // calibration happens when calib_state != 0. If it takes more than 10 seconds, continue to operation
        switch (calib_state) {
        case 1: // extend A at low power until stall
            motA_nbytes = send_O32_cmd(0xA, CMD_SET_VOLTAGE, twoscomplement14(calib_cmd), motA_rx);
            motB_nbytes = send_O32_cmd(0xB, CMD_SET_VOLTAGE, 0, motB_rx);
            if (cur_tot > calib_cur_thres) {
                delay(1000);
                calib_pos_A = pad28(motA_rx[0], motA_rx[1], motA_rx[2], motA_rx[3]);

                calib_state = 2;
            }
            break;
        case 2: // extend B at low power until stall
            motA_nbytes = send_O32_cmd(0xA, CMD_SET_VOLTAGE, 0, motA_rx);
            motB_nbytes = send_O32_cmd(0xB, CMD_SET_VOLTAGE, twoscomplement14(calib_cmd), motB_rx);
            if (cur_tot > calib_cur_thres) {
                delay(1000);
                calib_pos_B = pad28(motB_rx[0], motB_rx[1], motB_rx[2], motB_rx[3]);

                motA_nbytes = send_O32_cmd(0xA, CMD_SET_VOLTAGE, 0, motA_rx);
                motA_nbytes = send_O32_cmd(0xA, CMD_SET_VOLTAGE, 0, motA_rx);
                calib_state = 0;
            }
            break;
        default:
            break;
        }

        if (print_timer > print_period && Serial.availableForWrite()) {
            print_timer = 0;


            Serial.print("elapsed: ");
            Serial.println(elapsed);
            Serial.print("calib_state: ");
            Serial.println(calib_state);
            Serial.print("cur_tot: ");
            Serial.println(cur_tot);
            Serial.print("calib_pos_A: ");
            Serial.println(calib_pos_A);
            Serial.print("calib_pos_B: ");
            Serial.println(calib_pos_B);
            Serial.println("\t");
        }

        return;
    }

    // normal operation
    int16_t cmd_A = (pos_A > calib_pos_A) ? 0 : cmd.motors[0];
    int16_t cmd_B = (pos_B > calib_pos_B) ? 0 : cmd.motors[1];
    motA_nbytes = send_O32_cmd(0xA, CMD_SET_VOLTAGE, twoscomplement14(cmd_A), motA_rx);
    motB_nbytes = send_O32_cmd(0xB, CMD_SET_VOLTAGE, twoscomplement14(cmd_B), motB_rx);

    pos_A = pad28(motA_rx[0], motA_rx[1], motA_rx[2], motA_rx[3]);
    pos_B = pad28(motB_rx[0], motB_rx[1], motB_rx[2], motB_rx[3]);

    // for (int i = 0; i < 5; i++) {
    //     // uint16_t duty = map(cmd.servos[i], 0, 180, 0, 4095); //map 0-180º to 0-4095 for dynamixel position
    //     uint16_t duty = map(180, 0, 360, 0, 4095); // map 0-180º to 0-4095 for dynamixel position
    //     dxl_write(dxl_ids[i], 116, duty);
    //     // delayMicroseconds(300);
    // }
    dxl_syncread(0x0084, 4, dxl_rx);
    for(uint8_t i=0; i<5; i++){
        if(dxl_rx[15*i + 7] == 0x55){ //check if it's a return status packet
            state.dxl_pos[i] = (dxl_rx[15*i + 10] << 8) + dxl_rx[15*i + 9]; 
        }
    }

    if (Serial.available()) { // restart if first character is 'R'
        char user_cmd = Serial.read();
        switch (user_cmd){
        case 'm':
            if(state.dxl_on) dxl_disable();
            else dxl_enable();
            break;
        case 'r':
            timerWrite(timer_elapsed, 0); //reset elapsed time
            break;
        case 'R':
            ESP.restart();
            break;
        default:
            break;
        }

        while (Serial.available())
            Serial.read();
    }

    if (print_timer > print_period && Serial.availableForWrite()) {
        print_timer = 0;

        if (motA_nbytes == 7) {
            int16_t rpm = pad14(motA_rx[4], motA_rx[5]);
            int16_t temp_ntc = pad14(0x0, motA_rx[6]);

            Serial.print("cmd_A: ");
            Serial.println(cmd.motors[0]);
            Serial.print("pos_A: ");
            Serial.println(pos_A);
            Serial.print("temp_ntc_A: ");
            Serial.println(temp_ntc);
        }

        if (motB_nbytes == 7) {
            int16_t rpm = pad14(motB_rx[4], motB_rx[5]);
            int16_t temp_ntc = pad14(0x0, motB_rx[6]);

            Serial.print("cmd_B: ");
            Serial.println(cmd.motors[1]);
            Serial.print("pos_B: ");
            Serial.println(pos_B);
            Serial.print("temp_ntc_B: ");
            Serial.println(temp_ntc);
        }

        sprintf(
            print_buf, 
            "elapsed: %ld\n"
            // "cur_tot: %f\n"
            // "calib_pos_A: %ld\n"
            // "calib_pos_B: %ld\n"

            "servos[0]: %ld\n"
            "servos[1]: %ld\n"
            "servos[2]: %ld\n"
            "servos[3]: %ld\n"
            "servos[4]: %ld\n"
            
            "dxl_pos[0]: %ld\n"
            "dxl_pos[1]: %ld\n"
            "dxl_pos[2]: %ld\n"
            "dxl_pos[3]: %ld\n"
            "dxl_pos[4]: %ld\n"
            "dxl_on: %ld\t\n",

            elapsed, 
            // cur_tot,
            // calib_pos_A,
            // calib_pos_B,

            cmd.servos[0],
            cmd.servos[1],
            cmd.servos[2],
            cmd.servos[3],
            cmd.servos[4],

            state.dxl_pos[0],
            state.dxl_pos[1],
            state.dxl_pos[2],
            state.dxl_pos[3],
            state.dxl_pos[4],
            state.dxl_on
        );
        Serial.write(print_buf);
    }
}

uint8_t send_O32_cmd(uint8_t addr, uint8_t CMD_TYPE, uint16_t data, uint8_t *rx) {
    while (rs485_0.available())
        rs485_0.read(); // clear rx buffer

    uint8_t uart2_TX[3] = {0}; // command RS485 to Ø32
    uart2_TX[0] = CMD_TYPE | addr;
    uart2_TX[1] = (data >> 7) & 0b01111111;
    uart2_TX[2] = (data) & 0b01111111;

    rs485_0.write(uart2_TX, 3);
    rs485_0.flush();
    delayMicroseconds(200); // should be enough for up to 20 bytes of response at 1Mbaud
    // uint8_t numread = rs485_0.readBytesUntil(MIN_INT8, rx, 10);
    uint8_t numread = rs485_0.readBytes(rx, 7);
    return numread;
}

void dxl_enable(){
    for (int i = 0; i < 5; i++) {
        dxl_write(dxl_ids[i], 64, 1); // servos torque on
    }
    state.dxl_on = true;
}
void dxl_disable(){
    for (int i = 0; i < 5; i++) {
        dxl_write(dxl_ids[i], 64, 0); // servos torque off
    }
    state.dxl_on = false;
}

void dxl_write(uint8_t id, uint32_t reg_addr, uint32_t value) {
    dxl_tx[0] = 0xFF; // header
    dxl_tx[1] = 0xFF; // header
    dxl_tx[2] = 0xFD; // header
    dxl_tx[3] = 0x00; // reserved
    dxl_tx[4] = id;   // device ID
    dxl_tx[5] = 0x09; // length L (length of instruction, parameters, and crc)
    dxl_tx[6] = 0x00; // length H

    dxl_tx[7] = 0x03; // instruction (write)

    dxl_tx[8] = (uint8_t)(reg_addr & 0x00FF);        // register address L
    dxl_tx[9] = (uint8_t)((reg_addr >> 8) & 0x00FF); // register address H

    dxl_tx[10] = (uint8_t)(value & 0x00FF);         // data 0 LSB
    dxl_tx[11] = (uint8_t)((value >> 8) & 0x00FF);  // data 1
    dxl_tx[12] = (uint8_t)((value >> 16) & 0x00FF); // data 2
    dxl_tx[13] = (uint8_t)((value >> 24) & 0x00FF); // data 3 MSB

    uint16_t CRC = update_crc(0, dxl_tx, 14);            // 12 = 5 + Packet Length(7)
    uint8_t CRC_L = (uint8_t)(CRC & 0x00FF); // Little-endian
    uint8_t CRC_H = (uint8_t)((CRC >> 8) & 0x00FF);

    dxl_tx[14] = CRC_L;
    dxl_tx[15] = CRC_H;

    dxl_serial.write(dxl_tx, 16);
    // dxl_serial.flush();
    delayMicroseconds(300);
}

void dxl_syncread(uint32_t reg_addr, uint32_t bytes_to_read, uint8_t* rx) { //reads from all the servos
    dxl_tx[0] = 0xFF; // header
    dxl_tx[1] = 0xFF; // header
    dxl_tx[2] = 0xFD; // header
    dxl_tx[3] = 0x00; // reserved
    dxl_tx[4] = 0xFE; // ID: broadcast
    dxl_tx[5] = 0x0C; // length L (length of instruction, parameters, and crc)
    dxl_tx[6] = 0x00; // length H

    dxl_tx[7] = 0x82; // instruction (sync read)

    dxl_tx[8] = (uint8_t)(reg_addr & 0x00FF);        // P1: register address L
    dxl_tx[9] = (uint8_t)((reg_addr >> 8) & 0x00FF); // P2: register address H

    dxl_tx[10] = (uint8_t)(bytes_to_read & 0x00FF);         // P3: num bytes L
    dxl_tx[11] = (uint8_t)((bytes_to_read >> 8) & 0x00FF);  // P4: num bytes H
    dxl_tx[12] = (uint8_t)(dxl_ids[0] & 0x00FF);           // P5: ID of 1st device to read from
    dxl_tx[13] = (uint8_t)(dxl_ids[1] & 0x00FF);           // P6: ID of 2nd device to read from
    dxl_tx[14] = (uint8_t)(dxl_ids[2] & 0x00FF);           // P7: ID of 3rd device to read from
    dxl_tx[15] = (uint8_t)(dxl_ids[3] & 0x00FF);           // P8: ID of 4th device to read from
    dxl_tx[16] = (uint8_t)(dxl_ids[4] & 0x00FF);           // P9: ID of 5th device to read from

    unsigned short CRC = update_crc(0, dxl_tx, 17);            // last number is packet length until now
    uint8_t CRC_L = (uint8_t)(CRC & 0x00FF); // Little-endian
    uint8_t CRC_H = (uint8_t)((CRC >> 8) & 0x00FF);

    dxl_tx[17] = CRC_L;
    dxl_tx[18] = CRC_H;

    while (dxl_serial.available()) dxl_serial.read(); // clear rx buffer

    dxl_serial.write(dxl_tx, 19);
    // while (dxl_serial.available()) dxl_serial.read(); // clear rx buffer

    dxl_serial.flush();
    delayMicroseconds(500);

    *rx = dxl_serial.readBytes(rx, 100);
}


void dxl_read() {
}

uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size) {
    uint16_t i, j;
    uint16_t crc_table[256] = {
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
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}
