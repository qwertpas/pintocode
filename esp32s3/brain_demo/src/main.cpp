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

uint8_t estop_mac_addr[] = {0x30, 0x30, 0xF9, 0x34, 0x52, 0xA0};
uint8_t NO_SIGNAL = true;

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

// communication buffers
uint8_t dxl_tx[100];
uint8_t dxl_rx[100];
char print_buf[1024];

hw_timer_t *timer_1khz = NULL;    // watchdog
hw_timer_t *timer_elapsed = NULL; // keeping track of elapsed time

typedef struct cmd_struct {
    uint16_t servos[6];
    int16_t motors[2];
    uint16_t aux;
    uint16_t aux_prev;
} cmd_struct;
cmd_struct cmd;

typedef struct state_struct {
    uint8_t motor_power_on;
    uint8_t dxl_torque_on;
    uint16_t dxl_pos[5];

    int32_t pos_A;
    int32_t pos_B;

    int32_t encpos1_raw;
    int32_t encpos1_offset;
    int32_t encpos1;
    int32_t encpos2_raw;
    int32_t encpos2_offset;
    int32_t encpos2;

    int16_t acc[3];
    int16_t ang_rate[3];

    int16_t rpmA;
    uint8_t temp_ntcA;
    float vbusA;

    int16_t rpmB;
    uint8_t temp_ntcB;
    float vbusB;

    float vbus;

    uint32_t elapsed;
    float cur_tot;
} state_struct;
state_struct state = {0};

char state_str[1024];

uint32_t recv_watchdog = 0;

elapsedMillis print_timer;

elapsedMillis telemetry_timer;

uint8_t enable_motors_flag = 0;
uint8_t disable_motors_flag = 0;

String receivedString = "";
String recv_vars = "";

// function declarations
void reset_cmd(cmd_struct _cmd);
uint8_t send_O32_cmd(uint8_t addr, uint8_t CMD_TYPE, uint16_t data, uint8_t *rx);
uint8_t send_rs485_cmd(HardwareSerial *rs485, uint8_t addr, uint8_t CMD_TYPE, uint16_t data, uint8_t *rx, uint8_t bytes_to_read, uint16_t delay);
void set_dxl_torque_on();
void set_dxl_torque_off();
void dxl_write(uint8_t id, uint32_t reg_addr, uint32_t value);
void dxl_syncread(uint32_t reg_addr, uint32_t value, uint8_t *rx);
void dxl_syncwrite(uint32_t reg_addr, uint8_t bytes_to_write, uint32_t *data);
uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);


void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LED_UNLIT);

    pinMode(MOTOR_ON, OUTPUT);
    digitalWrite(MOTOR_ON, LOW);
    state.motor_power_on = false;

    pinMode(GPIO_D1, INPUT);

    Serial.begin(115200); // USB print
    Serial.setTimeout(0);
    Serial.setTxTimeoutMs(0);

    rs485_0.begin(1000000, SERIAL_8N1);                // baudrate, parity
    rs485_0.setPins(UART0_RX, UART0_TX, -1, UART0_DE); // RX, TX, CTS, DE
    rs485_0.setMode(UART_MODE_RS485_HALF_DUPLEX);
    rs485_0.setTimeout(1);

    rs485_1.begin(1000000, SERIAL_8N1);
    rs485_1.setPins(UART1_RX, UART1_TX, -1, UART1_DE);
    rs485_1.setMode(UART_MODE_RS485_HALF_DUPLEX);
    rs485_1.setTimeout(1);

    dxl_serial.begin(1000000, SERIAL_8N1, UART2_RX, UART2_TX);
    dxl_serial.setPins(UART2_RX, UART2_TX, -1, UART2_DE); // CTS pin should be an unused GPIO, otherwise USB serial disappears
    dxl_serial.setMode(UART_MODE_RS485_HALF_DUPLEX);
    dxl_serial.setTimeout(1);

    delay(1000);

    Serial.println("Turning on motors...");
    reset_cmd(cmd);
    digitalWrite(LED_BUILTIN, LED_LIT);
    digitalWrite(MOTOR_ON, HIGH);
    state.motor_power_on = true;

    delay(2000); // wait for motors to initialize
    // set_dxl_torque_on();

    timer_elapsed = timerBegin(1, 80, true); // also 1khz, should be enough because 64bit timer
    timerStart(timer_elapsed);
}

float cur_tot = 0.0f;
float alpha_cur_tot = 0.5;

uint8_t motA_rx[12] = {0};
uint8_t motA_nbytes = 0;
uint8_t motB_rx[12] = {0};
uint8_t motB_nbytes = 0;

uint32_t elapsed;
uint32_t elapsed_prev;

uint32_t o32cnt = 0;

void loop() {
    elapsed_prev = elapsed;
    elapsed = timerReadMilis(timer_elapsed);
    state.elapsed = elapsed;
    uint32_t looptime = elapsed - elapsed_prev;

    // READ TOTAL CURRENT
    uint32_t sns_adc = analogRead(GPIO_D1);
    if (sns_adc == 0) {
        cur_tot = 0;
    } else {
        cur_tot = alpha_cur_tot * (sns_adc * 11.224f + 222.865f) + (1 - alpha_cur_tot) * cur_tot;
    }
    state.cur_tot = cur_tot;

    // SET BLDC POWER
    uint8_t zeros[4] = {0};
    rs485_0.write(zeros, 4);
    rs485_0.flush();
    delayMicroseconds(500);
    int16_t cmd_A = cmd.motors[0];
    int16_t cmd_B = cmd.motors[1];
    motA_nbytes = send_O32_cmd(0xA, CMD_SET_VOLTAGE, twoscomplement14(cmd_A), motA_rx);
    motB_nbytes = send_O32_cmd(0xB, CMD_SET_VOLTAGE, twoscomplement14(cmd_B), motB_rx);
    state.pos_A = pad28(motA_rx[0], motA_rx[1], motA_rx[2], motA_rx[3]);
    state.pos_B = pad28(motB_rx[0], motB_rx[1], motB_rx[2], motB_rx[3]);

    uint8_t imu_rx[12] = {0};
    uint8_t imu_nbytes = send_rs485_cmd(&rs485_0, 0x6, CMD_GET_POSITION, 0x0, imu_rx, 12, 0);
    if(imu_nbytes == 12){
        state.acc[0] = pad14(imu_rx[0], imu_rx[1]);
        state.acc[1] = pad14(imu_rx[2], imu_rx[3]);
        state.acc[2] = pad14(imu_rx[4], imu_rx[5]);
        state.ang_rate[0] = pad14(imu_rx[6], imu_rx[7]);
        state.ang_rate[1] = pad14(imu_rx[8], imu_rx[9]);
        state.ang_rate[2] = pad14(imu_rx[10], imu_rx[11]);
    }

    //READ RS485 ENCODERS
    uint8_t enc_rx[4] = {0};
    uint8_t enc_nbytes = send_rs485_cmd(&rs485_0, 0x1, CMD_GET_POSITION, 0x0, enc_rx, 4, 82); //rs485_1 doesnt recv anything for some reason
    float enc_alpha = 1; //low pass filter, lower this number the smoother
    if(enc_nbytes == 4){
        state.encpos1_raw = enc_alpha * pad28(enc_rx[0], enc_rx[1], enc_rx[2], enc_rx[3]) + (1-enc_alpha)*state.encpos1_raw;
        state.encpos1 = state.encpos1_raw - state.encpos1_offset;
    } //add encoder 2 later
    enc_nbytes = send_rs485_cmd(&rs485_0, 0x2, CMD_GET_POSITION, 0x0, enc_rx, 4, 82); //rs485_1 doesnt recv anything for some reason
    if(enc_nbytes == 4){
        state.encpos2_raw = enc_alpha * pad28(enc_rx[0], enc_rx[1], enc_rx[2], enc_rx[3])  + (1-enc_alpha)*state.encpos2_raw;
        state.encpos2 = state.encpos2_raw - state.encpos2_offset;
    }

    dxl_write(dxl_ids[1], 116, cmd.servos[1]);
    dxl_write(dxl_ids[0], 116, cmd.servos[0]);
    dxl_write(dxl_ids[2], 116, cmd.servos[2]);
    dxl_write(dxl_ids[4], 116, cmd.servos[4]);
    dxl_write(dxl_ids[3], 116, cmd.servos[3]);

    // GET BLDC DATA
    if (motA_nbytes == 9) {
        state.rpmA = pad14(motA_rx[4], motA_rx[5]);
        state.temp_ntcA = motA_rx[6];
        state.vbusA = pad14(motA_rx[7], motA_rx[8]) * 3.3 * 5.12 / 4096.;
        o32cnt++;
    } else {
        state.rpmA = 0;
        state.temp_ntcA = 0;
        state.vbusA = 0;
    }
    if (motB_nbytes == 9) {
        state.rpmB = pad14(motB_rx[4], motB_rx[5]);
        state.temp_ntcB = motB_rx[6];
        state.vbusB = pad14(motB_rx[7], motB_rx[8]) * 3.3 * 5.12 / 4096.;
    } else {
        state.rpmB = 0;
        state.temp_ntcB = 0;
        state.vbusB = 0;
    }
    if (state.vbusA != 0 && state.vbusB != 0) {
        state.vbus = (state.vbusA + state.vbusB) / 2.;
    }else if (state.vbusA != 0){
        state.vbus = state.vbusA;
    }else if(state.vbusB != 0){
        state.vbus = state.vbusB;
    } else {
        state.vbus = 0;
    }


    // SERIAL PRINT OUT
    if (print_timer > 10 && Serial.availableForWrite()) {
        print_timer = 0;

        sprintf(
            print_buf,

            // "elapsed: %ld\n"
            // "looptime: %d\n"
            "cur_tot: %.2f\n"

            // "s[0]: %ld\n"
            // "s[1]: %ld\n"
            // "s[2]: %ld\n"
            // "s[3]: %ld\n"
            // "s[4]: %ld\n"
            // "m[0]: %ld\n"
            // "m[1]: %ld\n"

            // "tempA: %d\n"
            // "tempB: %d\n"
            // "vbus: %.2f\n"

            "cmdA: %d\n"
            "cmdB: %d\n"

            "posA: %d\n"
            "posB: %d\n"

            "rpmA: %d\n"
            "rpmB: %d\n"

            "encpos1: %d\n"

            "acc0: %d\n"
            "acc1: %d\n"
            "acc2: %d\n"
            "rate0: %d\n"
            "rate1: %d\n"
            "rate2: %d\n"

            // "dxl_pos[0]: %ld\n"
            // "dxl_pos[1]: %ld\n"
            // "dxl_pos[2]: %ld\n"
            // "dxl_pos[3]: %ld\n"
            // "dxl_pos[4]: %ld\n"
            // "dxl_torque_on: %ld\t\n",
            "\t\n",

            // elapsed,
            // looptime,
            cur_tot,

            // cmd.servos[0],
            // cmd.servos[1],
            // cmd.servos[2],
            // cmd.servos[3],
            // cmd.servos[4],
            // cmd.motors[0],
            // cmd.motors[1],

            // state.temp_ntcA,
            // state.temp_ntcB,
            // state.vbus,

            cmd.motors[0],
            cmd.motors[1],

            state.pos_A,
            state.pos_B,

            state.rpmA,
            state.rpmB,

            state.encpos1,

            state.acc[0],
            state.acc[1],
            state.acc[2],
            state.ang_rate[0],
            state.ang_rate[1],
            state.ang_rate[2]

            // state.dxl_pos[0],
            // state.dxl_pos[1],
            // state.dxl_pos[2],
            // state.dxl_pos[3],
            // state.dxl_pos[4]
            // state.dxl_torque_on
        );

        // if (NO_SIGNAL)
        //     Serial.println("No signal to ESTOP ~~~~~~~~~~~~~~~~~~~~");
        // Serial.println(o32cnt);
        Serial.write(print_buf);
    }

    // SERIAL COMMAND
    if (Serial.available()) {
        char user_cmd = Serial.read();
        switch (user_cmd) {
        case 't':
            if (state.dxl_torque_on) {
                set_dxl_torque_off();
            } else {
                set_dxl_torque_on();
            }
            break;
        case 'd':
        {
            char which_servo = Serial.read();
            int receivedInt = Serial.parseInt(); // Read the incoming integer
            if(which_servo == 'a')
                cmd.servos[0] = clip(receivedInt, 0, 4095);
            else if(which_servo = 'b')
                cmd.servos[1] = clip(receivedInt, 0, 4095);
            else if(which_servo = 'c')
                cmd.servos[2] = clip(receivedInt, 0, 4095);
            else if(which_servo = 'd')
                cmd.servos[3] = clip(receivedInt, 0, 4095);
            else if(which_servo = 'e')
                cmd.servos[4] = clip(receivedInt, 0, 4095);
            break;
        }
        case 'm':
        {
            char which_motor = Serial.read();
            int receivedInt = Serial.parseInt(); // Read the incoming integer
            if(which_motor == 'a')
                cmd.motors[0] = clip(receivedInt, -511, 511);
            else if(which_motor = 'b')
                cmd.motors[1] = clip(receivedInt, -511, 511);
            break;
        }
        case 'z':
            state.encpos1_offset = state.encpos1_raw; // zero ecnoder
            state.encpos2_offset = state.encpos2_raw; // zero ecnoder
            break;
        case 'l': // example: "l21" sets led on encoder 2 to high
        {
            if (!Serial.available())
                break;
            uint8_t enc_addr = Serial.read() - '0';
            if (!Serial.available())
                break;
            uint8_t led_cmd = Serial.read() - '0';
            send_rs485_cmd(&rs485_0, enc_addr, CMD_SET_POSITION, led_cmd, NULL, 0, 0);
            Serial.print("set led ");
            Serial.print(enc_addr);
            Serial.print(" to ");
            Serial.println(led_cmd);
            break;
        }
        case 'r':
            timerWrite(timer_elapsed, 0); // reset elapsed time
            break;
        case 'R':
            ESP.restart();
            break;
        default:
            break;
        }
        while (Serial.available()) Serial.read(); // clear RX buffer
    }
}

// FUNCTIONS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void reset_cmd(cmd_struct _cmd) {
    _cmd.servos[0] = 2048;
    _cmd.servos[1] = 2048;
    _cmd.servos[2] = 2048;
    _cmd.servos[3] = 2048;
    _cmd.servos[4] = 2048;
    _cmd.motors[0] = 0;
    _cmd.motors[1] = 0;
    _cmd.aux = 0;
    _cmd.aux_prev = 0;
}

uint8_t send_O32_cmd(uint8_t addr, uint8_t CMD_TYPE, uint16_t data, uint8_t *rx) {
    while (rs485_0.available())
        rs485_0.read(); // clear rx buffer

    uint8_t uart2_TX[4] = {0}; // command RS485 to Ø32
    uart2_TX[0] = CMD_TYPE | addr;
    uart2_TX[1] = (data >> 7) & 0b01111111;
    uart2_TX[2] = (data) & 0b01111111;

    uart2_TX[3] = ((uint8_t)(uart2_TX[0] + uart2_TX[1] + uart2_TX[2])) & 0b01111111; //checksum

    rs485_0.write(uart2_TX, 4);
    rs485_0.flush();
    delayMicroseconds(200); // should be enough for up to 20 bytes of response at 1Mbaud
    // uint8_t numread = rs485_0.readBytesUntil(MIN_INT8, rx, 10); //don't use because data itself might contain MIN_INT8
    uint8_t numread = rs485_0.readBytes(rx, 9);
    return numread;
}

uint8_t send_rs485_cmd(HardwareSerial *rs485, uint8_t addr, uint8_t CMD_TYPE, uint16_t data, uint8_t *rx, uint8_t bytes_to_read, uint16_t delay) {
    while (rs485->available())
        rs485->read(); // clear rx buffer

    uint8_t uart2_TX[3] = {0}; // command RS485 to Ø32
    uart2_TX[0] = CMD_TYPE | addr;
    uart2_TX[1] = (data >> 7) & 0b01111111;
    uart2_TX[2] = (data) & 0b01111111;

    rs485->write(uart2_TX, 3);
    rs485->flush();
    // delayMicroseconds(82 + bytes_to_read*10); // give each byte 10us if 1Mbaud
    delayMicroseconds(delay + bytes_to_read * 10);
    // delayMicroseconds(200);
    uint8_t numread = rs485->readBytes(rx, bytes_to_read);
    return numread;
}

void set_dxl_torque_on() {
    for (int i = 0; i < 5; i++) {
        dxl_write(dxl_ids[i], 64, 1); // servos torque on
    }
    // uint32_t data[5] = {1,1,1,1,1};
    // dxl_syncwrite(64, 4, data);
    state.dxl_torque_on = true;
}
void set_dxl_torque_off() {
    for (int i = 0; i < 5; i++) {
        dxl_write(dxl_ids[i], 64, 0); // servos torque off
    }
    state.dxl_torque_on = false;
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

    uint16_t CRC = update_crc(0, dxl_tx, 14); // number of bytes until now including header
    uint8_t CRC_L = (uint8_t)(CRC & 0x00FF);  // Little-endian
    uint8_t CRC_H = (uint8_t)((CRC >> 8) & 0x00FF);

    dxl_tx[14] = CRC_L;
    dxl_tx[15] = CRC_H;

    dxl_serial.write(dxl_tx, 16);
    // dxl_serial.flush();
    delayMicroseconds(300);
}

void dxl_syncread(uint32_t reg_addr, uint32_t bytes_to_read, uint8_t *rx) { // reads from all the servos
    dxl_tx[0] = 0xFF;                                                       // header
    dxl_tx[1] = 0xFF;                                                       // header
    dxl_tx[2] = 0xFD;                                                       // header
    dxl_tx[3] = 0x00;                                                       // reserved
    dxl_tx[4] = 0xFE;                                                       // ID: broadcast
    dxl_tx[5] = 0x0C;                                                       // length L (length of instruction, parameters, and crc)
    dxl_tx[6] = 0x00;                                                       // length H

    dxl_tx[7] = 0x82; // instruction (sync read)

    dxl_tx[8] = (uint8_t)(reg_addr & 0x00FF);        // P1: register address L
    dxl_tx[9] = (uint8_t)((reg_addr >> 8) & 0x00FF); // P2: register address H

    dxl_tx[10] = (uint8_t)(bytes_to_read & 0x00FF);        // P3: num bytes L
    dxl_tx[11] = (uint8_t)((bytes_to_read >> 8) & 0x00FF); // P4: num bytes H
    dxl_tx[12] = (uint8_t)(dxl_ids[0] & 0x00FF);           // P5: ID of 1st device to read from
    dxl_tx[13] = (uint8_t)(dxl_ids[1] & 0x00FF);           // P6: ID of 2nd device to read from
    dxl_tx[14] = (uint8_t)(dxl_ids[2] & 0x00FF);           // P7: ID of 3rd device to read from
    dxl_tx[15] = (uint8_t)(dxl_ids[3] & 0x00FF);           // P8: ID of 4th device to read from
    dxl_tx[16] = (uint8_t)(dxl_ids[4] & 0x00FF);           // P9: ID of 5th device to read from

    unsigned short CRC = update_crc(0, dxl_tx, 17); // last number is packet length until now
    uint8_t CRC_L = (uint8_t)(CRC & 0x00FF);        // Little-endian
    uint8_t CRC_H = (uint8_t)((CRC >> 8) & 0x00FF);

    dxl_tx[17] = CRC_L;
    dxl_tx[18] = CRC_H;

    while (dxl_serial.available())
        dxl_serial.read(); // clear rx buffer

    dxl_serial.write(dxl_tx, 19);
    // while (dxl_serial.available()) dxl_serial.read(); // clear rx buffer

    dxl_serial.flush();
    delayMicroseconds(500);

    *rx = dxl_serial.readBytes(rx, 100);
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
