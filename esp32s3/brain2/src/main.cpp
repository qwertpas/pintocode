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

#define LSM6DSV16X_ACC_SENSITIVITY_FS_2G   0.061f  //default
#define LSM6DSV16X_ACC_SENSITIVITY_FS_4G   0.122f
#define LSM6DSV16X_ACC_SENSITIVITY_FS_8G   0.244f
#define LSM6DSV16X_ACC_SENSITIVITY_FS_16G  0.488f

#define LSM6DSV16X_GYRO_SENSITIVITY_FS_125DPS     4.375f //default
#define LSM6DSV16X_GYRO_SENSITIVITY_FS_250DPS     8.750f
#define LSM6DSV16X_GYRO_SENSITIVITY_FS_500DPS    17.500f
#define LSM6DSV16X_GYRO_SENSITIVITY_FS_1000DPS   35.000f
#define LSM6DSV16X_GYRO_SENSITIVITY_FS_2000DPS   70.000f
#define LSM6DSV16X_GYRO_SENSITIVITY_FS_4000DPS  140.000f

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
char print_buf[200];

hw_timer_t *timer_1khz = NULL;    // watchdog
hw_timer_t *timer_elapsed = NULL; // keeping track of elapsed time

typedef struct cmd_struct {
    uint16_t servos[6];
    int16_t motor_pwrs[2];
    int16_t motor_pos[2];
    uint16_t aux;
    uint16_t aux_prev;
} cmd_struct;
cmd_struct cmd;

typedef struct state_struct {
    uint8_t motor_power_on; //B
    uint8_t dxl_torque_on; //B
    uint16_t dxl_pos[5]; //5H

    int32_t pos_A; //i
    int32_t pos_B; //i

    int16_t sent_cmd_A; //h
    int16_t sent_cmd_B; //h

    int32_t encposB_raw; //i
    int32_t encposB_offset; //i
    int32_t encposB; //i
    int32_t encposA_raw; //i
    int32_t encposA_offset; //i
    int32_t encposA; //i

    float acc[3];      //3f
    float ang_rate[3]; //3f
    float ang_rate_prev[3]; //for trapezoidal integration //3f
    float ang_gyro[3]; //3f
    float ang_acc[2];//2f
    float ang_filt[2];//2f
    
    int16_t rpmA; //h
    uint8_t temp_ntcA; //B
    float vbusA; //f

    int16_t rpmB; //h
    uint8_t temp_ntcB; //B
    float vbusB; //f

    float vbus; //f

    uint32_t elapsed; //I
    float cur_tot; //f

    uint8_t no_signal_cnt; //B

    int16_t setpos_endstop[2];
    int16_t setpos_trigger[2];
    int16_t setpos_retract[2];
    int16_t setpos_reset[2];
    int16_t latchtiming;
    uint8_t do_timed_latch;
    uint8_t disable_o32_comm; //aux setpoint writing overrides motor_pos so motor comms need to be disabled for one cycle

} state_struct;
state_struct state = {0};

// state_struct log[10] = {0};


char state_str[1024];

uint32_t recv_watchdog = 0;

elapsedMillis print_timer;

elapsedMillis telemetry_timer;

uint8_t enable_motors_flag = 0;

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







//CALLBACKS
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) { // len should not be longer than 250bytes
    receivedString = String((char *)incomingData);

    if (receivedString.indexOf("s0") != -1) { // data exists

        if (recv_watchdog > 100) {
            enable_motors_flag = 1; //reenable motors
        }
        NO_SIGNAL = false;
        recv_watchdog = 0;
    }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Serial.print("\r\nLast Packet Send Status:\t");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void IRAM_ATTR timer_1khz_ISR() { // 1kHz
    recv_watchdog++;
}







void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LED_UNLIT);

    pinMode(MOTOR_ON, OUTPUT);
    digitalWrite(MOTOR_ON, LOW);
    state.motor_power_on = false;

    pinMode(GPIO_D1, INPUT);

    Serial.begin(1000000); //USB print
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

    WiFi.mode(WIFI_MODE_STA);
    Serial.println(WiFi.macAddress());

    delay(3000);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        delay(3000);
        ESP.restart();
    }

    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, estop_mac_addr, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);


    delay(1000);

    Serial.println("My MAC address:");
    Serial.println(WiFi.macAddress());

    Serial.println("Turning on motors...");
    reset_cmd(cmd);
    digitalWrite(LED_BUILTIN, LED_LIT);
    digitalWrite(MOTOR_ON, HIGH);
    state.motor_power_on = true;

    delay(2000); // wait for motors to initialize
    // set_dxl_torque_on();

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

uint32_t elapsed;
uint32_t elapsed_prev;
unsigned long imu_time_prev = 0;

uint32_t o32cnt = 0;

void loop() {
    elapsed_prev = elapsed;
    elapsed = timerReadMilis(timer_elapsed);
    state.elapsed = elapsed;
    uint32_t looptime = elapsed - elapsed_prev;

    state.disable_o32_comm = 0; //motor comm enabled by default, things within this loop can disable it for one cycle

    //READ TOTAL CURRENT
    uint32_t sns_adc = analogRead(GPIO_D1);
    if (sns_adc == 0) {
        cur_tot = 0;
    } else {
        cur_tot = alpha_cur_tot * (sns_adc * 11.224f + 222.865f) + (1 - alpha_cur_tot) * cur_tot;
    }
    state.cur_tot = cur_tot;


    //DXL READ POSITION
    dxl_syncread(0x0084, 4, dxl_rx);
    for (uint8_t i = 0; i < 5; i++) {
        if (dxl_rx[15 * i + 7] == 0x55) { // check if it's a return status packet
            state.dxl_pos[i] = (dxl_rx[15 * i + 10] << 8) + dxl_rx[15 * i + 9];
        }
    }


    //READ RS485 ENCODERS
    uint8_t enc_rx[4] = {0};
    // float enc_alpha = 0.4; //low pass filter, lower this number the smoother
    float enc_alpha = 0.8; //low pass filter, lower this number the smoother
    uint8_t enc_nbytes = send_rs485_cmd(&rs485_0, 0x1, CMD_GET_POSITION, 0x0, enc_rx, 4, 82); //rs485_1 doesnt recv anything for some reason
    if(enc_nbytes == 4){ //bottom motor B
        state.encposB_raw = enc_alpha * pad28(enc_rx[0], enc_rx[1], enc_rx[2], enc_rx[3]) + (1-enc_alpha)*state.encposB_raw;
        state.encposB = state.encposB_raw - state.encposB_offset;
    }
    enc_nbytes = send_rs485_cmd(&rs485_0, 0x2, CMD_GET_POSITION, 0x0, enc_rx, 4, 82); //rs485_1 doesnt recv anything for some reason
    if(enc_nbytes == 4){ //top motor A
        state.encposA_raw = enc_alpha * -pad28(enc_rx[0], enc_rx[1], enc_rx[2], enc_rx[3])  + (1-enc_alpha)*state.encposA_raw; //negative sign for positive extension
        state.encposA = state.encposA_raw - state.encposA_offset;
    }

    //READ IMU
    uint8_t imu_rx[12] = {0};
    uint8_t imu_nbytes = send_rs485_cmd(&rs485_0, 0x6, CMD_GET_POSITION, 0x0, imu_rx, 12, 0);
    if(imu_nbytes == 12){

        for(int i = 0; i < 3; i++){
            state.ang_rate_prev[i] = state.ang_rate[i];
        }

        state.acc[0] = pad14(imu_rx[0], imu_rx[1]) * LSM6DSV16X_ACC_SENSITIVITY_FS_2G; //X
        state.acc[1] = pad14(imu_rx[2], imu_rx[3]) * LSM6DSV16X_ACC_SENSITIVITY_FS_2G; //Y
        state.acc[2] = pad14(imu_rx[4], imu_rx[5]) * LSM6DSV16X_ACC_SENSITIVITY_FS_2G; //Z
        state.ang_rate[0] = pad14(imu_rx[6], imu_rx[7]) * LSM6DSV16X_GYRO_SENSITIVITY_FS_125DPS;
        state.ang_rate[1] = pad14(imu_rx[8], imu_rx[9]) * LSM6DSV16X_GYRO_SENSITIVITY_FS_125DPS;
        state.ang_rate[2] = pad14(imu_rx[10], imu_rx[11]) * LSM6DSV16X_GYRO_SENSITIVITY_FS_125DPS;

        unsigned long dt = micros() - imu_time_prev;
        for(int i = 0; i < 3; i++){
            state.ang_gyro[i] = state.ang_rate[i] * dt;
        }
        state.ang_acc[0]  = atan2(state.acc[1], state.acc[2]) * RAD_TO_DEG;
        state.ang_acc[1] = atan(-state.acc[0] / sqrt(state.acc[1] * state.acc[1] + state.acc[2] * state.acc[2])) * RAD_TO_DEG;
    }
    // Serial.println(imu_nbytes);
    // Serial.println(imu_rx[0]);
    // Serial.println(imu_rx[1]);
    // Serial.println(imu_rx[2]);
    // Serial.println(imu_rx[3]);
    // Serial.println(imu_rx[4]);
    // Serial.println(imu_rx[5]);
    // if(enc_nbytes == 4){
    //     state.encposB_raw = pad28(enc_rx[0], enc_rx[1], enc_rx[2], enc_rx[3]);
    //     state.encposB = state.encposB_raw - state.encposB_offset;
    // }


    //AUX COMMANDS
    if((cmd.aux & 0b00001) && !(cmd.aux_prev & 0b00001)){ //if rising edge of aux bit 0, toggle motor power
        if(state.motor_power_on){
            state.motor_power_on = false;
            digitalWrite(MOTOR_ON, LOW);
        }else{
            state.motor_power_on = true;
            digitalWrite(MOTOR_ON, HIGH);
        }
    }
    if((cmd.aux & 0b00010) && !(cmd.aux_prev & 0b00010)){ //if rising edge of aux bit 1, turn on servos
        set_dxl_torque_on();


        if(state.encposB_raw < 0){ //bring encoders back into range
            state.encposB_offset = (state.encposB_raw/32768 - 1) * 32768; // -48000 - (-1 - 1)*32768
        }else{
            state.encposB_offset = state.encposB_raw/32768 * 32768; 
        }

        if(state.encposA_raw < 0){
            state.encposA_offset = (state.encposA_raw/32768 - 1) * 32768; // -48000 - (-1 - 1)*32768
        }else{
            state.encposA_offset = state.encposA_raw/32768 * 32768; 
        }
        // state.encposB_offset = copysign(state.encposB_raw, state.encposB_raw/32768 * 32768);
        // state.encposA_offset = copysign(state.encposA_raw, state.encposA_raw/32768 * 32768);
    }
    uint8_t aux_setpos = (cmd.aux >> 2) & 0b011; // upper 3 bits of aux has info about which encoder setpoint to update or timed action 
    if(aux_setpos){ //if there's a setpoint to update or timed setpoint action
        state.disable_o32_comm = 1;        
        if(aux_setpos == 0){ //update setpoint reset 
            state.setpos_reset[0] = cmd.motor_pos[0];
            state.setpos_reset[1] = cmd.motor_pos[1];
        }else if(aux_setpos == 1){ //update setpoint retract
            state.setpos_retract[0] = cmd.motor_pos[0];
            state.setpos_retract[1] = cmd.motor_pos[1];
        }else if(aux_setpos == 2){ //update setpoint trigger
            state.setpos_trigger[0] = cmd.motor_pos[0];
            state.setpos_trigger[1] = cmd.motor_pos[1];
        }else if(aux_setpos == 3){ //update setpoint endstop
            state.setpos_endstop[0] = cmd.motor_pos[0];
            state.setpos_endstop[1] = cmd.motor_pos[1];
        }else if(aux_setpos == 4){ //update latch timing difference
            state.latchtiming = cmd.motor_pos[0]; //use motor_pos[0] as milliseconds delay
        }else if(aux_setpos == 7){ //try to do timed jump
            state.do_timed_latch = (
                state.setpos_reset[0]!=0 && state.setpos_retract[0]!=0 && state.setpos_trigger[0]!=0 && state.setpos_endstop[0]!=0 && 
                state.setpos_reset[1]!=0 && state.setpos_retract[1]!=0 && state.setpos_trigger[1]!=0 && state.setpos_endstop[1]!=0                 
            ); //do timed latch if all setpoints are nonzero
            state.disable_o32_comm = 0; //allow motor comm because motor_pos still valid
        }
    }

    //BLDC POS CALIBRATION
    if (!NO_SIGNAL && calib_state && elapsed < 10000) { // calibration happens when calib_state != 0. If it takes more than 10 seconds, continue to operation
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

        //Calibration prints
        if (print_timer > 10 && Serial.availableForWrite()) {
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

        return; // don't do other stuff if calibration not complete
    }



    // SET BLDC AND SERVO POWERS
    if(!NO_SIGNAL){
        if(!state.disable_o32_comm){
            if(state.do_timed_latch){ //timed latch control

                state.sent_cmd_A = cmd.motor_pwrs[0]; //default is apply power
                state.sent_cmd_B = cmd.motor_pwrs[1];

                if(state.encposA < state.setpos_endstop[0] && state.encposB < state.setpos_endstop[1]){ //check if finished

                    if(state.encposA < state.setpos_trigger[0] && state.encposB > state.setpos_trigger[1]){
                        state.sent_cmd_B = 0; //have B wait for A
                    }else if(state.encposA > state.setpos_trigger[0] && state.encposB < state.setpos_trigger[1])
                    if(state.encposB < state.setpos_trigger[1]){
                        state.sent_cmd_A = 0; //have A wait for B
                    }

                }else{
                    state.sent_cmd_A = 0;
                    state.sent_cmd_B = 0;
                    state.do_timed_latch = 0;
                }

            }else{ //proportional setpoint control
                // int16_t state.sent_cmd_A = (DO_CALIB && state.pos_A > calib_pos_A) ? 0 : cmd.motor_pwrs[0];
                // int16_t state.sent_cmd_B = (DO_CALIB && state.pos_B > calib_pos_B) ? 0 : cmd.motor_pwrs[1];

                float KP_A = -512/6000.0f;
                float KP_B = -512/6000.0f;
            
                state.sent_cmd_A = KP_A * (cmd.motor_pos[0] - state.encposA);
                state.sent_cmd_B = KP_B * (cmd.motor_pos[1] - state.encposB);
                
            }

            state.sent_cmd_A = clip(state.sent_cmd_A, -abs(cmd.motor_pwrs[0]), abs(cmd.motor_pwrs[0]));
            if(abs(state.sent_cmd_A) < 20) state.sent_cmd_A = 0;
            state.sent_cmd_B = clip(state.sent_cmd_B, -abs(cmd.motor_pwrs[1]), abs(cmd.motor_pwrs[1])); //should be in [-511, 511]
            if(abs(state.sent_cmd_B) < 20) state.sent_cmd_B = 0;
            
            // motA_nbytes = send_O32_cmd(0xA, CMD_SET_VOLTAGE, twoscomplement14(0), motA_rx);
            motA_nbytes = send_O32_cmd(0xA, CMD_SET_VOLTAGE, twoscomplement14(state.sent_cmd_A), motA_rx);
            state.pos_A = pad28(motA_rx[0], motA_rx[1], motA_rx[2], motA_rx[3]);

            // motB_nbytes = send_O32_cmd(0xB, CMD_SET_VOLTAGE, twoscomplement14(0), motB_rx);
            motB_nbytes = send_O32_cmd(0xB, CMD_SET_VOLTAGE, twoscomplement14(state.sent_cmd_B), motB_rx);
            state.pos_B = pad28(motB_rx[0], motB_rx[1], motB_rx[2], motB_rx[3]);
        }

        dxl_write(dxl_ids[0], 116, cmd.servos[0]);//set position
        dxl_write(dxl_ids[1], 116, cmd.servos[1]); 
        dxl_write(dxl_ids[2], 116, cmd.servos[2]);
        dxl_write(dxl_ids[3], 116, cmd.servos[3]);
        dxl_write(dxl_ids[4], 116, cmd.servos[4]);
    }else{
        state.no_signal_cnt++;
    }


    // GET BLDC DATA
    if (motA_nbytes == 9) {
        state.rpmA = pad14(motA_rx[4], motA_rx[5]);
        state.temp_ntcA = motA_rx[6];
        state.vbusA = pad14(motA_rx[7], motA_rx[8]) * 3.3*5.1020f/4096.;
        o32cnt++;
    }else{
        state.rpmA = 0;
        state.temp_ntcA = 0;
        state.vbusA = 0;
    }
    if (motB_nbytes == 9) {
        state.rpmB = pad14(motB_rx[4], motB_rx[5]);
        state.temp_ntcB = motB_rx[6];
        state.vbusB = pad14(motB_rx[7], motB_rx[8]) * 3.3*4.9575f/4096.;
    }else{
        state.rpmB = 0;
        state.temp_ntcB = 0;
        state.vbusB = 0;
    }
    if(state.vbusA!=0 && state.vbusB!=0){
        state.vbus = (state.vbusA+state.vbusB)/2.;
    }else{
        state.vbus = 0;
    }


    //TODO: MOVE INSIDE INTRRUPT TO AVOID STRING STUFF EVERY LOOP
    // Convert ESP-now message to motor commands
    if (receivedString.indexOf("s0") != -1) {
        cmd.aux_prev = cmd.aux;
        // stupid way to read recieved string but oh well gotta go fast
        cmd.servos[0] = receivedString.substring(receivedString.indexOf("s0") + 3, receivedString.indexOf("s0") + 8).toInt();
        cmd.servos[1] = receivedString.substring(receivedString.indexOf("s1") + 3, receivedString.indexOf("s1") + 8).toInt();
        cmd.servos[2] = receivedString.substring(receivedString.indexOf("s2") + 3, receivedString.indexOf("s2") + 8).toInt();
        cmd.servos[3] = receivedString.substring(receivedString.indexOf("s3") + 3, receivedString.indexOf("s3") + 8).toInt();
        cmd.servos[4] = receivedString.substring(receivedString.indexOf("s4") + 3, receivedString.indexOf("s4") + 8).toInt();
        cmd.motor_pwrs[0] = receivedString.substring(receivedString.indexOf("mw0") + 4, receivedString.indexOf("mw0") + 9).toInt();
        cmd.motor_pwrs[1] = receivedString.substring(receivedString.indexOf("mw1") + 4, receivedString.indexOf("mw1") + 9).toInt();
        cmd.motor_pos[0] = 100 * receivedString.substring(receivedString.indexOf("mo0") + 4, receivedString.indexOf("mo0") + 9).toInt();
        cmd.motor_pos[1] = 100 * receivedString.substring(receivedString.indexOf("mo1") + 4, receivedString.indexOf("mo1") + 9).toInt();
        cmd.aux = receivedString.substring(receivedString.indexOf("a") + 2, receivedString.indexOf("a") + 7).toInt();

    }


    // JOYSTICK WATCHDOG
    if (recv_watchdog > 200) { // didn't receive anything from joystick in a while
        NO_SIGNAL = true; // don't drive motors

        reset_cmd(cmd);
        state.sent_cmd_A = 0;
        state.sent_cmd_B = 0;
        motA_nbytes = send_O32_cmd(0xA, CMD_SET_VOLTAGE, 0, motA_rx);
        motB_nbytes = send_O32_cmd(0xB, CMD_SET_VOLTAGE, 0, motB_rx);
        set_dxl_torque_off();
    }

    //REENABLE MOTORS
    if (enable_motors_flag) {
        enable_motors_flag = 0;
        delay(100);
        set_dxl_torque_on();
    }


    //TELEMETRY
    if(telemetry_timer > 1){
        telemetry_timer = 0;
        size_t state_str_size = sprintf(state_str,
            // "motor_power_on:%d\n"
            // "dxl_torque_on:%d\n"
            "d0:%d\n"
            "d1:%d\n"
            "d2:%d\n"
            "d3:%d\n"
            "d4:%d\n"
            // "ntcA:%d\n"
            // "ntcB:%d\n"
            "v:%.2f\n"
            "pA:%d\n"
            "pB:%d\n"
            "mA:%d\n"
            "mB:%d\n"
            "eA:%d\n"
            "eB:%d\n"
            "t:%d\n"
            "I:%.2f\n"
            ,
            // state.motor_power_on,
            // state.dxl_torque_on,
            state.dxl_pos[0],
            state.dxl_pos[1],
            state.dxl_pos[2],
            state.dxl_pos[3],
            state.dxl_pos[4],
            // state.temp_ntcA,
            // state.temp_ntcB,
            state.vbus,
            state.sent_cmd_A,
            state.sent_cmd_B,
            state.pos_A,
            state.pos_B,
            state.encposA,
            state.encposB,
            state.elapsed,
            state.cur_tot
        );



        // char logbuf[200] = {0};
        // memcpy(logbuf, &state, sizeof(state));    

        // Serial.printf("%d\n", elapsed);

        // for(int i = 0; i < sizeof(state); ++i){
        //     Serial.printf("%02X ", (unsigned char)logbuf[i]);
        // }
        // Serial.println('\t');



        // esp_now_send(estop_mac_addr, (uint8_t *) logbuf, sizeof(state));
        esp_now_send(estop_mac_addr, (uint8_t *) state_str, state_str_size);
    }

    
    // SERIAL PRINT OUT
    if (print_timer > 10 && Serial.availableForWrite()) {
        print_timer = 0;

        // Serial.printf("%d\n", sizeof(state));
        Serial.printf("ton n8 jery \n");

        // memset(print_buf,0,strlen(print_buf));


        // sprintf(
        //     print_buf,

        //     // "elapsed: %ld\n"
        //     "looptime: %d\n"
        //     // "cur_tot: %.2f\n"
        //     // "calib_pos_A: %ld\n"
        //     // "calib_pos_B: %ld\n"

        //     // "s[0]: %ld\n"
        //     // "s[1]: %ld\n"
        //     // "s[2]: %ld\n"
        //     // "s[3]: %ld\n"
        //     // "s[4]: %ld\n"
        //     "mw[0]: %ld\n"
        //     "mw[1]: %ld\n"
        //     "mo[0]: %ld\n"
        //     "mo[1]: %ld\n"
        //     // "aux: %d\n"

        //     // "nosigcnt: %d\n"

        //     "ntcA: %d\n"
        //     "ntcB: %d\n"
        //     "vbus: %.2f\n"

        //     "encposB: %ld\n"
        //     "encposA: %ld\n"

        //     // "sentA: %d\n"
        //     // "sentB: %d\n"

        //     "acc0: %.2f\n"
        //     "acc1: %.2f\n"
        //     "acc2: %.2f\n"
        //     "rate0: %.2f\n"
        //     "rate1: %.2f\n"
        //     "rate2: %.2f\n"
        //     // "ang_gyro[0] : %f\n"
        //     // "ang_gyro[1] : %f\n"
        //     // "ang_gyro[2] : %f\n"
        //     // "ang_acc[0]: %f\n"
        //     // "ang_acc[1]: %f\n"

        //     // "dxl_pos[0]: %ld\n"
        //     // "dxl_pos[1]: %ld\n"
        //     // "dxl_pos[2]: %ld\n"
        //     // "dxl_pos[3]: %ld\n"
        //     // "dxl_pos[4]: %ld\n"
        //     // "dxl_torque_on: %ld\t\n",
        //     "\t\n",

        //     looptime,

        //     // cmd.servos[0],
        //     // cmd.servos[1],
        //     // cmd.servos[2],
        //     // cmd.servos[3],
        //     // cmd.servos[4],
        //     cmd.motor_pwrs[0],
        //     cmd.motor_pwrs[1],
        //     cmd.motor_pos[0],
        //     cmd.motor_pos[1],
        //     // cmd.aux,

        //     // state.no_signal_cnt,

        //     state.temp_ntcA,
        //     state.temp_ntcB,
        //     state.vbus,

        //     state.encposB,
        //     state.encposA,

        //     // state.sent_cmd_A,
        //     // state.sent_cmd_B,

        //     state.acc[0],
        //     state.acc[1],
        //     state.acc[2],
        //     state.ang_rate[0],
        //     state.ang_rate[1],
        //     state.ang_rate[2]
        //     // state.ang_gyro[0],
        //     // state.ang_gyro[1],
        //     // state.ang_gyro[2],
        //     // state.ang_acc[0],
        //     // state.ang_acc[1],

        //     // state.dxl_pos[0],
        //     // state.dxl_pos[1],
        //     // state.dxl_pos[2],
        //     // state.dxl_pos[3],
        //     // state.dxl_pos[4]
        //     // state.dxl_torque_on
        // );

        // char logbuf[200] = {0};
        // memcpy(logbuf, &state, sizeof(state));        
        
        // if(NO_SIGNAL) Serial.println("No signal to ESTOP ~~~~~~~~~~~~~~~~~~~~");
        // Serial.println(o32cnt);
        // Serial.write(print_buf);

        // Serial.printf("%d\n", elapsed);

        // for(int i = 0; i < sizeof(state); ++i){
        //     Serial.printf("%02X ", (unsigned char)logbuf[i]);
        // }
        // Serial.println('\t');
        // Serial.printf()
    }



    // SERIAL COMMAND
    if (Serial.available()) {
        char user_cmd = Serial.read();
        switch (user_cmd) {
        case 't':
            if (state.dxl_torque_on){
                set_dxl_torque_off();
            }else{
                set_dxl_torque_on();
            }
            break;
        case 'm':
            if (state.motor_power_on){
                digitalWrite(MOTOR_ON, LOW);
                state.motor_power_on = false;
            }else{
                digitalWrite(MOTOR_ON, HIGH);
                state.motor_power_on = true;
            }
            break;
        case 'z':
            state.encposB_offset = state.encposB_raw; // zero ecnoder
            state.encposA_offset = state.encposA_raw; // zero ecnoder
            break;
        case 'l': // example: "l21" sets led on encoder 2 to high
            {
                if(!Serial.available()) break;
                uint8_t enc_addr = Serial.read() - '0';
                if(!Serial.available()) break;
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
        while (Serial.available()) Serial.read(); //clear RX buffer
    }


}








//FUNCTIONS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


void reset_cmd(cmd_struct _cmd) {
    _cmd.servos[0] = 2048;
    _cmd.servos[1] = 2048;
    _cmd.servos[2] = 2048;
    _cmd.servos[3] = 2048;
    _cmd.servos[4] = 2048;
    _cmd.motor_pwrs[0] = 0;
    _cmd.motor_pwrs[1] = 0;
    _cmd.aux = 0;
    _cmd.aux_prev = 0;
}

uint8_t send_O32_cmd(uint8_t addr, uint8_t CMD_TYPE, uint16_t data, uint8_t *rx) {
    while (rs485_0.available()) rs485_0.read(); // clear rx buffer

    uint8_t uart2_TX[3] = {0}; // command RS485 to Ø32
    uart2_TX[0] = CMD_TYPE | addr;
    uart2_TX[1] = (data >> 7) & 0b01111111;
    uart2_TX[2] = (data) & 0b01111111;

    rs485_0.write(uart2_TX, 3);
    rs485_0.flush();
    delayMicroseconds(200); // should be enough for up to 20 bytes of response at 1Mbaud
    // uint8_t numread = rs485_0.readBytesUntil(MIN_INT8, rx, 10); //don't use because data itself might contain MIN_INT8
    uint8_t numread = rs485_0.readBytes(rx, 9);
    return numread;
}

uint8_t send_rs485_cmd(HardwareSerial *rs485, uint8_t addr, uint8_t CMD_TYPE, uint16_t data, uint8_t *rx, uint8_t bytes_to_read, uint16_t delay) {
    while (rs485->available()) rs485->read(); // clear rx buffer

    uint8_t uart2_TX[3] = {0}; // command RS485 to Ø32
    uart2_TX[0] = CMD_TYPE | addr;
    uart2_TX[1] = (data >> 7) & 0b01111111;
    uart2_TX[2] = (data) & 0b01111111;

    rs485->write(uart2_TX, 3);
    rs485->flush();
    // delayMicroseconds(82 + bytes_to_read*10); // give each byte 10us if 1Mbaud
    delayMicroseconds(delay + bytes_to_read*10);
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

// doesn't work right now
void dxl_syncwrite(uint32_t reg_addr, uint8_t bytes_to_write, uint32_t *data) { // bytes_to_write max 4 bytes because data is 32bit
    dxl_tx[0] = 0xFF;                                                           // header
    dxl_tx[1] = 0xFF;                                                           // header
    dxl_tx[2] = 0xFD;                                                           // header
    dxl_tx[3] = 0x00;                                                           // reserved
    dxl_tx[4] = 0xFE;                                                           // ID: broadcast

    uint8_t param_len = sizeof(dxl_ids) * (bytes_to_write + 1);

    dxl_tx[5] = (param_len + 7); // length L (length of instruction, parameters, and crc)
    dxl_tx[6] = 0x00;            // length H

    dxl_tx[7] = 0x83; // instruction (write)

    dxl_tx[8] = (uint8_t)(reg_addr & 0x00FF);        // P1: register address L
    dxl_tx[9] = (uint8_t)((reg_addr >> 8) & 0x00FF); // P2: register address H

    dxl_tx[10] = (uint8_t)(bytes_to_write & 0x00FF);        // P3: num bytes L
    dxl_tx[11] = (uint8_t)((bytes_to_write >> 8) & 0x00FF); // P4: num bytes H

    for (uint8_t i = 0; i < sizeof(dxl_ids); i++) {
        dxl_tx[12 + i * (bytes_to_write + 1)] = (uint8_t)(dxl_ids[i] & 0x00FF); // P5: target device ID
        for (uint8_t j = 0; j < bytes_to_write; j++) {
            dxl_tx[13 + i * (bytes_to_write + 1) + j] = (uint8_t)((data[i] >> (j * 8)) & 0x00FF); // P6: target device ID
        }
    }

    uint16_t CRC = update_crc(0, dxl_tx, param_len + 12); // number of bytes until now including header
    uint8_t CRC_L = (uint8_t)(CRC & 0x00FF);              // Little-endian
    uint8_t CRC_H = (uint8_t)((CRC >> 8) & 0x00FF);

    dxl_tx[param_len + 12] = CRC_L;
    dxl_tx[param_len + 13] = CRC_H;

    dxl_serial.write(dxl_tx, param_len + 14);
    delayMicroseconds(500);
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
