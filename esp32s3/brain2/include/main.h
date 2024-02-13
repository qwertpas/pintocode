#include <stdlib.h>
#include "hal/uart_types.h" //includes RS485 definition
#include <HardwareSerial.h>
#include <esp_now.h>
#include <WiFi.h>
#include <elapsedMillis.h>

#include "comdef.h"
#include "util.h"



//for RS485 bus
uint8_t send_O32_cmd(HardwareSerial serial, uint8_t addr, uint8_t CMD_TYPE, uint16_t data, uint8_t *rx);

//for dynamixels
void dxl_write(uint32_t addr, uint32_t value);
void update_dxl();
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
