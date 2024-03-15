#include <cstdint>
#include <stdio.h>



int main(int argc, char const *argv[])
{
    uint8_t dxl_tx[200] = {0};
    uint8_t dxl_ids[] = {
        3, // right back
        4, // right front
        5, // left front
        6, // left back
        7  // roll
    };

    uint32_t reg_addr = 13;
    
    uint8_t bytes_to_write = 4;
    
    uint32_t data[] = {12,32,43,45};


    dxl_tx[0] = 0xFF; // header
    dxl_tx[1] = 0xFF; // header
    dxl_tx[2] = 0xFD; // header
    dxl_tx[3] = 0x00; // reserved
    dxl_tx[4] = 0xFE; // ID: broadcast

    uint8_t param_len = sizeof(dxl_ids) * (bytes_to_write + 1);

    dxl_tx[5] = 0x09; // length L (length of instruction, parameters, and crc)
    dxl_tx[6] = 0x00; // length H

    dxl_tx[7] = 0x03; // instruction (write)

    dxl_tx[8] = (uint8_t)(reg_addr & 0x00FF);        // P1: register address L
    dxl_tx[9] = (uint8_t)((reg_addr >> 8) & 0x00FF); // P2: register address H

    dxl_tx[10] = (uint8_t)(bytes_to_write & 0x00FF);         // P3: num bytes L
    dxl_tx[11] = (uint8_t)((bytes_to_write >> 8) & 0x00FF);  // P4: num bytes H

    for(uint8_t i=0; i<sizeof(dxl_ids); i++){
        dxl_tx[12 + i*(bytes_to_write+1)] = (uint8_t)(dxl_ids[i] & 0x00FF);  // P5: target device ID
        for(uint8_t j=0; j<bytes_to_write; j++){
            dxl_tx[13 + i*(bytes_to_write+1) + j] = (uint8_t)((data[i]>>(j*8)) & 0x00FF);  // P6: target device ID
        }
        printf("filling %d\n", i);
    }
    


    for(int i=0; i<30; i++){
        printf("dxl_tx[%d]: %d", i, dxl_tx[i]);
    }
}
