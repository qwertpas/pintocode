#include <stdlib.h>
#define MIN_INT8 (0x80) //most negative int8

int16_t clip(int16_t x, int16_t min, int16_t max) {
  if (x > max) {
    return max;
  } else if (x < min) {
    return min;
  } else {
    return x;
  }
}

uint16_t deadband(int16_t x, int16_t deadband) {
  if (abs(x) < deadband){
    return 0;
  }else if(x > 0){
    return x - deadband;
  }else{ //x is negative
    return x + deadband;
  }
}

int16_t twoscomplement14(int16_t num16) {
  // Convert a twos complement int16_t into a twos complement 14 bit number
  int16_t num14 = num16 & 0x3FFF;
  if (num16 & 0x8000) {
    num14 = -((~num14) + 1);
  }
  return num14;
}

int16_t pad14(uint8_t num7_0, uint8_t num7_1) {
  int16_t res = (num7_0 << 7) | (num7_1);
  if (res & 0x2000) return res | 0xC000;
  else return res;
}

int32_t pad28(uint8_t num7_0, uint8_t num7_1, uint8_t num7_2, uint8_t num7_3) {
  int32_t res = (num7_0 << 21) | (num7_1 << 14) | (num7_2 << 7) | (num7_3);
  if (res & 0x08000000) return res | 0xF8000000;
  else return res;
}

int32_t min(int32_t a, int32_t b){
  return (a<b) ? a : b; 
}

int32_t max(int32_t a, int32_t b){
  return (a>b) ? a : b; 
}
