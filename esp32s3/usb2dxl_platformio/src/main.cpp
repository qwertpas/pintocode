#include <Arduino.h>
#include <HardwareSerial.h>
#include <hal/uart_types.h> //includes RS485 definition

void update_dxl();

//sqrlbrain board pinout
#define MOTOR_ON 1
#define GPIO_D1 2 //breakout GPIO
#define UART1_DE 3
#define UART1_TX 4
#define UART1_RX 5
#define UART0_DE 6
#define UART0_TX 43
#define UART0_RX 44
#define UART2_DE 7
#define UART2_TX 8 //dynamixel 
#define UART2_RX 9 //dynamixel
#define LED_LIT LOW
#define LED_UNLIT HIGH

#define CMD_PORT Serial //USB
HardwareSerial DBG_PORT(1); //for debugging
HardwareSerial DXL_PORT(2); //comm with dynamixel using usual connector

#define DXL_TX_BUFFER_LENGTH  1024
unsigned char tx_buffer[DXL_TX_BUFFER_LENGTH];


void setup() {
  CMD_PORT.begin(115200);
  CMD_PORT.setTimeout(1);

  //have to update this each time baud rate is changed in wizard
  // DXL_PORT.begin(57600, SERIAL_8N1, UART2_RX, UART2_TX);
  DXL_PORT.begin(1000000, SERIAL_8N1, UART2_RX, UART2_TX);
  DXL_PORT.setPins(UART2_RX, UART2_TX, GPIO_D1, UART2_DE); // CTS pin should be an unused GPIO, otherwise USB serial disappears
  DXL_PORT.setMode(UART_MODE_RS485_HALF_DUPLEX);
  DXL_PORT.setTimeout(1);


  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LED_LIT);

  pinMode(MOTOR_ON, OUTPUT);
  digitalWrite(MOTOR_ON, HIGH);

  
  delay(10);
}

void loop() {

  //doesn't seem to detect when CMD_PORT changes baudrate
  // CMD_PORT.begin(0);
  // if( CMD_PORT.baudRate() != DXL_PORT.baudRate() ){
  //   // DXL_PORT.end();
  //   DXL_PORT.updateBaudRate(CMD_PORT.baudRate());
  // }

  //passthrough
  update_dxl();


}


void update_dxl(){
  int length;
  int i;

  //-- USB -> DXL
  length = CMD_PORT.available();
  if( length > 0 ) {
    for(i=0; i<length; i++) {
      DXL_PORT.write(CMD_PORT.read());
      DXL_PORT.flush();
    }
  }

  //-- DXL -> USB
  length = DXL_PORT.available();
  if( length > 0 ) {
    if( length > DXL_TX_BUFFER_LENGTH ) {
      length = DXL_TX_BUFFER_LENGTH;
    }
    for(i=0; i<length; i++) {
      tx_buffer[i] = DXL_PORT.read();
    }
    CMD_PORT.write(tx_buffer, length);
  }
}