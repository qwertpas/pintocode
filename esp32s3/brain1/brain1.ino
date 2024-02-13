// Need this for the lower level access to set them up.
#include <HardwareSerial.h>
#include "hal/uart_types.h" //includes RS485 definition

#define MOTOR_ON 1
#define GPIO_D1 2 //breakout GPIO
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


HardwareSerial hwserial0(0);
HardwareSerial hwserial1(1);
HardwareSerial hwserial2(2);


void setup()
{
    // For the USB, just use Serial as normal:
    Serial.begin(115200);
    Serial.setTimeout(1);

    hwserial0.begin(115200, SERIAL_8N1); //baudrate, parity
    hwserial0.setPins(UART0_RX, UART0_TX, -1, UART0_DE); //RX, TX, CTS, RTS (DE)
    hwserial0.setMode(UART_MODE_RS485_HALF_DUPLEX);
    hwserial0.print("hwserial0");

    hwserial1.begin(115200, SERIAL_8N1);
    hwserial1.setPins(UART1_RX, UART1_TX, -1, UART1_DE);
    hwserial1.setMode(UART_MODE_RS485_HALF_DUPLEX);
    hwserial1.print("hwserial1");

    hwserial2.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);

    pinMode(LED_BUILTIN, OUTPUT);
    
    pinMode(MOTOR_ON, OUTPUT);
    digitalWrite(MOTOR_ON, HIGH);

//    pinMode(GPIO_D1, OUTPUT);
//    digitalWrite(GPIO_D1, LOW);

//    pinMode(UART0_DE, OUTPUT);
//    digitalWrite(UART0_DE, HIGH);
//
//    pinMode(UART1_DE, OUTPUT);
//    digitalWrite(UART1_DE, HIGH);

//    timer0 = timerBegin(0, 80, true); //80MHz clock, 80div=1us precision, true=count up
//    timerAttachInterrupt(timer0, &timer0_ISR, true); //true=edgetriggered
//    timerAlarmWrite(timer0, 50000, true); //microseconds, true=autoreload
//    timerAlarmEnable(timer0); //start counting
}

void loop()
{
  
    digitalWrite(LED_BUILTIN, LED_UNLIT);
    delay(10);
  
    digitalWrite(LED_BUILTIN, LED_LIT);

    Serial.println("AAA");
    hwserial0.println("BBB");
    hwserial1.println("CCC");
    hwserial2.println("DDD");

    delay(10);
    

}


//uint8_t send_motor_cmd(uint8_t addr, uint8_t CMD_TYPE, uint16_t data){
////  while(Serial1.available()) Serial1.read(); //clear rx buffer
//
//  uint8_t uart2_TX[3] = {0}; //command RS485 to Ø32
//  uart2_TX[0] = CMD_TYPE | addr;
//  uart2_TX[1] = (data >> 7) & 0b01111111;
//  uart2_TX[2] = (data)      & 0b01111111;
//
//  digitalWrite(RS485_DE, HIGH);
//  digitalWrite(RS485_RE, HIGH);
//  Serial1.write(uart2_TX, 3);
//  Serial1.flush();
//  digitalWrite(RS485_DE, LOW);
//  digitalWrite(RS485_RE, LOW);
////  int numread = Serial1.readBytesUntil(MIN_INT8, rx, 10);
////  return numread; 
//  return 1;
//}
