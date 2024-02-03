// Need this for the lower level access to set them up.
#include <HardwareSerial.h>

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
#define LIT LOW
#define UNLIT HIGH


//Define two Serial devices mapped to the two internal UARTs
HardwareSerial MySerial0(0);
HardwareSerial MySerial1(1);
HardwareSerial MySerial2(2);


hw_timer_t *timer0 = NULL;
void IRAM_ATTR timer0_ISR()
{
    digitalWrite(GPIO_D1, !digitalRead(GPIO_D1));
    timerStop(timer0);

}


void setup()
{
    // For the USB, just use Serial as normal:
    Serial.begin(115200);
    Serial.setTimeout(1);

    // SerialX.begin(baudrate, parity, RX pin, TX pin);
    MySerial0.begin(115200, SERIAL_8N1, UART0_RX, UART0_TX);
    MySerial0.print("MySerial0");

    MySerial1.begin(115200, SERIAL_8N1, UART1_RX, UART1_TX);
    MySerial1.print("MySerial1");

    MySerial2.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);

    pinMode(LED_BUILTIN, OUTPUT);
    
    pinMode(MOTOR_ON, OUTPUT);
    digitalWrite(MOTOR_ON, LOW);

    pinMode(GPIO_D1, OUTPUT);
    digitalWrite(GPIO_D1, LOW);

    pinMode(UART0_DE, OUTPUT);
    digitalWrite(UART0_DE, HIGH);

    pinMode(UART1_DE, OUTPUT);
    digitalWrite(UART1_DE, HIGH);

    timer0 = timerBegin(0, 80, true); //80MHz clock, 80div=1us precision, true=count up
    timerAttachInterrupt(timer0, &timer0_ISR, true); //true=edgetriggered
    timerAlarmWrite(timer0, 50000, true); //microseconds, true=autoreload
    timerAlarmEnable(timer0); //start counting
}

void loop()
{
  
    digitalWrite(LED_BUILTIN, UNLIT);
    delay(10);
  
    digitalWrite(LED_BUILTIN, LIT);

    Serial.println("AAA");
    MySerial0.println("BBB");
    MySerial1.println("CCC");
    MySerial2.println("DDD");

    delay(10);
    



}
