#include <ADS1256.h>
#include <Arduino.h>
#include <stdlib.h>
#include "SPI.h"


#define CS 3

ADS1256 A(4, 0, 5, CS, 2.500); // DRDY, RESET, SYNC(PDWN), CS, VREF(float).    //Teensy 4.0 mine

float N_per_mV = 2 / 0.00199;
float mV_zero[4] = {0};

void calibrate_force_angle() {
    Serial.println("Initializing ADS1256");

    while(A.readRegister(DRATE_REG) != DRATE_1000SPS){
        Serial.println("Setting data reg");
        Serial.println(A.readRegister(DRATE_REG));
        A.setDRATE(DRATE_1000SPS);
    }
    A.setPGA(PGA_64);
    // while((A.readRegister(IO_REG) & 0b00000111) != PGA_64){
    //     Serial.println("Setting pga reg: ");
    //     Serial.println((A.readRegister(IO_REG)));
    //     A.setPGA(PGA_64);
    // }

    //   digitalWrite(CS, LOW);
    // digitalWrite(CS, HIGH);
    // delay(100);
    // digitalWrite(CS, LOW);
    // A.setPGA(PGA_64); //224
    // digitalWrite(CS, HIGH);
    // delay(10);
    // digitalWrite(CS, LOW);
    // A.setPGA(PGA_64); //224
    // digitalWrite(CS, HIGH);
    // delay(100);
    // digitalWrite(CS, LOW);
    // A.setMUX(DIFF_0_1); 
    // digitalWrite(CS, HIGH);
    // delay(100);
    // digitalWrite(CS, LOW);
    // A.setDRATE(DRATE_1000SPS); //146
    // digitalWrite(CS, HIGH);
    // delay(10);
    // digitalWrite(CS, LOW);
    // A.setDRATE(DRATE_1000SPS); //146
    // digitalWrite(CS, HIGH);
    // delay(100);



    float alpha = 0.5;
    for (int samples = 20; samples > 0;) {

        Serial.print("calibrating: ");

        for (int i = 0; i < 4; i++) {
            float mV = 1000*A.convertToVoltage(A.cycleDifferential());
            mV_zero[i] = alpha * mV + (1 - alpha) * mV_zero[i];

            Serial.print("mv_zero[");
            Serial.print(i);
            Serial.print("]: ");
            Serial.println(mV_zero[i], 6);
        }


        Serial.print("\t\n");
        samples--;

        delay(100);
    }
    A.stopConversion();
}

void setup() {
    Serial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(CS, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    A.InitializeADC();

    calibrate_force_angle();

    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(CS, LOW);
}

elapsedMillis sample_timer;
elapsedMillis elapsed;

void loop() {
    if(Serial.available() > 0){
        if(Serial.read() == 'c'){
            calibrate_force_angle();
            while(Serial.available()){Serial.read();}
        }
    }
    if (sample_timer > 2) {
        sample_timer = 0;

        Serial.print("elapsed: ");
        Serial.println(elapsed);

        for (int i = 0; i < 4; i++) {
            Serial.print("force[");
            Serial.print(i);
            Serial.print("]: ");
            Serial.println(1000*A.convertToVoltage(A.cycleDifferential()) - mV_zero[i], 4);
        }
        Serial.println("\t\n");
    }
}