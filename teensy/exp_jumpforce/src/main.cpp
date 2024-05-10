#include "SPI.h"
#include <ADS1256.h>
#include <Arduino.h>
#include <stdlib.h>

#define PIN_CS 3
#define PIN_DRDY 4
#define PIN_SYNC 5
#define CLOCK_MHZ 7.68 // crystal frequency used on ADS1256
#define VREF 2.5       // voltage reference

ADS1256 adc = ADS1256(PIN_CS, PIN_DRDY, PIN_SYNC, CLOCK_MHZ, VREF);

float N_per_mV[4] = {
    1,
    9.81*0.561/-0.4190,
    1,
    9.81*0.463/0.34569 //calibrated 4/25/24 by Chris with waterbottle and solder
};
//561/-0.1072

float mV_zero[4] = {0};
float adc_mV[4] = {0}; //millivolts raw
float adc_N[4] = {0}; //newtons with corrected scale and offset


void zero_adc() {
    Serial.println("Zeroing start");
    digitalWrite(LED_BUILTIN, LOW);

    for (int samples = 100; samples > 0; samples--) {

        adc.waitDRDY();
        adc.setChannel(0, 1);
        adc.readCurrentChannel(); //throw away first data, its bad for some reason

        adc.waitDRDY();
        adc.setChannel(2, 3); //set channel for the next reading
        adc_mV[0] = 1000*adc.readCurrentChannel(); // data from 12

        adc.waitDRDY(); 
        adc.setChannel(4, 5);
        adc_mV[1] = 1000*adc.readCurrentChannel(); //data from 23

        adc.waitDRDY();
        adc.setChannel(6, 7);
        adc_mV[2] = 1000*adc.readCurrentChannel(); //data from 45

        adc.waitDRDY();
        // adc.setChannel(6,7);
        adc_mV[3] = 1000*adc.readCurrentChannel(); //data from 67

    
        float alpha = 0.1;
        for (int i = 0; i < 4; i++) {
            mV_zero[i] = alpha * adc_mV[i] + (1 - alpha) * mV_zero[i];
        }

        Serial.print("adc01: ");    
        Serial.println(adc_mV[0], 10);
        Serial.print("adc23: ");    
        Serial.println(adc_mV[1], 10);
        Serial.print("adc45: ");    
        Serial.println(adc_mV[2], 10);
        Serial.print("adc67: ");    
        Serial.println(adc_mV[3], 10);
        Serial.print("zero01: ");    
        Serial.println(mV_zero[0], 10);
        Serial.print("zero23: ");    
        Serial.println(mV_zero[1], 10);
        Serial.print("zero45: ");    
        Serial.println(mV_zero[2], 10);
        Serial.print("zero67: ");    
        Serial.println(mV_zero[3], 10);

        Serial.print("\t\n");

        delay(1);
    }

    Serial.println("Zeroing complete");
    adc.setChannel(0, 1);
    digitalWrite(LED_BUILTIN, HIGH);
}



void setup() {
    Serial.begin(1000000);

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.println("Starting ADC");
    adc.begin(ADS1256_DRATE_7500SPS, ADS1256_GAIN_64, false);

    zero_adc();
    adc.setChannel(0, 1);
}

elapsedMillis elapsed;

unsigned long startTime = 0;

void loop() {

    unsigned long duration = micros() - startTime;
    startTime = micros();

    

    // adc.waitDRDY();
    // adc.setChannel(2, 3); //set channel for the next reading
    // adc_mV[0] = 1000*adc.readCurrentChannel(); // data from 12

    adc.waitDRDY(); 
    adc.setChannel(6, 7);
    adc_mV[1] = 1000*adc.readCurrentChannel(); //data from 23

    // adc.waitDRDY();
    // adc.setChannel(6, 7);
    // adc_mV[2] = 1000*adc.readCurrentChannel(); //data from 45

    adc.waitDRDY();
    adc.setChannel(2, 3);
    adc_mV[3] = 1000*adc.readCurrentChannel(); //data from 67

    float alpha_N = 0.1;
    for(int i=0; i<4; i++){
        adc_N[i] = alpha_N*N_per_mV[i]*(adc_mV[i] - mV_zero[i]) + (1-alpha_N)*adc_N[i];
    }
    // Serial.print("adc01: ");    
    // Serial.println(adc_mV[0], 10);
    // Serial.print("adc23: ");    
    // Serial.println(adc_mV[1], 10);
    // Serial.print("adc45: ");    
    // Serial.println(adc_mV[2], 10);
    // Serial.print("adc67: ");    
    // Serial.println(adc_mV[3], 10);
    // Serial.print("zero01: ");    
    // Serial.println(mV_zero[0], 10);
    // Serial.print("zero23: ");    
    // Serial.println(mV_zero[1], 10);
    // Serial.print("zero45: ");    
    // Serial.println(mV_zero[2], 10);
    // Serial.print("zero67: ");    
    // Serial.println(mV_zero[3], 10);

    Serial.print("elapsed: ");
    Serial.println(elapsed);

    Serial.print("looptime: ");
    Serial.println(duration);

    // Serial.print("N01: ");    
    // Serial.println(adc_N[0], 10);
    Serial.print("N23: ");    
    Serial.println(adc_N[1], 10);
    // Serial.print("N45: ");    
    // Serial.println(adc_N[2], 10);
    Serial.print("N67: ");    
    Serial.println(adc_N[3], 10);

    Serial.print("\t\n");
    if (Serial.available() > 0) {
        if (Serial.read() == 'z') {
            zero_adc();
            while (Serial.available()) {
                Serial.read();
            }
        }
    }
}