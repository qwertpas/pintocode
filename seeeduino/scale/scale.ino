#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 8;
const int LOADCELL_SCK_PIN = 7;

float N_per_ct = 270/527200.38;
float N_zero = 0;

HX711 scale;
int isCalibrated = 0;

void calibrate(){
  isCalibrated = 0;
  float alpha = 0.5;
  for(int samples = 20; samples > 0;){
    if (scale.is_ready()) {
      float newtons = scale.read() * N_per_ct;
      N_zero = alpha*newtons + (1-alpha)*N_zero;
      Serial.print("calibrating... ");
      Serial.println(N_zero,3);
      samples--;
    }
  }
  Serial.print("calibrated: ");
  Serial.println(N_zero,3);
  isCalibrated = 1;
}

void setup() {
  Serial.begin(9600);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  pinMode(9, INPUT_PULLUP);
  calibrate();
}

float reading_filt = 0;
float filt_alpha = 0.5;

void loop() {

  if (scale.is_ready() && isCalibrated == 1) {
    float reading = scale.read()*N_per_ct - N_zero;
    reading_filt = filt_alpha*reading_filt + (1-filt_alpha)*reading;
    Serial.print("HX711: ");
    Serial.println(reading_filt, 2);
  }

  if(digitalRead(9) == 0){
    calibrate();
  }

  if(Serial.available() > 0){
    if(Serial.read() == 'c'){
      calibrate();
      while(Serial.available()){Serial.read();}
    }
  }

//  delay(200);
  
}
