#include <Wire.h>

#define interval 1000
#define speedpercentage 40
#define speedmax 255
#define konstanta 1
#define debit 100
#define pwmPin 25 // Pin D25
#define ChannelPWM 1
const int adcPins[] = {32, 33, 27, 14, 12}; // Pins D32, D33, D27, D14, and D12
int adcValues[5];
unsigned long lastmillis = 0;
uint8_t speedmotor = 0;

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 5; i++) {
    pinMode(adcPins[i], INPUT);  // Set ADC pins as inputs
  }
  pinMode(pwmPin, OUTPUT);  // Set PWM pin as output

  speedmotor = (speedpercentage * speedmax) / 100;
  
  ledcAttachPin(pwmPin, ChannelPWM);
  ledcSetup(ChannelPWM, 12000, 8); // 12 kHz PWM, 8-bit resolution
}

void loop() {
  if(millis() - lastmillis >= interval) {
      for (int i = 0; i < 5; i++) {
      adcValues[i] = analogRead(adcPins[i]);  // Read ADC values
      Serial.print("ADC value on pin D"); Serial.print(adcPins[i]); Serial.print(": ");
      Serial.println(adcValues[i]);

      int debitair = adcValues[i] * konstanta;

      if(debitair < debit) speedmotor++;
      else if(adcValues[i] > 0) speedmotor--;
      
      if(speedmotor > speedmax) speedmotor = speedmax;
    }
    ledcWrite(ChannelPWM, speedmotor);
  }

}
