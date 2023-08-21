#include <Arduino.h>

#define APin 33

void setup() {
  pinMode(APin, INPUT);
}

void loop() {
  int valoradc;
  valoradc = analogRead(APin);
  printf("\n\rValor ADC = %d", valoradc);
}