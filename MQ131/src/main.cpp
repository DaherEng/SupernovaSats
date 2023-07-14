#include <MQ131.h>
#include <Arduino.h>

void setup() {
Serial.begin(115200);

// Init the sensor
// - Heater control on pin 2
// - Sensor analog read on pin A0
// - Model HIGH_CONCENTRATION
// - Load resistance RL of 1MOhms (1000000 Ohms)
Serial.println("Chegou aqui");
MQ131.begin(2, 34, HIGH_CONCENTRATION, 10000);

Serial.println("Calibration in progress...");

MQ131.calibrate();

Serial.println("Calibration done!");
Serial.print("R0 = ");
Serial.print(MQ131.getR0());
Serial.println(" Ohms");
Serial.print("Time to heat = ");
Serial.print(MQ131.getTimeToRead());
Serial.println(" s");
}

void loop() {
Serial.println("Chegou no loop");
Serial.println("Sampling...");
MQ131.sample();
Serial.print("Concentration O3 : ");
Serial.print(MQ131.getO3(PPM));
Serial.println(" ppm");
Serial.print("Concentration O3 : ");
Serial.print(MQ131.getO3(PPB));
Serial.println(" ppb");
Serial.print("Concentration O3 : ");
Serial.print(MQ131.getO3(MG_M3));
Serial.println(" mg/m3");
Serial.print("Concentration O3 : ");
Serial.print(MQ131.getO3(UG_M3));
Serial.println(" ug/m3");

delay(5000);
}