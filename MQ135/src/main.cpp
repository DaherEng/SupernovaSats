#include <Arduino.h>
#include <MQ-Sensor-SOLDERED.h>

#define RatioMQ135CleanAir 3.6 // RS / R0 = 3.6 ppm

// Pino analógico utilizado
MQ2 mq135(34);

void setup()
{
  Serial.begin(115200);
// Valores para calibrar para medição de CO2
mq135.setRegressionMethod(1);
mq135.setA(36974);
mq135.setB(-3.109);

mq135.begin(0x30);
  Serial.print("Calibrating please wait.");

  float calcR0 = 0;
    for (int i = 1; i <= 10; i++)
    {
      mq135.update();
        calcR0 +=mq135.calibrate(RatioMQ135CleanAir);
        Serial.print(".");
    }
  mq135.setR0(calcR0 / 10);
    Serial.println("  done!.");

    if (isinf(calcR0))
    {
        Serial.println("Warning: Conection issue founded! Check easyC cable, connector and I2C address!");
        while (1)
            ;
    }
    if (calcR0 == 0)
    {
        Serial.println("Warning: Conection issue founded! Check easyC cable, connector and I2C address!");
        while (1)
            ;
    }
    /*****************************  MQ Calibration ********************************************/

  mq135.serialDebug(true);
}

void loop()
{
  mq135.update();      // Update data, the arduino will be read the voltage on the analog pin
  mq135.readSensor();  // Sensor will read PPM concentration using the model and a and b values setted before or in the
  mq135.serialDebug(); // Will print the table on the serial port
  delay(2000);        // Sampling frequency
}