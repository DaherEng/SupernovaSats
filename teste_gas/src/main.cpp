#include <Arduino.h>
#include <math.h>

#define RL 10     // Resistência ao lado do DOUT_LED
#define APin 33   // Pino analógico utilizado

float curve[2] = {-0.32372, 0.648};  // Curva do gráfico em log do MQ135 para CO2 (a, b)

float R0 = 0;

float calibracao(){
  int cont;
  float val=0;
  // Calcula o valor de RS no cenário de ar limpo 50 vezes e pega a média
  for (cont=0;cont<50;cont++) {
                val += ((float)RL * (4095-analogRead(APin)) / analogRead(APin));
                delay(500);
  }
  val = val/50;                                                                                        
  return val;
}

float read_PPM()
{
  double ADCread=0;
  double RS, RSR0, Y, X, PPM;

  //5 Leituras e tira a media
  for (int count=0;count<5;count++) {
                ADCread += analogRead(APin);
                delay(50);
  }
  ADCread = ADCread/5;

  //Calcula RS
  RS = (float)RL * (4095-ADCread) / ADCread;

  //Calcula RS/R0
  RSR0 = RS/R0;

  //Tira o Log de RSR0 para utilizar na curva log-log (Y)
  Y = log10(RSR0);

  //Calcula o X
  X = (Y - curve[1])/curve[0];

  //Retorna 10^X = PPM
  return pow10(X);
}

void setup() {
  printf("Calibrando...");
  pinMode(APin, INPUT);
  R0 = calibracao();
  printf("\n\rR0 = %f\n\r", R0);
}

void loop() {

  float ppm_co2 = read_PPM();
  float valoradc_co2;
  valoradc_co2 = analogRead(APin);
  printf("\n\rValor ADC = %f", valoradc_co2);
  printf("\nTaxa de CO2 : %f ppm\n\r", ppm_co2);
  
  delay(2000);
}