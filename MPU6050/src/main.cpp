#include <Arduino.h>
#include <Wire.h>

#define FSA 16384
#define FSG 131

// Endereço I2C do sensor
const int MPU = 0x68;

// Variáveis para armazenar os dados coletados
float AccX, AccY, AccZ, Temp, GirX, GirY, GirZ;

void setup() {
  // Inicialização da comunicação serial
  Serial.begin(9600);

  // Inicialização do MPU-6050
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Configuração do fundo de escala do giroscópio
  /*
  Fundo de escala em +/- 250°/s >> Segundo Wire.write(0x00);
  Fundo de escala em +/- 500°/s >> Segundo Wire.write(0x08);
  Fundo de escala em +/- 1000°/s >> Segundo Wire.write(0x10);
  Fundo de escala em +/- 2000°/s >> Segundo Wire.write(0x18);
  */
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x18);
  Wire.endTransmission(true);

  // Configuração do fundo de escala do acelerômetro
  /*
  Fundo de escala em +/- 2G >> Segundo Wire.write(0x00);
  Fundo de escala em +/- 4G >> Segundo Wire.write(0x08);
  Fundo de escala em +/- 8G >> Segundo Wire.write(0x10);
  Fundo de escala em +/- 16G >> Segundo Wire.write(0x18);
  */
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void loop() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Endereço de inicio da solicitação de dados
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // Solicita 14 bytes de dados a partir do primeiro endereço
  // As variáveis de aceleração, giro e temperatura são de 2 bytes cada (um high e um low, em hexa 0xHL)

  // Armazena os dados
  AccX = Wire.read() << 8 | Wire.read(); // Lê os bytes 3B e 3C e os coloca na mesma variável
  AccY = Wire.read() << 8 | Wire.read(); // Lê os bytes 3D e 3E e os coloca na mesma variável
  AccZ = Wire.read() << 8 | Wire.read(); // Lê os bytes 3F e 40 e os coloca na mesma variável
  Temp = Wire.read() << 8 | Wire.read(); // Lê os bytes 41 e 42 e os coloca na mesma variável
  GirX = Wire.read() << 8 | Wire.read(); // Lê os bytes 43 e 44 e os coloca na mesma variável
  GirY = Wire.read() << 8 | Wire.read(); // Lê os bytes 45 e 46 e os coloca na mesma variável
  GirZ = Wire.read() << 8 | Wire.read(); // Lê os bytes 47 e 48 e os coloca na mesma variável

  // Escreve na serial as variaveis
/*
  É necessário alterar os valores da divisão conforme o fundo de escala utilizando o #define FSA e FSG!

  Acelerômetro
  +/- 2G  >> 16384
  +/- 4G  >> 8192
  +/- 8G  >> 4096
  +/- 16G >> 2048

  Giroscópio
  +/- 250°/s  >> 131
  +/- 500°/s  >> 65.6
  +/- 1000°/s >> 32.8
  +/- 2000°/s >> 16.4
*/
  Serial.print("Aceleracao: ");
  Serial.print(AccX /FSA);
  Serial.print(" ");
  Serial.print(AccY /FSA);
  Serial.print(" ");
  Serial.print(AccZ /FSA);
  Serial.print("\nGiro: ");
  Serial.print(GirX /FSG);
  Serial.print(" ");
  Serial.print(GirY /FSG);
  Serial.print(" ");
  Serial.print(GirZ /FSG);
  Serial.print("\n");


    delay(2000);
  }