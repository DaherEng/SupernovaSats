// Biblioteca padrão do Arduino, de comunicação I2C e de matemática
#include <Arduino.h> 
#include <Wire.h>
#include <math.h>

// Biblioteca para sensores Adafruit
#include <Adafruit_Sensor.h> 

//Biblioteca BMP
#include <Adafruit_BMP280.h> 

//Biblioteca AHT10
#include <Adafruit_AHTX0.h> 

//Biblioteca sensor de corrente
#include <Adafruit_INA219.h>

//Biblioteca GPS
#include <TinyGPS++.h> 

//Biblioteca acelerômetro
#include <MPU6050.h> 

#include <SoftwareSerial.h> // Biblioteca Serial


//Parâmetros do Acelerometro
#define FSA 16384
#define FSG 131
const int MPU = 0x68;// Endereço I2C do sensor
float AccX, AccY, AccZ, Temp, GirX, GirY, GirZ;// Variáveis para armazenar os dados coletados

//Definição dos parâmetros do sensor de CO2
#define RL_co2 10     // Resistência ao lado do DOUT_LED
#define APin_co2 33   // Pino analógico utilizado

float curve_co2[2] = {-0.32372, 0.648};  // Curva do gráfico em log do MQ135 para CO2 (a, b)

float R0_co2 = 0;

//Definição dos parâmetros do sensor de O3
#define RL_o3 10     // Resistência ao lado do DOUT_LED
#define APin_o3 34   // Pino analógico utilizado

float curve_o3[2] = {0.05775, 0.2647};  // Curva do gráfico em log do MQ135 para CO2 (a, b)

float R0_o3 = 0;

//Definição dos Pinos - Comentado porque ele está usando o padrão do ESP, então só em carater informativo
//#define BMP_SDA 21 //Ligado o BMP280 ao Pino 21 do ESP - SDA é responsavel de enviar e receber os dados
//#define BMP_SCL 22 //Ligado o BMP280 ao Pino 22 do ESP - SCL para criar um clock que sincroniza os sistemas

//Definindo os PINOS RX e TX para GPS
int RXPin = 16;
int TXPin = 17;
int GPSBaud = 9600;

//BMP280
Adafruit_BMP280 bmp; // Cria um objeto para o sensor BMP280 usando I2C

//AHT10
Adafruit_AHTX0 aht; //Declarando o AHT criando objeto

//INA219
Adafruit_INA219 ina219; //Declarando o INA219 criando objeto 

//GPS
TinyGPSPlus gps; //Declarando o TinyGPS criando objeto
SoftwareSerial gpsSerial (RXPin, TXPin); // Criando a porta serial "gpsSerial" para falar com o modulo

//Variáveis
float humidity = 0.0; //variavel global para umidade do AHT10

//Variaveis para o sensor de corrente
float shuntVoltage = 0;
float busVoltage = 0;
float current_mA = 0;
float loadVoltage = 0;

float read_PPM_co2()
{
  double ADCread=0;
  double RS, RSR0, Y, X, PPM;

  //5 Leituras e tira a media
  for (int count=0;count<5;count++) {
                ADCread += analogRead(APin_co2);
                delay(50);
  }
  ADCread = ADCread/5;

  //Calcula RS
  RS = (float)RL_co2 * (4095-ADCread) / ADCread;

  //Calcula RS/R0
  RSR0 = RS/R0_co2;

  //Tira o Log de RSR0 para utilizar na curva log-log (Y)
  Y = log10(RSR0);

  //Calcula o X
  X = (Y - curve_co2[1])/curve_co2[0];

  //Retorna 10^X = PPM
  return pow10(X);
}

float calibracao_co2(){
  int cont;
  float val=0;
  // Calcula o valor de RS no cenário de ar limpo 50 vezes e pega a média
  for (cont=0;cont<50;cont++) {
                val += ((float)RL_co2 * (4095-analogRead(APin_co2)) / analogRead(APin_co2));
                delay(500);
  }
  val = val/50;                                                                                        
  return val;
}

float read_PPM_o3()
{
  double ADCread=0;
  double RS, RSR0, Y, X, PPM;

  //5 Leituras e tira a media
  for (int count=0;count<5;count++) {
                ADCread += analogRead(APin_o3);
                delay(50);
  }
  ADCread = ADCread/5;

  //Calcula RS
  RS = (float)RL_o3 * (4095-ADCread) / ADCread;

  //Calcula RS/R0
  RSR0 = RS/R0_o3;

  //Tira o Log de RSR0 para utilizar na curva log-log (Y)
  Y = log10(RSR0);

  //Calcula o X
  X = (Y - curve_o3[1])/curve_o3[0];

  //Retorna 10^X = PPM
  return pow10(X);
}

float calibracao_o3(){
  int cont;
  float val=0;
  // Calcula o valor de RS no cenário de ar limpo 50 vezes e pega a média
  for (cont=0;cont<50;cont++) {
                val += ((float)RL_o3 * (4095-analogRead(APin_o3)) / analogRead(APin_o3));
                delay(500);
  }
  val = val/50;                                                                                        
  return val;
}
//Função para ler os dados do sensor BMP280 e retornar uma string formatada com os valores
String readBMP280Sensor() {
  String result = "";
  result += "Temp = ";
  result += bmp.readTemperature(); //Faz o objeto bmp ler a temperatura
  result += " *C\n"; //Coloca "*C" e pula linha
  result += "Pressao = ";
  result += bmp.readPressure() / 100.0F; //Faz o objeto bmp ler a pressao
  result += " hPa\n"; //Coloca "hPa" e pula linha
  result += "Alt aprox = ";
  result += bmp.readAltitude(1013.25); // Esse parâmetro pode ser ajustado
  result += " m\n"; //Coloca "m" e pula linha
  return result;
}

// Função de leitura de dados do GPS
void displayInfo()
{
  if (gps.location.isValid()) // SE A LOCALIZACAO DO SINAL ENCONTRADO E VALIDA , ENTAO
  {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6) ; // IMPRIME NA SERIAL O VALOR DA LATIDUE LIDA
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng() , 6) ; // IMPRIME NA SERIAL O VALOR DA LONGITUDE LIDA
  }
  else
  {
    Serial.println ("Nao detectamos a localizacao "); // SE NAO HOUVER NENHUMA LEITURA , IMPRIME A MENSAGEM DE ERRO NA SERIAL
  }
    Serial.print("Data: ");
  if (gps.date.isValid()) // IMPRIME A DATA NA SERIAL
  {
    Serial.print(gps.date.day()); // LEITURA DO DIA
    Serial.print("/");
    Serial.print(gps.date.month()); // LEITURA DO MES
    Serial.print("/");
    Serial.println(gps.date.year()); // LEITURA DO ANO}
  }
  else
  {
    Serial.println("Erro"); // SE NAO HOUVER NENHUMA LEITURA , IMPRIME A MENSAGEM DE ERRO NA SERIAL
  }

  Serial.print("Time: "); // LEITURA DA HORA PARA SER IMPRESSA NA SERIAL
  if (gps.time.isValid())
  {
  if (gps.time.hour() < 10)
    Serial.print(F("0"));
    Serial.print( gps . time . hour () - 3) ; // AJUSTA O FUSO HORARIO PARA NOSSA REGIAO ( FUSO DE SP03:00 , POR ISSO O -3 NO CODIGO ) E IMPRIME NA SERIAL
    Serial.print(":");
  if (gps.time.minute() < 10)
    Serial.print(F("0"));
    Serial.print(gps.time.minute()); // IMPRIME A INFORMACAO DOS MINUTOS NA SERIAL
    Serial.print(":");
  if (gps.time.second() < 10)
    Serial.print(F("0"));
    Serial.print(gps.time.second()); // IMPRIME A INFORMACAO DOS SEGUNDOS NA SERIAL
  }
  else
  {
    Serial.println(" Nao detectamos o horario atual ");
    // SE NAO HOUVER NENHUMA LEITURA , IMPRIME A MENSAGEM DE ERRO NA SERIAL
  }

  Serial.println();
  Serial.println();
  delay(1000);
}
  
void setup() {
  Serial.begin(9600); // Inicia a comunicação Serial na velocidade de 9600 - validar esse BaudRate
  gpsSerial.begin(GPSBaud); //Inincia a comunicacao serial com o GPS
  

//SETUP DO ACELEROMETRO
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


// ----------------
// Fim do código
// ----------------

// ----------------
// Inicio do Loop
// -----------------

void loop() { //Loop que fica rodando para exibir os dados

    Adafruit_Sensor *aht_sensor = aht.getTemperatureSensor();
    sensors_event_t event;
    aht_sensor->getEvent(&event);
    float temperature = event.temperature;

    Adafruit_Sensor *humiditySensor = aht.getHumiditySensor();
    sensors_event_t humidityEvent;
    humiditySensor->getEvent(&humidityEvent);
    humidity = humidityEvent.relative_humidity;

   
    String data = readBMP280Sensor(); // Lê os dados do sensor e adiciona as informações do AHT10
    data += "Temp AHT10: ";
    data += temperature;
    data += " °C\n";
    data += "Umid AHT10: ";
    data += humidity;
    data += " %\n";
    
    Serial.println(data); // Exibe os dados na Serial
    


// Dados do Sensor de Corrente sendo escritas na Serial
  shuntVoltage = ina219.getShuntVoltage_mV();
  busVoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  loadVoltage = busVoltage + (shuntVoltage / 1000);

  Serial.print("Tensão do barramento: ");
  Serial.print(busVoltage);
  Serial.println(" V");
  
  Serial.print("Tensão da carga: ");
  Serial.print(loadVoltage);
  Serial.println(" V");

  Serial.print("Corrente: ");
  Serial.print(current_mA);
  Serial.println(" mA");

    delay(2000); // Aguarda 2 segundos antes de ler os dados novamente


//LOOP DO GPS
// TODA VEZ QUE FOR LIDA UMA NOVA SENTENCA NMEA , CHAMAREMOS A FUNCAO displayInfo () PARA MOSTRAR OS DADOS NA TELA
  while (gpsSerial.available() > 0)
  if (gps.encode (gpsSerial.read()))
  displayInfo();

  // SE EM 5 SEGUNDOS NAO FOR DETECTADA NENHUMA NOVA LEITURA PELO MODULO , SERA MOSTRADO ESTA MENSGEM DE ERRO .
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(" Sinal GPS nao detectado ");
  while (true);
  }


//LOOP DO ACELEROMETRO
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Endereço de inicio da solicitação de dados
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, 1); // Solicita 14 bytes de dados a partir do primeiro endereço
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

  // Leitura do sensor de CO2
  float ppm_co2 = read_PPM_co2();
  float valoradc_co2;
  valoradc_co2 = analogRead(APin_co2);
  printf("\n\rValor ADC = %f", valoradc_co2);
  printf("\nTaxa de CO2 : %f ppm\n\r", ppm_co2);

 // Leitura do sensor de O3
  float ppm_o3 = read_PPM_o3();
  float valoradc_o3;
  valoradc_o3 = analogRead(APin_o3);
  printf("\n\rValor ADC = %f", valoradc_o3);
  printf("\nTaxa de CO2 : %f ppm\n\r", ppm_o3);
}