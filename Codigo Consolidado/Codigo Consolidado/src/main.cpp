/* Versão beta de código feito por Louise Alcino para leitura de um sensor BMP280 ligado a um ESP32s
Upgrades:
Envio de informações via Bluetooth
Envio de informações via https por um WebServer conectado a rede Wifi Local
Exposição de dados em Display OLED

-
A Biblioteca do sensor BMP280 foi utlizada em seu padrão para as leituras de Temperatura, Pressão e Altitute mas pode ser facilmente customizada


======== PlatformIO.ini abaixo
Como inicialmente está sendo feito com VSCode e PlatformIO lembrar de parametrizar o platformIO.ini com os dados abaixo ( já está lá nesta versão)

[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
board_build.partitions = huge_app.csv
lib_deps = 
	adafruit/Adafruit BMP280 Library @ ^2.3.1
	adafruit/Adafruit SSD1306@^2.5.7
	;espressif/esp32-camera@^2.0.0
	;makuna/NeoPixelBus@^2.7.5
	adafruit/Adafruit AHTX0@^2.0.0
	;enjoyneering/AHT10@^1.1.0
	;adafruit/Adafruit Sensor Lab@^0.8.0
    Adafruit INA219
    ostaquet/MQ131 gas sensor@^1.5.2
   	mikalhart/TinyGPSPlus@^1.0.3
  	plerup/EspSoftwareSerial@^8.0.3
    jrowberg/I2Cdevlib-MPU6050@^1.0.0
*/

//-------------------
// Início do código
//-------------------

//Incluindo as bibliotecas para fucionamento do sistema
#include <Arduino.h> // Biblioteca padrão do Arduino
#include <Wire.h> // Biblioteca para comunicação I2C
#include <Adafruit_Sensor.h> // Biblioteca para sensores Adafruit
#include <Adafruit_BMP280.h> // Biblioteca para o sensor BMP280
#include <WiFi.h> // Biblioteca para conexão WiFi
#include <WebServer.h> // Biblioteca para criar um servidor web
#include <BluetoothSerial.h> // Biblioteca para comunicação Bluetooth
#include <Adafruit_SSD1306.h> //Adafruit_SSD1306.h - Biblioteca para o display
#include <Adafruit_GFX.h> //Adafruit_GFX.h - Biblioteca auxiliar para o display
#include <Adafruit_AHTX0.h> //Adafruit_AHTX0.h - Biblioteca para o sensor AHT10
#include <Adafruit_INA219.h> //Adafruit_INA219 - Biblioteca para o sensor de corrente
#include <TinyGPS++.h> //Biblioteca para o GPS
#include <MPU6050.h> //Biblioteca do Acelerometro
#include <SoftwareSerial.h> // Biblioteca Serial - ??? Conferir necessidade ????
#include <MQ131.h> //Biblioteca do sensor de gas

//Parâmetros do Display OLED - Largura e altura
#define SCREEN_WIDTH 128 // Largura da tela OLED em pixels
#define SCREEN_HEIGHT 32 // Altura da tela OLED em pixels

//Parâmetros do Acelerometro
#define FSA 16384
#define FSG 131
const int MPU = 0x68;// Endereço I2C do sensor
float AccX, AccY, AccZ, Temp, GirX, GirY, GirZ;// Variáveis para armazenar os dados coletados

// Vamos conectar em uma rede WiFi pra compartilhar os dados em uma pagina Web
const char* ssid = "Republica";
const char* password = "46784678";

//Definição dos Pinos - Comentado porque ele está usando o padrão do ESP, então só em carater informativo
//#define BMP_SDA 21 //Ligado o BMP280 ao Pino 21 do ESP - SDA é responsavel de enviar e receber os dados
//#define BMP_SCL 22 //Ligado o BMP280 ao Pino 22 do ESP - SCL para criar um clock que sincroniza os sistemas

//Definindo os PINOS RX e TX para GPS? 
// - Obs o criador:  LEMBRE -SE: O PINO ONDE VOCE CONECTOU O TX SERA O RX AQUI NO CODIGO E VICE - VERSA . ISSO E POR CAUSA DA COMUNICACAO SERIAL
int RXPin = 16;
int TXPin = 17;
int GPSBaud = 9600;

//Levantando Servidor Web para resultados em Navegdor, Bluetooth para resultado em BT além dos resultados em Display
WebServer server(80); // Cria um servidor web na porta 80 para ser acessar por Navegador Web
BluetoothSerial SerialBT; // Cria um objeto uma conexão Bluetooth
Adafruit_BMP280 bmp; // Cria um objeto para o sensor BMP280 usando I2C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // Cria um objeto do Display
Adafruit_AHTX0 aht; //Declarando o AHT criando objeto
Adafruit_INA219 ina219; //Declarando o INA219 criando objeto 
TinyGPSPlus gps; //Declarando o TinyGPS criando objeto

SoftwareSerial gpsSerial (RXPin, TXPin); // Criando a porta serial "gpsSerial" para falar com o modulo

//Variáveis
float humidity = 0.0; //variavel global para umidade do AHT10

//variaveis para o sensor de corrente
float shuntVoltage = 0;
float busVoltage = 0;
float current_mA = 0;
float loadVoltage = 0;

// Função para ler os dados do sensor BMP280 e retornar uma string formatada com os valores
String readBMP280Sensor() {
  String result = "";
  result += "Temp = ";
  result += bmp.readTemperature(); //Faz o objeto bmp ler a temperatura
  result += " *C\n"; //Coloca "*C" e pula linha
  result += "Pressao = ";
  result += bmp.readPressure() / 100.0F; //Faz o objeto bmp ler a temperatura
  result += " hPa\n"; //Coloca "hPa" e pula linha
  result += "Alt aprox = ";
  result += bmp.readAltitude(1013.25); // Esse parâmetro pode ser ajustado
  result += " m\n"; //Coloca "m" e pula linha
  result += "IP = ";
  result += WiFi.localIP().toString(); // Converte o endereço IP em uma string para ser exibida no painel
  result += "\n"; //Coloca m e pula linha
  return result;
}

// Função para escrever os dados do sensor no Display OLED
void MostraDisplay(String data) {
  display.clearDisplay(); // Limpa o Display OLED
  display.setTextSize(0.5); // Define o tamanho da fonte
  display.setTextColor(WHITE); // Define a cor do texto
  display.setCursor(0,0); // Define a posição do cursor
  display.println(data);
  display.display(); // Atualiza o Display OLED
}

// Função chamada quando o usuário acessa a página inicial do servidor web
void handle_OnConnect() {
  String html = "<html><head><meta http-equiv='refresh' content='5'></head><body>";//O código HTML tem um autorefresh a cada 5 segundos para não ficar sobrecarregado
  html += "<h1>Dados do Sensor BMP280</h1>";
  html += "<pre>";
  html += readBMP280Sensor(); // Lê os dados do sensor (String montada acima) e adiciona na página HTML
  html += "</pre>";
  html += "</body></html>";
  
  server.send(200, "text/html", html); // Envia a página HTML para o usuário
}

void setup() {
  Serial.begin(9600); // Inicia a comunicação Serial na velocidade de 9600 - validar esse BaudRate
  gpsSerial.begin(GPSBaud); //Inincia a comunicacao serial com o GPS
  
  // Inicia a comunicação com o display OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Não foi possível iniciar o display OLED"));
    while (1);
  }

  if (!bmp.begin(0x76)) { // Inicia a comunicação com o sensor BMP280
    Serial.println(F("Não foi possível encontrar o sensor BMP280"));
    while (1);
  }

  if (!aht.begin()) { // Inicia a comunicação com o sensor AHT10
    Serial.println(F("Não foi possível encontrar o sensor AHT10"));
    while (1);
  }

  if (!ina219.begin()) { // Inicia a comunicação com o sensor INA219
    Serial.println("Não foi possível encontrar o INA219");
    while (1);
  }

//Tentando Wifi também para exibir em rede para o Servidor Web
  WiFi.begin(ssid, password); // Chama conexão ao WiFi
  int tentawifi = 0; //Inicia contagem de tentativas de conexão WiFi
  while (tentawifi < 10) { //Limitamos em 10 tentativas antes de continuar o programa
    if (WiFi.status() != WL_CONNECTED) { // Aguarda a conexão com o WiFi
    Serial.println("Conectando ao WiFi...");
    delay(1000); //Acontece a cada 1 segundo para não dar sobrecarga
  }

//Se conseguir ele printa os dados
    else {
      Serial.println("Conectado ao WiFi");
      Serial.print("Endereço IP: ");
      Serial.print(WiFi.localIP());
      Serial.println("");
  // A conexão Wi-Fi foi estabelecida com sucesso, acenda a LED interna que fica no Pin2
     pinMode(2, OUTPUT);
     digitalWrite(2, HIGH);
    break;
    }
  //Se nao conseguiu, tenta novamente até 10 vezes
    tentawifi++;
  }
  

  server.on("/", handle_OnConnect); // Define a função que será chamada quando o usuário acessar a página inicial
  server.begin(); // Inicia o servidor web

  SerialBT.begin("ESP32-BMP280"); // Inicia a conexão Bluetooth com o nome "ESP32-BMP280"

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


//SETUP DO ACELEROMETRO
//Iniciando o MQ131
// Init the sensor
// - Heater control on pin 2
// - Sensor analog read on pin A0
// - Model LOW_CONCENTRATION
// - Load resistance RL of 995 Ohm
  MQ131.begin(2,A0, HIGH_CONCENTRATION, 995);  

  Serial.println("Calibration parameters");
  Serial.print("R0 = ");
  Serial.print(MQ131.getR0());
  Serial.println(" Ohms");
  Serial.print("Time to heat = ");
  Serial.print(MQ131.getTimeToRead());
  Serial.println(" s");

}


// FUNÇÃO DIRECIONADO AO MODULO GPS
void displayInfo() // FUNCAO RESPONSAVEL PELA LEITURA DOS DADOS
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

    server.handleClient(); // Processa as requisições do servidor web
    
    String data = readBMP280Sensor(); // Lê os dados do sensor e adiciona as informações do AHT10
    data += "Temp AHT10: ";
    data += temperature;
    data += " °C\n";
    data += "Umid AHT10: ";
    data += humidity;
    data += " %\n";
    
    Serial.println(data); // Exibe os dados na Serial
    
    if (SerialBT.available()) { // Se houver uma conexão Bluetooth ativa
      SerialBT.println(data); // Envia os dados via Bluetooth
    }

    MostraDisplay(data); // Escreve os dados no Display OLED


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

//LOOP DO SENSOR DE GAS/OZONIO
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
 
}