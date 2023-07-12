// INCLUSAO DE BIBLIOTECAS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>


/* DEFINICAO DE PINOS DO ARDUINO . LEMBRE -SE: O PINO ONDE VOCE CONECTOU O TX SERA O RX AQUI NO
CODIGO E VICE - VERSA . ISSO E POR CAUSA DA
COMUNICACAO SERIAL . PARA MAIS DETALHES , VOCE PODE CONSULTAR NOSSO BLOG */
int RXPin = 16;
int TXPin = 17;

int GPSBaud = 9600;

// CRIANDO UM OBJETO PARA COMUNICAR COM A BIBLIOTECA
TinyGPSPlus gps;

// CRIANDO UMA PORTA SERIAL gpsSerial PARA CONVERSAR COM MODULO
SoftwareSerial gpsSerial (RXPin, TXPin);

void setup()
{
// INICIA A SERIAL
  Serial.begin(115200);

 // INICIA A PORTA SERIAL DO SOFTWARE NO BAUD PADRAO DO GPS , COMO DETERMINAMOS ACIMA :9600
  gpsSerial.begin(GPSBaud);
}


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

void loop()
{
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
}
