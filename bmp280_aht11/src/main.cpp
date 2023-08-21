#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <WebServer.h>
#include <BluetoothSerial.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <AHT10.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

const char* ssid = "Supernova Rocketry";
const char* password = "foguetaos2";

WebServer server(80);
BluetoothSerial SerialBT;
Adafruit_BMP280 bmp;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
AHT10 aht;

String readBMP280Sensor() {
  String result = "";
  result += "Temp BMP= ";
  result += bmp.readTemperature();
  result += " *C\n";
  result += "Pressao BMP= ";
  result += bmp.readPressure() / 100.0F;
  result += " hPa\n";
  result += "Alt BMP = ";
  result += bmp.readAltitude(1013.25);
  result += " m\n";
  result += "Temp AHT10 = ";
  result += aht.readTemperature();
  result += " °C\n";
  result += "Hum AHT10 = ";
  result += aht.readHumidity();
  result += " %\n";
  result += "IP = ";
  result += WiFi.localIP().toString();
  result += "\n";
  return result;
}

void MostraDisplay(void *parameter) {
  while (1) {
    String data = (String) parameter;
    display.clearDisplay();
    display.setTextSize(0.5);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println(data);
    display.display();
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Aguarda 1 segundo
  }
}

void handle_OnConnect() {
  String html = "<html><head><meta http-equiv='refresh' content='5'></head><body>";
  html += "<h1>Dados do Sensor BMP280</h1>";
  html += "<pre>";
  html += readBMP280Sensor();
  html += "</pre>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

void serverTask(void *parameter) {
  while (1) {
    server.handleClient();
    vTaskDelay(1); // Aguarda um tempo mínimo para evitar bloqueio de tarefas
  }
}

void bluetoothTask(void *parameter) {
  while (1) {
    if (SerialBT.available()) {
      String data = (String) parameter;
      SerialBT.println(data);
    }
    vTaskDelay(1);
  }
}

void setup() {
  Serial.begin(9600);
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Não foi possível iniciar o display OLED");
  }

  if (!bmp.begin(0x76)) {
    Serial.println("Não foi possível encontrar o sensor BMP280");
  }

  if (!aht.begin()) {
    Serial.println("Não foi possível encontrar o sensor AHT10");
  }

 

 WiFi.begin(ssid, password);
  int tentawifi = 0;
  while (tentawifi < 10) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Conectando ao WiFi...");
      delay(1000);
    } else {
      Serial.println("Conectado ao WiFi");
      Serial.print("Endereço IP: ");
      Serial.print(WiFi.localIP());
      Serial.println("");
      pinMode(2, OUTPUT);
      digitalWrite(2, HIGH);
      break;
    }
    tentawifi++;
  }
  
  server.on("/", handle_OnConnect);
  server.begin();

  SerialBT.begin("ESP32-BMP280");

  xTaskCreate(MostraDisplay, "DisplayTask", 4096, NULL, 1, NULL); // Cria a tarefa para o display
  xTaskCreate(serverTask, "ServerTask", 8192, NULL, 1, NULL); // Cria a tarefa para o servidor web
  xTaskCreate(bluetoothTask, "BluetoothTask", 8192, NULL, 1, NULL); // Cria a tarefa para o Bluetooth
}

void loop() {
  String data = readBMP280Sensor();
  Serial.println(data);
  vTaskDelay(2000 / portTICK_PERIOD_MS); // Aguarda 2 segundos
  vTaskList(buffer);
  Serial.println(buffer);
}