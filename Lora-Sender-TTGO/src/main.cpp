//Software para envio de dados via LORA com o TTGO ESP32
//Dados enviados: Latitude, Longitude, Temperatura, Pressão e Altitude
//Data de modificação: 20/12/2022

//Inclusão de todas as bibliotecas a serem utilizadas

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "Arduino.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "BMP085.h"
#include <SPI.h>
#include <LoRa.h>

//Definição das variáveis

BMP085 barometer;

float temperature;
float pressure;
int32_t altitude;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

//Definir as portas que serão utilizadas como RX e TX no GPS

SoftwareSerial serial1(34, 35); // RX, TX
TinyGPS gps1;


// Pinout do LORA
#define LORA_MISO 19
#define LORA_CS 18
#define LORA_MOSI 27
#define LORA_SCK 5
#define LORA_RST 14
#define LORA_IRQ 26 
// Frequência do LORA -> 433hz ou 915hz
#define LORA_BAND 433E6


// Função de inicialização do Lora
void initLoRa() {
  Serial.println("Inicializando Lora ...");
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  // Start LoRa using the frequency
  int result = LoRa.begin(LORA_BAND);
  if (result != 1) {
    Serial.println("Falha na inicialização do Lora");
    for (;;);
  }
  Serial.println("LoRa inicializado");
  delay(2000);
}

void setup() {
   serial1.begin(9600); //Software Serial do GPS definida anteriormente
   Serial.begin(9600);

   Serial.println("GPS aguardando sinal dos satélites");
   
   Wire.begin(21,22); //Pinos SDA e SCL utilizados pelo GY-80

   Serial.println("Inicializando dispositivos I2C");
   barometer.initialize();

    //Verificando conexões
    Serial.println("Testando conexões...");
    Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");

    pinMode(LED_PIN, OUTPUT);
    Serial.println("Configurando LoRa Sender...");
    initLoRa(); //Função que inicializa o Lora
}

void loop() {
  bool recebido = false; //Variável que indica se o sinal de GPS foi recebido

  while (serial1.available()) {
     char cIn = serial1.read();
     recebido = gps1.encode(cIn); //Se receber o sinal a variável recebe o valor true
  }

  if (recebido) {
     //Latitude e Longitude
     long latitude, longitude;
     unsigned long idadeInfo;
     gps1.get_position(&latitude, &longitude, &idadeInfo);     

     //Leitura da Temperatura
     barometer.setControl(BMP085_MODE_TEMPERATURE);
    
    //Guardando o valor em Celsius (ºC) na variável
     temperature = barometer.getTemperatureC();

    //Leitura da Pressão (3x oversampling mode, high detail, 23.5ms delay)
     barometer.setControl(BMP085_MODE_PRESSURE_3);

    //Guardando o valor da temperatura calibrada em Pascal (Pa)
     pressure = barometer.getPressure();

    //Calcula a altitude absoluta em metros baseada na pressão conhecida
    //Pode ser passado um segundo parâmentro com a pressão a nível do mar
    //Caso contrário será o usado o valor padrão de 101325 Pa
     altitude = barometer.getAltitude(pressure);


     //Criando uma string para armazenar os dados e enviar através do Lora
     char stemp [120];
     sprintf(stemp, "* %10.6f ; %10.6f ; %10.2f ; %10.2f ; %10.2f*", float(latitude) / 1000000, float(longitude) / 1000000, temperature, pressure, altitude);
     Serial.println(stemp);
     LoRa.beginPacket();
     LoRa.print(stemp);
     LoRa.endPacket();

  }
}