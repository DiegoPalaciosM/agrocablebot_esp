#ifndef DEFINES
#define DEFINES

#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <AsyncMqttClient.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

// Robot
// Estructura
union {
  struct
  {
    char name[8];
    int height;
    int width;
    int length;
    int ef;
    int ef_length;
  } data;
} robotSetup;

char buf[64];
char bufindex;

// Pines
#define A_STEP_PIN 23
#define A_DIR_PIN 22
#define A_ENABLE_PIN 21

#define B_STEP_PIN 19
#define B_DIR_PIN 18
#define B_ENABLE_PIN 16

#define C_STEP_PIN 32
#define C_DIR_PIN 33
#define C_ENABLE_PIN 25

#define D_STEP_PIN 26
#define D_DIR_PIN 27
#define D_ENABLE_PIN 12

#define BUZZER 17

// Macros
#define A_STEP_HIGH digitalWrite(A_STEP_PIN, HIGH);
#define A_STEP_LOW digitalWrite(A_STEP_PIN, LOW);
#define A_DIR_HIGH digitalWrite(A_DIR_PIN, HIGH);
#define A_DIR_LOW digitalWrite(A_DIR_PIN, LOW);
#define A_ENABLE digitalWrite(A_ENABLE_PIN, LOW);
#define A_DISABLE digitalWrite(A_ENABLE_PIN, HIGH);

#define B_STEP_HIGH digitalWrite(B_STEP_PIN, HIGH);
#define B_STEP_LOW digitalWrite(B_STEP_PIN, LOW);
#define B_DIR_HIGH digitalWrite(B_DIR_PIN, HIGH);
#define B_DIR_LOW digitalWrite(B_DIR_PIN, LOW);
#define B_ENABLE digitalWrite(B_ENABLE_PIN, LOW);
#define B_DISABLE digitalWrite(B_ENABLE_PIN, HIGH);

#define C_STEP_HIGH digitalWrite(C_STEP_PIN, HIGH);
#define C_STEP_LOW digitalWrite(C_STEP_PIN, LOW);
#define C_DIR_HIGH digitalWrite(C_DIR_PIN, HIGH);
#define C_DIR_LOW digitalWrite(C_DIR_PIN, LOW);
#define C_ENABLE digitalWrite(C_ENABLE_PIN, LOW);
#define C_DISABLE digitalWrite(C_ENABLE_PIN, HIGH);

#define D_STEP_HIGH digitalWrite(D_STEP_PIN, HIGH);
#define D_STEP_LOW digitalWrite(D_STEP_PIN, LOW);
#define D_DIR_HIGH digitalWrite(D_DIR_PIN, HIGH);
#define D_DIR_LOW digitalWrite(D_DIR_PIN, LOW);
#define D_ENABLE digitalWrite(D_ENABLE_PIN, LOW);
#define D_DISABLE digitalWrite(D_ENABLE_PIN, HIGH);

#define BUZZER_ON digitalWrite(BUZZER, HIGH);
#define BUZZER_OFF digitalWrite(BUZZER, LOW);



float curX, curY, curZ;  // Posicion actual XYZ
float tarX, tarY, tarZ;  // Posicion objetivo XYZ

float curCA, curCB, curCC, curCD;  // Longitud actual de los cables ABCD
float tarCA, tarCB, tarCC, tarCD;  // Longitud objetivo de los cables ABCD

float angCA, angCB, angCC, angCD;                  // Angulo polea
float CompRGXCA, CompRGYCA, CompRGXCB, CompRGYCB;  // Angulo de compensacion
float CompRGXCC, CompRGYCC, CompRGXCD, CompRGYCD;  // Angulo de compensacion

long tarA, tarB, tarC, tarD;  // Objetivo de los motores
long posA, posB, posC, posD;  // Actual de los motores

int8_t motorAfw, motorBfw, motorCfw, motorDfw = 1;   // Direccion de giro positiva
int8_t motorAbk, motorBbk, motorCbk, motorDbk = -1;  // Direccion de giro negativa

float initStringCA, initStringCB, initStringCC, initStringCD;                  // Longitud inicial de los cables
float pxCA, pyCA, pzCA, pxCB, pyCB, pzCB, pxCC, pyCC, pzCC, pxCD, pyCD, pzCD;  // Coordenadas inicales de los puntos de anclaje
float pxCAcg, pyCAcg, pxCBcg, pyCBcg, pxCCcg, pyCCcg, pxCDcg, pyCDcg;          // Cordenadas en X y Y desde los centro de giro de poleas

// Parámetros de los motores paso a paso:
long stepAuxDelay = 0;
long stepdelay_min = 100;  // RÁPIDO: 100 LENTO: 1000
long stepdelay_max = 500;  // RÁPIDO: 500 LENTO: 2000
int MOM = 0;               // Move Only Motor

#define SPEED_STEP 1

// Parámetros geométricos del robot:
#define STEPS_PER_CIRCLE 3200.0f                                   // configurado a un 1/16 del paso del motor que es 1.8°
#define WIDTH 1170                                                 // distanica entre centros de radio de giro de poleas
#define HEIGHT 1170                                                // distanica entre centros de radio de giro de poleas
#define LENGTH 320                                                 // distancia entre los centros de poleas y el suelo (poleas en la mitad)(cables a tension) !!NOTA:Z se invierte¡¡
#define EF 180                                                     // distancia entre centros de poleas del efector a lo largo de los ejes X y Y //150mm tamaño cuadrado rojo
#define EF_LENGTH 100                                              // distancia entre centros de poleas del efector a lo largo del eje Z
#define DIAMETER 81.80                                             // diámetro del carrete
#define GEARBOX 10                                                 // Relacion de la caja de reduccion coran sin fin
#define STEPS_PER_MM (STEPS_PER_CIRCLE / PI / DIAMETER) * GEARBOX  // relación de pasos por milímetro
#define RGp 45                                                     // radio de giro del sistema de poleas

void pinEnabler() {
  A_ENABLE;
  B_ENABLE;
  C_ENABLE;
  D_ENABLE;
}

void pinDisabler() {
  A_DISABLE;
  B_DISABLE;
  C_DISABLE;
  D_DISABLE;
}

void stepperA(bool dir) {
  if (!dir)  // define la direccion de giro del motor A
  {
    A_DIR_LOW;
  } else {
    A_DIR_HIGH;
  }

  A_STEP_HIGH;  // enciende y apaga el pin de step del driver para controlar los pasos del motor A
  delayMicroseconds(50);
  A_STEP_LOW;
}

void stepperB(bool dir) {
  if (!dir)  // define la direccion de giro del motor B
  {
    B_DIR_LOW;
  } else {
    B_DIR_HIGH;
  }

  B_STEP_HIGH;  // enciende y apaga el pin de stpes del driver para controlar los pasos del motor B
  delayMicroseconds(50);
  B_STEP_LOW;
}

void stepperC(bool dir) {
  if (!dir)  // define la direccion de giro del motor C
  {
    C_DIR_LOW;
  } else {
    C_DIR_HIGH;
  }

  C_STEP_HIGH;  // enciende y apaga el pin de stpes del driver para controlar los pasos del motor C
  delayMicroseconds(50);
  C_STEP_LOW;
}

void stepperD(bool dir) {
  if (!dir)  // define la direccion de giro del motor D
  {
    D_DIR_LOW;
  } else {
    D_DIR_HIGH;
  }

  D_STEP_HIGH;  // enciende y apaga el pin de stpes del driver para controlar los pasos del motor D
  delayMicroseconds(50);
  D_STEP_LOW;
}

void beep(int time) {
  BUZZER_ON;
  delay(time);
  BUZZER_OFF;
}

void resetBuff() {
  memset(buf, 0, 64);
  bufindex = 0;
}

// OTA
bool enableOTA = false;
const char* passwordOTA = "imacuna";

void initOTA() {
  ArduinoOTA.setPassword(passwordOTA);
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else
        type = "filesystem";
    });
  ArduinoOTA.setHostname("agrocablebot");
  ArduinoOTA.begin();
}

// Conexion

// MQTT
AsyncMqttClient clientMqtt;
bool mqttmessageAvailable = false;

// Webserver
WiFiClient client;
AsyncWebServer server(80);
AsyncWebSocket ws("/info");


// HTTP form
String ssid;
String password;
String mqttHost;

// Archivos de configuracion
const char* ssidPath = "/ssid.txt";
const char* passPath = "/password.txt";
const char* mqttHostPath = "/mqttHost.txt";
const char* configPath = "/config.json";

// Tiempo antes de crear el AP
unsigned long previousMillis = 0;
const long interval = 10000;

// SPIFFS

String readFile(const char* path) {
  if (!SPIFFS.exists(path)){
    return "";
  }
  File file = SPIFFS.open(path);
  String fileContent;
  while (file.available()) {
    fileContent += file.readString();
    break;
  }
  return fileContent;
}

void writeFile(const char* path, const char* data) {
  File file = SPIFFS.open(path, FILE_WRITE);
  file.print(data);
  file.close();
}

// JSON
StaticJsonDocument<256> cmdJson;

void printAll() {
  Serial.print(curX);
  Serial.print(' ');
  Serial.print(curY);
  Serial.print(' ');
  Serial.print(curZ);
  Serial.print(' ');
  Serial.print(tarX);
  Serial.print(' ');
  Serial.print(tarY);
  Serial.print(' ');
  Serial.print(tarZ);
  Serial.print(' ');
  Serial.print(curCA);
  Serial.print(' ');
  Serial.print(curCB);
  Serial.print(' ');
  Serial.print(curCC);
  Serial.print(' ');
  Serial.print(curCD);
  Serial.print(' ');
  Serial.print(tarCA);
  Serial.print(' ');
  Serial.print(tarCB);
  Serial.print(' ');
  Serial.print(tarCC);
  Serial.print(' ');
  Serial.print(tarCD);
  Serial.print(' ');
  Serial.print(angCA);
  Serial.print(' ');
  Serial.print(angCB);
  Serial.print(' ');
  Serial.print(angCC);
  Serial.print(' ');
  Serial.print(angCD);
  Serial.print(' ');
  Serial.print(CompRGXCA);
  Serial.print(' ');
  Serial.print(CompRGXCB);
  Serial.print(' ');
  Serial.print(CompRGXCC);
  Serial.print(' ');
  Serial.print(CompRGXCD);
  Serial.print(' ');
  Serial.print(tarA);
  Serial.print(' ');
  Serial.print(tarB);
  Serial.print(' ');
  Serial.print(tarC);
  Serial.print(' ');
  Serial.print(tarD);
  Serial.print(' ');
  Serial.print(posA);
  Serial.print(' ');
  Serial.print(posB);
  Serial.print(' ');
  Serial.print(posC);
  Serial.print(' ');
  Serial.println(posD);
}

void testPines(bool high) {
  if (high) {
    Serial.print("Pines: ");
    A_DIR_HIGH;
    Serial.print(A_DIR_PIN);
    Serial.print(" ");
    A_STEP_HIGH;
    Serial.print(A_STEP_PIN);
    Serial.print(" ");
    A_DISABLE;
    Serial.print(A_ENABLE_PIN);
    Serial.print(" ");
    B_DIR_HIGH;
    Serial.print(B_DIR_PIN);
    Serial.print(" ");
    B_STEP_HIGH;
    Serial.print(B_STEP_PIN);
    Serial.print(" ");
    B_DISABLE;
    Serial.print(B_ENABLE_PIN);
    Serial.print(" ");
    C_DIR_HIGH;
    Serial.print(C_DIR_PIN);
    Serial.print(" ");
    C_STEP_HIGH;
    Serial.print(C_STEP_PIN);
    Serial.print(" ");
    C_DISABLE;
    Serial.print(C_ENABLE_PIN);
    Serial.print(" ");
    D_DIR_HIGH;
    Serial.print(D_DIR_PIN);
    Serial.print(" ");
    D_STEP_HIGH;
    Serial.print(D_STEP_PIN);
    Serial.print(" ");
    D_DISABLE;
    Serial.print(D_ENABLE_PIN);
    Serial.println(" Puestos en nivel logico ALTO");
  } else {
    Serial.print("Pines: ");
    A_DIR_LOW;
    Serial.print(A_DIR_PIN);
    Serial.print(" ");
    A_STEP_LOW;
    Serial.print(A_STEP_PIN);
    Serial.print(" ");
    A_ENABLE;
    Serial.print(A_ENABLE_PIN);
    Serial.print(" ");
    B_DIR_LOW;
    Serial.print(B_DIR_PIN);
    Serial.print(" ");
    B_STEP_LOW;
    Serial.print(B_STEP_PIN);
    Serial.print(" ");
    B_ENABLE;
    Serial.print(B_ENABLE_PIN);
    Serial.print(" ");
    C_DIR_LOW;
    Serial.print(C_DIR_PIN);
    Serial.print(" ");
    C_STEP_LOW;
    Serial.print(C_STEP_PIN);
    Serial.print(" ");
    C_ENABLE;
    Serial.print(C_ENABLE_PIN);
    Serial.print(" ");
    D_DIR_LOW;
    Serial.print(D_DIR_PIN);
    Serial.print(" ");
    D_STEP_LOW;
    Serial.print(D_STEP_PIN);
    Serial.print(" ");
    D_ENABLE;
    Serial.print(D_ENABLE_PIN);
    Serial.println(" Puestos en nivel logico BAJO");
  }
}

#endif  // DEFINES