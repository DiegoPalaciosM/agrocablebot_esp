#include "acb.h"
#include "connection.h"
#include "defines.h"

unsigned long tInit = 0;
unsigned long tEnd = 0;

void setup() {
  Serial.begin(115200);
  Serial.println('1');
  initWiFi();
  Serial.println('2');
  initRobotSetup();
  Serial.println('3');
  initPosition();
  Serial.println('4');
  initOTA();
  Serial.println('5');
  notifyClients();
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
}

void loop() {
  if (tEnd - tInit > 5000){
    tInit = tEnd;
    notifyClients();
  }
  if (enableOTA) {
    ArduinoOTA.handle();
  }
  if (Serial.available()) {
    char comando = Serial.read();
    if (comando == '\n') {
      Serial.printf("Serial: %s\n", buf);
      parseCmd();
      Serial.println("____________________________");
    } else {
      buf[bufindex++] = comando;
    }
  }
  if (mqttmessageAvailable) {
    Serial.printf("MQTT: %s\n", buf);
    parseCmd();
  }
  ws.cleanupClients();
  tEnd = millis();
}
