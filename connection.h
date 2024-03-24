#ifndef CONNECTION
#define CONNECTION

#include <ESPmDNS.h>
#include <WiFi.h>

#include "acb.h"
#include "defines.h"

void createRoutes() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });
  server.on("/mqttHost", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/plain", mqttHost);
  });
  server.serveStatic("/", SPIFFS, "/");
  server.on("/changeWiFi", HTTP_POST, [](AsyncWebServerRequest* request) {
    int params = request->params();
    for (int i = 0; i < params; i++) {
      AsyncWebParameter* p = request->getParam(i);
      if (p->isPost()) {
        if (p->name() == "ssid") {
          ssid = p->value().c_str();
          writeFile(ssidPath, ssid.c_str());
        }
        if (p->name() == "pass") {
          password = p->value().c_str();
          writeFile(passPath, password.c_str());
        }
      }
    }
    request->send(200, "text/plain", "Se reiniciara con las credenciales de wifi nuevas");
    delay(5000);
    ESP.restart();
  });
  server.on("/changeMQTT", HTTP_POST, [](AsyncWebServerRequest* request) {
    int params = request->params();
    for (int i = 0; i < params; i++) {
      AsyncWebParameter* p = request->getParam(i);
      if (p->isPost()) {
        if (p->name() == "mqttHost") {
          mqttHost = p->value().c_str();
          writeFile(mqttHostPath, mqttHost.c_str());
        }
      }
    }
    clientMqtt.disconnect();
    request->redirect("/");
  });
  server.begin();
}

void notifyClients() {
  String data = readFile(configPath);
  ws.textAll(String(data));
}

bool startWiFi() {
  if (ssid == "") {
    return false;
  }
  Serial.printf("Conectando a: %s\n", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());

  unsigned long currentMillis = millis();
  previousMillis = currentMillis;

  while (WiFi.status() != WL_CONNECTED) {
    currentMillis = millis();
    Serial.printf("%lu ", (currentMillis - previousMillis) / 1000);
    if (currentMillis - previousMillis >= interval) {
      Serial.println(' ');
      return false;
    }
    delay(1000);
  }
  Serial.println(' ');
  return true;
}

void connectToMqtt() {
  clientMqtt.setCredentials("imacuna", "pi");
  if (mqttHost == "") {
    return;
  }
  clientMqtt.setServer(mqttHost.c_str(), 1883);
  Serial.printf("connectToMqtt: %s\n", mqttHost);
  clientMqtt.connect();
}

void connectToWifi() {
  if (startWiFi()) {
    Serial.printf("Conectado a: %s\n", ssid);
    Serial.printf("IP: %s\n", WiFi.localIP().toString());
  connectToMqtt();


  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Agrocablebot", "imacuna2023.");
    Serial.println("AP Creado");
    Serial.printf("IP: %s\n", WiFi.softAPIP().toString());
  }
  if (!MDNS.begin("agrocablebot")) {
    Serial.println("Error configurando MDNS");
  } else {
    Serial.println("MDNS Creado");
  }

}

void onMqttConnect(bool sessionPresent) {
  clientMqtt.subscribe("comandos", 0);
  clientMqtt.publish("status", 0, false, "{\"esp\":\"connected\"}");
  publishPosition();
  Serial.println("MQTT Conectado");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  if (WiFi.isConnected()) {
    connectToMqtt();
  }
}


void onMqttMessage(char* topic, char* payload,
                   const AsyncMqttClientMessageProperties& properties,
                   const size_t& len, const size_t& index,
                   const size_t& total) {
  String message;
  const char* command;
  if (!strcmp(topic, "comandos")) {
    for (int i = 0; i < len; i++) {
      message += payload[i];
    }
    cmdJson.clear();
    deserializeJson(cmdJson, message);
    Serial.println("MQTT on_message");
    if (cmdJson.containsKey("GCODE")) {
      command = cmdJson["GCODE"];
      String aux = String(command);
      int auxLen = strlen(command);
      for (int i = 0; i < auxLen; i++) {
        buf[i] = aux.charAt(i);
      }
      if (buf[0] == 'M') {
        parseCmd();
      } else {
        mqttmessageAvailable = true;
      }
    } else if (cmdJson.containsKey("command")) {
      if (cmdJson["command"] == "enableOTA") {
        enableOTA = true;
      }
    }
  }
}

void initWiFi() {
  SPIFFS.begin(true);

  ssid = readFile(ssidPath);
  password = readFile(passPath);
  mqttHost = readFile(mqttHostPath);

  clientMqtt.onConnect(onMqttConnect);
  clientMqtt.onDisconnect(onMqttDisconnect);
  clientMqtt.onMessage(onMqttMessage);

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");

  connectToWifi();
  createRoutes();
  server.addHandler(&ws);
}

#endif  // CONNECTION