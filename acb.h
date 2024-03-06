#include "esp32-hal.h"
#ifndef ROBOT
#define ROBOT

#include "defines.h"

void publishPosition() {
  cmdJson.clear();
  cmdJson["x"] = curX;
  cmdJson["y"] = curY;
  cmdJson["z"] = curZ;
  char output[128];
  serializeJson(cmdJson, output);
  clientMqtt.publish("status", 0, false, output);
}

void initPosition() {  // Posicion al encender el robot
                       // Leer ultima longitud registrada
  cmdJson.clear();
  deserializeJson(cmdJson, readFile(configPath));
  float aux;
  // Inicialización de variables:
  posA = 0;
  posB = 0;
  curX = 0;
  curY = 0;
  curZ = 0;  // Posición actual XYZ del Efector-Final
  posC = 0;
  posD = 0;  // Posición de los motores ABCD
  tarA = 0;
  tarB = 0;
  tarC = 0;
  tarD = 0;  // Posición de los motores ABCD objetivo

  // Calculo de la longitud de los cables:

  // Cable A

  pzCA = pzCB = pzCC = pzCD = (robotSetup.data.length / 2) - (robotSetup.data.ef_length / 2);  //  L/2 -  l/2

  pxCAcg = (-robotSetup.data.width / 2) - (-robotSetup.data.ef / 2);      // -W/2 - -w/2
  pyCAcg = (robotSetup.data.height / 2) - (robotSetup.data.ef / 2);       //  H/2 -  h/2

  angCA = atan(pyCAcg / pxCAcg);  // ángulo en radianes
  angCA = angCA > 0 ? angCA : -angCA;

  CompRGXCA = RGp * cos(angCA);  // compensacion de giro
  CompRGYCA = RGp * sin(angCA);

  pxCA = pxCAcg + (2 * CompRGXCA);
  pyCA = pyCAcg - (2 * CompRGYCA);

  initStringCA = curCA = sqrt((pxCA * pxCA) + (pyCA * pyCA) + (pzCA * pzCA));
  aux = cmdJson["curCA"];
  curCA = aux != 0.00 ? aux : curCA;

  // Cable B

  pxCBcg = (robotSetup.data.width / 2) - (robotSetup.data.ef / 2);        // W/2 - w/2
  pyCBcg = (robotSetup.data.height / 2) - (robotSetup.data.ef / 2);       // H/2 - h/2

  angCB = atan(pyCBcg / pxCBcg);  // ángulo en radianes
  angCB = angCB > 0 ? angCB : -angCB;

  CompRGXCB = RGp * cos(angCB);  // compensacion de giro
  CompRGYCB = RGp * sin(angCB);

  pxCB = (robotSetup.data.width / 2 - (CompRGXCB)) - (robotSetup.data.ef / 2 + (CompRGXCB));   // W/2 - w/2
  pyCB = (robotSetup.data.height / 2 - (CompRGYCB)) - (robotSetup.data.ef / 2 + (CompRGYCB));  // H/2 - h/2

  initStringCB = curCB = sqrt((pxCB * pxCB) + (pyCB * pyCB) + (pzCB * pzCB));
  aux = cmdJson["curCB"];
  curCB = aux != 0 ? aux : curCB;

  // Cable C

  pxCCcg = (robotSetup.data.width / 2) - (robotSetup.data.ef / 2);        //  W/2 -  w/2
  pyCCcg = (-robotSetup.data.height / 2) - (-robotSetup.data.ef / 2);     // -H/2 - -h/2

  angCC = atan(pyCCcg / pxCCcg);  // ángulo en radianes
  angCC = angCC > 0 ? angCC : -angCC;

  CompRGXCC = RGp * cos(angCC);  // compensacion de giro
  CompRGYCC = RGp * sin(angCC);

  pxCC = (robotSetup.data.width / 2 - (CompRGXCC)) - (robotSetup.data.ef / 2 + (CompRGXCC));     //  W/2 -  w/2
  pyCC = (-robotSetup.data.height / 2 + (CompRGYCC)) - (-robotSetup.data.ef / 2 - (CompRGYCC));  // -H/2 - -h/2

  initStringCC = curCC = sqrt((pxCC * pxCC) + (pyCC * pyCC) + (pzCC * pzCC));
  aux = cmdJson["curCC"];
  curCC = aux != 0 ? aux : curCC;

  // Cable D

  pxCDcg = (-robotSetup.data.width / 2) - (-robotSetup.data.ef / 2);      // -W/2 - -w/2
  pyCDcg = (-robotSetup.data.height / 2) - (-robotSetup.data.ef / 2);     // -H/2 - -h/2

  angCD = atan(pyCDcg / pxCDcg);  // ángulo en radianes
  angCD = angCD > 0 ? angCD : -angCD;

  CompRGXCD = RGp * cos(angCD);  // compensacion de giro
  CompRGYCD = RGp * sin(angCD);

  pxCD = (-robotSetup.data.width / 2 + (CompRGXCD)) - (-robotSetup.data.ef / 2 - (CompRGXCD));   // -W/2 - -w/2
  pyCD = (-robotSetup.data.height / 2 + (CompRGYCD)) - (-robotSetup.data.ef / 2 - (CompRGYCD));  // -H/2 - -h/2

  initStringCD = curCD = sqrt((pxCD * pxCD) + (pyCD * pyCD) + (pzCD * pzCD));
  aux = cmdJson["curCD"];
  curCD = aux != 0 ? aux : curCD;

  aux = cmdJson["curX"];
  curX = aux;
  aux = cmdJson["curY"];
  curY = aux;
  aux = cmdJson["curZ"];
  curZ = aux;
  aux = cmdJson["posA"];
  posA = aux;
  aux = cmdJson["posB"];
  posB = aux;
  aux = cmdJson["posC"];
  posC = aux;
  aux = cmdJson["posD"];
  posD = aux;

  publishPosition();

  Serial.println("Longitud en X0 Y0 Z0");
  Serial.printf("Cable A: %0.4f mm, ", initStringCA);
  Serial.printf("Cable B: %0.4f mm, ", initStringCB);
  Serial.printf("Cable C: %0.4f mm, ", initStringCC);
  Serial.printf("Cable D: %0.4f mm\n", initStringCD);

  Serial.printf("Longitud en X%0.1f Y%0.1f Z%0.1f\n", curX, curY, curZ);
  Serial.printf("Cable A: %0.4f mm, ", curCA);
  Serial.printf("Cable B: %0.4f mm, ", curCB);
  Serial.printf("Cable C: %0.4f mm, ", curCC);
  Serial.printf("Cable D: %0.4f mm\n", curCD);
}

void initRobotSetup() {
  pinMode(A_STEP_PIN, OUTPUT);
  pinMode(A_DIR_PIN, OUTPUT);
  pinMode(A_ENABLE_PIN, OUTPUT);
  pinMode(B_STEP_PIN, OUTPUT);
  pinMode(B_DIR_PIN, OUTPUT);
  pinMode(B_ENABLE_PIN, OUTPUT);
  pinMode(C_STEP_PIN, OUTPUT);
  pinMode(C_DIR_PIN, OUTPUT);
  pinMode(C_ENABLE_PIN, OUTPUT);
  pinMode(D_STEP_PIN, OUTPUT);
  pinMode(D_DIR_PIN, OUTPUT);
  pinMode(D_ENABLE_PIN, OUTPUT);

  pinMode(BUZZER, OUTPUT);

  pinDisabler();

  robotSetup.data.height = HEIGHT;
  robotSetup.data.width = WIDTH;
  robotSetup.data.length = LENGTH;
  robotSetup.data.ef = EF;
  robotSetup.data.ef_length = EF_LENGTH;
}

void savePosition() {
  cmdJson.clear();
  cmdJson["curCA"] = curCA;
  cmdJson["curCB"] = curCB;
  cmdJson["curCC"] = curCC;
  cmdJson["curCD"] = curCD;
  cmdJson["curX"] = curX;
  cmdJson["curY"] = curY;
  cmdJson["curZ"] = curZ;
  char output[128];
  serializeJson(cmdJson, output);
  writeFile(configPath, output);
}

void doMove() {  // Convierte la distancia en pasos y hace el movimiento
  Serial.println("doMove");
  long tini = millis();
  long tiniA = tini;
  long mDelay = stepdelay_max;
  long speedDiff = -SPEED_STEP;
  long maxDA1, maxDA2, maxD;  // Numero maximo de pasos
  long dA, dB, dC, dD;
  long abs_dA, abs_dB, abs_dC, abs_dD;           // |Delta ABDC|
  float stepA, stepB, stepC, stepD;              // Tamaño del paso ABCD
  float cntA = 0, cntB = 0, cntC = 0, cntD = 0;  // Contador de pasos ABCD
  bool d;

  dA = tarA;
  dB = tarB;
  dC = tarC;
  dD = tarD;

  abs_dA = dA > 0 ? dA : -dA;
  abs_dB = dB > 0 ? dB : -dB;
  abs_dC = dC > 0 ? dC : -dC;
  abs_dD = dD > 0 ? dD : -dD;

  // Numero maximo de pasos
  maxDA1 = max(abs_dA, abs_dB);
  maxDA2 = max(abs_dC, abs_dD);
  maxD = max(maxDA1, maxDA2);

  // Tamaño del paso ABCD
  stepA = (float)abs_dA / (float)maxD;
  stepB = (float)abs_dB / (float)maxD;
  stepC = (float)abs_dC / (float)maxD;
  stepD = (float)abs_dD / (float)maxD;

  Serial.printf("dA: A(%ld) B(%ld) C(%ld) D(%ld)\n", dA, dB, dC, dD);
  Serial.printf("tar: A(%ld) B(%ld) C(%ld) D(%ld)\n", tarA, tarB, tarC, tarD);

  // Contar pasos movidos mientras sea necesario
  for (long i = 0; (posA != tarA) || (posB != tarB) || (posC != tarC) || (posD != tarD); i++) {
    digitalWrite(4, LOW);
    delayMicroseconds(50);
    if ((millis() - tiniA) / 1000 > 1) {
      Serial.printf("pos: A(%ld) B(%ld) C(%ld) D(%ld)\n", posA, posB, posC, posD);
      tiniA = millis();
    }
    if (posA != tarA) {
      cntA += stepA;

      if (cntA >= 1) {
        d = dA > 0;
        posA += (dA > 0 ? 1 : -1);
        stepperA(d);
        cntA -= 1;
      }
    }

    if (posB != tarB) {
      cntB += stepB;

      if (cntB >= 1) {
        d = dB > 0;
        posB += (dB > 0 ? 1 : -1);
        stepperB(d);
        cntB -= 1;
      }
    }

    if (posC != tarC) {
      cntC += stepC;

      if (cntC >= 1) {
        d = dC > 0;
        posC += (dC > 0 ? 1 : -1);
        stepperC(d);
        cntC -= 1;
      }
    }

    if (posD != tarD) {
      cntD += stepD;

      if (cntD >= 1) {
        d = dD > 0;
        posD += (dD > 0 ? 1 : -1);
        stepperD(d);
        cntD -= 1;
      }
    }

    mDelay = constrain(mDelay + speedDiff, stepdelay_min, stepdelay_max) + stepAuxDelay;
    if (mDelay > 10000) {
      delay(mDelay / 1000);
      delayMicroseconds(mDelay % 1000);
    } else {
      delayMicroseconds(mDelay);
    }
    if ((maxD - i) < ((stepdelay_max - stepdelay_min) / SPEED_STEP)) {
      speedDiff = SPEED_STEP;
    }
  }
  digitalWrite(4, HIGH);
  posA -= tarA;
  posB -= tarB;
  posC -= tarC;
  posD -= tarD;
  Serial.printf("pos: A(%ld), B(%ld), C(%ld), D(%ld)\n", posA, posB, posC, posD);
  Serial.println((millis() - tini) / 1000);
}
void prepareMove() {  // Obtiene la distancia necesaria de los cables
  Serial.println("prepareMove");
  if (MOM == 1) {
    Serial.printf("tar: A(%ld), B(%ld), C(%ld), D(%ld)\n", tarA, tarB, tarC, tarD);

    doMove();

    MOM = 0;
    return;
  }

  float dx = tarX - curX;
  float dy = tarY - curY;
  float dz = tarZ - curZ;
  float distance = sqrt(dx * dx + dy * dy + dz * dz);

  Serial.printf("Distancia a recorrer: %0.4f\n", distance);

  if (distance < 0.1) {
    return;
  }

  // Longitud del cable A

  float CAx = pxCAcg - tarX;
  float CAy = pyCAcg - tarY;
  float CAz = pzCA - tarZ;

  angCA = atan(CAy / CAx);  // angulo en radianes
  angCA = angCA > 0 ? angCA : -angCA;

  CompRGXCA = RGp * cos(angCA);
  CompRGYCA = RGp * sin(angCA);

  CAx = (CAx + 2 * (CompRGXCA));
  CAy = (CAy - 2 * (CompRGYCA));

  tarCA = sqrt(CAx * CAx + CAy * CAy + CAz * CAz);

  // Longitud del cable B

  float CBx = pxCBcg - tarX;
  float CBy = pyCBcg - tarY;
  float CBz = pzCB - tarZ;

  angCB = atan(CBy / CBx);  // angulo en radianes
  angCB = angCB > 0 ? angCB : -angCB;

  CompRGXCB = RGp * cos(angCB);
  CompRGYCB = RGp * sin(angCB);

  CBx = (CBx - 2 * (CompRGXCB));
  CBy = (CBy - 2 * (CompRGYCB));

  tarCB = sqrt(CBx * CBx + CBy * CBy + CBz * CBz);

  // Longitud del cable C

  float CCx = pxCCcg - tarX;
  float CCy = pyCCcg - tarY;
  float CCz = pzCC - tarZ;

  angCC = atan(CCy / CCx);  // angulo en radianes
  angCC = angCC > 0 ? angCC : -angCC;

  CompRGXCC = RGp * cos(angCC);
  CompRGYCC = RGp * sin(angCC);

  CCx = (CCx - 2 * (CompRGXCC));
  CCy = (CCy + 2 * (CompRGYCC));

  tarCC = sqrt(CCx * CCx + CCy * CCy + CCz * CCz);

  // Longitud del cable D

  float CDx = pxCDcg - tarX;
  float CDy = pyCDcg - tarY;
  float CDz = pzCD - tarZ;

  angCD = atan(CDy / CDx);  // ángnlo en radianes
  angCD = angCD > 0 ? angCD : -angCD;

  CompRGXCD = RGp * cos(angCD);
  CompRGYCD = RGp * sin(angCD);

  CDx = (CDx + 2 * (CompRGXCD));
  CDy = (CDy + 2 * (CompRGYCD));

  tarCD = sqrt(CDx * CDx + CDy * CDy + CDz * CDz);

  tarA = (tarCA - curCA) * STEPS_PER_MM;
  tarB = (tarCB - curCB) * STEPS_PER_MM;
  tarC = (tarCC - curCC) * STEPS_PER_MM;
  tarD = (tarCD - curCD) * STEPS_PER_MM;

  Serial.printf("tar: CA(%0.4f), CB(%0.4f), CC(%0.4f), CD(%0.4f)\n", tarCA, tarCB, tarCC, tarCD);
  Serial.printf("cur: CA(%0.4f), CB(%0.4f), CC(%0.4f), CD(%0.4f)\n", curCA, curCB, curCC, curCD);

  Serial.printf("tar: A(%ld), B(%ld), C(%ld), D(%ld)\n", tarA, tarB, tarC, tarD);

  doMove();

  curCA = tarCA;
  curCB = tarCB;
  curCC = tarCC;
  curCD = tarCD;
  curX = tarX;
  curY = tarY;
  curZ = tarZ;
}

void parseMotorOffset(char *cmd) {
  char *token;
  char *rest = cmd;
  tarA = 0;
  tarB = 0;
  tarC = 0;
  tarD = 0;
  posA = 0;
  posB = 0;
  posC = 0;
  posD = 0;
  while ((token = strtok_r(rest, " ", &rest))) {
    if (String(token).charAt(0) == 'A') {
      tarA = atoi(token + 1) * -STEPS_PER_MM;
      MOM = 1;
    } else if (String(token).charAt(0) == 'B') {
      tarB = atoi(token + 1) * -STEPS_PER_MM;
      MOM = 1;
    } else if (String(token).charAt(0) == 'C') {
      tarC = atoi(token + 1) * -STEPS_PER_MM;
      MOM = 1;
    } else if (String(token).charAt(0) == 'D') {
      tarD = atoi(token + 1) * -STEPS_PER_MM;
      MOM = 1;
    }
  }
  prepareMove();
}

void parseCoordinate(char *cmd) {
  char *token;
  char *rest = cmd;
  while ((token = strtok_r(rest, " ", &rest))) {
    //delay(10);  // Delay de soporte, Sin él, el esp se crashea
    if (String(token).charAt(0) == 'X') {
      tarX = atoi(token + 1);  // X Objetivo
    } else if (String(token).charAt(0) == 'Y') {
      tarY = atoi(token + 1);  // Y Objetivo
    } else if (String(token).charAt(0) == 'Z') {
      tarZ = atoi(token + 1);  // Z Objetivo
    }
  }
  prepareMove();
}

void echoRobotSetup() {
  Serial.print("Altura del cuerpo del robot: ");
  Serial.println(robotSetup.data.height);
  Serial.print("Ancho del cuerpo del robot: ");
  Serial.println(robotSetup.data.width);
  Serial.print("Largo del cuerpo del robot: ");
  Serial.println(robotSetup.data.length);
  Serial.print("Ancho y Largo del efector: ");
  Serial.println(robotSetup.data.ef);
  Serial.print("Altura del efector: ");
  Serial.println(robotSetup.data.ef_length);
}

void echoConnectionSetup() {
  if (WiFi.isConnected()) {
    Serial.printf("SSID: %s, LocalIP: %s\n", ssid, WiFi.localIP().toString());
    Serial.printf("MQTT Broker: %s\n", mqttHost);
  } else {
    Serial.printf("AP: %s, IP: %s\n", ssid, WiFi.softAPIP().toString());
  }
}

void echoRobotStatus() {
  Serial.printf("Posicion actual del efector: \nA %0.4f, B %0.4f, C %0.4f\n", curX, curY, curZ);
  Serial.printf("Longitud actual del cable: \nA %0.4f, B %0.4f, C %0.4f, D %0.4f\n", curCA, curCB, curCC, curCD);
  Serial.printf("pos: A(%ld), B(%ld), C(%ld), D(%ld)\n", posA, posB, posC, posD);
}

void setupConnection(char *cmd) {
  char *token;
  char *rest = cmd;
  while ((token = strtok_r(rest, " ", &rest))) {
    if (String(token).charAt(0) == 'S') {
      ssid = String(token).substring(1);
      writeFile(ssidPath, ssid.c_str());
    } else if (String(token).charAt(0) == 'P') {
      password = String(token).substring(1);
      writeFile(passPath, password.c_str());
    } else if (String(token).charAt(0) == 'M') {
      mqttHost = String(token).substring(1);
      writeFile(mqttHostPath, mqttHost.c_str());
    }
  }
  ESP.restart();
}

void resetConfig() {
  writeFile(configPath, " ");
  curX = 0;
  curY = 0;
  curZ = 0;
  tarX = 0;
  tarY = 0;
  tarZ = 0;
  initPosition();
  tarA = 0;
  tarB = 0;
  tarC = 0;
  tarD = 0;
  posA = 0;
  posB = 0;
  posC = 0;
  posD = 0;
  Serial.println("Estado reiniciado");
  //ESP.restart();
}

void parseGcode(char *cmd) {
  int code;
  code = atoi(cmd);
  Serial.printf("Code: G%u \n", code);
      pinEnabler();

  switch (code) {
    case 1:  // Mover
      parseCoordinate(cmd);
      break;
    case 10:  // Mover individualmente cada motor un numero de pasos dado
      parseMotorOffset(cmd);  // Mover individualmente cada motor un numero de pasos dado
      break;
    case 28:               // Ir home
      tarX = 0;
      tarY = 0;
      tarZ = 0;
      prepareMove();
      break;
    default:
      break;
  }
      //pinDisabler();

  savePosition();
  publishPosition();
}

void parseMcode(char *cmd) {
  int code;
  code = atoi(cmd);
  Serial.printf("Code: M%u \n", code);
  switch (code) {
    case 1:  // Imprimir configuracion robot
      echoRobotSetup();
      break;
    case 2:  // Imprimir estado del robot
      echoRobotStatus();
      break;
    case 3:  // Imprimir estado de la conexion
      echoConnectionSetup();
      break;
    case 4:  // Configurar conexion Sssid Ppassword Mmqtthost
      setupConnection(cmd);
      break;
    case 5:
      publishPosition();
      break;
    case 6:
      resetConfig(); // Eliminar configuracion de posicion
      break;
    case 7:
      testPines(true); // Todos los pines en HIGH
      break;
    case 8:
      testPines(false); // Todos los pines en LOW
      break;
    default:
      break;
  }
}

void parseCmd() {
  char *cmd = buf;
  if (cmd[0] == 'G') {
    parseGcode(cmd + 1);
    Serial.println("OK");
    clientMqtt.publish("status", 0, false, "{\"esp\":\"OK\"}");
  } else if (cmd[0] == 'M') {
    parseMcode(cmd + 1);
    Serial.println("OK");
    clientMqtt.publish("status", 0, false, "{\"esp\":\"OK\"}");
  }
  mqttmessageAvailable = false;
  resetBuff();
}

#include "connection.h"

#endif
