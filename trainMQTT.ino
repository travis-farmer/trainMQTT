#include <SD.h>
#include <SPI.h>
#include <WiFi101.h>
#include <PubSubClient.h>
#include <IniFile.h>
#include "arduino_secrets.h"
#include <tjf_mcp23017.h>
#include <PCA9685.h>
#include <Wire.h>
#include "Nextion.h"

#define SD_SELECT 4
int cntSensor = 0;
int cntTurnout = 0;
int cntLight = 0;
int confTurnoutMap[32][5];
int confLightNum[32][2];
int confSensorNum[32][2];
bool debugSys = false;

const size_t bufferLen = 80;
char buffer[bufferLen];
const char *filename = "/config.ini";

IniFile ini(filename);
WiFiClient wClient;
PubSubClient client(wClient);
tjf_mcp23017 mcp(2);
ServoDriver servo;

NexText t1 = NexText(0, 17, "t1");
NexText t2 = NexText(0, 18, "t2");
NexText t3 = NexText(0, 19, "t3");
NexText t4 = NexText(0, 20, "t4");
NexText t5 = NexText(0, 21, "t5");
NexText t6 = NexText(0, 22, "t6");
NexText t7 = NexText(0, 23, "t7");
NexText t8 = NexText(0, 24, "t8");
NexText t9 = NexText(0, 25, "t9");
NexText t10 = NexText(0, 26, "t10");
NexText t11 = NexText(0, 27, "t11");
NexText t12 = NexText(0, 28, "t12");
NexText t13 = NexText(0, 29, "t13");
NexText t14 = NexText(0, 30, "t14");
NexText t15 = NexText(0, 31, "t15");
NexText t16 = NexText(0, 32, "t16");
NexText t17 = NexText(0, 33, "t17");
NexText t18 = NexText(0, 34, "t18");
NexText t19 = NexText(0, 35, "t19");
NexText t20 = NexText(0, 36, "t20");
NexText t21 = NexText(0, 37, "t21");
NexText t22 = NexText(0, 38, "t22");
NexText t23 = NexText(0, 39, "t23");
NexText t24 = NexText(0, 40, "t24");
NexText t25 = NexText(0, 41, "t25");
NexText t26 = NexText(0, 42, "t26");
NexText t27 = NexText(0, 43, "t27");
NexText t28 = NexText(0, 44, "t28");
NexText t29 = NexText(0, 45, "t29");
NexText t30 = NexText(0, 46, "t30");
NexText t31 = NexText(0, 47, "t31");
NexText t32 = NexText(0, 48, "t32");
NexText t33 = NexText(0, 49, "t33");
NexText t34 = NexText(0, 50, "t34");
NexText t35 = NexText(0, 51, "t35");
NexText t36 = NexText(0, 52, "t36");
NexText t37 = NexText(0, 53, "t37");
NexText t38 = NexText(0, 54, "t38");
NexText t39 = NexText(0, 55, "t39");
NexText t40 = NexText(0, 56, "t40");
NexText t41 = NexText(0, 57, "t41");
NexText t42 = NexText(0, 58, "t42");
NexText t43 = NexText(0, 59, "t43");
NexText t44 = NexText(0, 60, "t44");
NexText t45 = NexText(0, 61, "t45");
NexText t46 = NexText(0, 62, "t46");
NexText t47 = NexText(0, 63, "t47");
NexText t48 = NexText(0, 64, "t48");
NexText t49 = NexText(0, 65, "t49");
NexText t50 = NexText(0, 66, "t50");
NexText t51 = NexText(0, 67, "t51");
NexText t52 = NexText(0, 68, "t52");
NexText t53 = NexText(0, 69, "t53");
NexText t54 = NexText(0, 70, "t54");
NexText t55 = NexText(0, 71, "t55");
NexText t56 = NexText(0, 72, "t56");
NexText t57 = NexText(0, 73, "t57");
NexText t58 = NexText(0, 74, "t58");
NexText t59 = NexText(0, 75, "t59");
NexText t60 = NexText(0, 76, "t60");
NexText t61 = NexText(0, 77, "t61");
NexText t62 = NexText(0, 78, "t62");
NexText t63 = NexText(0, 79, "t63");
NexText t64 = NexText(0, 80, "t64");
NexGauge z0  = NexGauge(0, 1, "z0");
NexGauge z1  = NexGauge(0, 2, "z1");
NexGauge z2  = NexGauge(0, 3, "z2");
NexGauge z3  = NexGauge(0, 4, "z3");
NexGauge z4  = NexGauge(0, 5, "z4");
NexGauge z5  = NexGauge(0, 6, "z5");
NexGauge z6  = NexGauge(0, 7, "z6");
NexGauge z7  = NexGauge(0, 8, "z7");
NexGauge z8  = NexGauge(0, 9, "z8");
NexGauge z9  = NexGauge(0, 10, "z9");
NexGauge z10  = NexGauge(0, 11, "z10");
NexGauge z11  = NexGauge(0, 12, "z11");
NexGauge z12  = NexGauge(0, 13, "z12");
NexGauge z13  = NexGauge(0, 14, "z13");
NexGauge z14  = NexGauge(0, 15, "z14");
NexGauge z15  = NexGauge(0, 16, "z15");




// Update these with values suitable for your hardware/network.
IPAddress server(192, 168, 1, 171);

// from arduino_secrets.h
char ssid[] = WSSID;    // your SSID
char pass[] = WPSWD;       // your SSID Password

void readConf() {
  String strBuffer = "";
  if (ini.getValue("counts", "debug", buffer, bufferLen)) {
    strBuffer = buffer;
    if (strBuffer.toInt() == 1) debugSys = true;
  }
  if (ini.getValue("counts", "sensor", buffer, bufferLen)) {
    strBuffer = buffer;
    cntSensor = strBuffer.toInt();
  }
  if (ini.getValue("counts", "turnout", buffer, bufferLen)) {
    strBuffer = buffer;
    cntTurnout = strBuffer.toInt();
  }
  if (ini.getValue("counts", "light", buffer, bufferLen)) {
    strBuffer = buffer;
    cntLight = strBuffer.toInt();
  }
  for (int i=0; i<cntTurnout; i++) {
    char sz[4];
    sprintf(sz, "%d", i+1);
    if (ini.getValue("turnout", sz, buffer, bufferLen)) {
      if (debugSys) Serial.print("Turnout ");
      strBuffer = buffer;
      String tmpStr = "";
      int tmpInt = strBuffer.indexOf(":");
      tmpStr = strBuffer.substring(0,tmpInt);
      confTurnoutMap[i][0] = tmpStr.toInt();
      if (debugSys) Serial.print(tmpStr);
      if (debugSys) Serial.print(" - ");
      int oldTmpInt = tmpInt;
      tmpInt = strBuffer.indexOf(":",tmpInt + 1);
      tmpStr = strBuffer.substring(oldTmpInt+1,tmpInt);
      confTurnoutMap[i][1] = tmpStr.toInt();
      if (debugSys) Serial.print(tmpStr);
      if (debugSys) Serial.print(" - ");
      oldTmpInt = tmpInt;
      tmpInt = strBuffer.indexOf(":",tmpInt + 1);
      tmpStr = strBuffer.substring(oldTmpInt+1,tmpInt);
      confTurnoutMap[i][2] = tmpStr.toInt();
      if (debugSys) Serial.print(tmpStr);
      oldTmpInt = tmpInt;
      tmpInt = strBuffer.indexOf(":",tmpInt + 1);
      if (tmpInt > 1) {
        if (debugSys) Serial.print(" - ");
        tmpStr = strBuffer.substring(oldTmpInt+1,tmpInt);
        confTurnoutMap[i][3] = tmpStr.toInt();
        if (debugSys) Serial.print(tmpStr);
        if (debugSys) Serial.print(" - ");
        oldTmpInt = tmpInt;
        tmpInt = strBuffer.indexOf(":",tmpInt + 1);
        tmpStr = strBuffer.substring(oldTmpInt+1,tmpInt);
        confTurnoutMap[i][4] = tmpStr.toInt();
        if (debugSys) Serial.println(tmpStr);
      } else {
        tmpInt = 0;
        if (debugSys) Serial.println();
      }

    }
  }
  for (int i=0; i<cntSensor; i++) {
    char sz[4];
    sprintf(sz, "%d", i+1);
    if (ini.getValue("sensor", sz, buffer, bufferLen)) {
      if (debugSys) Serial.print("Sensor ");
      strBuffer = buffer;
      String tmpStrB = "";
      int tmpIntB = strBuffer.indexOf(":");
      tmpStrB = strBuffer.substring(0,tmpIntB);
      if (debugSys) Serial.print(tmpStrB);
      if (debugSys) Serial.print(" - ");
      confSensorNum[i][0] = tmpStrB.toInt();
      int oldTmpIntB = tmpIntB;
      tmpIntB = strBuffer.indexOf(":",tmpIntB + 1);
      tmpStrB = strBuffer.substring(oldTmpIntB+1,tmpIntB);
      if (debugSys) Serial.println(tmpStrB);
      confSensorNum[i][1] = tmpStrB.toInt();
    }
  }
  for (int i=0; i<cntLight; i++) {
    char sz[4];
    sprintf(sz, "%d", i+1);
    if (ini.getValue("light", sz, buffer, bufferLen)) {
      strBuffer = buffer;
      String tmpStrC = "";
      int tmpIntC = strBuffer.indexOf(":");
      tmpStrC = strBuffer.substring(0,tmpIntC);
      confLightNum[i][0] = tmpStrC.toInt();
      int oldTmpIntC = tmpIntC;
      tmpIntC = strBuffer.indexOf(":",tmpIntC + 1);
      tmpStrC = strBuffer.substring(oldTmpIntC+1,tmpIntC);
      confLightNum[i][1] = tmpStrC.toInt();
    }
  }


  // set GPIO
  //Turnouts (servos are ignored, as they will be PCA9685 based)
  //outputs will eventually be MCP23017 based
  for (int i =0; i<cntTurnout;i++) {
    if (confTurnoutMap[i][0] == 0) {
      mcp.pinMode(confTurnoutMap[i][2],OUTPUT);
    }
  }

  // factor in the Lights
  for (int i=0; i<cntLight;i++) {
    mcp.pinMode(confLightNum[i],OUTPUT);
  }

  // now sensors
  for (int i=0; i<cntSensor;i++) {
    mcp.pinMode(confSensorNum[i],INPUT_PULLUP);
  }
}

void setNexLED(int Led, bool Val) {
  if (Led == 1) t1.setText(Val ? "1" : "0");
  else if (Led == 2) t2.setText(Val ? "1" : "0");
  else if (Led == 3) t3.setText(Val ? "1" : "0");
  else if (Led == 4) t4.setText(Val ? "1" : "0");
  else if (Led == 5) t5.setText(Val ? "1" : "0");
  else if (Led == 6) t6.setText(Val ? "1" : "0");
  else if (Led == 7) t7.setText(Val ? "1" : "0");
  else if (Led == 8) t8.setText(Val ? "1" : "0");
  else if (Led == 9) t9.setText(Val ? "1" : "0");
  else if (Led == 10) t11.setText(Val ? "1" : "0");
  else if (Led == 11) t11.setText(Val ? "1" : "0");
  else if (Led == 12) t12.setText(Val ? "1" : "0");
  else if (Led == 13) t13.setText(Val ? "1" : "0");
  else if (Led == 14) t14.setText(Val ? "1" : "0");
  else if (Led == 15) t15.setText(Val ? "1" : "0");
  else if (Led == 16) t16.setText(Val ? "1" : "0");
  else if (Led == 17) t17.setText(Val ? "1" : "0");
  else if (Led == 18) t18.setText(Val ? "1" : "0");
  else if (Led == 19) t19.setText(Val ? "1" : "0");
  else if (Led == 20) t20.setText(Val ? "1" : "0");
  else if (Led == 21) t21.setText(Val ? "1" : "0");
  else if (Led == 22) t22.setText(Val ? "1" : "0");
  else if (Led == 23) t23.setText(Val ? "1" : "0");
  else if (Led == 24) t24.setText(Val ? "1" : "0");
  else if (Led == 25) t25.setText(Val ? "1" : "0");
  else if (Led == 26) t26.setText(Val ? "1" : "0");
  else if (Led == 27) t27.setText(Val ? "1" : "0");
  else if (Led == 28) t28.setText(Val ? "1" : "0");
  else if (Led == 29) t29.setText(Val ? "1" : "0");
  else if (Led == 30) t30.setText(Val ? "1" : "0");
  else if (Led == 31) t31.setText(Val ? "1" : "0");
  else if (Led == 32) t32.setText(Val ? "1" : "0");
  else if (Led == 33) t33.setText(Val ? "1" : "0");
  else if (Led == 34) t34.setText(Val ? "1" : "0");
  else if (Led == 35) t35.setText(Val ? "1" : "0");
  else if (Led == 36) t36.setText(Val ? "1" : "0");
  else if (Led == 37) t37.setText(Val ? "1" : "0");
  else if (Led == 38) t38.setText(Val ? "1" : "0");
  else if (Led == 39) t39.setText(Val ? "1" : "0");
  else if (Led == 40) t40.setText(Val ? "1" : "0");
  else if (Led == 41) t41.setText(Val ? "1" : "0");
  else if (Led == 42) t42.setText(Val ? "1" : "0");
  else if (Led == 43) t43.setText(Val ? "1" : "0");
  else if (Led == 44) t44.setText(Val ? "1" : "0");
  else if (Led == 45) t45.setText(Val ? "1" : "0");
  else if (Led == 46) t46.setText(Val ? "1" : "0");
  else if (Led == 47) t47.setText(Val ? "1" : "0");
  else if (Led == 48) t48.setText(Val ? "1" : "0");
  else if (Led == 49) t49.setText(Val ? "1" : "0");
  else if (Led == 50) t50.setText(Val ? "1" : "0");
  else if (Led == 51) t51.setText(Val ? "1" : "0");
  else if (Led == 52) t52.setText(Val ? "1" : "0");
  else if (Led == 53) t53.setText(Val ? "1" : "0");
  else if (Led == 54) t54.setText(Val ? "1" : "0");
  else if (Led == 55) t55.setText(Val ? "1" : "0");
  else if (Led == 56) t56.setText(Val ? "1" : "0");
  else if (Led == 57) t57.setText(Val ? "1" : "0");
  else if (Led == 58) t58.setText(Val ? "1" : "0");
  else if (Led == 59) t59.setText(Val ? "1" : "0");
  else if (Led == 60) t60.setText(Val ? "1" : "0");
  else if (Led == 61) t61.setText(Val ? "1" : "0");
  else if (Led == 62) t62.setText(Val ? "1" : "0");
  else if (Led == 63) t63.setText(Val ? "1" : "0");
  else if (Led == 64) t64.setText(Val ? "1" : "0");
}

void setNexServo(int Pin, int Val) {
  Pin -=1;
  if (Pin == 0) z0.setValue(Val);
  else if (Pin == 1) z1.setValue(Val);
  else if (Pin == 2) z2.setValue(Val);
  else if (Pin == 3) z3.setValue(Val);
  else if (Pin == 4) z4.setValue(Val);
  else if (Pin == 5) z5.setValue(Val);
  else if (Pin == 6) z6.setValue(Val);
  else if (Pin == 7) z7.setValue(Val);
  else if (Pin == 8) z8.setValue(Val);
  else if (Pin == 9) z9.setValue(Val);
  else if (Pin == 10) z10.setValue(Val);
  else if (Pin == 11) z11.setValue(Val);
  else if (Pin == 12) z12.setValue(Val);
  else if (Pin == 13) z13.setValue(Val);
  else if (Pin == 14) z14.setValue(Val);
  else if (Pin == 15) z15.setValue(Val);
}

void ProcLED(int lPin, int lVal) {
  mcp.digitalWrite(lPin,lVal);
  setNexLED(lPin,lVal);
  if (debugSys) Serial.print("LED ");
  if (debugSys) Serial.print(lPin);
  if (debugSys) Serial.print(": ");
  if (debugSys) Serial.println(lVal);
}

void ProcServo(int sPin, int sVal) {
  servo.setAngle(sPin, sVal);
  setNexServo(sPin, sVal);
  if (debugSys) Serial.print("SERVO ");
  if (debugSys) Serial.print(sPin);
  if (debugSys) Serial.print(": ");
  if (debugSys) Serial.println(sVal);
}

void procTurnout(int turnoutID, int turnoutValue) {
  for (int i=0; i<cntTurnout;i++) {
    if (confTurnoutMap[i][1] == turnoutID) {
      if (confTurnoutMap[i][0] == 0) {
        ProcLED(confTurnoutMap[i][2],turnoutValue);
      } else {
        ProcServo(confTurnoutMap[i][2],turnoutValue ? confTurnoutMap[i][3] : confTurnoutMap[i][4]);
      }
      break;
    }
  }
}

void sendSensors() {
  char topic[32];
  for (int i=0; i<cntSensor;i++) {

    sprintf(topic, "trains/track/sensor/%d", confSensorNum[i][0]);
    if (mcp.digitalRead(confSensorNum[i][1]) == LOW) {
      client.publish(topic,"ACTIVE");
    } else {
      client.publish(topic,"INACTIVE");
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String tmpTopic = topic;
  char tmpStr[length+1];
  for (int x=0; x<length; x++) {
    tmpStr[x] = (char)payload[x]; // payload is a stream, so we need to chop it by length.
  }
  tmpStr[length] = 0x00; // terminate the char string with a null
  String Value = tmpStr;

  if (tmpTopic.startsWith("trains/track/light/")) {
    int tmpLight = 0;
    tmpTopic.remove(0,19);
    tmpLight = tmpTopic.toInt();
    for (int i=0; i<cntLight;i++) {
      if (confLightNum[i][0] == tmpLight) {
        ProcLED(confLightNum[i][1],(Value == "ON") ? 1 : 0);
        if (debugSys) Serial.print("LIGHT ");
        if (debugSys) Serial.print(confLightNum[i][1]);
        if (debugSys) Serial.print(": ");
        if (debugSys) Serial.println((Value == "ON") ? 1 : 0);
        break;
      }
    }
  }
  else if (tmpTopic.startsWith("trains/track/turnout/")) {
    int tmpTurnout = 0;
    tmpTopic.remove(0,21);
    tmpTurnout = tmpTopic.toInt();
    if (Value == "THROWN") {
      procTurnout(tmpTurnout,1);
    } else {
      procTurnout(tmpTurnout,0);
    }
  }



}


long lastReconnectAttempt = 0;
unsigned long lastSensorReading = 0UL;

boolean reconnect() {
  if (client.connect("arduinoClient3")) {
    client.subscribe("trains/track/turnout/#");
    client.subscribe("trains/track/light/#");
  }
  return client.connected();
}

void setup()
{
  Serial.begin (115200);

  nexInit();
  if (!SD.begin(SD_SELECT))
    while (1)
      Serial.println("SD.begin() failed");

  if (!ini.open()) {
    Serial.print("Ini file ");
    Serial.print(filename);
    Serial.println(" does not exist");
    // Cannot do anything else
    while (1)
      ;
  }
  if (!ini.validate(buffer, bufferLen)) {
    Serial.print("ini file ");
    Serial.print(ini.getFilename());
    Serial.print(" not valid: ");
    // Cannot do anything else
    while (1)
      ;
  }
  mcp.addMCP(0x01); // address (001) of first MCP (decimal value of the binary equivalent of address pins A0, A1, A2)
  mcp.addMCP(0x02); // address (010) of second MCP
  mcp.begin();
  servo.init(0x7f);
  client.setServer(server, 1883);
  client.setCallback(callback);


  //WiFi.setPins(53,48,49);
  int status = WiFi.begin(ssid, pass);
  if ( status != WL_CONNECTED) {
    Serial.println("Couldn't get a wifi connection");
    while(true);
  }
  // print out info about the connection:
  else {
    Serial.println("Connected to network");
    IPAddress ip = WiFi.localIP();
    Serial.print("My IP address is: ");
    Serial.println(ip);
  }
  delay(1500);
  lastReconnectAttempt = 0;

  Serial.println("Train MQTT online!");

  readConf();
}

void loop()
{
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected

    client.loop();
  }

  if (millis()-lastSensorReading > 500) {
    lastSensorReading = millis();
    sendSensors();
  }
}
