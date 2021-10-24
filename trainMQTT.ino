#include <SD.h>
#include <SPI.h>
#include <WiFi101.h>
#include <PubSubClient.h>
#include <IniFile.h>
#include "arduino_secrets.h"
#include <tjf_mcp23017.h>
#include <PCA9685.h>
#include <Wire.h>

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

void ProcLED(int lPin, int lVal) {
  mcp.digitalWrite(lPin,lVal);
  if (debugSys) Serial.print("LED ");
  if (debugSys) Serial.print(lPin);
  if (debugSys) Serial.print(": ");
  if (debugSys) Serial.println(lVal);
}

void ProcServo(int sPin, int sVal) {
  servo.setAngle(sPin, sVal);
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
