#include <SD.h>
#include <SPI.h>
#include <WiFi101.h>
#include <PubSubClient.h>
#include <IniFile.h>
#include "arduino_secrets.h"

#define SD_SELECT 4
int cntSensor = 0;
int cntTurnout = 0;
int cntLight = 0;
int confTurnoutMap[32][5];
int confLightNum[32];
int confSensorNum[32];

const size_t bufferLen = 80;
char buffer[bufferLen];
const char *filename = "/config.ini";
IniFile ini(filename);

WiFiClient wClient;
PubSubClient client(wClient);

// Update these with values suitable for your hardware/network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xEC };
IPAddress server(192, 168, 1, 171);

// WiFi card example
char ssid[] = WSSID;    // your SSID
char pass[] = WPSWD;       // your SSID Password

void readConf() {
  String strBuffer = "";

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
      //Serial.print("Turnout ");
      strBuffer = buffer;
      String tmpStr = "";
      int tmpInt = strBuffer.indexOf(":");
      tmpStr = strBuffer.substring(0,tmpInt);
      confTurnoutMap[i][0] = tmpStr.toInt();
      //Serial.print(tmpStr);
      //Serial.print(" - ");
      int oldTmpInt = tmpInt;
      tmpInt = strBuffer.indexOf(":",tmpInt + 1);
      tmpStr = strBuffer.substring(oldTmpInt+1,tmpInt);
      confTurnoutMap[i][1] = tmpStr.toInt();
      //Serial.print(tmpStr);
      //Serial.print(" - ");
      oldTmpInt = tmpInt;
      tmpInt = strBuffer.indexOf(":",tmpInt + 1);
      tmpStr = strBuffer.substring(oldTmpInt+1,tmpInt);
      confTurnoutMap[i][2] = tmpStr.toInt();
      //Serial.print(tmpStr);
      oldTmpInt = tmpInt;
      tmpInt = strBuffer.indexOf(":",tmpInt + 1);
      if (tmpInt > 1) {
        //Serial.print(" - ");
        tmpStr = strBuffer.substring(oldTmpInt+1,tmpInt);
        confTurnoutMap[i][3] = tmpStr.toInt();
        //Serial.print(tmpStr);
        //Serial.print(" - ");
        oldTmpInt = tmpInt;
        tmpInt = strBuffer.indexOf(":",tmpInt + 1);
        tmpStr = strBuffer.substring(oldTmpInt+1,tmpInt);
        confTurnoutMap[i][4] = tmpStr.toInt();
        //Serial.println(tmpStr);
      } else {
        tmpInt = 0;
        //Serial.println();
      }

    }
  }
  for (int i=0; i<cntSensor; i++) {
    if (ini.getValue("sensor", i+1, buffer, bufferLen)) {
      strBuffer = buffer;
      confSensorNum[i] = strBuffer.toInt();
    }
  }
  for (int i=0; i<cntLight; i++) {
    if (ini.getValue("light", i+1, buffer, bufferLen)) {
      strBuffer = buffer;
      confLightNum[i] = strBuffer.toInt();
    }
  }

}



void procTurnout(int turnoutID, int turnoutValue) {
  String ledORservo = "";
  for (int i=0; i<cntTurnout;i++) {
    if (confTurnoutMap[i][1] == turnoutID) {
      if (confTurnoutMap[i][0] == 0) {
        // led
        ledORservo.remove(0,1);
        Serial.print("LED: ");
        Serial.print(confTurnoutMap[i][2]);
        Serial.print(" ");
        Serial.println(turnoutValue);
        //ProcLED(confTurnoutMap[i][2],turnoutValue);
      } else {
        // servo
        ledORservo.remove(0,1);
        Serial.print("SERVO: ");
        Serial.print(confTurnoutMap[i][2]);
        Serial.print(" ");
        Serial.println(turnoutValue ? confTurnoutMap[i][3] : confTurnoutMap[i][4]);
        //ProcServo(confTurnoutMap[i][2],turnoutValue);
      }
      break;
    }
  }
}

void sendSensor(int sensorID, bool sensorValue) {
  char topic[32];
  sprintf(topic, "trains/track/sensor/%s", confSensorNum[sensorID]);
  if (sensorValue == true) {
    client.publish(topic,"ACTIVE");
  } else {
    client.publish(topic,"INACTIVE");
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
    if (Value == "ON") {
      //procLight(tmpLight,1);
    } else {
      //procLight(tmpLight,0);
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


boolean reconnect() {
  if (client.connect("arduinoClient3")) {
    client.subscribe("trains/track/turnout/#");





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
}
