#include <SD.h>
#include <SPI.h>
#include <WiFi101.h>
#include <PubSubClient.h>
#include <IniFile.h>
#include "arduino_secrets.h"
#include <tjf_mcp23017.h>
#include <PCA9685.h>
#include <Wire.h>
#include <EEPROM.h>

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

// WiFi card example
char ssid[] = WSSID;    // your SSID
char pass[] = WPSWD;       // your SSID Password

void readConf() {
  int eeReadBuffer[6];
  for (int i=0; i<510;i=i+6) { // 85 config lines... not very efficiant yet
    for (int j=0; j<6; j++) {
      EEPROM.get((i+j), eeReadBuffer[j]);
    }
    if (eeReadBuffer[0] == 255) break; // EOF, rest of data stored is bogus
    if (eeReadBuffer[0] == 1) {
      // mode is header
      // bytes are: debugSys,
      if (eeReadBuffer[1] == 1) debugSys = true;
    }
    else if (eeReadBuffer[0] == 2) {
      //mode is sensor config
      // Bytes: ID, MQTT pin, MCP pin
      confSensorNum[eeReadBuffer[1]][0] = eeReadBuffer[2];
      confSensorNum[eeReadBuffer[1]][1] = eeReadBuffer[3];
    }
    else if (eeReadBuffer[0] == 3) {
      //mode is light config
      // Bytes: ID, MQTT pin, MCP pin
      confLightNum[eeReadBuffer[1]][0] = eeReadBuffer[2];
      confLightNum[eeReadBuffer[1]][1] = eeReadBuffer[3];
    }
    else if (eeReadBuffer[0] == 4) {
      // mode is turnout config
      // Bytes: ID, servo(1)/LED(0), MQTT pin, out pin, servo on, servo off
      confTurnoutMap[eeReadBuffer[1]][0] = eeReadBuffer[2];
      confTurnoutMap[eeReadBuffer[1]][1] = eeReadBuffer[3];
      confTurnoutMap[eeReadBuffer[1]][2] = eeReadBuffer[4];
      confTurnoutMap[eeReadBuffer[1]][3] = eeReadBuffer[5];
      confTurnoutMap[eeReadBuffer[1]][4] = eeReadBuffer[6];

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
}
