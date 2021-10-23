#include <SPI.h>
#include <WiFi101.h>
#include <PubSubClient.h>
#include "arduino_secrets.h"

// Update these with values suitable for your hardware/network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xEC };
IPAddress server(192, 168, 1, 171);

// WiFi card example
char ssid[] = WSSID;    // your SSID
char pass[] = WPSWD;       // your SSID Password

void procTurnout(int turnoutID, int turnoutValue) {
  Serial.print("Turnout ");
  Serial.print(turnoutID);
  Serial.print(": ");
  Serial.println(turnoutValue);

}

void procLight(int lightID, int lightValue) {
  Serial.print("Light ");
  Serial.print(lightID);
  Serial.print(": ");
  Serial.println(lightValue);

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
      procLight(tmpLight,1);
    } else {
      procLight(tmpLight,0);
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

WiFiClient wClient;
PubSubClient client(wClient);

long lastReconnectAttempt = 0;


boolean reconnect() {
  if (client.connect("arduinoClient3")) {
    client.subscribe("trains/track/light/#");
    client.subscribe("trains/track/turnout/#");





  }
  return client.connected();
}

void setup()
{
  Serial.begin (115200);

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

  Serial.println("Weather Display online!");

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
