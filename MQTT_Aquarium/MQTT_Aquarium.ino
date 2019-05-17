#include <ESP8266WiFi.h>  
#include <ESP8266HTTPClient.h>

#include <ArduinoJson.h>          //Needed to store configuration as JSON: https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>       // used for MQTT

// declarations for MQTT and WiFi
char gMqttServer[40] = "xxx.xxx.xxx.xxx";
int  gMqttPort = 1883;
char gMqttUser[256] = "";
char gMqttPassword[256] = "";
char gMqttTopicSubscribe[256] = "aquarium/licht";

char gSSID[256] = "****";
char gWifiPassword[256] = "****";

const char* APPLICATION_NAME = "Aquarium";

HTTPClient http;
WiFiClient wifiClient;
PubSubClient mqttClient;

const int relayPin = D8;


void setup()   
{
  Serial.begin(115200);

  pinMode(relayPin,OUTPUT);
  
  // Connect to WiFi
  WiFi.begin(gSSID, gWifiPassword);

  Serial.print("Connecting to wireless network SSID: ");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  //if you get here you have connected to the WiFi
  Serial.println("");Serial.print("Connected to WIFI ("); Serial.print(WiFi.SSID()); Serial.print("), local ip: "); Serial.println(WiFi.localIP());
  
  mqttClient.setClient(wifiClient);
  mqttClient.setServer(gMqttServer, gMqttPort);
  mqttClient.setCallback(callback);
}
 
void loop() 
{
  if (!mqttClient.connected()) {
      reconnect();
    }
    mqttClient.loop();
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(APPLICATION_NAME)) {
      Serial.println("connected");
      // ... and resubscribe
      mqttClient.subscribe(gMqttTopicSubscribe);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  String mqttTopic = String(topic);

 
 if (mqttTopic == gMqttTopicSubscribe) {
      Serial.print("Relay status = ");
      setRelayStatus(payload);
  }
  else {
    Serial.println("Why am I here???");
  }
}

void setRelayStatus(byte* payload) {
  if ((char)payload[0] == '0') {
        Serial.println("OFF");
        digitalWrite(relayPin,LOW);
      }
      else {
        Serial.println("ON");
        digitalWrite(relayPin,HIGH);
      }
}


