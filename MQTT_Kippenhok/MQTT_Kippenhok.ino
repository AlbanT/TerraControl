#include <ESP8266WiFi.h>  
#include <ESP8266HTTPClient.h>

#include <ArduinoJson.h>          //Needed to store configuration as JSON: https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>       // used for MQTT

// declarations for MQTT and WiFi
char gMqttServer[40] = "xxx.xxx.xxx.xxx";
int  gMqttPort = 1883;
char gMqttUser[256] = "";
char gMqttPassword[256] = "";
char gMqttTopicSubscribe[256] = "timer";
char gMqttTopicPublish[256] = "kippenhok/temperatuur";

char gSSID[256] = "****";
char gWifiPassword[256] = "****";

const char* APPLICATION_NAME = "Kippenhok";

HTTPClient http;
WiFiClient wifiClient;
PubSubClient mqttClient;


#include <WEMOS_SHT3X.h>

SHT3X sht30(0x45);



void setup()   
{
  Serial.begin(115200);

  
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
      publishMQTT(gMqttTopicPublish,sensorData(payload));
  }
  else {
    Serial.println("Why am I here???");
  }
}

// reads the sensors and returns the values as JSON
char* sensorData(byte* timestamp){
  // Make sure the total message size does not exceed 128 bytes.
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  if(sht30.get()==0){
    Serial.print("Temperature in Celsius : ");
    Serial.println(sht30.cTemp);
    Serial.print("Relative Humidity : ");
    Serial.println(sht30.humidity);
    Serial.println();

    root["temp"] = sht30.cTemp;
    root["hum"] = sht30.humidity;
    root["timestamp"] = timestamp;
  }

  

  

  char mqttData[256] = {0};
  root.printTo(mqttData, sizeof(mqttData));
  return mqttData;
}


void publishMQTT(const char* mqttTopic, const char* mqttMessage) {
  mqttClient.publish(mqttTopic,mqttMessage);
}



