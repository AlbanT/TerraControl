#include <ESP8266WiFi.h>  
#include <ESP8266HTTPClient.h>

#include <ArduinoJson.h>          //Needed to store configuration as JSON: https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>       // used for MQTT

// declarations for MQTT and WiFi
char gMqttServer[40] = "xxx.xxx.xxx.xxx";
int  gMqttPort = 1883;
char gMqttUser[256] = "";
char gMqttPassword[256] = "";
char gMqttTopic[256] = "sensor/data";
char gMqttTopicSubscribe1[256] = "terrarium/sproeier";
char gMqttTopicSubscribe2[256] = "terrarium/daglicht1";
char gMqttTopicSubscribe3[256] = "terrarium/daglicht2";
char gMqttTopicSubscribe4[256] = "terrarium/UV1";
char gMqttTopicSubscribe5[256] = "terrarium/UV2";
char gMqttTopicSubscribe6[256] = "terrarium/IR1";
char gMqttTopicSubscribe7[256] = "terrarium/IR2";
char gMqttTopicSubscribe8[256] = "timer";

char gMqttTopicPublish[256] = "terrarium/temperatuur";




char gSSID[256] = "****";
char gWifiPassword[256] = "****";

const char* APPLICATION_NAME = "VellemanRS232";

HTTPClient http;
WiFiClient wifiClient;
PubSubClient mqttClient;

#include <K8056Velleman.h>

K8056Velleman Relay;


#include <SHT1x.h>              // used for the SHT11 temperature/humidity sensor
#include <OneWire.h>			// used for the DS18B20 temperature sensor
#include <DallasTemperature.h>	// used for the DS18B20 temperature sensor
/*
// Specify data and clock connections and instantiate SHT1x object
const int dataPin = D5;
const int clockPin = D0;
SHT1x sht1x(dataPin, clockPin);

struct SHTmeasurement {
  float Temperature;
  int Humidity;
} ;
*/
const int OneWirePin = D4;

struct DS18B20measurement {
  float Temperature0;
  float Temperature1;
  float Temperature2;
  float Temperature3;
  char* jsonPacket;
} ;

OneWire  oneWire(OneWirePin);  // on pin D4 (a 4.7K resistor is necessary)

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature DS18B20sensor(&oneWire);


void setup()   
{
  Serial.begin(115200);
  Relay.begin();

  // Setup the one-wire DS18B20 temperature sensors
  DS18B20sensor.begin();
	pinMode(OneWirePin, OUTPUT); // this line is needed because of a problem with the onewire library and ESP8266. Temperature conversion 85 deg...
  
  
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
    //Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(APPLICATION_NAME)) {
      //Serial.println("connected");
      // ... and resubscribe
      mqttClient.subscribe(gMqttTopicSubscribe1);
      mqttClient.subscribe(gMqttTopicSubscribe2);
      mqttClient.subscribe(gMqttTopicSubscribe3);
      mqttClient.subscribe(gMqttTopicSubscribe4);
      mqttClient.subscribe(gMqttTopicSubscribe5);
      mqttClient.subscribe(gMqttTopicSubscribe6);
      mqttClient.subscribe(gMqttTopicSubscribe7);
      mqttClient.subscribe(gMqttTopicSubscribe8);




    } else {
      //Serial.print("failed, rc=");
      //Serial.print(mqttClient.state());
      //Serial.println(" try again in 5 seconds");
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
    //Serial.print((char)payload[i]);
  }
 // Serial.println();

  String mqttTopic = String(topic);

  if (mqttTopic == gMqttTopicSubscribe1) {
    setRelayStatus(payload,1);
  }
  else if (mqttTopic == gMqttTopicSubscribe2) {
      setRelayStatus(payload,2);
  }
  else if (mqttTopic == gMqttTopicSubscribe3) {
      setRelayStatus(payload,3);
  }
  else if (mqttTopic == gMqttTopicSubscribe4) {
      setRelayStatus(payload,4);
  }
  else if (mqttTopic == gMqttTopicSubscribe5) {
      setRelayStatus(payload,5);
  }
  else if (mqttTopic == gMqttTopicSubscribe6) {
      setRelayStatus(payload,6);
  }
  else if (mqttTopic == gMqttTopicSubscribe7) {
      setRelayStatus(payload,7);
  }
  else if (mqttTopic == gMqttTopicSubscribe8) {
      // get temperature data
      DS18B20measurement DS18B20;
      DS18B20 = getDS18B20Measurement();
      // Serial.print("DS18B20 temperature 0 = ");Serial.print(DS18B20.Temperature0);Serial.println("°C");
	    // Serial.print("DS18B20 temperature 1 = ");Serial.print(DS18B20.Temperature1);Serial.println("°C");
	    // Serial.print("DS18B20 temperature 2 = ");Serial.print(DS18B20.Temperature2);Serial.println("°C");
      publishMQTT(gMqttTopicPublish,DS18B20.jsonPacket);
  }
  else {
   // Serial.println("Why am I here???");
  }
}

void setRelayStatus(byte* payload, byte relayID) {
 // Serial.print("Set Relay ");Serial.print(relayID); Serial.print(" to ");
  if ((char)payload[0] == '0') {
       // Serial.println("OFF");
        Relay.OFF(1,relayID);
      }
      else {
       // Serial.println("ON");
        Relay.ON(1,relayID);
      }
}


DS18B20measurement getDS18B20Measurement() {
	DS18B20measurement result;
	DS18B20sensor.requestTemperatures(); // Send the command to get temperatures

	result.Temperature0 = DS18B20sensor.getTempCByIndex(0);
	result.Temperature1 = DS18B20sensor.getTempCByIndex(1);
	result.Temperature2 = DS18B20sensor.getTempCByIndex(2);
  result.Temperature3 = DS18B20sensor.getTempCByIndex(3);

  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  
  root["Temperature0"] = result.Temperature0;
  root["Temperature1"] = result.Temperature1;
  root["Temperature2"] = result.Temperature2;
  root["Temperature3"] = result.Temperature3;

  char mqttData[256] = {0};
  root.printTo(mqttData, sizeof(mqttData));
  result.jsonPacket = mqttData;


	return result;
}


void publishMQTT(const char* mqttTopic, const char* mqttMessage) {
  mqttClient.publish(mqttTopic,mqttMessage);
}


