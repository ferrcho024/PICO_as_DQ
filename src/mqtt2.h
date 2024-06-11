#define WARN Serial.println

#define MQTTCLIENT_QOS2 1

#include <SPI.h>
#include <IPStack.h>
#include <Countdown.h>
#include <MQTTClient.h>

// MQTT settings
const String ID = "1";
const String BROKER = "192.168.1.104";
const String CLIENT_NAME = "pico_"+ID;
//const String TOPIC = "80";

// in_txt;
bool callback = false;

int arrivedcount = 0;

void messageArrived(MQTT::MessageData& md)
{
  MQTT::Message &message = md.message;
  
  Serial.print("Message ");
  Serial.print(++arrivedcount);
  Serial.print(" arrived: qos ");
  Serial.print(message.qos);
  Serial.print(", retained ");
  Serial.print(message.retained);
  Serial.print(", dup ");
  Serial.print(message.dup);
  Serial.print(", packetid ");
  Serial.println(message.id);
  Serial.print("Payload ");
  Serial.println((char*)message.payload);
  //in_txt = message.payload;
  callback = true;
}


WiFiClient c; // replace by a YunClient if running on a Yun
IPStack ipstack(c);
MQTT::Client<IPStack, Countdown, 50, 1> client = MQTT::Client<IPStack, Countdown, 50, 1>(ipstack);

byte mac[] = { 0x28, 0xcd, 0xc1, 0x06, 0x61, 0xf0 };  // replace with your device's MAC
const char* topic = "80";

void MQTTconnect()
{
  char hostname[] = "192.168.1.104";
  int port = 1883;

  Serial.print("Connecting to ");
  Serial.print(hostname);
  Serial.print(":");
  Serial.println(port);
 
  int rc = ipstack.connect(hostname, port);
  if (rc != 1)
  {
    Serial.print("rc from TCP connect is ");
    Serial.println(rc);
  }
 
  Serial.println("MQTT connecting");
  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;       
  data.MQTTVersion = 3;
  data.clientID.cstring = (char*)"Pico_MQTT2";
  rc = client.connect(data);
  if (rc != 0)
  {
    Serial.print("rc from MQTT connect is ");
    Serial.println(rc);
  }
  Serial.println("MQTT connected");
  
  rc = client.subscribe(topic, MQTT::QOS2, messageArrived);  
  Serial.println("MQTT Subscripting");
  if (rc != 0)
  {
    Serial.print("rc from MQTT subscribe is ");
    Serial.println(rc);
  }
  Serial.println("MQTT subscribed");
}