/****************************************************************************************************************************
  WiFiMQTT.ino

  MQTT and MQTT over WebSoket Client for Arduino

  For nRF52, SAMD21, SAMD51, STM32F/L/H/G/WB/MP1, Teensy, SAM DUE, RP2040-based boards, besides ESP8266,
  ESP32 (ESP32, ESP32_S2, ESP32_S3 and ESP32_C3) and WT32_ETH01

  Ethernet shields W5100, W5200, W5500, ENC28J60, Teensy 4.1 NativeEthernet/QNEthernet.

  Based on and modified from MQTTPubSubClient Library (https://github.com/hideakitai/MQTTPubSubClient)

  Built by Khoi Hoang https://github.com/khoih-prog/MQTTPubSubClient_Generic
  Licensed under MIT license
 *****************************************************************************************************************************/
#if !defined(ESP32)
  #error This code is intended to run on the ESP32 platform! Please check your Tools->Board setting.
#endif

#define TIMER_INTERRUPT_DEBUG       0
#define ISR_SERVO_DEBUG             1

// For ESP32_C3, select ESP32 timer number (0-1)
// For ESP32 and ESP32_S2, select ESP32 timer number (0-3)
#if defined( ARDUINO_ESP32C3_DEV )
  #define USE_ESP32_TIMER_NO          1
#else
  #define USE_ESP32_TIMER_NO          3
#endif
#include "defines.h"
#include "ESP32_New_ISR_Servo.h"
int status = WL_IDLE_STATUS;     // the Wifi radio's status

#include <MQTTPubSubClient_Generic.h>

WiFiClient client;
MQTTPubSubClient mqttClient;

//#define MQTT_SERVER         "192.168.2.30"
#define MQTT_SERVER           "192.168.206.69"
#define MQTT_PORT             1883
#define MIN_MICROS      800  //544
#define MAX_MICROS      2450
#define PIN_D3            3         // Pin D3 mapped to pin GPIO3/RX0 of ESP32
#define PIN_D4            4         // Pin D4 mapped to pin GPIO4/ADC10/TOUCH0 of ESP32
const char *PubTopic    = "/mqttPubSub";                                  // Topic to publish
const char *PubMessage  = "Hello from " BOARD_NAME " with " SHIELD_TYPE;       // Topic Message to publish
int servoIndex1  = -1;
int servoIndex2  = -1;
void printWifiStatus()
{
  // print the SSID of the network you're attached to:
  // you're connected now, so print out the data
  Serial.print(F("You're connected to the network, IP = "));
  Serial.println(WiFi.localIP());

  Serial.print(F("SSID: "));
  Serial.print(WiFi.SSID());

  // print the received signal strength:
  int32_t rssi = WiFi.RSSI();
  Serial.print(F(", Signal strength (RSSI):"));
  Serial.print(rssi);
  Serial.println(F(" dBm"));
}

void setup()
{
  // Debug console
  Serial.begin(115200);

  while (!Serial && millis() < 5000);

  Serial.print(F("\nStart WiFiMQTT on "));
  Serial.print(BOARD_NAME);
  Serial.print(F(" with "));
  Serial.println(SHIELD_TYPE);
  Serial.println(WIFI_WEBSERVER_VERSION);
  Serial.println(MQTT_PUBSUB_CLIENT_GENERIC_VERSION);
  ESP32_ISR_Servos.useTimer(USE_ESP32_TIMER_NO);

  servoIndex1 = ESP32_ISR_Servos.setupServo(PIN_D3, MIN_MICROS, MAX_MICROS);
  servoIndex2 = ESP32_ISR_Servos.setupServo(PIN_D4, MIN_MICROS, MAX_MICROS);

  if (servoIndex1 != -1)
    Serial.println(F("Setup Servo1 OK"));
  else
    Serial.println(F("Setup Servo1 failed"));

  if (servoIndex2 != -1)
    Serial.println(F("Setup Servo2 OK"));
  else
    Serial.println(F("Setup Servo2 failed"));
#if WIFI_USING_ESP_AT

  // initialize serial for ESP module
  EspSerial.begin(115200);
  // initialize ESP module
  WiFi.init(&EspSerial);

  Serial.println(F("WiFi shield init done"));

#endif

#if !(ESP32 || ESP8266)

  // check for the presence of the shield
#if USE_WIFI_NINA

  if (WiFi.status() == WL_NO_MODULE)
#else
  if (WiFi.status() == WL_NO_SHIELD)
#endif
  {
    Serial.println(F("WiFi shield not present"));

    // don't continue
    while (true);
  }

#if USE_WIFI_NINA
  String fv = WiFi.firmwareVersion();

  if (fv < WIFI_FIRMWARE_LATEST_VERSION)
  {
    Serial.println(F("Please upgrade the firmware"));
  }

#endif

#endif

  Serial.print(F("Connecting to SSID: "));
  Serial.println(ssid);

  status = WiFi.begin(ssid, pass);

  delay(1000);

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED)
  {
    delay(500);

    // Connect to WPA/WPA2 network
    status = WiFi.status();
  }

  // you're connected now, so print out the data
  printWifiStatus();

  Serial.print("Connecting to host ");
  Serial.println(MQTT_SERVER);

  while (!client.connect(MQTT_SERVER, MQTT_PORT))
  {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nConnected!");

  // initialize mqtt client
  mqttClient.begin(client);

  Serial.print("Connecting to mqtt broker...");

  while (!mqttClient.connect("arduino", "public", "public"))
  {
    Serial.print(".");
    delay(1000);
  }

  Serial.println(" connected!");

  // subscribe callback which is called when every packet has come
  mqttClient.subscribe([](const String & topic, const String & payload, const size_t size)
  {
    (void) size;

    Serial.println("MQTT received: " + topic + " - " + payload);
    if (payload == "on")
    {
        ESP32_ISR_Servos.setPosition(servoIndex1, 60);
        delay(250);
        ESP32_ISR_Servos.setPosition(servoIndex1, 0);    
    }
    else if(payload == "off"){
        ESP32_ISR_Servos.setPosition(servoIndex2, 60);
        delay(250);
        ESP32_ISR_Servos.setPosition(servoIndex2, 0);
      }
  });

  // subscribe topic and callback which is called when /hello has come
  mqttClient.subscribe(PubTopic, [](const String & payload, const size_t size)
  {
    (void) size;

    Serial.print("Subcribed to ");
    Serial.print(PubTopic);
    Serial.print(" => ");
    Serial.println(payload);
  });

  mqttClient.publish(PubTopic, PubMessage);
}

void loop()
{
  mqttClient.update();  // should be called

}
