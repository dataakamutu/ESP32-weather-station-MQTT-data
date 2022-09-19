#include <AsyncTCP.h>

#include <AsyncMqttClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include <ArduinoJson.h>



//#define  D4  5


#include <Wire.h>

//#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <SFE_BMP180.h>
#include <DHT.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}



#define ALTITUDE PROVIDE_ALTITUDE_VALUE // Altitude in meters
float temperature;

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  1800    /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;




#define WIFI_SSID "WIFI_USER_NAME"
#define  WIFI_PASSWORD "WIFI_PASSWORD"


// Raspberry Pi Mosquitto MQTT Broker

#define MQTT_HOST IPAddress(150, 230, 14, 8)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MqqtID "MQTT_USER_NAME"
#define MqqtPW "MQTT_PASSWORD"

#define MQTT_PORT 1883

// Temperature MQTT Topics
//#define MQTT_PUB_TEMP "esp/dht/temperature"
//#define MQTT_PUB_HUM "esp/dht/humidity"
#define MQTT_PUB_HUM_TEMP "weather"
#define MQTT_PUB_PRES "esp/bmp/pressur"

// Digital pin connected to the DHT sensor
#define DHTPIN 2


// Uncomment whatever DHT sensor type you're using
#define DHTTYPE DHT11   // DHT 11


// Initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE);

// Initialize bmp 180 pressure sensor
SFE_BMP180 pressure;

// Variables to hold sensor readings
float temp;
float hum;

float cordinate[]={
 PROVIDE_LONGITUDE_HERE,PROVIDE_LATITIUDE_HERE
  };

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 1 * 60000;      // Interval at which to publish sensor readings



/*
  Method to print the reason by which ESP32
  has been awaken from sleep
*/
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}


void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.onEvent(WiFiEvent);


}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();

  //mqttClient.onConnect(onMqttConnect);


}

void WiFiEvent(WiFiEvent_t event) {

  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case  SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

/*void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
  }
  void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  }*/

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  //disable brownout detector

  Serial.begin(115200);
  Serial.println();

  //connectToWifi();

  dht.begin();

  //BMP180 Sensor
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    Serial.println("BMP180 init fail\n\n");
    while (1); // Pause forever.
  }


  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  //WiFiEvent();

  mqttClient.onConnect(onMqttConnect);
  //Serial.print("Session present finished ");
  //mqttClient.onDisconnect(onMqttDisconnect);

  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  mqttClient.setClientId(MqqtID);
  mqttClient.setCredentials(MqqtID,WIFI_PASSWORD);
  //mqttClient.onConnect(onMqttConnect);

  connectToWifi();

  /*  //Increment boot number and print it every reboot
    ++bootCount;
    Serial.println("Boot number: " + String(bootCount));
    //Print the wakeup reason for ESP32
    print_wakeup_reason();
  */
  /*
    First we configure the wake up source
    We set our ESP32 to wake up every 5 seconds
  */

  /* esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
                  " Seconds");*/

  /*
    Now that we have setup a wake cause and if needed setup the
    peripherals state in deep sleep, we can now start going to
    deep sleep.
    In the case that no wake up sources were provided but deep
    sleep was started, it will sleep forever unless hardware
    reset occurs.
  */

  /*Serial.println("Going to sleep now");
    Serial.flush();
    esp_deep_sleep_start();
    Serial.println("This will never be printed");*/
}

void loop() {
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds)
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    // New DHT sensor readings
    hum = dht.readHumidity();
    // Read temperature as Celsius (the default)
    temp = dht.readTemperature();
    delay(2000);

    float  pressureVal;
    pressureVal = readPressureAndTemperature();
    Serial.printf("pressure: %.2f \n", pressureVal);

    StaticJsonDocument<256> doc1;
    doc1["pressure"] = pressureVal;
    JsonArray coordinates = doc1.createNestedArray("coordinates");

    //
    coordinates.add(PROVIDE_LONGITUDE_HERE);
    coordinates.add(MQTT_PASSWORD);

    //
    char out1[128];
    int b1 = serializeJson(doc1, out1);
    Serial.print("bytes = ");
    Serial.println(b1, DEC);

    //Publish an MQTT message on topic esp/bmp/pressur
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_PRES, 1, true, out1);
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_PRES, packetIdPub1);
    //Serial.printf("Message: %.2f \n", temp);

    StaticJsonDocument<256> doc;
    doc1["temperature"] = temp;
    doc1["humidity"] = hum;
    char out[128];
    int b = serializeJson(doc1, out);
    Serial.print("bytes = ");
    Serial.println(b, DEC  );

    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_HUM_TEMP , 1, true, out);
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM_TEMP, packetIdPub3);
    Serial.printf("Message: %.2f \n", temp);
    Serial.printf("Message: %.2f \n", hum);

    //Increment boot number and print it every reboot
    ++bootCount;
    Serial.println("Boot number: " + String(bootCount));

    //Print the wakeup reason for ESP32
    print_wakeup_reason();

    /*
      First we configure the wake up source
      We set our ESP32 to wake up every 5 seconds
    */

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
                   " Seconds");

    /*
      Now that we have setup a wake cause and if needed setup the
      peripherals state in deep sleep, we can now start going to
      deep sleep.
      In the case that no wake up sources were provided but deep
      sleep was started, it will sleep forever unless hardware
      reset occurs.
    */
    delay (5000);
    
    Serial.println("Going to sleep now");
    Serial.flush();
    esp_deep_sleep_start();
    Serial.println("This will never be printed");


  }
}
float readPressureAndTemperature()
{
  char status;
  double T, P, p0, a;

  status = pressure.startTemperature();
  if (status != 0)
  {
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      temperature = T;
      status = pressure.startPressure(3);
      if (status != 0)
      {
        delay(status);
        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          p0 = pressure.sealevel(P, ALTITUDE);
          return p0;
        }
      }
    }
  }
}
