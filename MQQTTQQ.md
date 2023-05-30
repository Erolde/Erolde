#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define wifi_ssid "BME"
#define wifi_password "bme-pi-2023-wlan" //"bme-pi-2023-wlan"

#define mqtt_server "raspi9"
#define mqtt_user "bme-pi-2023"
#define mqtt_password "bme-pi-2023" //"bme-pi-2023-mqtt"

#define humidity_topic "esp1/humidity"
#define temperature_topic "esp1/temperature"
#define pressure_topic "esp1/pressure"

float temp = 0.0;
float hum = 0.0;
float pres = 0.0;
float diff = 1.0;

Adafruit_BME280 bme; // I2C
unsigned long delayTime;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);

  while (!Serial); // Zeit für die Initialisierung der seriellen Kommunikation
  unsigned status;
  status = bme.begin();
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    while (1);
  }
  delayTime = 1000;
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    getValues();
  }
}

void setup_wifi() {
  delay(10);
  // Verbindung zum WiFi-Netzwerk herstellen
  Serial.println();
  Serial.print("Verbindung zu ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi verbunden");
  Serial.println("IP-Adresse: ");
  Serial.println(WiFi.localIP());
}

bool checkBound(float newValue, float prevValue, float maxDiff) {
  return !isnan(newValue) &&
         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}

void getValues() {
  float newPres = bme.readPressure() / 100.0F;
  float newTemp = bme.readTemperature();
  float newHum = bme.readHumidity();

  if (checkBound(newTemp, temp, diff)) {
    temp = newTemp;
    Serial.print("Neue Temperatur: ");
    Serial.println(String(temp).c_str());
    client.publish(temperature_topic, String(temp).c_str(), true);
  }

  if (checkBound(newHum, hum, diff)) {
    hum = newHum;
    Serial.print("Neue Luftfeuchtigkeit: ");
    Serial.println(String(hum).c_str());
    client.publish(humidity_topic, String(hum).c_str(), true);
  }

  if (checkBound(newPres, pres, diff)) {
    pres = newPres;
    Serial.print("Neuer Luftdruck: ");
    Serial.println(String(pres).c_str());
    client.publish(pressure_topic, String(pres).c_str(), true);
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Versuche MQTT-Verbindung...");
    String clientId = "bme";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("verbunden");
      client.publish("outTopic", "ESP01 alive");
      client.subscribe("inTopic");
    } else {
      Serial.print("Fehler, rc=");
      Serial.print(client.state());
      Serial.println(" - Neuer Versuch in 5 Sekunden");
      delay(5000);
    }
  }
}
