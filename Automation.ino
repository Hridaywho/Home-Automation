/* Rate-limited & low-latency control version
   - Use mqtt.processPackets(0) for non-blocking handling
   - Smaller main loop delay (20 ms)
   - DHT publishes every 30 s
   - IR publish-on-change and heartbeat
   - Relay set immediately on command and ACK published only on change
   - UPDATED: Relay logic inverted (active-LOW relay)
*/

#include <WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <DHT.h>

// ********** REPLACE LOCALLY **********
#define WIFI_SSID     "iPhone"           // <= paste your WiFi SSID
#define WIFI_PASS     "12345678"         // <= paste your WiFi password

#define AIO_USERNAME  "your_aio_username"   // <= paste your Adafruit IO username
#define AIO_KEY       "your_key_to_be_inserted_here"   // <= paste Adafruit IO KEY
// *************************************

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883

// pins
#define RELAY_PIN 4
#define LED_PIN   2
#define IR_PIN    15
#define DHT_PIN   16

#define DHTTYPE DHT22
DHT dht(DHT_PIN, DHTTYPE);

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish pub_temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish pub_humidity    = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish pub_ir_raw      = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/ir_raw");
Adafruit_MQTT_Publish pub_presence    = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/presence");
Adafruit_MQTT_Publish pub_led_control = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/led-control");

Adafruit_MQTT_Subscribe sub_led_control =
  Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/led-control");

// rate-limits
const unsigned long MIN_TEMP_HUM_INTERVAL = 30000UL; // 30s
const unsigned long HEARTBEAT_IR_MS = 60000UL; // 60s
unsigned long lastPublishTempHumMs = 0;
unsigned long lastPublishIrMs = 0;

int lastIrRaw = -1;
bool lastPresencePublished = false;
bool lastRelayStatePublished = false;

// presence detection
const unsigned long PRESENCE_MS = 5000UL;
unsigned long irActiveStart = 0;
unsigned long irInactiveStart = 0;
bool presenceDetected = false;

// relay state
volatile bool relayState = false;   // logical relay state
bool onboardLEDState = false;

/* ===========================
   ACTIVE-LOW RELAY CONTROL
   relay ON  = pin LOW
   relay OFF = pin HIGH
   =========================== */

void writeRelay(bool logicalOn) {
  digitalWrite(RELAY_PIN, logicalOn ? HIGH : LOW);
}

void connectMQTT() {
  while (!mqtt.connected()) {
    Serial.print("Connecting to Adafruit IO... ");
    int8_t ret = mqtt.connect();
    if (ret == 0) {
      Serial.println("Connected!");
      mqtt.subscribe(&sub_led_control);
      if (relayState != lastRelayStatePublished) {
        pub_led_control.publish(relayState ? "1" : "0");
        lastRelayStatePublished = relayState;
      }
    } else {
      Serial.print("Failed: ");
      Serial.print(mqtt.connectErrorString(ret));
      Serial.println(" -- retrying in 3s");
      delay(3000);
    }
  }
}

void publishLedStateAckIfChanged() {
  if (relayState != lastRelayStatePublished) {
    pub_led_control.publish(relayState ? "1" : "0");
    lastRelayStatePublished = relayState;
  }
}

void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(RELAY_PIN, OUTPUT);
  writeRelay(false);  // relay OFF at boot (HIGH)

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(IR_PIN, INPUT);

  dht.begin();

  Serial.print("Connecting to WiFi ");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(200);
    if (millis() - t0 > 20000) break;
  }
  Serial.println();
  Serial.print("WiFi IP: ");
  Serial.println(WiFi.localIP());

  mqtt.subscribe(&sub_led_control);

  lastPublishTempHumMs = 0;
  lastPublishIrMs = 0;
  lastIrRaw = -1;
  lastPresencePublished = presenceDetected;
  lastRelayStatePublished = relayState;
}

void loop() {
  if (!mqtt.connected()) connectMQTT();

  mqtt.processPackets(0);

  Adafruit_MQTT_Subscribe *sub;
  while ((sub = mqtt.readSubscription(0))) {
    if (sub == &sub_led_control) {
      String cmd = String((char*)sub_led_control.lastread);
      cmd.trim();
      cmd.toUpperCase();

      bool newState = false;
      if (cmd == "1" || cmd == "ON" || cmd == "TRUE") newState = true;
      else if (cmd == "0" || cmd == "OFF" || cmd == "FALSE") newState = false;
      else newState = (atoi(cmd.c_str()) != 0);

      relayState = newState;
      onboardLEDState = newState;

      writeRelay(relayState);                       // ACTIVE-LOW FIX
      digitalWrite(LED_PIN, onboardLEDState ? HIGH : LOW);

      Serial.print("Command -> Relay set: ");
      Serial.println(relayState ? "ON" : "OFF");

      publishLedStateAckIfChanged();
    }
  }

  unsigned long now = millis();

  if (now - lastPublishTempHumMs >= MIN_TEMP_HUM_INTERVAL) {
    lastPublishTempHumMs = now;
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t)) {
      pub_temperature.publish((double)t, 2, false);
      Serial.print("Temp published: "); Serial.println(t);
    }
    if (!isnan(h)) {
      pub_humidity.publish((double)h, 2, false);
      Serial.print("Hum published: "); Serial.println(h);
    }
  }

  int irRaw = digitalRead(IR_PIN);
  if (irRaw != lastIrRaw || (now - lastPublishIrMs >= HEARTBEAT_IR_MS)) {
    lastPublishIrMs = now;
    lastIrRaw = irRaw;
    pub_ir_raw.publish((int32_t)irRaw);
    Serial.print("IR published: "); Serial.println(irRaw);
  }

  bool isActive = (irRaw == LOW);
  if (isActive) {
    irInactiveStart = 0;
    if (irActiveStart == 0) irActiveStart = now;
    else if (!presenceDetected && (now - irActiveStart >= PRESENCE_MS)) {
      presenceDetected = true;
      if (presenceDetected != lastPresencePublished) {
        pub_presence.publish((int32_t)1);
        lastPresencePublished = presenceDetected;
        Serial.println("Presence CONFIRMED");
      }
    }
  } else {
    irActiveStart = 0;
    if (irInactiveStart == 0) irInactiveStart = now;
    else if (presenceDetected && (now - irInactiveStart >= PRESENCE_MS)) {
      presenceDetected = false;
      if (presenceDetected != lastPresencePublished) {
        pub_presence.publish((int32_t)0);
        lastPresencePublished = presenceDetected;
        Serial.println("Presence CLEARED");
      }
    }
  }

  delay(20);
}