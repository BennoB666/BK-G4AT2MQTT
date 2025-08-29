#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

// ---- Konfiguration ----
const char* ssid = "SSID";
const char* password = "Password";
const char* mqtt_server = "192.168.178.1"; // MQTT Broker IP
const int mqtt_port = 1883;
const char* mqtt_topic = "gaszaehler/verbrauch";

WiFiClient espClient;
PubSubClient client(espClient);

// ---- M-Bus UART ----
HardwareSerial mbusSerial(1); // UART1
const int MBUS_RX_PIN = 20;   // anpassen
const int MBUS_TX_PIN = 21;   // anpassen
const long MBUS_BAUD = 2400;

// ---- MBUS State Maschine ----
enum MBusState { MBUS_IDLE, MBUS_WAIT_RESPONSE };
MBusState mbusState = MBUS_IDLE;
unsigned long mbusLastAction = 0;
const unsigned long MBUS_POLL_INTERVAL = 60000; // 60 Sekunden
const unsigned long MBUS_RESPONSE_TIMEOUT = 500; // ms

uint8_t mbusBuffer[256];
size_t mbusLen = 0;

// ---- WLAN Setup ----
void setup_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Verbinde mit WLAN");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Verbunden! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WLAN-Verbindung fehlgeschlagen!");
  }
}

// ---- MQTT Reconnect ----
void reconnect() {
  while (!client.connected()) {
    Serial.print("Verbinde zu MQTT Broker: ");
    Serial.print(mqtt_server);
    Serial.print(":");
    Serial.println(mqtt_port);
    if (client.connect("ESP32GasClient")) {
      Serial.println("MQTT verbunden");
    } else {
      Serial.print("Fehler, rc=");
      Serial.print(client.state());
      Serial.println(" -> Neuer Versuch in 5 Sekunden");
      delay(5000);
    }
  }
}

// ---- BCD Parser für Gaszähler ----
float parseGasVolumeBCD(const uint8_t* data, size_t len) {
    for (size_t i = 0; i + 5 < len; i++) {
        if (data[i] == 0x0C && data[i+1] == 0x13) { // DIF=0x0C, VIF=0x13
            uint32_t value = 0;
            uint32_t factor = 1;
            for (int b = 0; b < 4; b++) {
                uint8_t byte = data[i+2+b];
                uint8_t lsn = byte & 0x0F;
                uint8_t msn = (byte >> 4) & 0x0F;
                value += lsn * factor; factor *= 10;
                value += msn * factor; factor *= 10;
            }
            return value / 1000.0; // 2 Dezimalstellen → m³
        }
    }
    return -1; // nicht gefunden
}

// ---- OTA Setup ----
void setupOTA() {
  ArduinoOTA.setHostname("esp32-gas");
  ArduinoOTA.onStart([]() { Serial.println("Start OTA Update"); });
  ArduinoOTA.onEnd([]() { Serial.println("\nOTA Ende"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Fortschritt: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Fehler[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth fehlgeschlagen");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin fehlgeschlagen");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Verbindung fehlgeschlagen");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Empfang fehlgeschlagen");
    else if (error == OTA_END_ERROR) Serial.println("End fehlgeschlagen");
  });
  ArduinoOTA.begin();
  Serial.println("OTA bereit");
}

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);

  mbusSerial.begin(MBUS_BAUD, SERIAL_8E1, MBUS_RX_PIN, MBUS_TX_PIN);
  Serial.println("M-Bus UART bereit");

  setupOTA();
  mbusLastAction = millis() - MBUS_POLL_INTERVAL; // sofort Poll starten
}

// ---- Loop ----
void loop() {
  if (!client.connected()) reconnect();
  client.loop();
  ArduinoOTA.handle();

  unsigned long now = millis();

  switch (mbusState) {
    case MBUS_IDLE:
      if (now - mbusLastAction >= MBUS_POLL_INTERVAL) {
        // Poll senden
        uint8_t pollFrame[5] = {0x10, 0x5B, 0x00, 0x5B, 0x16};
        mbusSerial.write(pollFrame, sizeof(pollFrame));
        mbusSerial.flush();
        mbusLen = 0;
        mbusLastAction = now;
        mbusState = MBUS_WAIT_RESPONSE;
        Serial.println("MBUS Poll gesendet, warte auf Antwort...");
      }
      break;

    case MBUS_WAIT_RESPONSE:
      while (mbusSerial.available() && mbusLen < sizeof(mbusBuffer)) {
        mbusBuffer[mbusLen++] = mbusSerial.read();
      }

      if ((now - mbusLastAction >= MBUS_RESPONSE_TIMEOUT) || mbusLen >= sizeof(mbusBuffer)) {
        if (mbusLen > 0) {
          Serial.print("MBUS Antwort empfangen (");
          Serial.print(mbusLen);
          Serial.println(" Bytes)");

          float volume = parseGasVolumeBCD(mbusBuffer, mbusLen);
          if (volume >= 0) {
            char payload[16];
            dtostrf(volume, 0, 2, payload);
            client.publish(mqtt_topic, payload);
            Serial.print("Verbrauch gesendet: ");
            Serial.println(payload);
          } else {
            Serial.println("Kein Volumenwert gefunden!");
          }
        } else {
          Serial.println("Keine MBUS Antwort erhalten");
        }
        mbusState = MBUS_IDLE; // wieder bereit für nächsten Poll
      }
      break;
  }
}
