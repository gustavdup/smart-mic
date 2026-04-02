/*
 * iBeacon Scanner — XIAO ESP32-S3 + Seeed Expansion Board
 *
 * Libraries required (install via Arduino Library Manager):
 *   - NimBLE-Arduino  (h2zero)
 *   - ArduinoJson     (Benoit Blanchon)
 *   - U8g2            (oliver)
 *
 * Board: XIAO_ESP32S3
 * Partition scheme: default
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <NimBLEDevice.h>
#include <ArduinoJson.h>
#include <U8g2lib.h>
#include <Wire.h>

// ── Config ────────────────────────────────────────────────────────────────────
const char* WIFI_SSID     = "Home DSL";
const char* WIFI_PASSWORD = "1234544321";
const char* SERVER_URL    = "http://192.168.1.67:5001/api/beacon";  // Mac's IP

// Target iBeacon UUID: E2C56DB5-DFFB-48D2-B060-D0F5A71096E0
const uint8_t TARGET_UUID[16] = {
  0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48, 0xD2,
  0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0
};

#define NUM_BEACONS 5
#define STALE_MS    10000   // grey out beacon after 10s without update
#define DISPLAY_MS  500     // redraw display every 500ms

// ── OLED display (SSD1306 128x64, I2C) ───────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ── Beacon state (indexed by major-1, minor is always 1 in this deployment) ──
struct BeaconState {
  bool     active;
  int      rssi;
  float    distance;
  uint32_t lastSeenMs;
};

BeaconState beacons[NUM_BEACONS] = {};
SemaphoreHandle_t beaconsMutex;

// ── Beacon queue (BLE callback → main loop) ───────────────────────────────────
struct BeaconData {
  char     address[18];
  uint16_t major;
  uint16_t minor;
  int      rssi;
  int8_t   txPower;
};

QueueHandle_t beaconQueue;

// ── Distance estimate ─────────────────────────────────────────────────────────
float rssiToDistance(int rssi, int8_t txPower) {
  if (rssi == 0) return -1.0f;
  float ratio = (txPower - rssi) / (10.0f * 2.5f);
  return powf(10.0f, ratio);
}

// ── BLE scan callback ─────────────────────────────────────────────────────────
class ScanCallback : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* device) override {
    if (!device->haveManufacturerData()) return;

    const uint8_t* d   = (const uint8_t*)device->getManufacturerData().data();
    size_t          len = device->getManufacturerData().size();

    // Debug: print first 6 bytes of every manufacturer packet
    Serial.printf("[scan] len=%d bytes:", len);
    for (int i = 0; i < min((int)len, 6); i++) Serial.printf(" %02X", d[i]);
    Serial.println();

    if (len < 25) return;
    if (d[0] != 0x4C || d[1] != 0x00) return;   // Apple company ID
    if (d[2] != 0x02 || d[3] != 0x15) return;   // iBeacon type/length
    if (memcmp(d + 4, TARGET_UUID, 16) != 0) return;

    BeaconData b;
    strncpy(b.address, device->getAddress().toString().c_str(), sizeof(b.address) - 1);
    b.address[sizeof(b.address) - 1] = '\0';
    b.major   = (d[20] << 8) | d[21];
    b.minor   = (d[22] << 8) | d[23];
    b.txPower = (int8_t)d[24];
    b.rssi    = device->getRSSI();

    xQueueSendFromISR(beaconQueue, &b, nullptr);
  }
};

// ── Display ───────────────────────────────────────────────────────────────────
void updateDisplay() {
  u8g2.clearBuffer();

  // Header
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "iBeacon Scanner");
  u8g2.drawHLine(0, 12, 128);

  // One row per beacon
  u8g2.setFont(u8g2_font_5x7_tf);
  uint32_t now = millis();

  for (int i = 0; i < NUM_BEACONS; i++) {
    int y = 22 + i * 10;
    char line[32];

    if (xSemaphoreTake(beaconsMutex, 0) == pdTRUE) {
      BeaconState s = beacons[i];
      xSemaphoreGive(beaconsMutex);

      bool stale = !s.active || (now - s.lastSeenMs > STALE_MS);
      uint32_t ageSec = s.active ? (now - s.lastSeenMs) / 1000 : 0;

      if (!s.active) {
        snprintf(line, sizeof(line), "T%d  --", i + 1);
      } else {
        snprintf(line, sizeof(line), "T%d %4ddBm %4.1fm %2lus",
                 i + 1, s.rssi, s.distance, (unsigned long)ageSec);
      }

      if (stale) {
        // Draw dimmed (just the label) when stale
        u8g2.setDrawColor(1);
        char label[4];
        snprintf(label, sizeof(label), "T%d", i + 1);
        u8g2.drawStr(0, y, label);
        u8g2.drawStr(18, y, "out of range");
      } else {
        u8g2.drawStr(0, y, line);
      }
    }
  }

  u8g2.sendBuffer();
}

// ── WiFi helpers ──────────────────────────────────────────────────────────────
void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi failed — continuing without server");
  }
}

void postBeacon(const BeaconData& b) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(2000);

  StaticJsonDocument<128> doc;
  doc["address"]  = b.address;
  doc["major"]    = b.major;
  doc["minor"]    = b.minor;
  doc["rssi"]     = b.rssi;
  doc["tx_power"] = b.txPower;

  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  if (code > 0) {
    Serial.printf("[beacon] %d.%d  rssi=%d  http=%d\n", b.major, b.minor, b.rssi, code);
  } else {
    Serial.printf("[err] POST failed: %s\n", http.errorToString(code).c_str());
  }
  http.end();
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  beaconsMutex = xSemaphoreCreateMutex();
  beaconQueue  = xQueueCreate(32, sizeof(BeaconData));

  // Display
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 20, "Starting...");
  u8g2.sendBuffer();

  connectWiFi();

  // Show IP on display
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(0, 10, "WiFi OK");
  u8g2.drawStr(0, 20, WiFi.localIP().toString().c_str());
  u8g2.drawStr(0, 32, "Starting BLE...");
  u8g2.sendBuffer();
  delay(1000);

  NimBLEDevice::init("");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new ScanCallback(), true);
  scan->setActiveScan(false);
  scan->setInterval(16);   // 10ms (units of 0.625ms)
  scan->setWindow(16);     // 10ms — 100% duty cycle
  scan->setMaxResults(0);
  scan->start(0, false);

  Serial.println("BLE scanning...");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  static uint32_t lastDisplay = 0;

  // Drain beacon queue
  BeaconData b;
  while (xQueueReceive(beaconQueue, &b, pdMS_TO_TICKS(10)) == pdTRUE) {
    // Update beacon state
    if (b.major >= 1 && b.major <= NUM_BEACONS) {
      float dist = rssiToDistance(b.rssi, b.txPower);
      if (xSemaphoreTake(beaconsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        BeaconState& s = beacons[b.major - 1];
        s.active     = true;
        s.rssi       = b.rssi;
        s.distance   = dist;
        s.lastSeenMs = millis();
        xSemaphoreGive(beaconsMutex);
      }
    }
    postBeacon(b);
  }

  // Refresh display at DISPLAY_MS interval
  if (millis() - lastDisplay >= DISPLAY_MS) {
    lastDisplay = millis();
    updateDisplay();
  }
}
