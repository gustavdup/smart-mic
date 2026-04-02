/*
 * iBeacon Scanner + Audio Recorder
 * XIAO ESP32-S3 Sense + Seeed Expansion Board v1
 *
 * Libraries (install via Library Manager):
 *   - NimBLE-Arduino  (h2zero)
 *   - U8g2            (oliver)
 *
 * Board: XIAO_ESP32S3
 */

// BLESnapshot must be defined before #includes so Arduino's auto-generated
// prototype for writeBLEChunk (inserted after the #include block) can see it.
#define NUM_BEACONS  5

#pragma pack(push, 1)
struct BLESnapshot {
  uint32_t offset_ms;
  struct { bool active; int8_t rssi; float distance; }
    beacon[NUM_BEACONS];
};
#pragma pack(pop)

#include <WiFi.h>
#include <NimBLEDevice.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include "driver/i2s_pdm.h"
#include "mbedtls/md.h"
#include "time.h"
#include "esp_log.h"
// Forward-declare idle hook API (avoids pulling in esp_freertos_hooks.h which conflicts with packed structs)
extern "C" esp_err_t esp_register_freertos_idle_hook_for_cpu(bool (*hook)(void), UBaseType_t cpuid);

// ── WiFi (MinIO upload only) ──────────────────────────────────────────────────
const char* WIFI_SSID     = "Home DSL";
const char* WIFI_PASSWORD = "123454321";

// ── MinIO ─────────────────────────────────────────────────────────────────────
const char* MINIO_HOST    = "192.168.1.72";  // Mac IP — update if changed
const int   MINIO_PORT    = 9000;
const char* MINIO_BUCKET  = "audio";
const char* MINIO_ACCESS  = "minioadmin";
const char* MINIO_SECRET  = "minioadmin";

// ── iBeacon UUID ──────────────────────────────────────────────────────────────
const uint8_t TARGET_UUID[16] = {
  0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48, 0xD2,
  0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0
};

// ── Pins ──────────────────────────────────────────────────────────────────────
#define BUTTON_PIN   2   // User button on Seeed expansion board (D1 = GPIO2)
#define SD_CS_PIN    21  // SD card chip select on expansion board
#define PDM_CLK      42  // Built-in PDM microphone clock
#define PDM_DATA     41  // Built-in PDM microphone data
#define PDM_PWR      14  // PDM mic power enable (HIGH = on)

// ── Recording ─────────────────────────────────────────────────────────────────
#define SAMPLE_RATE   16000
#define SEGMENT_S     30     // Segment and upload every 30 seconds
#define MIC_GAIN      16     // Boost microphone gain

// ── Beacon / Display ──────────────────────────────────────────────────────────
// NUM_BEACONS defined above (before #includes) to fix Arduino prototype ordering
#define STALE_MS     10000
#define DISPLAY_MS   500

// ── OLED ──────────────────────────────────────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ── Beacon state ──────────────────────────────────────────────────────────────
struct BeaconState {
  bool     active;
  int      rssi;
  float    distance;
  uint32_t lastSeenMs;
};
BeaconState    beacons[NUM_BEACONS] = {};
SemaphoreHandle_t beaconsMutex;

// ── BLE queue ─────────────────────────────────────────────────────────────────
struct BeaconData {
  char     address[18];
  uint16_t major;
  uint16_t minor;
  int      rssi;
  int8_t   txPower;
};
QueueHandle_t beaconQueue;

// ── I2S handle (new ESP-IDF 5.x PDM API) ─────────────────────────────────────
static i2s_chan_handle_t i2s_rx_handle = NULL;

// ── Recording / upload state ──────────────────────────────────────────────────
volatile bool isRecording   = false;
volatile bool isUploading   = false;
volatile bool stopRequested = false;

SemaphoreHandle_t uploadReady;  // signalled when a new file is ready to upload
SemaphoreHandle_t sdMutex;      // SD card access lock

bool     ntpSynced     = false;
int      fileCounter   = 0;
String   currentFile   = "";
volatile bool bleScanning = false;

// ── CPU load tracking (idle-hook, sticky baseline) ────────────────────────────
static volatile uint32_t s_idle0 = 0;
static volatile uint32_t s_idle1 = 0;
static uint32_t s_base0 = 1, s_base1 = 1;  // high-water mark — never decreases

static bool IRAM_ATTR idleHook0() { s_idle0++; return false; }
static bool IRAM_ATTR idleHook1() { s_idle1++; return false; }

// ── BLE snapshot log (one entry per second, written as custom WAV chunk) ─────
// BLESnapshot struct defined above (before #includes) — see top of file.
static BLESnapshot bleLog[SEGMENT_S + 5];  // +5 safety margin
static int         bleLogCount = 0;

// ── WAV header ────────────────────────────────────────────────────────────────
#pragma pack(push, 1)
struct WAV_HEADER {
  char  riff[4]        = {'R','I','F','F'};
  int32_t flength      = 0;
  char  wave[4]        = {'W','A','V','E'};
  char  fmt[4]         = {'f','m','t',' '};
  int32_t chunk_size   = 16;
  int16_t format_tag   = 1;
  int16_t num_chans    = 1;
  int32_t srate        = SAMPLE_RATE;
  int32_t bytes_per_sec = SAMPLE_RATE * 2;
  int16_t bytes_per_samp = 2;
  int16_t bits_per_samp  = 16;
  char  dat[4]         = {'d','a','t','a'};
  int32_t dlength      = 0;
} wavHeader;
#pragma pack(pop)

// ── I2S PDM microphone ────────────────────────────────────────────────────────
bool initMic() {
  pinMode(PDM_PWR, OUTPUT);
  digitalWrite(PDM_PWR, HIGH);  // power on the PDM mic
  delay(10);                    // let it stabilise

  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  if (i2s_new_channel(&chan_cfg, NULL, &i2s_rx_handle) != ESP_OK) {
    Serial.println("[mic] channel create failed");
    return false;
  }

  i2s_pdm_rx_config_t pdm_cfg = {
    .clk_cfg  = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .clk = (gpio_num_t)PDM_CLK,
      .din = (gpio_num_t)PDM_DATA,
      .invert_flags = { .clk_inv = false },
    },
  };

  if (i2s_channel_init_pdm_rx_mode(i2s_rx_handle, &pdm_cfg) != ESP_OK) {
    Serial.println("[mic] PDM init failed");
    i2s_del_channel(i2s_rx_handle);
    i2s_rx_handle = NULL;
    return false;
  }

  if (i2s_channel_enable(i2s_rx_handle) != ESP_OK) {
    Serial.println("[mic] channel enable failed");
    i2s_del_channel(i2s_rx_handle);
    i2s_rx_handle = NULL;
    return false;
  }

  Serial.println("[mic] PDM enabled");
  return true;
}

// ── WAV helpers ───────────────────────────────────────────────────────────────
void createWAV(const char* path) {
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) != pdTRUE) return;
  SD.remove(path);
  File f = SD.open(path, FILE_WRITE);
  if (f) {
    f.write((uint8_t*)&wavHeader, sizeof(wavHeader));
    f.close();
  }
  xSemaphoreGive(sdMutex);
}

void finalizeWAV(const char* path) {
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) != pdTRUE) return;
  File f = SD.open(path, "r+");
  if (f) {
    int32_t size = f.size();
    wavHeader.flength = size - 8;
    wavHeader.dlength = size - sizeof(wavHeader);
    f.seek(0);
    f.write((uint8_t*)&wavHeader, sizeof(wavHeader));
    f.close();
    float dur = (float)wavHeader.dlength / (SAMPLE_RATE * 2);
    Serial.printf("[rec] Finalized %s (%.1fs, %d bytes)\n", path, dur, size);
  }
  xSemaphoreGive(sdMutex);
}

void appendAudio(const char* path) {
  static int16_t buf[1024];  // 2048 bytes, reused — no heap allocation
  size_t bytesRead = 0;
  i2s_channel_read(i2s_rx_handle, buf, sizeof(buf), &bytesRead, pdMS_TO_TICKS(100));

  if (bytesRead > 0 && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    for (int i = 0; i < (int)(bytesRead / 2); i++)
      buf[i] = (int16_t)constrain((long)buf[i] * MIC_GAIN, -32768, 32767);
    File f = SD.open(path, FILE_APPEND);
    if (f) { f.write((uint8_t*)buf, bytesRead); f.close(); }
    xSemaphoreGive(sdMutex);
  }
}

// Appends a custom 'ble_' chunk and updates the RIFF length
static void writeBLEChunk(const char* path, BLESnapshot* log, int count) {
  if (count == 0) return;
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) != pdTRUE) return;
  File f = SD.open(path, "r+");
  if (!f) { xSemaphoreGive(sdMutex); return; }

  f.seek(f.size());
  uint32_t chunkSize = (uint32_t)(count * sizeof(BLESnapshot));
  f.write((const uint8_t*)"ble_", 4);
  f.write((uint8_t*)&chunkSize, 4);
  f.write((uint8_t*)log, chunkSize);

  // Update RIFF flength to cover the new chunk
  uint32_t riffLen = (uint32_t)(f.size() - 8);
  f.seek(4);
  f.write((uint8_t*)&riffLen, 4);
  f.close();
  xSemaphoreGive(sdMutex);
  Serial.printf("[rec] BLE chunk: %d snapshots (%u bytes)\n", count, chunkSize);
}

// ── AWS4 signing helpers ──────────────────────────────────────────────────────
void toHex(unsigned char* in, size_t len, char* out) {
  for (size_t i = 0; i < len; i++) sprintf(out + i * 2, "%02x", in[i]);
  out[len * 2] = '\0';
}

void hmac256(const unsigned char* key, size_t klen,
             const unsigned char* msg, size_t mlen,
             unsigned char* out) {
  mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                  key, klen, msg, mlen, out);
}

void sha256str(const char* msg, char* hexOut) {
  unsigned char hash[32];
  mbedtls_md(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
             (const unsigned char*)msg, strlen(msg), hash);
  toHex(hash, 32, hexOut);
}


// ── MinIO upload ──────────────────────────────────────────────────────────────
bool uploadToMinIO(const char* filename) {
  time_t uploadStart = time(nullptr);
  struct tm* tmStart = gmtime(&uploadStart);
  Serial.printf("[upload] %s — starting at %02d:%02d:%02d\n",
                filename, tmStart->tm_hour, tmStart->tm_min, tmStart->tm_sec);
  isUploading = true;

  // Get file size
  String fpath = "/" + String(filename);
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000)) != pdTRUE) { isUploading = false; return false; }
  if (!SD.exists(fpath.c_str())) { xSemaphoreGive(sdMutex); isUploading = false; return false; }
  File tmp = SD.open(fpath.c_str(), FILE_READ);
  size_t fileSize = tmp.size();
  tmp.close();
  xSemaphoreGive(sdMutex);

  // Use UNSIGNED-PAYLOAD — MinIO accepts this over plain HTTP
  const char* fileHash = "UNSIGNED-PAYLOAD";

  // Timestamp
  time_t now = time(nullptr);
  struct tm* t = gmtime(&now);
  char date[9], tstr[7], dt[17];
  strftime(date, sizeof(date), "%Y%m%d", t);
  strftime(tstr, sizeof(tstr), "%H%M%S", t);
  snprintf(dt, sizeof(dt), "%sT%sZ", date, tstr);

  // Canonical request
  char canonReq[640];
  snprintf(canonReq, sizeof(canonReq),
    "PUT\n/%s/%s\n\nhost:%s:%d\nx-amz-content-sha256:%s\nx-amz-date:%s\n\n"
    "host;x-amz-content-sha256;x-amz-date\n%s",
    MINIO_BUCKET, filename, MINIO_HOST, MINIO_PORT, fileHash, dt, fileHash);

  char canonHash[65];
  sha256str(canonReq, canonHash);

  // String to sign
  char sts[512];
  snprintf(sts, sizeof(sts),
    "AWS4-HMAC-SHA256\n%s\n%s/%s/s3/aws4_request\n%s",
    dt, date, "us-east-1", canonHash);

  // Signing key chain
  char keySecret[80] = "AWS4";
  strncat(keySecret, MINIO_SECRET, 75);
  unsigned char kDate[32], kRegion[32], kService[32], kSign[32], sig[32];
  hmac256((unsigned char*)keySecret, strlen(keySecret), (unsigned char*)date, strlen(date), kDate);
  hmac256(kDate, 32, (unsigned char*)"us-east-1", 9, kRegion);
  hmac256(kRegion, 32, (unsigned char*)"s3", 2, kService);
  hmac256(kService, 32, (unsigned char*)"aws4_request", 12, kSign);
  hmac256(kSign, 32, (unsigned char*)sts, strlen(sts), sig);
  char sigHex[65];
  toHex(sig, 32, sigHex);

  // Auth header
  char auth[512];
  snprintf(auth, sizeof(auth),
    "AWS4-HMAC-SHA256 Credential=%s/%s/%s/s3/aws4_request,"
    " SignedHeaders=host;x-amz-content-sha256;x-amz-date, Signature=%s",
    MINIO_ACCESS, date, "us-east-1", sigHex);

  // TCP connect
  Serial.printf("[upload] TCP connecting to %s:%d...\n", MINIO_HOST, MINIO_PORT);
  WiFiClient tcp;
  if (!tcp.connect(MINIO_HOST, MINIO_PORT)) {
    Serial.println("[upload] TCP failed");
    isUploading = false;
    return false;
  }
  Serial.printf("[upload] TCP connected (+%lus)\n", (unsigned long)(time(nullptr) - uploadStart));

  // HTTP headers
  tcp.printf("PUT /%s/%s HTTP/1.1\r\n", MINIO_BUCKET, filename);
  tcp.printf("Host: %s:%d\r\n", MINIO_HOST, MINIO_PORT);
  tcp.printf("Content-Length: %d\r\n", (int)fileSize);
  tcp.print("Content-Type: audio/wav\r\n");
  tcp.printf("x-amz-date: %s\r\n", dt);
  tcp.printf("x-amz-content-sha256: %s\r\n", fileHash);
  tcp.printf("Authorization: %s\r\n", auth);
  tcp.print("Connection: close\r\n\r\n");

  Serial.printf("[upload] Streaming %d bytes (+%lus)\n", (int)fileSize, (unsigned long)(time(nullptr) - uploadStart));

  // Stream file — open under mutex, read without holding it so recording isn't blocked
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(3000)) != pdTRUE) {
    tcp.stop(); isUploading = false; return false;
  }
  File f = SD.open(fpath.c_str(), FILE_READ);
  xSemaphoreGive(sdMutex);  // release immediately after open

  if (f) {
    // Try full file in PSRAM → 64KB PSRAM → 8KB internal RAM
    uint8_t* buf      = (uint8_t*)ps_malloc(fileSize);
    size_t   BUF_SIZE = buf ? fileSize : 0;
    if (!buf) { buf = (uint8_t*)ps_malloc(65536);  BUF_SIZE = 65536; }
    if (!buf) { buf = (uint8_t*)malloc(8192);       BUF_SIZE = 8192;  }
    Serial.printf("[upload] buf %u bytes (%s)\n", BUF_SIZE, buf ? "ok" : "FAILED");
    size_t sent = 0;
    if (buf) {
      while (f.available()) {
        size_t n = f.read(buf, BUF_SIZE);
        tcp.write(buf, n);
        sent += n;
      }
      free(buf);
    }
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      f.close();
      xSemaphoreGive(sdMutex);
    }
    Serial.printf("[upload] Sent %d bytes (+%lus)\n", sent, (unsigned long)(time(nullptr) - uploadStart));
  }

  // Response
  String resp = "";
  unsigned long t0 = millis();
  while (millis() - t0 < 8000) {
    if (tcp.available()) { resp = tcp.readStringUntil('\n'); break; }
    delay(10);
  }
  tcp.stop();
  Serial.printf("[upload] Response: %s\n", resp.c_str());

  bool ok = resp.indexOf("200") != -1;
  if (ok) {
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      SD.remove(fpath.c_str());
      xSemaphoreGive(sdMutex);
    }
    Serial.printf("[upload] Done in %lus\n", (unsigned long)(time(nullptr) - uploadStart));
  } else {
    Serial.printf("[upload] Failed after %lus — file kept for retry\n", (unsigned long)(time(nullptr) - uploadStart));
  }
  isUploading = false;
  return ok;
}

void syncNTP() {
  if (ntpSynced) return;
  configTime(0, 0, "pool.ntp.org");
  time_t now = time(nullptr);
  int retries = 0;
  while (now < 1672531200 && retries < 20) { vTaskDelay(pdMS_TO_TICKS(500)); now = time(nullptr); retries++; }
  ntpSynced = (now >= 1672531200);
  Serial.printf("[ntp] %s\n", ntpSynced ? "synced" : "failed");
}

// ── Upload task ───────────────────────────────────────────────────────────────
void uploadTask(void* arg) {
  while (true) {
    // Block until a segment is ready (semaphore is also given on reboot if files exist)
    xSemaphoreTake(uploadReady, portMAX_DELAY);

    unsigned long wifiStart = millis();
    Serial.println("[wifi] Connecting...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    int r = 0;
    while (WiFi.status() != WL_CONNECTED && r < 20) { vTaskDelay(pdMS_TO_TICKS(500)); r++; }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[upload] No WiFi — will retry next segment");
      WiFi.mode(WIFI_OFF);
      continue;
    }
    Serial.printf("[wifi] Connected in %lus\n", (millis() - wifiStart) / 1000);
    syncNTP();

    // Scan SD for all .wav files and upload them, skipping the one currently recording
    while (true) {
      char nextFile[48] = "";

      if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
        File root = SD.open("/");
        while (true) {
          File entry = root.openNextFile();
          if (!entry) break;
          String name = String(entry.name());
          entry.close();
          if (name.endsWith(".wav") && ("/" + name) != currentFile) {
            strncpy(nextFile, name.c_str(), sizeof(nextFile) - 1);
            break;
          }
        }
        root.close();
        xSemaphoreGive(sdMutex);
      }

      if (nextFile[0] == '\0') break;  // no more files to upload

      int attempts = 0;
      while (!uploadToMinIO(nextFile) && attempts++ < 3) {
        Serial.printf("[upload] Retry %d/3 in 10s\n", attempts);
        vTaskDelay(pdMS_TO_TICKS(10000));
      }
    }

    WiFi.disconnect(false);
    WiFi.mode(WIFI_OFF);
    Serial.println("[wifi] Radio off — BLE active");
  }
}

// ── Filename helper ───────────────────────────────────────────────────────────
String makeFilename() {
  fileCounter++;
  if (ntpSynced) {
    time_t now = time(nullptr);
    struct tm* t = gmtime(&now);
    char buf[48];
    snprintf(buf, sizeof(buf), "/rec_%04d%02d%02d_%02d%02d%02d.wav",
             t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
             t->tm_hour, t->tm_min, t->tm_sec);
    return String(buf);
  }
  return "/rec_fallback_" + String(fileCounter) + ".wav";
}

// ── Recording task ────────────────────────────────────────────────────────────
void recordingTask(void* arg) {
  while (true) {
    while (!isRecording) { vTaskDelay(pdMS_TO_TICKS(100)); }

    if (!initMic()) {
      isRecording = false;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    currentFile = makeFilename();
    createWAV(currentFile.c_str());
    uint32_t segStart    = millis();
    uint32_t lastSnap    = millis();
    bleLogCount          = 0;
    stopRequested        = false;
    Serial.println("[rec] Started");

    while (!stopRequested) {
      appendAudio(currentFile.c_str());

      // Snapshot BLE state once per second
      if (millis() - lastSnap >= 1000 && bleLogCount < SEGMENT_S + 5) {
        BLESnapshot snap;
        snap.offset_ms = millis() - segStart;
        if (xSemaphoreTake(beaconsMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          for (int i = 0; i < NUM_BEACONS; i++) {
            snap.beacon[i].active   = beacons[i].active;
            snap.beacon[i].rssi     = (int8_t)beacons[i].rssi;
            snap.beacon[i].distance = beacons[i].distance;
          }
          xSemaphoreGive(beaconsMutex);
        }
        bleLog[bleLogCount++] = snap;
        lastSnap = millis();
      }

      if (millis() - segStart >= (uint32_t)SEGMENT_S * 1000) {
        finalizeWAV(currentFile.c_str());
        writeBLEChunk(currentFile.c_str(), bleLog, bleLogCount);
        xSemaphoreGive(uploadReady);

        currentFile = makeFilename();
        createWAV(currentFile.c_str());
        segStart    = millis();
        lastSnap    = millis();
        bleLogCount = 0;
        Serial.println("[rec] New segment");
      }
    }

    // Finalize last segment
    finalizeWAV(currentFile.c_str());
    writeBLEChunk(currentFile.c_str(), bleLog, bleLogCount);
    xSemaphoreGive(uploadReady);

    i2s_channel_disable(i2s_rx_handle);
    i2s_del_channel(i2s_rx_handle);
    i2s_rx_handle = NULL;
    isRecording = false;
    bleLogCount = 0;
    Serial.println("[rec] Stopped");
  }
}

// ── BLE callback ─────────────────────────────────────────────────────────────
class ScanCallback : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* device) override {
    if (!device->haveManufacturerData()) return;
    std::string mfr = device->getManufacturerData();  // keep alive
    const uint8_t* d = (const uint8_t*)mfr.data();
    size_t len = mfr.size();
    if (len < 25) return;
    if (d[0] != 0x4C || d[1] != 0x00) return;
    if (d[2] != 0x02 || d[3] != 0x15) return;
    if (memcmp(d + 4, TARGET_UUID, 16) != 0) return;
    BeaconData b;
    strncpy(b.address, device->getAddress().toString().c_str(), sizeof(b.address) - 1);
    b.address[sizeof(b.address) - 1] = '\0';
    b.major   = (d[20] << 8) | d[21];
    b.minor   = (d[22] << 8) | d[23];
    b.txPower = (int8_t)d[24];
    b.rssi    = device->getRSSI();
    xQueueSend(beaconQueue, &b, 0);
  }
};

// ── Display helpers ───────────────────────────────────────────────────────────
static void drawWifiIcon(uint8_t cx, uint8_t cy, bool active) {
  // dot at bottom, two arcs opening upward
  u8g2.drawPixel(cx, cy);
  u8g2.drawCircle(cx, cy, 2, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawCircle(cx, cy, 4, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
  if (!active)
    u8g2.drawLine(cx - 4, cy, cx + 4, cy - 4);  // / slash through icon
}

static void drawBLEIcon(uint8_t x, uint8_t y, bool active) {
  // Bluetooth symbol: vertical line + two right-pointing chevrons, 5×9 px
  u8g2.drawVLine(x + 1, y, 9);
  u8g2.drawLine(x + 1, y,     x + 3, y + 2);   // upper \
  u8g2.drawLine(x + 3, y + 2, x + 1, y + 4);   // upper /
  u8g2.drawLine(x + 1, y + 4, x + 3, y + 6);   // lower \
  u8g2.drawLine(x + 3, y + 6, x + 1, y + 8);   // lower /
  if (!active)
    u8g2.drawLine(x, y + 8, x + 4, y);          // / slash through icon
}

// ── Display ───────────────────────────────────────────────────────────────────
void updateDisplay() {
  static bool    blink    = true;
  static uint32_t lastBlink = 0;
  uint32_t now = millis();
  if (now - lastBlink >= 500) { blink = !blink; lastBlink = now; }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tf);

  // Header — CPU load per core + RAM%
  portDISABLE_INTERRUPTS();
  uint32_t raw0 = s_idle0; s_idle0 = 0;
  uint32_t raw1 = s_idle1; s_idle1 = 0;
  portENABLE_INTERRUPTS();
  // Baseline is fixed at calibration time (end of setup()) — do not update here.
  // Updating s_base in the display loop causes transient early-boot idle bursts to
  // inflate the baseline, which then makes steady-state load appear as ~50% CPU.
  int cpu0 = max(0, min(99, (int)(100 - (raw0 * 100UL / s_base0))));
  int cpu1 = max(0, min(99, (int)(100 - (raw1 * 100UL / s_base1))));
  int ramPct = (int)(ESP.getFreeHeap() * 100UL / ESP.getHeapSize());

  char hbuf[10];
  snprintf(hbuf, sizeof(hbuf), "C0:%2d%%", cpu0);  u8g2.drawStr(0,  7, hbuf);
  snprintf(hbuf, sizeof(hbuf), "C1:%2d%%", cpu1);  u8g2.drawStr(38, 7, hbuf);
  snprintf(hbuf, sizeof(hbuf), "R:%2d%%",  ramPct); u8g2.drawStr(76, 7, hbuf);
  u8g2.drawHLine(0, 9, 128);

  // Recording dot — flashes when recording
  if (isRecording && blink)
    u8g2.drawDisc(108, 4, 3);

  // BLE icon (x=114, y=0) and WiFi icon (cx=124, cy=7)
  drawBLEIcon(114, 0, bleScanning);
  drawWifiIcon(124, 7, WiFi.status() == WL_CONNECTED);

  // Beacon rows
  for (int i = 0; i < NUM_BEACONS; i++) {
    int y = 18 + i * 10;
    if (xSemaphoreTake(beaconsMutex, 0) != pdTRUE) continue;
    BeaconState s = beacons[i];
    xSemaphoreGive(beaconsMutex);

    char line[32];
    bool stale = !s.active || (now - s.lastSeenMs > STALE_MS);
    if (!s.active) {
      snprintf(line, sizeof(line), "T%d  --", i + 1);
      u8g2.drawStr(0, y, line);
    } else if (stale) {
      snprintf(line, sizeof(line), "T%d", i + 1);
      u8g2.drawStr(0, y, line);
      u8g2.drawStr(18, y, "out of range");
    } else {
      uint32_t age = (now - s.lastSeenMs) / 1000;
      snprintf(line, sizeof(line), "T%d %4ddBm %4.1fm %2lus",
               i + 1, s.rssi, s.distance, (unsigned long)age);
      u8g2.drawStr(0, y, line);
    }
  }

  u8g2.sendBuffer();
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  neopixelWrite(48, 0, 0, 0);  // turn off RGB LED (GPIO48)
  delay(500);
  esp_log_level_set("wifi", ESP_LOG_NONE);

  beaconsMutex = xSemaphoreCreateMutex();
  sdMutex      = xSemaphoreCreateMutex();
  uploadReady  = xSemaphoreCreateBinary();
  beaconQueue  = xQueueCreate(32, sizeof(BeaconData));

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Display
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(0, 20, "Starting...");
  u8g2.sendBuffer();

  // SD card
  if (!SD.begin(SD_CS_PIN)) Serial.println("[sd] Init failed — check card");

  // WiFi — connect once for NTP, then radio off until upload needed
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int att = 0;
  while (WiFi.status() != WL_CONNECTED && att < 20) { delay(500); att++; }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[wifi] Connected: %s\n", WiFi.localIP().toString().c_str());
    syncNTP();
  } else {
    Serial.println("[wifi] Failed to connect");
  }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tf);
  if (WiFi.status() == WL_CONNECTED) {
    u8g2.drawStr(0, 10, "WiFi OK");
    u8g2.drawStr(0, 20, WiFi.localIP().toString().c_str());
  } else {
    u8g2.drawStr(0, 10, "WiFi failed");
  }
  u8g2.drawStr(0, 32, "Starting BLE...");
  u8g2.sendBuffer();
  delay(1000);

  // BLE — init before turning WiFi off so coexistence module is set up correctly
  NimBLEDevice::init("");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new ScanCallback(), true);
  scan->setActiveScan(false);
  scan->setInterval(16);
  scan->setWindow(16);
  scan->setMaxResults(0);
  scan->start(0, false);
  bleScanning = true;

  // WiFi off now that BLE is running
  WiFi.disconnect(false);
  WiFi.mode(WIFI_OFF);
  Serial.println("[wifi] Radio off — BLE active");

  // CPU load hooks — must register before tasks start
  esp_register_freertos_idle_hook_for_cpu(idleHook0, 0);
  esp_register_freertos_idle_hook_for_cpu(idleHook1, 1);

  // FreeRTOS tasks — pinned to Core 1 so Core 0 stays free for WiFi/BLE radio
  xTaskCreatePinnedToCore(recordingTask, "rec", 8192,  NULL, 2, NULL, 1);  // Core 1 — uncontested I2S/SD
  xTaskCreatePinnedToCore(uploadTask,    "upl", 12288, NULL, 1, NULL, 0);  // Core 0 — co-located with WiFi/lwIP

  // Calibrate CPU baseline — wait for NimBLE to reach steady state (beacons start
  // arriving within ~300ms), then measure one clean window as the idle reference.
  delay(2000);
  portDISABLE_INTERRUPTS(); s_idle0 = 0; s_idle1 = 0; portENABLE_INTERRUPTS();
  delay(DISPLAY_MS);
  portDISABLE_INTERRUPTS();
  s_base0 = max((uint32_t)1, (uint32_t)s_idle0);
  s_base1 = max((uint32_t)1, (uint32_t)s_idle1);
  s_idle0 = 0; s_idle1 = 0;
  portENABLE_INTERRUPTS();
  Serial.printf("[cpu] baseline C0=%u C1=%u\n", s_base0, s_base1);

  Serial.println("Ready. Press button to record.");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  static uint32_t lastDisplay = 0;
  static int      lastBtnRaw  = HIGH;
  static int      btnStable   = HIGH;
  static uint32_t btnTime     = 0;

  // ── Button debounce ──
  int btnRaw = digitalRead(BUTTON_PIN);
  if (btnRaw != lastBtnRaw) { btnTime = millis(); lastBtnRaw = btnRaw; }
  if (millis() - btnTime > 50 && btnRaw != btnStable) {
    btnStable = btnRaw;
    if (btnStable == LOW) {
      if (!isRecording) {
        Serial.println("[btn] Start recording");
        isRecording   = true;
        stopRequested = false;
      } else {
        Serial.println("[btn] Stop recording");
        stopRequested = true;
      }
    }
  }

  // ── Drain BLE queue ──
  BeaconData b;
  while (xQueueReceive(beaconQueue, &b, 0) == pdTRUE) {
    if (b.major < 1 || b.major > NUM_BEACONS) continue;
    int slot = b.major - 1;

    float dist = powf(10.0f, (b.txPower - b.rssi) / 25.0f);
    if (xSemaphoreTake(beaconsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      BeaconState& s = beacons[slot];
      s.active     = true;
      s.rssi       = b.rssi;
      s.distance   = dist;
      s.lastSeenMs = millis();
      xSemaphoreGive(beaconsMutex);
    }
  }

  // ── Display ──
  if (millis() - lastDisplay >= DISPLAY_MS) {
    lastDisplay = millis();
    updateDisplay();
  }

  vTaskDelay(1);  // yield 1 tick so the idle task gets Core 1 time for CPU metering
}
