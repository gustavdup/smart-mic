/*
 * iBeacon Scanner + Audio Recorder
 * XIAO ESP32-S3 Sense + Seeed Expansion Board v1
 *
 * Libraries (install via Library Manager):
 *   - NimBLE-Arduino      (h2zero)
 *   - Adafruit SSD1306    (Adafruit)
 *   - Adafruit GFX        (Adafruit)
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
#include <esp_wifi.h>
#include <NimBLEDevice.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SD.h>
#include <SPI.h>
#include "driver/i2s_std.h"
#include "driver/temperature_sensor.h"
#include "mbedtls/md.h"
#include "time.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include <Preferences.h>
#include <ESP32QRCodeReader.h>
#include "quirc/quirc.h"   // direct quirc API for proven QR scanning
// Forward-declare idle hook API (avoids pulling in esp_freertos_hooks.h which conflicts with packed structs)
extern "C" esp_err_t esp_register_freertos_idle_hook_for_cpu(bool (*hook)(void), UBaseType_t cpuid);

// Atomic-safe pending upload counters (cross-core access)
// pendingPSRAM: segments queued in segmentQueue or currently uploading from PSRAM
// pendingSD:    WAV files sitting on SD card waiting to upload
#define PSRAM_INC() do { portENTER_CRITICAL_SAFE(&_pend_mux); pendingPSRAM++; portEXIT_CRITICAL_SAFE(&_pend_mux); } while(0)
#define PSRAM_DEC() do { portENTER_CRITICAL_SAFE(&_pend_mux); if (pendingPSRAM > 0) pendingPSRAM--; portEXIT_CRITICAL_SAFE(&_pend_mux); } while(0)
#define SD_INC()    do { portENTER_CRITICAL_SAFE(&_pend_mux); pendingSD++;    portEXIT_CRITICAL_SAFE(&_pend_mux); } while(0)
#define SD_DEC()    do { portENTER_CRITICAL_SAFE(&_pend_mux); if (pendingSD > 0) pendingSD--;    portEXIT_CRITICAL_SAFE(&_pend_mux); } while(0)
static portMUX_TYPE _pend_mux = portMUX_INITIALIZER_UNLOCKED;

// ── WiFi (MinIO upload only) ──────────────────────────────────────────────────
// Mutable so loadConfig() can overwrite from SD card or NVS at boot
char WIFI_SSID    [64] = "Home DSL";
char WIFI_PASSWORD[64] = "123454321";

// ── MinIO ─────────────────────────────────────────────────────────────────────
char MINIO_HOST   [64] = "192.168.1.72";  // Mac IP — update via config.txt
int  MINIO_PORT        = 9000;
char MINIO_BUCKET [32] = "audio";
char MINIO_ACCESS [32] = "minioadmin";
char MINIO_SECRET [32] = "minioadmin";

// ── Waiter identity (set via QR scan, persisted to NVS) ─────────────────────
char waiterName[32] = "";
char waiterCode[16] = "";

// ── Camera pins (OV2640 on XIAO ESP32-S3 Sense) ───────────────────────────────
// GPIO14 is shared with PDM_PWR — only used when mic is powered down (not recording)
#define CAM_PIN_PWDN   -1
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK   10
#define CAM_PIN_SIOD   40
#define CAM_PIN_SIOC   39
#define CAM_PIN_Y9     48
#define CAM_PIN_Y8     11
#define CAM_PIN_Y7     12
#define CAM_PIN_Y6     14
#define CAM_PIN_Y5     16
#define CAM_PIN_Y4     18
#define CAM_PIN_Y3     17
#define CAM_PIN_Y2     15
#define CAM_PIN_VSYNC  38
#define CAM_PIN_HREF   47
#define CAM_PIN_PCLK   13

// ── iBeacon UUID ──────────────────────────────────────────────────────────────
const uint8_t TARGET_UUID[16] = {
  0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48, 0xD2,
  0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0
};

// ── Pins ──────────────────────────────────────────────────────────────────────
#define BUTTON_PIN   4   // Button (D3 = GPIO4)
#define SD_CS_PIN    21  // SD card chip select on expansion board
#define PDM_PWR      14  // camera Y6 — must be LOW before esp_camera_init
#define I2S_BCLK      1  // D0 — ICS-43434 bit clock
#define I2S_WS        2  // D1 — ICS-43434 word select (LRCL)
#define I2S_DIN       3  // D2 — ICS-43434 data out
#define MOTOR_PIN    44  // Vibration coin motor (D7 = GPIO44)

// ── Recording ─────────────────────────────────────────────────────────────────
static int  SAMPLE_RATE = 16000;  // configurable via config.txt (sample_rate=)
#define SEGMENT_S     30     // Segment and upload every 30 seconds
#define MIC_GAIN_L        4      // Gain for left channel (mic1, table-facing)
#define MIC_GAIN_R        4      // Gain for right channel (mic2, waiter-facing)

// ── Beacon / Display ──────────────────────────────────────────────────────────
// NUM_BEACONS defined above (before #includes) to fix Arduino prototype ordering
#define STALE_MS     10000
#define DISPLAY_MS   500

// ── OLED ──────────────────────────────────────────────────────────────────────
// 0.49" SSD1306 64×32 on hardware I2C (GPIO5=SDA, GPIO6=SCL)
Adafruit_SSD1306 oled(64, 32, &Wire, -1);

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

// ── Temperature sensor ────────────────────────────────────────────────────────
static temperature_sensor_handle_t s_temp_sensor = NULL;

// ── Recording / upload state ──────────────────────────────────────────────────
volatile bool isRecording   = false;
volatile bool isUploading   = false;
volatile bool stopRequested = false;
volatile bool g_motorActive = false;  // suppresses updateDisplay() I2C during motor pulses
volatile int   uploadingBuf  = -1;   // index of PSRAM buffer currently uploading (-1 = none)
volatile float uploadKBps   = 0.0f; // live upload speed, updated during streaming

SemaphoreHandle_t uploadReady;  // signalled when a new file is ready to upload
SemaphoreHandle_t sdMutex;      // SD card access lock

bool     ntpSynced     = false;
int      fileCounter   = 0;
String   currentFile   = "";
volatile bool bleScanning = false;

// ── Multi-screen display & log ────────────────────────────────────────────────
static int  currentScreen  = 0;   // 0=main 1=waiter 2=rec 3=BLE T1-3 4=BLE T4-5 5=status 6=CPU 7=speedtest 8=logs
static volatile int pendingPSRAM = 0;  // segments in PSRAM pipeline (queued or uploading)
static volatile int pendingSD    = 0;  // WAV files on SD waiting to upload

#define LOG_LINES 5
#define LOG_COLS  22
static char logBuf[LOG_LINES][LOG_COLS];
static int  logIdx = 0;
static SemaphoreHandle_t logMutex = NULL;

static uint8_t  g_btnState     = 0;   // 0=idle 1=down 2=up(counting clicks)
static uint32_t g_btnStateTime = 0;   // millis() when current press/wait started
static uint8_t  g_clickCount   = 0;   // rapid click counter
static bool     g_screenOff     = false;
static uint32_t g_lastActivityMs = 0;  // millis() of last button press — drives 60s screen timeout
static bool     uploadsEnabled = true;

// ── Speed test results ────────────────────────────────────────────────────────
static bool          st_done  = false;
static size_t        st_bytes = 0;
static unsigned long st_ms    = 0;
static float         st_kbps  = 0;

// ── PSRAM double-buffer recording ─────────────────────────────────────────────
// Two 2 MB PSRAM buffers. recordingTask writes audio into the active buffer;
// at each segment boundary it queues the full buffer (WAV header + audio + BLE
// chunk already patched in) and flips to the other. uploadTask streams the
// queued buffer directly to MinIO — no SD reads needed.
// SD is only written as a fallback when WiFi is unavailable or upload fails.
#define PSRAM_BUF_SIZE  (2UL * 1024 * 1024)   // 2 MB per buffer

struct SegmentReady {
  uint8_t* buf;       // pointer to one of psramBuf[0/1]
  size_t   size;      // total bytes: WAV header + audio + BLE chunk
  char     filename[48];
};
static QueueHandle_t segmentQueue  = NULL;
static uint8_t*      psramBuf[2]   = {nullptr, nullptr};
static size_t        psramFill[2]  = {0, 0};  // audio bytes written (not counting WAV header)
static int           activeBuf     = 0;

void addLog(const char* msg) {
  if (!logMutex) return;
  if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    strncpy(logBuf[logIdx], msg, LOG_COLS - 1);
    logBuf[logIdx][LOG_COLS - 1] = '\0';
    logIdx = (logIdx + 1) % LOG_LINES;
    xSemaphoreGive(logMutex);
  }
}

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
  int16_t num_chans    = 2;
  int32_t srate        = 8000;   // set at runtime from SAMPLE_RATE in initMic()
  int32_t bytes_per_sec = 32000; // set at runtime: SAMPLE_RATE * 4
  int16_t bytes_per_samp = 4;   // 2 channels × 2 bytes per frame
  int16_t bits_per_samp  = 16;
  char  dat[4]         = {'d','a','t','a'};
  int32_t dlength      = 0;
} wavHeader;
#pragma pack(pop)

// ── I2S microphone (Adafruit ICS-43434) ──────────────────────────────────────
bool initMic() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  if (i2s_new_channel(&chan_cfg, NULL, &i2s_rx_handle) != ESP_OK) {
    Serial.println("[mic] channel create failed");
    return false;
  }

  i2s_std_config_t std_cfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_BCLK,
      .ws   = (gpio_num_t)I2S_WS,
      .dout = I2S_GPIO_UNUSED,
      .din  = (gpio_num_t)I2S_DIN,
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
    },
  };

  if (i2s_channel_init_std_mode(i2s_rx_handle, &std_cfg) != ESP_OK) {
    Serial.println("[mic] I2S std init failed");
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

  // Patch WAV header with runtime sample rate
  wavHeader.srate         = SAMPLE_RATE;
  wavHeader.bytes_per_sec = SAMPLE_RATE * 4;  // stereo 16-bit

  Serial.printf("[mic] ICS-43434 I2S enabled @ %d Hz stereo\n", SAMPLE_RATE);
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
    Serial.printf("[sd] Created %s\n", path);
  } else {
    Serial.printf("[sd] FAILED to create %s\n", path);
  }
  xSemaphoreGive(sdMutex);
}

void finalizeWAV(const char* path) {
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) != pdTRUE) return;
  File f = SD.open(path, "r+");
  if (f) {
    int32_t size = f.size();
    wavHeader.flength = size - 8;
    wavHeader.dlength = size - sizeof(wavHeader);
    f.seek(0);
    f.write((uint8_t*)&wavHeader, sizeof(wavHeader));
    f.close();
    float dur = (float)wavHeader.dlength / (float)(SAMPLE_RATE * 4);  // stereo: 2ch × 2 bytes
    Serial.printf("[rec] Finalized %s (%.1fs, %d bytes)\n", path, dur, size);
  }
  xSemaphoreGive(sdMutex);
}


// Appends a custom 'ble_' chunk and updates the RIFF length
static void writeBLEChunk(const char* path, BLESnapshot* log, int count) {
  if (count == 0) return;
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) != pdTRUE) return;
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
  Serial.printf("[upload] fpath=%s\n", fpath.c_str());
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
    Serial.println("[upload] FAIL: sdMutex timeout");
    isUploading = false; return false;
  }
  if (!SD.exists(fpath.c_str())) {
    Serial.printf("[upload] FAIL: file not found: %s\n", fpath.c_str());
    xSemaphoreGive(sdMutex); isUploading = false; return false;
  }
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
  tcp.setNoDelay(true);  // disable Nagle — send segments immediately
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
    // Internal RAM buffer for TCP writes — WiFi DMA cannot read PSRAM directly.
    // 16 KB reduces SD SPI round-trips ~4×; watchdog fed explicitly so no forced yield needed.
    const size_t BUF_SIZE = 16 * 1024;
    uint8_t* buf = (uint8_t*)heap_caps_malloc(BUF_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!buf) { buf = (uint8_t*)malloc(BUF_SIZE); }
    Serial.printf("[upload] buf %u bytes (%s)\n", BUF_SIZE, buf ? "ok" : "FAILED");
    size_t        sent        = 0;
    unsigned long streamStart = millis();
    if (buf) {
      while (f.available()) {
        size_t n = f.read(buf, BUF_SIZE);
        int written = tcp.write(buf, n);
        if (written > 0) {
          sent += (size_t)written;
          unsigned long el = millis() - streamStart;
          if (el > 0) uploadKBps = (float)sent / el * 1000.0f / 1024.0f;
        }
        vTaskDelay(1);  // yield so IDLE0 runs (feeds TWDT) — only ~60 yields/file at 16 KB chunks
      }
      free(buf);
    } else {
      Serial.println("[upload] FAIL: no buffer — file not sent");
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
  uploadKBps = 0.0f; isUploading = false;  // zero kbps before clearing flag to avoid stale display
  return ok;
}

// ── WiFi speed test ───────────────────────────────────────────────────────────
// Sends 2 MB of dummy data to MinIO with valid AWS Signature V4 auth so MinIO
// accepts the upload. Measures steady-state TCP send throughput over WiFi.
void wifiSpeedTest() {
  const size_t TEST_BYTES = 8UL * 1024 * 1024;  // 8 MB — good steady-state sample
  const size_t CHUNK      = 4 * 1024;            // 4 KB — internal RAM (DMA-safe)

  uint8_t* buf = (uint8_t*)heap_caps_malloc(CHUNK, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!buf) buf = (uint8_t*)malloc(CHUNK);
  if (!buf) { Serial.println("[speedtest] No memory"); return; }
  memset(buf, 0xAA, CHUNK);

  // Build AWS Signature V4 auth (same as uploadToMinIO)
  const char* fileHash = "UNSIGNED-PAYLOAD";
  time_t now_t = time(nullptr);
  struct tm* t = gmtime(&now_t);
  char date[9], tstr[7], dt[17];
  strftime(date, sizeof(date), "%Y%m%d", t);
  strftime(tstr, sizeof(tstr), "%H%M%S", t);
  snprintf(dt, sizeof(dt), "%sT%sZ", date, tstr);

  char canonReq[640];
  snprintf(canonReq, sizeof(canonReq),
    "PUT\n/%s/_speedtest\n\nhost:%s:%d\nx-amz-content-sha256:%s\nx-amz-date:%s\n\n"
    "host;x-amz-content-sha256;x-amz-date\n%s",
    MINIO_BUCKET, MINIO_HOST, MINIO_PORT, fileHash, dt, fileHash);
  char canonHash[65]; sha256str(canonReq, canonHash);

  char sts[512];
  snprintf(sts, sizeof(sts), "AWS4-HMAC-SHA256\n%s\n%s/%s/s3/aws4_request\n%s",
           dt, date, "us-east-1", canonHash);

  char keySecret[80] = "AWS4";
  strncat(keySecret, MINIO_SECRET, 75);
  unsigned char kDate[32], kRegion[32], kService[32], kSign[32], sig[32];
  hmac256((unsigned char*)keySecret, strlen(keySecret), (unsigned char*)date, strlen(date), kDate);
  hmac256(kDate, 32, (unsigned char*)"us-east-1", 9, kRegion);
  hmac256(kRegion, 32, (unsigned char*)"s3", 2, kService);
  hmac256(kService, 32, (unsigned char*)"aws4_request", 12, kSign);
  hmac256(kSign, 32, (unsigned char*)sts, strlen(sts), sig);
  char sigHex[65]; toHex(sig, 32, sigHex);

  char auth[512];
  snprintf(auth, sizeof(auth),
    "AWS4-HMAC-SHA256 Credential=%s/%s/%s/s3/aws4_request,"
    " SignedHeaders=host;x-amz-content-sha256;x-amz-date, Signature=%s",
    MINIO_ACCESS, date, "us-east-1", sigHex);

  WiFiClient tcp;
  tcp.setNoDelay(true);
  if (!tcp.connect(MINIO_HOST, MINIO_PORT)) {
    Serial.println("[speedtest] TCP failed");
    free(buf); return;
  }

  tcp.printf("PUT /%s/_speedtest HTTP/1.1\r\n", MINIO_BUCKET);
  tcp.printf("Host: %s:%d\r\n", MINIO_HOST, MINIO_PORT);
  tcp.printf("Content-Length: %u\r\n", (unsigned)TEST_BYTES);
  tcp.print("Content-Type: application/octet-stream\r\n");
  tcp.printf("x-amz-date: %s\r\n", dt);
  tcp.printf("x-amz-content-sha256: %s\r\n", fileHash);
  tcp.printf("Authorization: %s\r\n", auth);
  tcp.print("Connection: close\r\n\r\n");

  // Check if MinIO immediately rejects (bad auth, missing bucket, etc.)
  delay(200);
  if (tcp.available()) {
    String earlyResp = tcp.readStringUntil('\n');
    Serial.printf("[speedtest] MinIO early response: %s\n", earlyResp.c_str());
    tcp.stop(); free(buf); return;
  }

  unsigned long t0           = millis();
  unsigned long lastDisp     = 0;
  size_t        sent         = 0;
  unsigned long lastProgress = millis();
  while (sent < TEST_BYTES) {
    if (!tcp.connected()) break;
    if (millis() - lastProgress > 5000) { Serial.println("[speedtest] stalled"); break; }
    if (digitalRead(BUTTON_PIN) == LOW) { Serial.println("[speedtest] cancelled"); break; }
    size_t n = min((size_t)CHUNK, (size_t)(TEST_BYTES - sent));
    int w = tcp.write(buf, n);
    if (w > 0) { sent += w; lastProgress = millis(); }

    unsigned long now = millis();
    if (now - lastDisp >= 500 && sent > 0) {
      lastDisp = now;
      unsigned long el = now - t0;
      float mbps = (sent * 8.0f) / (el / 1000.0f) / 1e6f;
      char s1[24], s2[24];
      snprintf(s1, sizeof(s1), "%.2f Mbps", mbps);
      snprintf(s2, sizeof(s2), "%lu / %lu KB", (unsigned long)(sent/1024),
               (unsigned long)(TEST_BYTES/1024));
      oled.clearDisplay();
      oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
      oled.setCursor(0, 3); oled.print("Speedtest");
      oled.setCursor(0, 15); oled.print(s1);
      oled.setCursor(0, 27); oled.print(s2);
      oled.display();
      Serial.printf("[speedtest] %.2f Mbps — %lu KB\n", mbps, (unsigned long)(sent/1024));
    }
  }
  unsigned long elapsed = millis() - t0;
  bool cancelled = (digitalRead(BUTTON_PIN) == LOW);
  tcp.stop();
  free(buf);

  if (cancelled) {
    addLog("Speedtest cancelled");
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
    oled.setCursor(0, 11); oled.print("Speedtest");
    oled.setCursor(0, 23); oled.print("Cancelled");
    oled.display();
    delay(1500);
    return;
  }
  if (!sent || !elapsed) return;
  float kbps = (float)sent / elapsed * 1000.0f / 1024.0f;
  Serial.printf("[speedtest] %u bytes in %lums → %.0f KB/s (%.2f Mbps)\n",
                (unsigned)sent, elapsed, kbps, kbps * 8.0f / 1024.0f);
  st_bytes = sent;
  st_ms    = elapsed;
  st_kbps  = kbps;
  st_done  = true;
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

// ── Upload a segment directly from PSRAM (no SD reads) ───────────────────────
static bool uploadFromPSRAM(const SegmentReady& seg) {
  time_t uploadStart = time(nullptr);
  struct tm* tmStart = gmtime(&uploadStart);
  Serial.printf("[upload] PSRAM %s — starting at %02d:%02d:%02d (%u bytes)\n",
                seg.filename, tmStart->tm_hour, tmStart->tm_min, tmStart->tm_sec,
                (unsigned)seg.size);
  isUploading = true;
  // Track which PSRAM buffer is being uploaded so the display can show "UPL"
  if      (seg.buf == psramBuf[0]) uploadingBuf = 0;
  else if (seg.buf == psramBuf[1]) uploadingBuf = 1;
  else                             uploadingBuf = -1;
  const char* fileHash = "UNSIGNED-PAYLOAD";

  time_t now = time(nullptr);
  struct tm* t = gmtime(&now);
  char date[9], tstr[7], dt[17];
  strftime(date, sizeof(date), "%Y%m%d", t);
  strftime(tstr, sizeof(tstr), "%H%M%S", t);
  snprintf(dt, sizeof(dt), "%sT%sZ", date, tstr);

  char canonReq[640];
  snprintf(canonReq, sizeof(canonReq),
    "PUT\n/%s/%s\n\nhost:%s:%d\nx-amz-content-sha256:%s\nx-amz-date:%s\n\n"
    "host;x-amz-content-sha256;x-amz-date\n%s",
    MINIO_BUCKET, seg.filename, MINIO_HOST, MINIO_PORT, fileHash, dt, fileHash);
  char canonHash[65]; sha256str(canonReq, canonHash);

  char sts[512];
  snprintf(sts, sizeof(sts), "AWS4-HMAC-SHA256\n%s\n%s/%s/s3/aws4_request\n%s",
           dt, date, "us-east-1", canonHash);

  char keySecret[80] = "AWS4";
  strncat(keySecret, MINIO_SECRET, 75);
  unsigned char kDate[32], kRegion[32], kService[32], kSign[32], sig[32];
  hmac256((unsigned char*)keySecret, strlen(keySecret), (unsigned char*)date, strlen(date), kDate);
  hmac256(kDate,    32, (unsigned char*)"us-east-1",    9,  kRegion);
  hmac256(kRegion,  32, (unsigned char*)"s3",           2,  kService);
  hmac256(kService, 32, (unsigned char*)"aws4_request", 12, kSign);
  hmac256(kSign,    32, (unsigned char*)sts, strlen(sts), sig);
  char sigHex[65]; toHex(sig, 32, sigHex);

  char auth[512];
  snprintf(auth, sizeof(auth),
    "AWS4-HMAC-SHA256 Credential=%s/%s/%s/s3/aws4_request,"
    " SignedHeaders=host;x-amz-content-sha256;x-amz-date, Signature=%s",
    MINIO_ACCESS, date, "us-east-1", sigHex);

  Serial.printf("[upload] PSRAM TCP connecting to %s:%d...\n", MINIO_HOST, MINIO_PORT);
  WiFiClient tcp;
  if (!tcp.connect(MINIO_HOST, MINIO_PORT, 5000)) {
    Serial.println("[upload] PSRAM TCP failed");
    uploadingBuf = -1; isUploading = false; return false;
  }
  tcp.setNoDelay(true);  // must be AFTER connect — socket doesn't exist before connect
  Serial.printf("[upload] PSRAM TCP connected (+%lus)\n", (unsigned long)(time(nullptr) - uploadStart));

  tcp.printf("PUT /%s/%s HTTP/1.1\r\n", MINIO_BUCKET, seg.filename);
  tcp.printf("Host: %s:%d\r\n", MINIO_HOST, MINIO_PORT);
  tcp.printf("Content-Length: %u\r\n", (unsigned)seg.size);
  tcp.print("Content-Type: audio/wav\r\n");
  tcp.printf("x-amz-date: %s\r\n", dt);
  tcp.printf("x-amz-content-sha256: %s\r\n", fileHash);
  tcp.printf("Authorization: %s\r\n", auth);
  tcp.print("Connection: close\r\n\r\n");

  // Wait for headers to be flushed, sent, and ACKed before checking for early
  // rejection. With Nagle off (setNoDelay), headers leave immediately. LAN RTT
  // is ~1ms, so 500ms is more than enough for ACKs to come back.
  vTaskDelay(pdMS_TO_TICKS(500));
  if (tcp.available()) {
    String earlyResp = tcp.readStringUntil('\n');
    Serial.printf("[upload] PSRAM early MinIO response: %s\n", earlyResp.c_str());
    tcp.stop(); uploadingBuf = -1; isUploading = false; return false;
  }

  // Copy PSRAM → 16 KB internal buf → TCP (WiFi DMA cannot read PSRAM directly)
  const size_t CHUNK = 16 * 1024;
  uint8_t* ibuf = (uint8_t*)heap_caps_malloc(CHUNK, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!ibuf) { tcp.stop(); uploadingBuf = -1; isUploading = false; return false; }

  Serial.printf("[upload] PSRAM Streaming %u bytes (+%lus)\n",
                (unsigned)seg.size, (unsigned long)(time(nullptr) - uploadStart));
  size_t        sent         = 0;
  unsigned long streamStart  = millis();
  unsigned long lastProgress = millis();
  bool          stalled      = false;
  while (sent < seg.size) {
    if (!tcp.connected()) { Serial.println("[upload] PSRAM disconnected"); stalled = true; break; }
    if (millis() - lastProgress > 8000) { Serial.println("[upload] PSRAM stalled"); stalled = true; break; }
    size_t n = min(CHUNK, seg.size - sent);
    memcpy(ibuf, seg.buf + sent, n);
    int written = tcp.write(ibuf, n);
    if (written > 0) {
      sent += (size_t)written;
      lastProgress = millis();
      unsigned long el = millis() - streamStart;
      if (el > 0) uploadKBps = (float)sent / el * 1000.0f / 1024.0f;
    }
    vTaskDelay(1);  // yield to WiFi/lwIP task — same pattern as uploadToMinIO
  }
  free(ibuf);
  if (stalled) {
    tcp.stop();
    uploadingBuf = -1; uploadKBps = 0.0f; isUploading = false;
    return false;
  }
  Serial.printf("[upload] PSRAM Sent %u / %u bytes (+%lus)\n",
                (unsigned)sent, (unsigned)seg.size, (unsigned long)(time(nullptr) - uploadStart));

  String resp = "";
  unsigned long t0 = millis();
  while (millis() - t0 < 8000) {
    if (tcp.available()) { resp = tcp.readStringUntil('\n'); break; }
    delay(10);
  }
  tcp.stop();
  Serial.printf("[upload] PSRAM Response: %s\n", resp.c_str());

  bool ok = resp.indexOf("200") != -1;
  if (ok) {
    Serial.printf("[upload] PSRAM Done in %lus\n", (unsigned long)(time(nullptr) - uploadStart));
  } else {
    Serial.printf("[upload] PSRAM FAILED after %lus\n", (unsigned long)(time(nullptr) - uploadStart));
  }
  uploadingBuf = -1; uploadKBps = 0.0f; isUploading = false;
  return ok;
}

// ── Write a PSRAM segment to SD (fallback when WiFi down or upload fails) ─────
static void writePSRAMToSD(const SegmentReady& seg) {
  String path = "/" + String(seg.filename);
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) != pdTRUE) return;
  File f = SD.open(path.c_str(), FILE_WRITE);
  if (f) {
    f.write(seg.buf, seg.size);
    f.close();
    SD_INC();  // file now on SD waiting to upload
    Serial.printf("[sd] Fallback write: %s (%u bytes)\n", path.c_str(), (unsigned)seg.size);
  } else {
    Serial.printf("[sd] FAIL: fallback write failed for %s\n", path.c_str());
  }
  xSemaphoreGive(sdMutex);
}

// ── Upload task ───────────────────────────────────────────────────────────────
void uploadTask(void* arg) {
  while (true) {
    // Block until a segment is ready
    xSemaphoreTake(uploadReady, portMAX_DELAY);

    // If uploads disabled: drain PSRAM queue to SD so buffers don't overflow,
    // then sleep. Files will be picked up when uploads are re-enabled.
    if (!uploadsEnabled) {
      SegmentReady seg;
      while (xQueueReceive(segmentQueue, &seg, 0) == pdTRUE) {
        PSRAM_DEC();  // leaving PSRAM pipeline
        writePSRAMToSD(seg);  // SD_INC happens inside
      }
      continue;
    }

    unsigned long wifiStart = millis();
    Serial.println("[wifi] Connecting...");
    addLog("WiFi connecting...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    int r = 0;
    while (WiFi.status() != WL_CONNECTED && r < 20) { vTaskDelay(pdMS_TO_TICKS(500)); r++; }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[upload] No WiFi — falling back to SD");
      addLog("No WiFi->SD");
      WiFi.mode(WIFI_OFF);
      // Drain PSRAM queue to SD — if we just continue, the semaphore is consumed
      // but segments stay in the queue orphaned (nothing will retry them).
      { SegmentReady seg;
        while (xQueueReceive(segmentQueue, &seg, 0) == pdTRUE) {
          PSRAM_DEC();
          writePSRAMToSD(seg);
        }
      }
      // Re-signal after 60s so uploadTask retries SD files without needing a new recording segment
      vTaskDelay(pdMS_TO_TICKS(60000));
      xSemaphoreGive(uploadReady);
      continue;
    }
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    Serial.printf("[wifi] Connected in %lus\n", (millis() - wifiStart) / 1000);
    addLog("WiFi connected");
    syncNTP();
    // Brief settle after WiFi connect — ARP/lwIP needs a moment after radio was off.
    Serial.println("[upload] Waiting for TCP stack to settle...");
    vTaskDelay(pdMS_TO_TICKS(500));

    // ── Upload PSRAM segments (normal path — no SD reads) ──────────────────
    { SegmentReady seg;
      while (xQueueReceive(segmentQueue, &seg, 0) == pdTRUE) {
        if (!uploadsEnabled) { PSRAM_DEC(); writePSRAMToSD(seg); continue; }
        char logmsg[LOG_COLS];
        snprintf(logmsg, sizeof(logmsg), "UL %s", seg.filename);
        addLog(logmsg);
        // One retry only — PSRAM buffer is only valid for ~30s (until recordingTask
        // flips back to it). Multiple retries with long delays corrupt the buffer.
        bool ok = uploadFromPSRAM(seg);
        if (!ok) {
          Serial.println("[upload] PSRAM attempt 1 failed — retrying once");
          vTaskDelay(pdMS_TO_TICKS(1000));
          ok = uploadFromPSRAM(seg);
        }
        if (ok) {
          PSRAM_DEC();
          // Clear fill so display shows "free" instead of "ready"
          if      (seg.buf == psramBuf[0]) psramFill[0] = 0;
          else if (seg.buf == psramBuf[1]) psramFill[1] = 0;
          addLog("UL ok");
        } else { PSRAM_DEC(); writePSRAMToSD(seg); addLog("UL→SD"); }
      }
    }

    // ── Pre-count SD files so pendingSD reflects total before uploads start ──
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
      File root = SD.open("/");
      if (root && root.isDirectory()) {
        char skipFile[48] = "";
        strncpy(skipFile, currentFile.c_str(), sizeof(skipFile) - 1);
        int count = 0;
        while (true) {
          File entry = root.openNextFile();
          if (!entry) break;
          String name = String(entry.name());
          entry.close();
          if (name.endsWith(".wav") && ("/" + name) != String(skipFile)) count++;
        }
        root.close();
        portENTER_CRITICAL_SAFE(&_pend_mux); pendingSD = count; portEXIT_CRITICAL_SAFE(&_pend_mux);
      }
      xSemaphoreGive(sdMutex);
    }

    // ── Scan SD for leftover files (from previous sessions or failed uploads) ──
    while (true) {
      char nextFile[48] = "";

      if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
        // Re-init SD if needed — recovers from earlier init failures
        if (SD.cardSize() == 0) {
          Serial.println("[scan] SD not mounted — reinit...");
          SD.end();
          vTaskDelay(pdMS_TO_TICKS(100));
          if (!SD.begin(SD_CS_PIN)) {
            Serial.println("[scan] SD reinit FAILED");
            xSemaphoreGive(sdMutex);
            break;
          }
          Serial.println("[scan] SD reinit OK");
        }
        // Snapshot currentFile under sdMutex — Core 1 may reassign the String
        // at any segment boundary; comparing against a local copy is safe.
        char skipFile[48] = "";
        strncpy(skipFile, currentFile.c_str(), sizeof(skipFile) - 1);
        Serial.printf("[scan] SD size=%lluMB used=%lluMB currentFile=%s\n",
          SD.cardSize() / (1024*1024), SD.usedBytes() / (1024*1024), skipFile);
        File root = SD.open("/");
        if (!root || !root.isDirectory()) {
          Serial.println("[scan] root open failed");
          if (root) root.close();
          xSemaphoreGive(sdMutex);
          break;
        }
        int scanned = 0;
        while (true) {
          File entry = root.openNextFile();
          if (!entry) break;
          String name = String(entry.name());
          Serial.printf("[scan] found: '%s' wav=%d match=%d\n",
            name.c_str(), name.endsWith(".wav"), ("/" + name) == String(skipFile));
          scanned++;
          entry.close();
          if (name.endsWith(".wav") && ("/" + name) != String(skipFile)) {
            strncpy(nextFile, name.c_str(), sizeof(nextFile) - 1);
            break;
          }
        }
        if (scanned == 0) Serial.println("[scan] No files on SD");
        root.close();
        xSemaphoreGive(sdMutex);
      } else {
        Serial.println("[scan] sdMutex timeout");
      }

      if (nextFile[0] == '\0') break;  // no more files to upload
      if (!uploadsEnabled) break;      // uploads disabled mid-run — stop here

      char logmsg[LOG_COLS];
      snprintf(logmsg, sizeof(logmsg), "UL %s", nextFile);
      addLog(logmsg);

      bool ok = false;
      int attempts = 0;
      while (!(ok = uploadToMinIO(nextFile)) && attempts++ < 3) {
        Serial.printf("[upload] Retry %d/3 in 10s\n", attempts);
        vTaskDelay(pdMS_TO_TICKS(10000));
      }
      if (ok) { SD_DEC(); addLog("UL ok"); }
      else { addLog("UL FAILED"); }
    }

    WiFi.disconnect(false);
    WiFi.mode(WIFI_OFF);
    Serial.println("[wifi] Radio off — BLE active");
    addLog("WiFi off");
  }
}

// ── Filename helper ───────────────────────────────────────────────────────────
String makeFilename() {
  fileCounter++;
  if (ntpSynced) {
    time_t now = time(nullptr);
    struct tm* t = gmtime(&now);
    char buf[48];
    if (waiterCode[0] != '\0')
      snprintf(buf, sizeof(buf), "/rec_%04d%02d%02d_%02d%02d%02d_%s_%s.wav",
               t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
               t->tm_hour, t->tm_min, t->tm_sec, waiterCode, waiterName);
    else
      snprintf(buf, sizeof(buf), "/rec_%04d%02d%02d_%02d%02d%02d.wav",
               t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
               t->tm_hour, t->tm_min, t->tm_sec);
    return String(buf);
  }
  if (waiterCode[0] != '\0')
    return "/rec_fallback_" + String(fileCounter) + "_" + String(waiterCode) + "_" + String(waiterName) + ".wav";
  return "/rec_fallback_" + String(fileCounter) + ".wav";
}

// ── Segment finalize helper ───────────────────────────────────────────────────
// Patches the WAV header in the active PSRAM buffer, appends the BLE chunk,
// then queues the buffer for uploadTask. No SD writes on the happy path.
static void finalizeAndQueueSegment() {
  uint8_t* buf        = psramBuf[activeBuf];
  size_t   audioBytes = psramFill[activeBuf];

  // Patch WAV header at start of buffer (copy defaults then fix lengths)
  WAV_HEADER* hdr = (WAV_HEADER*)buf;
  *hdr = wavHeader;
  hdr->flength = (int32_t)(sizeof(WAV_HEADER) + audioBytes - 8);
  hdr->dlength = (int32_t)audioBytes;
  size_t totalSize = sizeof(WAV_HEADER) + audioBytes;

  // Append BLE chunk directly after audio data
  if (bleLogCount > 0) {
    uint32_t chunkBytes = (uint32_t)(bleLogCount * sizeof(BLESnapshot));
    uint8_t* p = buf + totalSize;
    memcpy(p, "ble_", 4);          p += 4;
    memcpy(p, &chunkBytes, 4);     p += 4;
    memcpy(p, bleLog, chunkBytes);
    totalSize += 8 + chunkBytes;
    hdr->flength = (int32_t)(totalSize - 8);  // update RIFF length to include BLE chunk
  }

  SegmentReady seg;
  seg.buf  = buf;
  seg.size = totalSize;
  strncpy(seg.filename, currentFile.c_str() + 1, sizeof(seg.filename) - 1);  // strip leading /
  seg.filename[sizeof(seg.filename) - 1] = '\0';

  if (xQueueSend(segmentQueue, &seg, pdMS_TO_TICKS(1000)) != pdTRUE)
    Serial.println("[rec] WARN: segmentQueue full — segment dropped!");
  xSemaphoreGive(uploadReady);
  PSRAM_INC();
  Serial.printf("[rec] Queued %s (%u bytes, %d BLE snaps)\n",
                seg.filename, (unsigned)totalSize, bleLogCount);
}

// ── Recording task ────────────────────────────────────────────────────────────
// Audio is written directly into PSRAM — no SD writes during recording.
// At each segment boundary finalizeAndQueueSegment() patches the WAV header
// and BLE chunk in-buffer and hands it to uploadTask via segmentQueue.
void recordingTask(void* arg) {
  while (true) {
    while (!isRecording) { vTaskDelay(pdMS_TO_TICKS(100)); }

    g_motorActive = true;  // suppress display for entire start sequence (initMic GPIO reconfig + motor)
    Serial.println("[rec] initMic starting...");
    if (!initMic()) {
      Serial.println("[rec] initMic FAILED — recording aborted");
      g_motorActive = false;
      isRecording = false;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    if (!psramBuf[0] || !psramBuf[1]) {
      Serial.println("[rec] PSRAM buffers unavailable — aborting");
      g_motorActive = false;
      isRecording = false;
      continue;
    }

    activeBuf         = 0;
    psramFill[0]      = psramFill[1] = 0;
    currentFile       = makeFilename();
    uint32_t segStart = millis();
    uint32_t lastSnap = millis();
    bleLogCount       = 0;
    stopRequested     = false;
    Serial.println("[rec] Started (PSRAM)");
    addLog("REC started");
    digitalWrite(MOTOR_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(80)); digitalWrite(MOTOR_PIN, LOW); vTaskDelay(pdMS_TO_TICKS(60));
    digitalWrite(MOTOR_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(80)); digitalWrite(MOTOR_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(50));
    g_motorActive = false;

    static int32_t rawBuf[1024];   // 32-bit DMA reads from ICS-43434
    static int16_t audioBuf[1024]; // converted 16-bit samples for WAV
    // Filter state
    float hpPrevInL = 0, hpPrevOutL = 0;   // HP stage 1
    float hpPrevInR = 0, hpPrevOutR = 0;
    float hp2PrevInL = 0, hp2PrevOutL = 0; // HP stage 2
    float hp2PrevInR = 0, hp2PrevOutR = 0;
    float hp3PrevInL = 0, hp3PrevOutL = 0; // HP stage 3 (cascade — 60dB/decade rolloff)
    float hp3PrevInR = 0, hp3PrevOutR = 0;
    float dcPrevInL = 0, dcPrevOutL = 0;
    float dcPrevInR = 0, dcPrevOutR = 0;
    const float HP_ALPHA = 0.9972f;  // ~80Hz high-pass at 16kHz; cascaded = 40dB/decade rolloff
    const float DC_ALPHA = 0.9992f;  // ~10Hz DC blocker — removes vibration DC drift

    while (!stopRequested) {
      if (!i2s_rx_handle) { Serial.println("[rec] i2s handle null — stopping"); break; }

      size_t bytesRead = 0;
      i2s_channel_read(i2s_rx_handle, rawBuf, sizeof(rawBuf), &bytesRead, pdMS_TO_TICKS(100));

      if (bytesRead > 0) {
        // Stereo DMA: L,R,L,R... each 32-bit — mic1 (SEL=GND)=L, mic2 (SEL=3V3)=R
        // Output interleaved 16-bit stereo: L0,R0,L1,R1,...
        int frames = bytesRead / 8;  // 8 bytes per stereo frame (L 32-bit + R 32-bit)
        for (int i = 0; i < frames; i++) {
          // Extract full 24 valid bits, normalised to 16-bit scale so gain values work as expected.
          // ICS-43434 is 24-bit MSB-aligned in 32-bit frame; >> 8 then / 256 = equivalent to >> 16
          // but float retains sub-LSB precision from the lower 8 bits.
          float inL = (float)(int32_t)(rawBuf[i * 2 + 1] >> 8) / 256.0f;  // mic1 SEL=GND → left
          float inR = (float)(int32_t)(rawBuf[i * 2]     >> 8) / 256.0f;  // mic2 SEL=3V3 → right

          // Step 2: DC blocker (~10Hz) — removes slow vibration/movement DC drift
          float dcL = inL - dcPrevInL + DC_ALPHA * dcPrevOutL;
          dcPrevInL = inL; dcPrevOutL = dcL; inL = dcL;
          float dcR = inR - dcPrevInR + DC_ALPHA * dcPrevOutR;
          dcPrevInR = inR; dcPrevOutR = dcR; inR = dcR;

          // Step 3: High-pass filter, 3-pole cascade (~80Hz, 60dB/decade) — steeper low-end rolloff
          float outL = HP_ALPHA * (hpPrevOutL + inL - hpPrevInL);
          float outR = HP_ALPHA * (hpPrevOutR + inR - hpPrevInR);
          hpPrevInL = inL; hpPrevOutL = outL;
          hpPrevInR = inR; hpPrevOutR = outR;
          // Second pole
          float s2L = HP_ALPHA * (hp2PrevOutL + outL - hp2PrevInL);
          float s2R = HP_ALPHA * (hp2PrevOutR + outR - hp2PrevInR);
          hp2PrevInL = outL; hp2PrevOutL = s2L; outL = s2L;
          hp2PrevInR = outR; hp2PrevOutR = s2R; outR = s2R;
          // Third pole
          float s3L = HP_ALPHA * (hp3PrevOutL + outL - hp3PrevInL);
          float s3R = HP_ALPHA * (hp3PrevOutR + outR - hp3PrevInR);
          hp3PrevInL = outL; hp3PrevOutL = s3L; outL = s3L;
          hp3PrevInR = outR; hp3PrevOutR = s3R; outR = s3R;

          // Step 4: Gain + soft limiter
          auto softClip = [](float x) -> int16_t {
            const float T = 24000.0f;
            if (x >  T) x =  T + (x - T) * 0.15f;
            if (x < -T) x = -T + (x + T) * 0.15f;
            return (int16_t)constrain((long)x, -32768, 32767);
          };
          audioBuf[i * 2]     = softClip(outL * MIC_GAIN_L);
          audioBuf[i * 2 + 1] = softClip(outR * MIC_GAIN_R);
        }
        bytesRead = frames * 4;  // 16-bit stereo output size (2 channels × 2 bytes)

        // Write after WAV header placeholder — reserve 2 KB at end for BLE chunk
        size_t offset    = sizeof(WAV_HEADER) + psramFill[activeBuf];
        size_t available = PSRAM_BUF_SIZE - offset - 2048;
        if (bytesRead <= available) {
          memcpy(psramBuf[activeBuf] + offset, audioBuf, bytesRead);
          psramFill[activeBuf] += bytesRead;
        } else {
          Serial.println("[rec] WARN: PSRAM buffer full — audio dropped");
        }
      }

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
        finalizeAndQueueSegment();
        activeBuf            ^= 1;          // flip to other buffer
        psramFill[activeBuf]  = 0;
        bleLogCount           = 0;
        currentFile           = makeFilename();
        segStart              = millis();
        lastSnap              = millis();
        addLog("REC new seg");
        Serial.println("[rec] New segment");
      }
    }

    finalizeAndQueueSegment();
    addLog("REC stopped");
    g_motorActive = true;  // suppress display across motor + I2S teardown
    digitalWrite(MOTOR_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(200)); digitalWrite(MOTOR_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(50));
    i2s_channel_disable(i2s_rx_handle);
    i2s_del_channel(i2s_rx_handle);
    i2s_rx_handle = NULL;
    g_motorActive = false;
    isRecording   = false;
    bleLogCount   = 0;
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
    if (xQueueSend(beaconQueue, &b, 0) != pdTRUE)
      Serial.println("[ble] queue full — beacon dropped");
  }
};

// ── Display helpers ───────────────────────────────────────────────────────────
// drawWifiIcon and drawBLEIcon removed — replaced by Adafruit SSD1306 driver

// ── Waiter NVS save ───────────────────────────────────────────────────────────
static void saveWaiter() {
  Preferences prefs;
  prefs.begin("cfg", false);
  prefs.putString("waiter_name", waiterName);
  prefs.putString("waiter_code", waiterCode);
  prefs.end();
  Serial.printf("[qr] Saved waiter: %s | %s\n", waiterName, waiterCode);
}

static void loadPlaceholderWaiter() {
  strncpy(waiterName, "John",  sizeof(waiterName) - 1);
  strncpy(waiterCode, "d03",   sizeof(waiterCode)  - 1);
  saveWaiter();
  addLog("Waiter:John|d03");
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
  oled.setCursor(0, 11); oled.print("Demo waiter set:");
  oled.setCursor(0, 19); oled.print("John");
  oled.setCursor(0, 27); oled.print("d03");
  oled.display();
  delay(1500);
}

// ── QR code scan (blocking, call from loop() only when not recording) ─────────
static void scanQRCode() {
  if (isRecording) { addLog("Stop rec first"); return; }

  // GPIO14 (PDM_PWR) shared with camera Y6 — must be LOW before camera init
  digitalWrite(PDM_PWR, LOW);
  delay(10);

  // ── Camera init — grayscale QVGA, proven for QR decode ───────────────────
  camera_config_t cfg = {};
  cfg.ledc_channel  = LEDC_CHANNEL_0;
  cfg.ledc_timer    = LEDC_TIMER_0;
  cfg.pin_d0        = CAM_PIN_Y2;  cfg.pin_d1 = CAM_PIN_Y3;
  cfg.pin_d2        = CAM_PIN_Y4;  cfg.pin_d3 = CAM_PIN_Y5;
  cfg.pin_d4        = CAM_PIN_Y6;  cfg.pin_d5 = CAM_PIN_Y7;
  cfg.pin_d6        = CAM_PIN_Y8;  cfg.pin_d7 = CAM_PIN_Y9;
  cfg.pin_xclk      = CAM_PIN_XCLK;
  cfg.pin_pclk      = CAM_PIN_PCLK;
  cfg.pin_vsync     = CAM_PIN_VSYNC;
  cfg.pin_href      = CAM_PIN_HREF;
  cfg.pin_sccb_sda  = CAM_PIN_SIOD;
  cfg.pin_sccb_scl  = CAM_PIN_SIOC;
  cfg.pin_pwdn      = CAM_PIN_PWDN;
  cfg.pin_reset     = CAM_PIN_RESET;
  cfg.xclk_freq_hz  = 20000000;
  cfg.pixel_format  = PIXFORMAT_GRAYSCALE;  // quirc needs raw pixels, not JPEG
  cfg.frame_size    = FRAMESIZE_QVGA;       // 320×240 — fast decode, enough resolution
  cfg.fb_count      = 2;                    // return one buffer while camera fills other
  cfg.fb_location   = CAMERA_FB_IN_PSRAM;
  cfg.grab_mode     = CAMERA_GRAB_WHEN_EMPTY;

  if (esp_camera_init(&cfg) != ESP_OK) {
    Serial.println("[qr] Camera init failed");
    addLog("Cam init failed");
    digitalWrite(PDM_PWR, HIGH);
    return;
  }

  // Camera GPIO reconfiguration corrupts I2C — recover before touching OLED
  Wire.end();
  Wire.begin(5, 6);
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.setRotation(2);

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_contrast(s, 2);
    s->set_sharpness(s, 2);
    s->set_saturation(s, -2);
    s->set_brightness(s, 0);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_ae_level(s, -1);                    // prevents phone screen blow-out
    s->set_gain_ctrl(s, 1);
    s->set_gainceiling(s, GAINCEILING_4X);     // 8X introduces bit errors
    s->set_lenc(s, 1);
    s->set_bpc(s, 1);
    s->set_wpc(s, 1);
    s->set_raw_gma(s, 1);
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);
    // set_sharpness() is broken on OV2640 — direct register fix
    s->set_reg(s, 0xff, 0xff, 0x00);
    s->set_reg(s, 0x92, 0xff, 0x01);
    s->set_reg(s, 0x93, 0xff, 0x30);
  }

  // Flush 20 frames so AEC converges before scanning
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
  oled.setCursor(0, 1); oled.print("QR:settling");
  oled.display();
  for (int i = 0; i < 20; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(80);
  }

  // Allocate quirc for QVGA
  struct quirc* q = quirc_new();
  if (!q || quirc_resize(q, 320, 240) < 0) {
    Serial.println("[qr] quirc init failed");
    addLog("quirc init failed");
    if (q) quirc_destroy(q);
    esp_camera_deinit();
    digitalWrite(PDM_PWR, HIGH);
    return;
  }

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
  oled.setCursor(0, 1); oled.print("Scan QR");
  if (waiterName[0]) {
    char wb[11]; snprintf(wb, sizeof(wb), "%.10s", waiterName);
    oled.setCursor(0, 9); oled.print(wb);
  }
  oled.setCursor(0, 17); oled.print("Timeout:30s");
  oled.display();

  bool found = false;
  unsigned long t0 = millis();
  unsigned long lastOled = 0;
  int frameCount = 0;

  while (millis() - t0 < 30000 && !found) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { delay(10); continue; }

    // Copy pixels into quirc buffer, release camera buffer IMMEDIATELY
    // so next frame can be captured during the ~75ms decode
    int qw, qh;
    uint8_t* qbuf = quirc_begin(q, &qw, &qh);
    memcpy(qbuf, fb->buf, (size_t)qw * qh);
    esp_camera_fb_return(fb);   // release BEFORE slow quirc_end
    frameCount++;

    quirc_end(q);   // detect finder patterns + extract grid (~75ms on QVGA)

    int n = quirc_count(q);
    for (int i = 0; i < n && !found; i++) {
      struct quirc_code code;
      struct quirc_data data;
      quirc_extract(q, i, &code);
      if (quirc_decode(&code, &data) == QUIRC_SUCCESS) {
        String payload = String((const char*)data.payload);
        Serial.printf("[qr] Found: %s\n", payload.c_str());
        int sep = payload.indexOf('|');
        if (sep > 0) {
          payload.substring(0, sep).toCharArray(waiterName, sizeof(waiterName));
          payload.substring(sep + 1).toCharArray(waiterCode, sizeof(waiterCode));
          saveWaiter();
          found = true;
          char msg[LOG_COLS]; snprintf(msg, sizeof(msg), "QR:%s|%s", waiterName, waiterCode);
          addLog(msg);
        }
      }
    }

    // Update OLED every second with countdown + frame count
    unsigned long now2 = millis();
    if (now2 - lastOled >= 1000) {
      lastOled = now2;
      int secsLeft = (int)((30000 - (now2 - t0)) / 1000);
      oled.clearDisplay();
      oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
      oled.setCursor(0, 1); oled.print("Scan QR");
      char fb2[11]; snprintf(fb2, sizeof(fb2), "f:%d", frameCount);
      oled.setCursor(0, 9); oled.print(fb2);
      char tb[11]; snprintf(tb, sizeof(tb), "t/o:%ds", secsLeft);
      oled.setCursor(0, 17); oled.print(tb);
      oled.display();
    }
  }

  quirc_destroy(q);
  esp_camera_deinit();

  g_motorActive = true;  // suppress display during motor kickback
  if (found) {
    digitalWrite(MOTOR_PIN, HIGH); delay(120);
    digitalWrite(MOTOR_PIN, LOW);  delay(80);
    digitalWrite(MOTOR_PIN, HIGH); delay(80);
    digitalWrite(MOTOR_PIN, LOW);
  } else {
    digitalWrite(MOTOR_PIN, HIGH); delay(100);
    digitalWrite(MOTOR_PIN, LOW);  delay(100);
    digitalWrite(MOTOR_PIN, HIGH); delay(100);
    digitalWrite(MOTOR_PIN, LOW);
  }
  delay(50);  // let motor inductive kickback dissipate
  g_motorActive = false;
  // Reset I2C — power glitch from motor can lock up SDA/SCL while loop() is blocked
  Wire.end();
  Wire.begin(5, 6);
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.setRotation(2);

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
  if (found) {
    oled.setCursor(0, 1); oled.print("Waiter set:");
    char wn[11]; strncpy(wn, waiterName, 10); wn[10] = '\0';
    oled.setCursor(0, 9); oled.print(wn);
    char wc[11]; strncpy(wc, waiterCode, 10); wc[10] = '\0';
    oled.setCursor(0, 17); oled.print(wc);
  } else {
    oled.setCursor(0, 1); oled.print("No QR found");
    oled.setCursor(0, 9); oled.print("Timed out");
  }
  oled.display();
  delay(2000);

  // Restore mic power
  digitalWrite(PDM_PWR, HIGH);
}

// ── Display ───────────────────────────────────────────────────────────────────
// 64×32 display: 10 chars wide, 4 lines at y=1/9/17/25, Adafruit SSD1306 5×7 font
// Screens 0-8, single-click cycles. Hold bar = bottom 2px (y=30, 64px wide).
void updateDisplay() {
  static bool     blink     = true;
  static uint32_t lastBlink = 0;
  uint32_t now = millis();
  if (now - lastBlink >= 500) { blink = !blink; lastBlink = now; }

  // Read CPU counters every display cycle
  portDISABLE_INTERRUPTS();
  uint32_t raw0 = s_idle0; s_idle0 = 0;
  uint32_t raw1 = s_idle1; s_idle1 = 0;
  portENABLE_INTERRUPTS();
  int cpu0 = max(0, min(99, (int)(100 - (raw0 * 100UL / s_base0))));
  int cpu1 = max(0, min(99, (int)(100 - (raw1 * 100UL / s_base1))));
  float cpuTemp = 0;
  if (s_temp_sensor) temperature_sensor_get_celsius(s_temp_sensor, &cpuTemp);

  // ── Screen timeout (60s) ─────────────────────────────────────────────────
  if (!g_screenOff && millis() - g_lastActivityMs > 60000) {
    oled.ssd1306_command(SSD1306_DISPLAYOFF);
    g_screenOff = true;
  }
  if (g_screenOff) return;
  if (g_motorActive) return;  // don't touch I2C while motor is drawing current

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);

  // ── Hold progress bar ─────────────────────────────────────────────────────
  if (g_btnState == 1 && (currentScreen == 0 || currentScreen == 1 || currentScreen == 2 || currentScreen == 7)) {
    uint32_t held     = millis() - g_btnStateTime;
    uint32_t holdFull = (currentScreen == 1) ? 6000 : 3000;
    int barW = (int)min((uint32_t)64, held * 64 / holdFull);
    oled.fillRect(0, 30, barW, 2, SSD1306_WHITE);
    if (currentScreen == 1) oled.drawFastVLine(32, 28, 4, SSD1306_WHITE);
  }

  char sbuf[12];

  if (currentScreen == 0) {
    // Main status screen
    if (isRecording) {
      oled.setCursor(0, 1); oled.print("Rec:ON 3s");
      if (blink) oled.fillCircle(60, 4, 2, SSD1306_WHITE);
    } else {
      oled.setCursor(0, 1); oled.print("Rec:OFF 3s");
    }
    if (waiterName[0]) {
      char w[11]; snprintf(w, sizeof(w), "W:%.8s", waiterName);
      oled.setCursor(0, 9); oled.print(w);
    } else {
      oled.setCursor(0, 9); oled.print("W: Unset");
    }
    if (isUploading && uploadKBps > 0)
      snprintf(sbuf, sizeof(sbuf), "%.0fKB/s", uploadKBps);
    else
      snprintf(sbuf, sizeof(sbuf), "UL: idle");
    oled.setCursor(0, 17); oled.print(sbuf);
    { int ps; portENTER_CRITICAL_SAFE(&_pend_mux); ps = pendingSD; portEXIT_CRITICAL_SAFE(&_pend_mux);
      snprintf(sbuf, sizeof(sbuf), "SD:%d pend", ps); }
    oled.setCursor(0, 25); oled.print(sbuf);

  } else if (currentScreen == 1) {
    char wc[9]; strncpy(wc, waiterCode[0] ? waiterCode : "no waiter", 8); wc[8] = '\0';
    oled.setCursor(0, 1); oled.print(wc);
    if (isRecording && blink) oled.fillCircle(60, 4, 2, SSD1306_WHITE);
    char wn[11]; strncpy(wn, waiterName[0] ? waiterName : "---", 10); wn[10] = '\0';
    oled.setCursor(0, 9); oled.print(wn);
    oled.setCursor(0, 17); oled.print("3s:scan QR");
    oled.setCursor(0, 25); oled.print("6s:demo");

  } else if (currentScreen == 2) {
    for (int i = 0; i < 2; i++) {
      const char* state;
      if      (uploadingBuf == i)             state = "UPL";
      else if (activeBuf == i && isRecording) state = "REC";
      else if (activeBuf == i)                state = "idl";
      else if (psramFill[i] > 0)             state = "rdy";
      else                                    state = "fre";
      size_t kb = psramFill[i] / 1024;
      if (kb >= 1000)
        snprintf(sbuf, sizeof(sbuf), "B%d:%-3s%2u.%uM", i, state, (unsigned)(kb/1024), (unsigned)((kb%1024)*10/1024));
      else
        snprintf(sbuf, sizeof(sbuf), "B%d:%-3s%3uK", i, state, (unsigned)kb);
      oled.setCursor(0, 1 + i * 8); oled.print(sbuf);
    }
    if (isUploading && uploadKBps > 0)
      snprintf(sbuf, sizeof(sbuf), "%.0fKB/s", uploadKBps);
    else
      snprintf(sbuf, sizeof(sbuf), "UL:idle");
    oled.setCursor(0, 17); oled.print(sbuf);
    { int ps; portENTER_CRITICAL_SAFE(&_pend_mux); ps = pendingSD; portEXIT_CRITICAL_SAFE(&_pend_mux);
      snprintf(sbuf, sizeof(sbuf), "SD pend:%d", ps); }
    oled.setCursor(0, 25); oled.print(sbuf);

  } else if (currentScreen == 3) {
    for (int i = 0; i < 3; i++) {
      if (xSemaphoreTake(beaconsMutex, 0) != pdTRUE) continue;
      BeaconState s = beacons[i];
      xSemaphoreGive(beaconsMutex);
      bool stale = !s.active || (now - s.lastSeenMs > STALE_MS);
      if (!s.active)  snprintf(sbuf, sizeof(sbuf), "%d --", i + 1);
      else if (stale) snprintf(sbuf, sizeof(sbuf), "%d stale", i + 1);
      else            snprintf(sbuf, sizeof(sbuf), "%d%3d %3.1fm", i + 1, s.rssi, s.distance);
      oled.setCursor(0, 1 + i * 8); oled.print(sbuf);
    }

  } else if (currentScreen == 4) {
    for (int i = 3; i < 5; i++) {
      if (xSemaphoreTake(beaconsMutex, 0) != pdTRUE) continue;
      BeaconState s = beacons[i];
      xSemaphoreGive(beaconsMutex);
      bool stale = !s.active || (now - s.lastSeenMs > STALE_MS);
      if (!s.active)  snprintf(sbuf, sizeof(sbuf), "%d --", i + 1);
      else if (stale) snprintf(sbuf, sizeof(sbuf), "%d stale", i + 1);
      else            snprintf(sbuf, sizeof(sbuf), "%d%3d %3.1fm", i + 1, s.rssi, s.distance);
      oled.setCursor(0, 1 + (i - 3) * 8); oled.print(sbuf);
    }

  } else if (currentScreen == 5) {
    snprintf(sbuf, sizeof(sbuf), "R:%s B:%s", isRecording ? "ON" : "off", bleScanning ? "ON" : "off");
    oled.setCursor(0, 1); oled.print(sbuf);
    snprintf(sbuf, sizeof(sbuf), "U:%s W:%s", isUploading ? "ON" : "off", WiFi.status() == WL_CONNECTED ? "ON" : "off");
    oled.setCursor(0, 9); oled.print(sbuf);
    { int pp, ps; portENTER_CRITICAL_SAFE(&_pend_mux); pp = pendingPSRAM; ps = pendingSD; portEXIT_CRITICAL_SAFE(&_pend_mux);
      snprintf(sbuf, sizeof(sbuf), "P:%dR %dSD", pp, ps); }
    oled.setCursor(0, 17); oled.print(sbuf);
    if (waiterName[0]) { char w[11]; snprintf(w, sizeof(w), "%.5s|%.4s", waiterName, waiterCode); oled.setCursor(0, 25); oled.print(w); }
    else { oled.setCursor(0, 25); oled.print("no waiter"); }

  } else if (currentScreen == 6) {
    snprintf(sbuf, sizeof(sbuf), "C0:%2d%%", cpu0);    oled.setCursor(0, 1);  oled.print(sbuf);
    snprintf(sbuf, sizeof(sbuf), "C1:%2d%%", cpu1);    oled.setCursor(0, 9);  oled.print(sbuf);
    snprintf(sbuf, sizeof(sbuf), "T:%.1fC", cpuTemp);  oled.setCursor(0, 17); oled.print(sbuf);
    if (isUploading && uploadKBps > 0) {
      snprintf(sbuf, sizeof(sbuf), "%.0fKB/s", uploadKBps);
      oled.setCursor(0, 25); oled.print(sbuf);
    }

  } else if (currentScreen == 7) {
    if (!st_done) {
      oled.setCursor(0,  1); oled.print("Speedtest");
      oled.setCursor(0,  9); oled.print("Hold 3s:run");
    } else {
      snprintf(sbuf, sizeof(sbuf), "%.0fKB/s", st_kbps);          oled.setCursor(0,  1); oled.print(sbuf);
      snprintf(sbuf, sizeof(sbuf), "%.2fMB/s", st_kbps/1024.0f);  oled.setCursor(0,  9); oled.print(sbuf);
      snprintf(sbuf, sizeof(sbuf), "%uK %.1fs", (unsigned)(st_bytes/1024), st_ms/1000.0f);
      oled.setCursor(0, 17); oled.print(sbuf);
      oled.setCursor(0, 25); oled.print("3s:rerun");
    }

  } else {
    if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      for (int i = 1; i <= 4; i++) {
        int slot = (logIdx + i) % LOG_LINES;
        if (logBuf[slot][0] != '\0') {
          char t[11]; strncpy(t, logBuf[slot], 10); t[10] = '\0';
          oled.setCursor(0, 1 + (i - 1) * 8); oled.print(t);
        }
      }
      xSemaphoreGive(logMutex);
    }
  }

  oled.display();
}

// ── Config loader — reads /config.txt from SD, falls back to NVS ─────────────
// Call in setup() after SD.begin() but before WiFi.begin().
// File format: one key=value per line, # for comments. Unknown keys ignored.
static const char* loadConfig() {
  Preferences prefs;
  bool fromSD = false;

  if (SD.exists("/config.txt")) {
    File f = SD.open("/config.txt", FILE_READ);
    if (f) {
      while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() == 0 || line[0] == '#') continue;
        int eq = line.indexOf('=');
        if (eq < 0) continue;
        String key = line.substring(0, eq);  key.trim();
        String val = line.substring(eq + 1); val.trim();
        if      (key == "wifi_ssid")    strncpy(WIFI_SSID,     val.c_str(), sizeof(WIFI_SSID)     - 1);
        else if (key == "wifi_pass")    strncpy(WIFI_PASSWORD, val.c_str(), sizeof(WIFI_PASSWORD)  - 1);
        else if (key == "minio_host")   strncpy(MINIO_HOST,    val.c_str(), sizeof(MINIO_HOST)    - 1);
        else if (key == "minio_port")   MINIO_PORT = val.toInt();
        else if (key == "minio_bucket") strncpy(MINIO_BUCKET,  val.c_str(), sizeof(MINIO_BUCKET)  - 1);
        else if (key == "minio_access") strncpy(MINIO_ACCESS,  val.c_str(), sizeof(MINIO_ACCESS)  - 1);
        else if (key == "minio_secret") strncpy(MINIO_SECRET,  val.c_str(), sizeof(MINIO_SECRET)  - 1);
        else if (key == "sample_rate")  SAMPLE_RATE = val.toInt();
      }
      f.close();
      fromSD = true;
      Serial.printf("[cfg] Loaded from SD — WiFi:%s  MinIO:%s:%d\n", WIFI_SSID, MINIO_HOST, MINIO_PORT);
    }
  }

  if (fromSD) {
    prefs.begin("cfg", false);
    prefs.putString("wifi_ssid",    WIFI_SSID);
    prefs.putString("wifi_pass",    WIFI_PASSWORD);
    prefs.putString("minio_host",   MINIO_HOST);
    prefs.putInt   ("minio_port",   MINIO_PORT);
    prefs.putString("minio_bucket", MINIO_BUCKET);
    prefs.putString("minio_access", MINIO_ACCESS);
    prefs.putString("minio_secret", MINIO_SECRET);
    prefs.putInt   ("sample_rate",  SAMPLE_RATE);
    prefs.end();
    Serial.println("[cfg] Saved to NVS");
    return "SD";
  } else {
    prefs.begin("cfg", true);
    if (prefs.isKey("wifi_ssid")) {
      prefs.getString("wifi_ssid",    WIFI_SSID,     sizeof(WIFI_SSID));
      prefs.getString("wifi_pass",    WIFI_PASSWORD, sizeof(WIFI_PASSWORD));
      prefs.getString("minio_host",   MINIO_HOST,    sizeof(MINIO_HOST));
      MINIO_PORT = prefs.getInt("minio_port", MINIO_PORT);
      prefs.getString("minio_bucket", MINIO_BUCKET,  sizeof(MINIO_BUCKET));
      prefs.getString("minio_access", MINIO_ACCESS,  sizeof(MINIO_ACCESS));
      prefs.getString("minio_secret", MINIO_SECRET,  sizeof(MINIO_SECRET));
      SAMPLE_RATE = prefs.getInt("sample_rate", SAMPLE_RATE);
      Serial.printf("[cfg] Loaded from NVS — WiFi:%s  MinIO:%s:%d  SR:%d\n", WIFI_SSID, MINIO_HOST, MINIO_PORT, SAMPLE_RATE);
      prefs.end();
      return "NVS";
    } else {
      Serial.println("[cfg] Using hardcoded defaults");
      prefs.end();
      return "default";
    }
  }

}

// quirc decode needs >8KB default loopTask stack
SET_LOOP_TASK_STACK_SIZE(32768);

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PIN, OUTPUT); digitalWrite(MOTOR_PIN, LOW);
  neopixelWrite(48, 0, 0, 0);  // turn off RGB LED (GPIO48)
  delay(500);
  esp_log_level_set("wifi", ESP_LOG_NONE);

  // Extend task watchdog timeout — uploads legitimately hold Core 0 for several seconds
  { esp_task_wdt_config_t wdt_cfg = {
      .timeout_ms     = 30000,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
      .trigger_panic  = true
    };
    esp_task_wdt_reconfigure(&wdt_cfg); }

  beaconsMutex = xSemaphoreCreateMutex();
  sdMutex      = xSemaphoreCreateMutex();
  logMutex     = xSemaphoreCreateMutex();
  uploadReady  = xSemaphoreCreateBinary();
  beaconQueue  = xQueueCreate(32, sizeof(BeaconData));
  segmentQueue = xQueueCreate(2, sizeof(SegmentReady));

  // Allocate PSRAM double-buffers — must be enabled: Tools → PSRAM → OPI PSRAM
  psramBuf[0] = (uint8_t*)ps_malloc(PSRAM_BUF_SIZE);
  psramBuf[1] = (uint8_t*)ps_malloc(PSRAM_BUF_SIZE);
  if (!psramBuf[0] || !psramBuf[1])
    Serial.println("[psram] FATAL: buffer alloc failed — PSRAM enabled?");
  else
    Serial.printf("[psram] Buffers OK: 2 × %lu KB\n", PSRAM_BUF_SIZE / 1024);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);
  gpio_set_drive_capability((gpio_num_t)MOTOR_PIN, GPIO_DRIVE_CAP_3);

  Wire.begin(5, 6);
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.setRotation(2);  // 180° flip for enclosure mounting
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 9);
  oled.print("Starting...");
  oled.display();

  // CPU temperature sensor
  { temperature_sensor_config_t tcfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    temperature_sensor_install(&tcfg, &s_temp_sensor);
    temperature_sensor_enable(s_temp_sensor); }

  // SD card — retry up to 5 times (card may need a moment after power-on)
  { bool sdOk = false;
    for (int i = 0; i < 5 && !sdOk; i++) {
      sdOk = SD.begin(SD_CS_PIN);
      if (!sdOk) { Serial.printf("[sd] Init attempt %d failed\n", i+1); vTaskDelay(pdMS_TO_TICKS(300)); }
    }
    if (!sdOk) Serial.println("[sd] SD init FAILED after retries");
    else        Serial.println("[sd] SD OK");
  }

  // Load config from SD card (if present) or NVS — must be before WiFi.begin()
  const char* cfgSrc = loadConfig();

  // Restore waiter identity from NVS (set via QR scan, not config.txt)
  { Preferences wp; wp.begin("cfg", true);
    wp.getString("waiter_name", waiterName, sizeof(waiterName));
    wp.getString("waiter_code", waiterCode, sizeof(waiterCode));
    wp.end(); }
  if (waiterName[0])
    Serial.printf("[cfg] Waiter: %s | %s\n", waiterName, waiterCode);

  if (strcmp(cfgSrc, "SD") == 0) {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
    oled.setCursor(0, 3); oled.print("Config: SD loaded");
    oled.setCursor(0, 15); oled.print(WIFI_SSID);
    oled.setCursor(0, 27); oled.print(MINIO_HOST);
    oled.display();
    delay(2000);
  }

  // WiFi — connect once for NTP, then radio off until upload needed
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int att = 0;
  while (WiFi.status() != WL_CONNECTED && att < 20) { delay(500); att++; }
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    Serial.printf("[wifi] Connected: %s\n", WiFi.localIP().toString().c_str());
    syncNTP();
    addLog("WiFi ok");
  } else {
    Serial.println("[wifi] Failed to connect");
    addLog("WiFi failed");
  }

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
  if (WiFi.status() == WL_CONNECTED) {
    oled.setCursor(0, 3); oled.print("WiFi OK");
    oled.setCursor(0, 13); oled.print(WiFi.localIP().toString().c_str());
  } else {
    oled.setCursor(0, 3); oled.print("WiFi failed");
  }
  oled.display();
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
  addLog("BLE scan on");

  // WiFi off now that BLE is running
  WiFi.disconnect(false);
  WiFi.mode(WIFI_OFF);
  Serial.println("[wifi] Radio off — BLE active");

  // CPU load hooks — must register before tasks start
  esp_register_freertos_idle_hook_for_cpu(idleHook0, 0);
  esp_register_freertos_idle_hook_for_cpu(idleHook1, 1);

  // FreeRTOS tasks — pinned to Core 1 so Core 0 stays free for WiFi/BLE radio
  xTaskCreatePinnedToCore(recordingTask, "rec", 12288, NULL, 2, NULL, 1);  // Core 1 — uncontested I2S/SD
  xTaskCreatePinnedToCore(uploadTask,    "upl", 20480, NULL, 1, NULL, 0);  // Core 0 — co-located with WiFi/lwIP

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

  // ── Button state machine ─────────────────────────────────────────────────
  // Single click   → next screen
  // Double click   → previous screen
  // 4 rapid clicks → toggle uploads on/off (brief confirmation shown)
  // Hold 3s        → QR scan (screen 1) / start/stop rec (screen 2) / speedtest (screen 7)
  // Hold 6s        → load placeholder waiter (screen 1 only, auto-fires while held)
  int btnRaw = digitalRead(BUTTON_PIN);
  if (btnRaw != lastBtnRaw) { btnTime = millis(); lastBtnRaw = btnRaw; }
  if (millis() - btnTime > 50 && btnRaw != btnStable) {
    btnStable = btnRaw;
    if (btnStable == LOW) {
      if (g_screenOff) {
        // Wake screen — return to main screen, consume the press
        oled.ssd1306_command(SSD1306_DISPLAYON);
        g_screenOff      = false;
        g_lastActivityMs = millis();
        currentScreen    = 0;
        g_btnState       = 0;
        g_clickCount     = 0;
      } else {
        // Button pressed — start or continue a click sequence
        g_lastActivityMs = millis();
        if (g_btnState == 0) {
          g_clickCount   = 1;
          g_btnState     = 1;
          g_btnStateTime = millis();
        } else if (g_btnState == 2) {
          g_clickCount++;
          g_btnState     = 1;
          g_btnStateTime = millis();
        }
      }
    } else {
      // Button released
      if (g_btnState == 1) {
        uint32_t held = millis() - g_btnStateTime;
        if (held >= 3000) {
          // Long press — execute screen action
          if (currentScreen == 7) {
            Serial.println("[btn] Run speedtest");
            addLog("Speedtest...");
            st_done = false;
            oled.clearDisplay();
            oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
            oled.setCursor(0,  1); oled.print("Speedtest");
            oled.setCursor(0,  9); oled.print("WiFi conn...");
            oled.display();
            WiFi.mode(WIFI_STA);
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            int att = 0;
            while (WiFi.status() != WL_CONNECTED && att < 20) { delay(500); att++; }
            if (WiFi.status() == WL_CONNECTED) {
              WiFi.setTxPower(WIFI_POWER_19_5dBm);
              WiFi.setSleep(false);
              esp_wifi_set_ps(WIFI_PS_NONE);
              oled.clearDisplay();
              oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
              oled.setCursor(0,  1); oled.print("Speedtest");
              oled.setCursor(0,  9); oled.print("Running...");
              oled.display();
              wifiSpeedTest();
              if (st_done) {
                char lb[22];
                snprintf(lb, sizeof(lb), "%.0f KB/s", st_kbps);
                addLog(lb);
              }
            } else {
              addLog("WiFi failed");
              oled.clearDisplay();
              oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
              oled.setCursor(0,  1); oled.print("Speedtest");
              oled.setCursor(0,  9); oled.print("WiFi failed");
              oled.display();
              delay(2000);
            }
            WiFi.disconnect(false);
            WiFi.mode(WIFI_OFF);
            updateDisplay();
          } else if (currentScreen == 0 || currentScreen == 2) {
            if (!isRecording) {
              if (!waiterName[0]) {
                oled.clearDisplay();
                oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
                oled.setCursor(0,  1); oled.print("No waiter!");
                oled.setCursor(0,  9); oled.print("Scan QR");
                oled.setCursor(0, 17); oled.print("first.");
                oled.display();
                delay(2000);
                currentScreen = 1;
              } else {
                Serial.println("[btn] Start recording"); isRecording = true; stopRequested = false;
                updateDisplay();
              }
            } else {
              Serial.println("[btn] Stop recording"); stopRequested = true;
            }
          } else if (currentScreen == 1) {
            Serial.println("[btn] Scan QR");
            scanQRCode();
          }
          g_btnState   = 0;
          g_clickCount = 0;
        } else {
          // Short press — wait to see if more clicks follow
          g_btnState     = 2;
          g_btnStateTime = millis();
        }
      }
    }
  }
  // Click-count timeout — fire action after 500ms with no new press
  if (g_btnState == 2 && millis() - g_btnStateTime >= 500) {
    if (g_clickCount == 1) {
      currentScreen = (currentScreen + 1) % 9;
    } else if (g_clickCount == 2) {
      currentScreen = (currentScreen + 8) % 9;  // go back one
    } else if (g_clickCount == 4) {
      uploadsEnabled = !uploadsEnabled;
      if (uploadsEnabled) xSemaphoreGive(uploadReady);
      addLog(uploadsEnabled ? "UL enabled" : "UL disabled");
      Serial.printf("[btn] Uploads %s\n", uploadsEnabled ? "enabled" : "disabled");
      oled.clearDisplay();
      oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
      oled.setCursor(0,  9); oled.print(uploadsEnabled ? "UL enabled" : "UL off");
      oled.display();
      delay(1000);
    }
    g_btnState   = 0;
    g_clickCount = 0;
  }
  // 6s hold on screen 1 auto-fires placeholder waiter
  if (g_btnState == 1 && currentScreen == 1 && millis() - g_btnStateTime >= 6000) {
    loadPlaceholderWaiter();
    g_btnState   = 0;
    g_clickCount = 0;
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
