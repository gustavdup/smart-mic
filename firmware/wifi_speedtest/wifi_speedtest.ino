/*
 * WiFi Speed Test
 * XIAO ESP32-S3 Sense + Seeed Expansion Board v1
 *
 * Uploads a dummy file to MinIO with valid AWS4 auth so MinIO
 * accepts and reads the full body — measures true LAN upload speed.
 *
 * Libraries: U8g2, WiFi (built-in)
 * Board: XIAO_ESP32S3
 * PSRAM: Tools → PSRAM → OPI PSRAM (required)
 */

#include <WiFi.h>
#include <esp_wifi.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "mbedtls/md.h"
#include "time.h"

// ── WiFi ──────────────────────────────────────────────────────────────────────
const char* WIFI_SSID     = "Home DSL";
const char* WIFI_PASSWORD = "123454321";

// ── MinIO ─────────────────────────────────────────────────────────────────────
const char* MINIO_HOST   = "192.168.1.72";
const int   MINIO_PORT   = 9000;
const char* MINIO_BUCKET = "audio";
const char* MINIO_ACCESS = "minioadmin";
const char* MINIO_SECRET = "minioadmin";

// ── Upload test ───────────────────────────────────────────────────────────────
#define UL_BYTES  (4UL * 1024 * 1024)   // 4 MB per run
#define UL_CHUNK  (4 * 1024)            // 4 KB internal-RAM chunks (DMA-safe)


U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ── Display ───────────────────────────────────────────────────────────────────
static void oledLines(const char* l1, const char* l2 = "",
                      const char* l3 = "", const char* l4 = "") {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tf);
  if (*l1) u8g2.drawStr(0,  9, l1);
  if (*l2) u8g2.drawStr(0, 21, l2);
  if (*l3) u8g2.drawStr(0, 33, l3);
  if (*l4) u8g2.drawStr(0, 45, l4);
  u8g2.sendBuffer();
}

// ── TCP ping ──────────────────────────────────────────────────────────────────
static int tcpPing(const char* host, uint16_t port) {
  WiFiClient c;
  unsigned long t = millis();
  bool ok = c.connect(host, port, 4000);
  int rtt = ok ? (int)(millis() - t) : -1;
  c.stop();
  Serial.printf("[ping] %s:%d -> %s (%d ms)\n", host, port, ok?"OK":"FAIL", rtt);
  return rtt;
}

// ── AWS4 signing ──────────────────────────────────────────────────────────────
static void toHex(const unsigned char* in, size_t len, char* out) {
  for (size_t i = 0; i < len; i++) sprintf(out + i*2, "%02x", in[i]);
  out[len*2] = '\0';
}
static void hmac256(const unsigned char* key, size_t klen,
                    const unsigned char* msg, size_t mlen,
                    unsigned char* out) {
  mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                  key, klen, msg, mlen, out);
}
static void sha256hex(const char* msg, char* hexOut) {
  unsigned char hash[32];
  mbedtls_md(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
             (const unsigned char*)msg, strlen(msg), hash);
  toHex(hash, 32, hexOut);
}

// ── MinIO upload with auth ────────────────────────────────────────────────────
static float measureUpload() {
  oledLines("Upload: signing...", MINIO_HOST);

  // Timestamp
  time_t now = time(nullptr);
  struct tm* t = gmtime(&now);
  char date[9], tstr[7], dt[17];
  strftime(date, sizeof(date), "%Y%m%d", t);
  strftime(tstr, sizeof(tstr), "%H%M%S", t);
  snprintf(dt, sizeof(dt), "%sT%sZ", date, tstr);

  const char* fileHash = "UNSIGNED-PAYLOAD";
  const char* objName  = "_speedtest.bin";

  // Canonical request
  char canonReq[640];
  snprintf(canonReq, sizeof(canonReq),
    "PUT\n/%s/%s\n\nhost:%s:%d\nx-amz-content-sha256:%s\nx-amz-date:%s\n\n"
    "host;x-amz-content-sha256;x-amz-date\n%s",
    MINIO_BUCKET, objName, MINIO_HOST, MINIO_PORT, fileHash, dt, fileHash);

  char canonHash[65];
  sha256hex(canonReq, canonHash);

  char sts[512];
  snprintf(sts, sizeof(sts),
    "AWS4-HMAC-SHA256\n%s\n%s/%s/s3/aws4_request\n%s",
    dt, date, "us-east-1", canonHash);

  char keySecret[80] = "AWS4";
  strncat(keySecret, MINIO_SECRET, 75);
  unsigned char kDate[32], kRegion[32], kService[32], kSign[32], sig[32];
  hmac256((unsigned char*)keySecret, strlen(keySecret), (unsigned char*)date, strlen(date), kDate);
  hmac256(kDate,    32, (unsigned char*)"us-east-1",    9,  kRegion);
  hmac256(kRegion,  32, (unsigned char*)"s3",           2,  kService);
  hmac256(kService, 32, (unsigned char*)"aws4_request", 12, kSign);
  hmac256(kSign,    32, (unsigned char*)sts, strlen(sts), sig);
  char sigHex[65];
  toHex(sig, 32, sigHex);

  char auth[512];
  snprintf(auth, sizeof(auth),
    "AWS4-HMAC-SHA256 Credential=%s/%s/%s/s3/aws4_request,"
    " SignedHeaders=host;x-amz-content-sha256;x-amz-date, Signature=%s",
    MINIO_ACCESS, date, "us-east-1", sigHex);

  // Connect
  oledLines("Upload: connecting...", MINIO_HOST);
  WiFiClient tcp;
  tcp.setNoDelay(true);
  if (!tcp.connect(MINIO_HOST, MINIO_PORT, 5000)) {
    oledLines("UL FAILED", "TCP connect failed", MINIO_HOST);
    Serial.println("[ul] connect failed");
    return -1.0f;
  }

  // HTTP headers
  tcp.printf("PUT /%s/%s HTTP/1.0\r\n", MINIO_BUCKET, objName);
  tcp.printf("Host: %s:%d\r\n", MINIO_HOST, MINIO_PORT);
  tcp.printf("Content-Length: %lu\r\n", (unsigned long)UL_BYTES);
  tcp.print("Content-Type: application/octet-stream\r\n");
  tcp.printf("x-amz-date: %s\r\n", dt);
  tcp.printf("x-amz-content-sha256: %s\r\n", fileHash);
  tcp.printf("Authorization: %s\r\n", auth);
  tcp.print("Connection: close\r\n\r\n");

  // Internal-RAM buffer (DMA-safe — PSRAM can't be read by WiFi DMA)
  uint8_t* buf = (uint8_t*)heap_caps_malloc(UL_CHUNK, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!buf) { tcp.stop(); oledLines("UL FAILED", "malloc"); return -1.0f; }
  memset(buf, 0xAB, UL_CHUNK);

  size_t        sent         = 0;
  unsigned long start        = millis();
  unsigned long lastDisp     = 0;
  unsigned long lastProgress = millis();

  while (sent < UL_BYTES) {
    if (!tcp.connected()) { Serial.println("[ul] disconnected"); break; }
    if (millis() - lastProgress > 8000) { Serial.println("[ul] stalled"); break; }

    size_t n = min((size_t)UL_CHUNK, (size_t)(UL_BYTES - sent));
    int written = tcp.write(buf, n);
    if (written > 0) { sent += written; lastProgress = millis(); }

    unsigned long now = millis();
    if (now - lastDisp >= 500 && sent > 0) {
      lastDisp = now;
      unsigned long el = now - start;
      float mbps = (sent * 8.0f) / (el / 1000.0f) / 1e6f;
      char s1[24], s2[24];
      snprintf(s1, sizeof(s1), "UL: %.2f Mbps", mbps);
      snprintf(s2, sizeof(s2), "%lu/%lu KB", (unsigned long)(sent/1024),
               (unsigned long)(UL_BYTES/1024));
      oledLines("Uploading (LAN)...", s1, s2, MINIO_HOST);
      Serial.printf("[ul] %.2f Mbps — %lu KB\n", mbps, (unsigned long)(sent/1024));
    }
  }

  unsigned long elapsed = millis() - start;
  tcp.stop();
  free(buf);

  if (!elapsed || !sent) return -1.0f;
  float mbps = (sent * 8.0f) / (elapsed / 1000.0f) / 1e6f;
  Serial.printf("[ul] FINAL: %.2f Mbps  %.0f KB/s  %lu KB in %lu ms\n",
                mbps, mbps*125.0f, (unsigned long)(sent/1024), elapsed);
  return mbps;
}

// ── Test run ──────────────────────────────────────────────────────────────────
static void runTest(int runNum) {
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  int rssi = WiFi.RSSI();
  Serial.printf("[wifi] RSSI:%d  IP:%s  GW:%s\n", rssi,
                WiFi.localIP().toString().c_str(),
                WiFi.gatewayIP().toString().c_str());

  int lanPing = tcpPing(MINIO_HOST, MINIO_PORT);

  char l1[24], l2[24];
  snprintf(l1, sizeof(l1), "LAN ping: %d ms", lanPing);
  snprintf(l2, sizeof(l2), "RSSI: %d dBm", rssi);
  oledLines(l1, l2, "", "Starting upload...");
  delay(1000);

  float ul = measureUpload();

  char r1[24], r2[24], r3[24];
  if (ul > 0) snprintf(r1, sizeof(r1), "UL: %.2f Mbps", ul);
  else        strncpy(r1, "UL: FAILED", sizeof(r1));
  snprintf(r2, sizeof(r2), "LAN ping: %d ms", lanPing);
  snprintf(r3, sizeof(r3), "RSSI:%d  #%d", rssi, runNum);

  oledLines(r1, r2, r3, MINIO_HOST);
  Serial.printf("[result] %s  %s\n", r1, r2);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  neopixelWrite(48, 0, 0, 0);
  delay(200);

  u8g2.begin();
  oledLines("WiFi Speed Test", "Connecting...");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int att = 0;
  while (WiFi.status() != WL_CONNECTED && att < 40) { delay(500); att++; }
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  if (WiFi.status() != WL_CONNECTED) {
    oledLines("WiFi FAILED");
    while (true) delay(1000);
  }

  // NTP needed for AWS4 signing
  configTime(0, 0, "pool.ntp.org");
  time_t now = time(nullptr);
  int retries = 0;
  while (now < 1672531200 && retries++ < 20) { delay(500); now = time(nullptr); }
  Serial.printf("[ntp] %s\n", now > 1672531200 ? "synced" : "failed");

  char ipStr[24];
  WiFi.localIP().toString().toCharArray(ipStr, sizeof(ipStr));
  oledLines("WiFi OK", ipStr,
            now > 1672531200 ? "NTP: synced" : "NTP: FAILED",
            ESP.getPsramSize() ? "PSRAM: OK" : "PSRAM: MISSING!");
  delay(1500);
}

static int testRun = 0;

void loop() {
  testRun++;
  runTest(testRun);
  delay(30000);
}
