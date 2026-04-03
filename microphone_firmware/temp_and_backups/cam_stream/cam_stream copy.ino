/*
 * Camera MJPEG Stream + QR Scanner
 * XIAO ESP32-S3 Sense
 *
 * /       → MJPEG stream (view in browser to check focus)
 * /scan   → scan for a QR code (10s), return payload as HTML
 *
 * Board:  XIAO_ESP32S3
 * PSRAM:  Tools → PSRAM → OPI PSRAM
 */

#include <WiFi.h>
#include "esp_camera.h"
#include <ESP32QRCodeReader.h>

const char* WIFI_SSID     = "Home DSL";
const char* WIFI_PASSWORD = "123454321";

// ── XIAO ESP32-S3 Sense camera pins ──────────────────────────────────────────
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

#define PDM_PWR        14  // shared with Y6 — power it LOW before camera init

WiFiServer server(80);

static bool initCamera() {
  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0       = CAM_PIN_Y2;
  cfg.pin_d1       = CAM_PIN_Y3;
  cfg.pin_d2       = CAM_PIN_Y4;
  cfg.pin_d3       = CAM_PIN_Y5;
  cfg.pin_d4       = CAM_PIN_Y6;
  cfg.pin_d5       = CAM_PIN_Y7;
  cfg.pin_d6       = CAM_PIN_Y8;
  cfg.pin_d7       = CAM_PIN_Y9;
  cfg.pin_xclk     = CAM_PIN_XCLK;
  cfg.pin_pclk     = CAM_PIN_PCLK;
  cfg.pin_vsync    = CAM_PIN_VSYNC;
  cfg.pin_href     = CAM_PIN_HREF;
  cfg.pin_sccb_sda = CAM_PIN_SIOD;
  cfg.pin_sccb_scl = CAM_PIN_SIOC;
  cfg.pin_pwdn     = CAM_PIN_PWDN;
  cfg.pin_reset    = CAM_PIN_RESET;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size   = FRAMESIZE_SVGA;  // 800×600 — more pixels, same focus
  cfg.jpeg_quality = 4;               // 0=best, 63=worst — low compression
  cfg.fb_count     = 2;
  cfg.fb_location  = CAMERA_FB_IN_PSRAM;
  cfg.grab_mode    = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&cfg) != ESP_OK) {
    Serial.println("[cam] init failed");
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_contrast(s, 2);
    s->set_sharpness(s, 2);
    s->set_brightness(s, 0);          // neutral — let AEC handle exposure
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_ae_level(s, 0);            // neutral AE bias
    s->set_gain_ctrl(s, 1);
    s->set_gainceiling(s, GAINCEILING_4X);  // cap noise
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_lenc(s, 1);       // lens correction — compensates for vignetting
    s->set_bpc(s, 1);        // bad pixel correction
    s->set_wpc(s, 1);        // white pixel correction
    s->set_raw_gma(s, 1);    // gamma correction — better tonal range
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);
  }
  Serial.println("[cam] JPEG stream ready");
  return true;
}

// ── MJPEG stream ──────────────────────────────────────────────────────────────
static void streamToClient(WiFiClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println();

  Serial.println("[stream] client connected — streaming");
  unsigned long frames = 0;
  unsigned long t0 = millis();

  while (client.connected()) {
    // Yield to any waiting connection (e.g. /scan) so loop() can handle it
    if (server.hasClient()) { Serial.println("[stream] yielding to new connection"); break; }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { Serial.println("[stream] fb_get failed"); delay(10); continue; }

    client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.print("\r\n");
    esp_camera_fb_return(fb);

    frames++;
    if (frames % 30 == 0) {
      float fps = frames * 1000.0f / (millis() - t0);
      Serial.printf("[stream] %lu frames  %.1f fps\n", frames, fps);
    }
  }
  Serial.printf("[stream] client disconnected after %lu frames\n", frames);
}

// ── Send one HTTP chunked chunk ───────────────────────────────────────────────
static void sendChunk(WiFiClient& client, const String& s) {
  client.printf("%x\r\n", s.length());
  client.print(s);
  client.print("\r\n");
}

// ── QR scan endpoint ─────────────────────────────────────────────────────────
static void handleScan(WiFiClient& client) {
  Serial.println("[scan] Starting QR scan...");

  // Send headers + "Scanning..." page immediately so browser doesn't wait blank
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Transfer-Encoding: chunked");
  client.println("Connection: close");
  client.println();
  sendChunk(client,
    "<!DOCTYPE html><html><body style='font-family:monospace;padding:20px'>"
    "<h2>Scanning for QR code...</h2>"
    "<p>Warming up camera (2s), then scanning for up to 20s.</p>"
    "<p>Keep QR code steady in front of camera.</p>"
  );

  // Deinit JPEG camera — ESP32QRCodeReader will reinit in grayscale
  esp_camera_deinit();

  CameraPins pins = {
    .PWDN_GPIO_NUM  = CAM_PIN_PWDN,
    .RESET_GPIO_NUM = CAM_PIN_RESET,
    .XCLK_GPIO_NUM  = CAM_PIN_XCLK,
    .SIOD_GPIO_NUM  = CAM_PIN_SIOD,
    .SIOC_GPIO_NUM  = CAM_PIN_SIOC,
    .Y9_GPIO_NUM    = CAM_PIN_Y9,
    .Y8_GPIO_NUM    = CAM_PIN_Y8,
    .Y7_GPIO_NUM    = CAM_PIN_Y7,
    .Y6_GPIO_NUM    = CAM_PIN_Y6,
    .Y5_GPIO_NUM    = CAM_PIN_Y5,
    .Y4_GPIO_NUM    = CAM_PIN_Y4,
    .Y3_GPIO_NUM    = CAM_PIN_Y3,
    .Y2_GPIO_NUM    = CAM_PIN_Y2,
    .VSYNC_GPIO_NUM = CAM_PIN_VSYNC,
    .HREF_GPIO_NUM  = CAM_PIN_HREF,
    .PCLK_GPIO_NUM  = CAM_PIN_PCLK,
  };

  ESP32QRCodeReader reader(pins, FRAMESIZE_VGA);  // 640×480 — XGA crashes (OOM)
  reader.setDebug(true);

  if (reader.setup() != SETUP_OK) {
    Serial.println("[scan] Camera init failed");
    sendChunk(client, "<h2 style='color:red'>Camera init failed</h2><p>Reboot device.</p>");
    client.print("0\r\n\r\n");
    initCamera();
    return;
  }

  // Sensor tuning for QR detection
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_contrast(s, 2);            // max — most impactful for module distinction
    s->set_sharpness(s, 2);           // high-level call (may be no-op on OV2640)
    s->set_saturation(s, -2);         // irrelevant in grayscale, zero cost
    s->set_brightness(s, 0);          // neutral — don't clip highlights
    s->set_exposure_ctrl(s, 1);       // AEC on
    s->set_aec2(s, 1);                // flicker filter — helps with phone screens
    s->set_ae_level(s, -1);           // slightly underexpose — prevents phone screen blow-out
    s->set_gain_ctrl(s, 1);           // AGC on
    s->set_gainceiling(s, GAINCEILING_4X);  // cap noise — 8X causes bit errors
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_lenc(s, 1);                // lens vignette correction
    s->set_bpc(s, 1);                 // bad pixel correction
    s->set_wpc(s, 1);                 // hot pixel correction
    s->set_raw_gma(s, 1);             // gamma correction
    // Direct register sharpness fix — set_sharpness() is broken on OV2640 (espressif/esp32-camera #196)
    s->set_reg(s, 0xff, 0xff, 0x00);  // switch to DSP bank
    s->set_reg(s, 0x92, 0xff, 0x01);  // sharpness: auto mode
    s->set_reg(s, 0x93, 0xff, 0x30);  // sharpness strength (0x00–0xFF)
  }

  // Flush frames until AEC converges — more reliable than blind delay
  Serial.println("[scan] Flushing frames for AEC settle...");
  for (int i = 0; i < 20; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(80);  // ~12.5 fps, 20 frames ≈ 1.6s
  }
  Serial.println("[scan] AEC settled, starting decode...");

  reader.beginOnCore(1);

  struct QRCodeData qrData;
  String payload = "";
  unsigned long t0 = millis();
  int attempts = 0;

  Serial.println("[scan] Scanning (20s)...");
  while (millis() - t0 < 20000) {
    if (reader.receiveQrCode(&qrData, 500)) {
      attempts++;
      if (qrData.valid) {
        payload = String((const char*)qrData.payload);
        Serial.printf("[scan] Found: %s\n", payload.c_str());
        break;
      }
    }
  }

  reader.end();
  delay(300);           // let reader task fully stop before deinit
  esp_camera_deinit();
  delay(300);           // let driver settle before JPEG reinit

  // Send result chunk, then end chunked response — before reiniting camera
  if (payload.length() > 0) {
    sendChunk(client,
      "<hr><h2 style='color:green'>QR Found</h2>"
      "<p style='font-size:1.4em'><b>" + payload + "</b></p>"
      "<p><a href='/'>Back to stream</a> &nbsp; <a href='/scan'>Scan again</a></p>"
      "</body></html>"
    );
  } else {
    char buf[128];
    snprintf(buf, sizeof(buf),
      "<hr><h2 style='color:red'>No QR code found</h2>"
      "<p>20s timeout &mdash; %d decode attempts</p>"
      "<p><a href='/'>Back to stream</a> &nbsp; <a href='/scan'>Try again</a></p>"
      "</body></html>", attempts);
    sendChunk(client, String(buf));
  }
  client.print("0\r\n\r\n");  // end chunked transfer

  // Reinit JPEG stream camera after closing response
  delay(100);
  initCamera();
}

// ── Setup / loop ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  neopixelWrite(48, 0, 0, 0);

  // GPIO14 shared with camera Y6 — pull LOW so camera owns it
  pinMode(PDM_PWR, OUTPUT);
  digitalWrite(PDM_PWR, LOW);
  delay(10);

  Serial.println("Camera stream — connecting WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nConnected:\n  Stream: http://%s/\n  QR scan: http://%s/scan\n",
                WiFi.localIP().toString().c_str(), WiFi.localIP().toString().c_str());

  if (!initCamera()) {
    Serial.println("Camera init FAILED — halting");
    while (true) delay(1000);
  }

  delay(1500);  // AE settle

  server.begin();
  Serial.println("HTTP server started.");
}

void loop() {
  WiFiClient client = server.accept();
  if (!client) return;

  // Wait for TCP data to arrive, then read headers
  unsigned long t = millis();
  while (client.connected() && !client.available() && millis() - t < 1000) delay(1);

  String requestLine = "";
  while (client.connected() && millis() - t < 2000) {
    if (client.available()) {
      String line = client.readStringUntil('\n');
      if (requestLine.isEmpty()) requestLine = line;
      if (line == "\r") break;
    }
  }

  requestLine.trim();
  Serial.printf("[http] '%s'\n", requestLine.c_str());

  if (requestLine.startsWith("GET /scan")) {
    handleScan(client);
  } else if (requestLine.startsWith("GET /")) {
    streamToClient(client);
  } else {
    // Ignore favicon, empty, or other requests
    client.println("HTTP/1.1 404 Not Found\r\nConnection: close\r\n");
  }
  client.stop();
}
