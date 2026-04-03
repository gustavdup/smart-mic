/*
 * Camera Live QR Scanner
 * XIAO ESP32-S3 Sense
 *
 * Port 80 /        → HTML page with live stream + auto QR result
 * Port 80 /result  → plain text last decoded QR payload
 * Port 80 /reset   → clear last result
 * Port 81 /        → MJPEG stream (grayscale)
 *
 * Board:  XIAO_ESP32S3
 * PSRAM:  Tools → PSRAM → OPI PSRAM
 */

#include <WiFi.h>
#include "esp_camera.h"
#include "img_converters.h"
#include <ESP32QRCodeReader.h>   // triggers compilation of quirc .c files
#include "quirc/quirc.h"

// loopTask default stack is 8KB — quirc decode blows it on success
SET_LOOP_TASK_STACK_SIZE(32768);

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

#define PDM_PWR        14  // shared with Y6 — pull LOW before camera init

WiFiServer apiServer(80);     // HTML + /result + /reset
WiFiServer streamServer(81);  // MJPEG stream — dedicated task, never blocks API

// ── QR state ─────────────────────────────────────────────────────────────────
static struct quirc*     g_quirc        = nullptr;
static char              lastPayload[1024] = "";
static unsigned long     lastPayloadTime  = 0;
static SemaphoreHandle_t g_payloadMux;

// ── Continuous QR decode task (Core 1) ───────────────────────────────────────
static void scanTask(void*) {
  g_quirc = quirc_new();
  if (!g_quirc) {
    Serial.println("[scan] quirc_new failed");
    vTaskDelete(NULL); return;
  }
  if (quirc_resize(g_quirc, 320, 240) < 0) {
    Serial.println("[scan] quirc_resize failed");
    vTaskDelete(NULL); return;
  }
  Serial.println("[scan] task running");

  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { vTaskDelay(10 / portTICK_PERIOD_MS); continue; }

    // Copy pixels into quirc buffer, then free camera buffer before decode
    int qw, qh;
    uint8_t* qbuf = quirc_begin(g_quirc, &qw, &qh);
    memcpy(qbuf, fb->buf, (size_t)qw * qh);
    esp_camera_fb_return(fb);   // release BEFORE slow quirc_end

    quirc_end(g_quirc);         // ~200ms on QVGA

    int n = quirc_count(g_quirc);
    for (int i = 0; i < n; i++) {
      struct quirc_code code;
      struct quirc_data data;
      quirc_extract(g_quirc, i, &code);
      if (quirc_decode(&code, &data) == QUIRC_SUCCESS) {
        xSemaphoreTake(g_payloadMux, portMAX_DELAY);
        strncpy(lastPayload, (char*)data.payload, sizeof(lastPayload) - 1);
        lastPayload[sizeof(lastPayload) - 1] = '\0';
        lastPayloadTime = millis();
        xSemaphoreGive(g_payloadMux);
        Serial.printf("[qr] Found: %s\n", lastPayload);
      }
    }
  }
}

// ── MJPEG stream (runs in streamTask, Core 0) ─────────────────────────────────
static void streamToClient(WiFiClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println();

  Serial.println("[stream] client connected");
  unsigned long frames = 0;
  unsigned long t0 = millis();

  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { vTaskDelay(10 / portTICK_PERIOD_MS); continue; }

    uint8_t* jpg = nullptr;
    size_t   jpg_len = 0;
    if (frame2jpg(fb, 80, &jpg, &jpg_len)) {
      client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", jpg_len);
      client.write(jpg, jpg_len);
      client.print("\r\n");
      free(jpg);
    }
    esp_camera_fb_return(fb);

    frames++;
    if (frames % 30 == 0) {
      float fps = frames * 1000.0f / (millis() - t0);
      Serial.printf("[stream] %lu frames  %.1f fps\n", frames, fps);
    }
  }
  Serial.printf("[stream] disconnected after %lu frames\n", frames);
}

// ── Stream server task (Core 0) ───────────────────────────────────────────────
static void streamTask(void*) {
  streamServer.begin();
  Serial.println("[stream] server started on port 81");

  while (true) {
    WiFiClient client = streamServer.accept();
    if (!client) { vTaskDelay(10 / portTICK_PERIOD_MS); continue; }

    // Drain headers
    unsigned long t = millis();
    while (client.connected() && !client.available() && millis() - t < 1000) vTaskDelay(1);
    while (client.connected()) {
      if (!client.available()) continue;
      String line = client.readStringUntil('\n');
      if (line == "\r") break;
    }
    streamToClient(client);
    client.stop();
  }
}

// ── HTML home page ────────────────────────────────────────────────────────────
static void handleRoot(WiFiClient& client) {
  String ip = WiFi.localIP().toString();

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();
  client.printf(R"HTML(<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>QR Scanner</title>
  <style>
    body{font-family:monospace;background:#222;color:#eee;text-align:center;padding:10px;margin:0}
    img{width:100%%;max-width:640px;border:3px solid #888;display:block;margin:0 auto;background:#555}
    #res{margin-top:14px;font-size:1.5em;min-height:2em;word-break:break-all}
    .found{color:#4f4}
    button{margin-top:10px;padding:6px 18px;font-size:1em;background:#333;color:#eee;
           border:1px solid #666;border-radius:4px;cursor:pointer}
    button:hover{background:#555}
  </style>
</head>
<body>
  <img src="http://%s:81/">
  <div id="res">Scanning...</div>
  <button onclick="doReset()">Reset</button>
  <script>
    let last='';
    async function doReset(){
      await fetch('/reset');
      last='';
      document.getElementById('res').innerHTML='Scanning...';
    }
    setInterval(async()=>{
      try{
        const t=(await(await fetch('/result')).text()).trim();
        if(t&&t!==last){
          last=t;
          document.getElementById('res').innerHTML=
            '<span class="found">&#10003; '+
            t.replace(/&/g,'&amp;').replace(/</g,'&lt;')+
            '</span>';
        }
      }catch(e){}
    },400);
  </script>
</body>
</html>
)HTML", ip.c_str());
}

static void handleResult(WiFiClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println();
  xSemaphoreTake(g_payloadMux, portMAX_DELAY);
  client.print(lastPayload);
  xSemaphoreGive(g_payloadMux);
}

static void handleReset(WiFiClient& client) {
  xSemaphoreTake(g_payloadMux, portMAX_DELAY);
  lastPayload[0] = '\0';
  lastPayloadTime = 0;
  xSemaphoreGive(g_payloadMux);
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.print("OK");
  Serial.println("[qr] result cleared");
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  neopixelWrite(48, 0, 0, 0);

  pinMode(PDM_PWR, OUTPUT);
  digitalWrite(PDM_PWR, LOW);
  delay(10);

  Serial.println("Connecting WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nConnected:\n  Page:   http://%s/\n  Stream: http://%s:81/\n",
                WiFi.localIP().toString().c_str(), WiFi.localIP().toString().c_str());

  // ── Camera — grayscale, QVGA, 2 buffers ──────────────────────────────────
  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0 = CAM_PIN_Y2;  cfg.pin_d1 = CAM_PIN_Y3;
  cfg.pin_d2 = CAM_PIN_Y4;  cfg.pin_d3 = CAM_PIN_Y5;
  cfg.pin_d4 = CAM_PIN_Y6;  cfg.pin_d5 = CAM_PIN_Y7;
  cfg.pin_d6 = CAM_PIN_Y8;  cfg.pin_d7 = CAM_PIN_Y9;
  cfg.pin_xclk     = CAM_PIN_XCLK;
  cfg.pin_pclk     = CAM_PIN_PCLK;
  cfg.pin_vsync    = CAM_PIN_VSYNC;
  cfg.pin_href     = CAM_PIN_HREF;
  cfg.pin_sccb_sda = CAM_PIN_SIOD;
  cfg.pin_sccb_scl = CAM_PIN_SIOC;
  cfg.pin_pwdn     = CAM_PIN_PWDN;
  cfg.pin_reset    = CAM_PIN_RESET;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_GRAYSCALE;
  cfg.frame_size   = FRAMESIZE_QVGA;          // 320×240
  cfg.fb_count     = 2;                       // scan task + stream task
  cfg.fb_location  = CAMERA_FB_IN_PSRAM;
  cfg.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  if (esp_camera_init(&cfg) != ESP_OK) {
    Serial.println("Camera init FAILED — halting");
    while (true) delay(1000);
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_contrast(s, 2);
    s->set_sharpness(s, 2);
    s->set_saturation(s, -2);
    s->set_brightness(s, 0);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_ae_level(s, -1);
    s->set_gain_ctrl(s, 1);
    s->set_gainceiling(s, GAINCEILING_4X);
    s->set_lenc(s, 1);
    s->set_bpc(s, 1);
    s->set_wpc(s, 1);
    s->set_raw_gma(s, 1);
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);
    s->set_reg(s, 0xff, 0xff, 0x00);  // direct sharpness fix
    s->set_reg(s, 0x92, 0xff, 0x01);
    s->set_reg(s, 0x93, 0xff, 0x30);
  }

  Serial.println("[cam] flushing frames for AEC settle...");
  for (int i = 0; i < 20; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(80);
  }
  Serial.println("[cam] ready");

  g_payloadMux = xSemaphoreCreateMutex();

  // scan on Core 1 (CPU intensive, isolated from WiFi)
  xTaskCreatePinnedToCore(scanTask,   "scan",   40960, NULL, 2, NULL, 1);
  // stream on Core 0 (co-located with WiFi/lwIP for best TCP throughput)
  xTaskCreatePinnedToCore(streamTask, "stream", 8192,  NULL, 1, NULL, 0);

  apiServer.begin();
  Serial.println("API server started on port 80.");
}

// ── API loop (loop runs on Core 1) ────────────────────────────────────────────
void loop() {
  WiFiClient client = apiServer.accept();
  if (!client) return;

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

  if      (requestLine.startsWith("GET /result")) handleResult(client);
  else if (requestLine.startsWith("GET /reset"))  handleReset(client);
  else if (requestLine.startsWith("GET /"))       handleRoot(client);
  else    client.println("HTTP/1.1 404 Not Found\r\nConnection: close\r\n");

  client.stop();
}
