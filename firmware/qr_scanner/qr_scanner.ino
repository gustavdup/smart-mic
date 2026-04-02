/*
 * Standalone QR Code Scanner
 * XIAO ESP32-S3 Sense + Seeed Expansion Board v1
 *
 * Scans continuously. On decode:
 *   - Shows payload on OLED
 *   - Beeps buzzer (D6 / GPIO43 on expansion board)
 *   - Flashes green NeoPixel
 *
 * No WiFi required.
 *
 * Board:  XIAO_ESP32S3
 * PSRAM:  Tools → PSRAM → OPI PSRAM
 *
 * Key learnings that make this work:
 *   - PIXFORMAT_GRAYSCALE + FRAMESIZE_QVGA (not JPEG, not VGA)
 *   - fb_count=2 + GRAB_WHEN_EMPTY for stable single-consumer loop
 *   - fb_return() BEFORE quirc_end() — releases camera buffer
 *     immediately so the next frame can be captured during the
 *     ~75ms decode, keeping the loop tight
 *   - ae_level=-1 prevents phone screen overexposure
 *   - Direct register sharpness fix (set_sharpness() is broken on OV2640)
 *   - 20-frame AEC flush before scanning so exposure is settled
 */

#include "esp_camera.h"
#include <U8g2lib.h>
#include <Wire.h>
#include <ESP32QRCodeReader.h>  // compiles quirc .c files
#include "quirc/quirc.h"

// loopTask default stack is 8KB — quirc decode blows it on success
SET_LOOP_TASK_STACK_SIZE(32768);

// ── Buzzer ────────────────────────────────────────────────────────────────────
#define BUZZER_PIN  43   // D6 on Seeed Expansion Board v1 — adjust if needed

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

#define PDM_PWR  14  // shared with camera Y6 — pull LOW before init

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

static struct quirc* g_quirc = nullptr;

// ── OLED helpers ──────────────────────────────────────────────────────────────
static void oledMsg(const char* line1, const char* line2 = "",
                    const char* line3 = "") {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tf);
  if (*line1) u8g2.drawStr(0,  9, line1);
  if (*line2) u8g2.drawStr(0, 21, line2);
  if (*line3) u8g2.drawStr(0, 33, line3);
  u8g2.sendBuffer();
}

// Word-wrap a long string across two OLED lines (21 chars each at 5x7 font)
static void oledResult(const char* payload) {
  char l1[22] = {}, l2[22] = {}, l3[22] = {};
  strncpy(l1, payload,      21);
  strncpy(l2, payload + 21, 21);
  strncpy(l3, payload + 42, 21);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(0,  9, "QR Found:");
  u8g2.drawStr(0, 21, l1);
  if (l2[0]) u8g2.drawStr(0, 33, l2);
  if (l3[0]) u8g2.drawStr(0, 45, l3);
  u8g2.sendBuffer();
}

// ── Beep ──────────────────────────────────────────────────────────────────────
static void beep() {
  tone(BUZZER_PIN, 2000, 120);  // 2 kHz, 120 ms
  delay(160);
  tone(BUZZER_PIN, 2800, 80);
  delay(100);
  noTone(BUZZER_PIN);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  neopixelWrite(48, 0, 0, 0);

  // GPIO14 shared with camera Y6 — must be LOW before camera init
  pinMode(PDM_PWR, OUTPUT);
  digitalWrite(PDM_PWR, LOW);
  delay(10);

  u8g2.begin();
  oledMsg("QR Scanner", "Initialising...");

  // ── Camera — grayscale QVGA, 2 buffers ───────────────────────────────────
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
  cfg.xclk_freq_hz  = 20000000;
  cfg.pixel_format  = PIXFORMAT_GRAYSCALE;  // quirc needs raw pixels
  cfg.frame_size    = FRAMESIZE_QVGA;       // 320×240 — fast decode, enough resolution
  cfg.fb_count      = 2;                    // return one buffer while camera fills other
  cfg.fb_location   = CAMERA_FB_IN_PSRAM;
  cfg.grab_mode     = CAMERA_GRAB_WHEN_EMPTY;

  if (esp_camera_init(&cfg) != ESP_OK) {
    oledMsg("Camera FAILED", "Check PSRAM", "Halting");
    Serial.println("[cam] init failed");
    while (true) delay(1000);
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_contrast(s, 2);             // max contrast — most impactful for QR modules
    s->set_sharpness(s, 2);            // high-level call (backed by register fix below)
    s->set_saturation(s, -2);          // irrelevant in grayscale
    s->set_brightness(s, 0);           // neutral — let AEC handle exposure
    s->set_exposure_ctrl(s, 1);        // AEC on
    s->set_aec2(s, 1);                 // flicker filter — helps with phone screens
    s->set_ae_level(s, -1);            // slight underexpose — prevents screen blow-out
    s->set_gain_ctrl(s, 1);            // AGC on
    s->set_gainceiling(s, GAINCEILING_4X);  // cap gain — 8X introduces bit errors
    s->set_lenc(s, 1);                 // lens vignette correction
    s->set_bpc(s, 1);                  // bad pixel correction
    s->set_wpc(s, 1);                  // hot pixel correction
    s->set_raw_gma(s, 1);              // gamma correction
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);
    // set_sharpness() is broken on OV2640 — use direct register writes
    s->set_reg(s, 0xff, 0xff, 0x00);
    s->set_reg(s, 0x92, 0xff, 0x01);
    s->set_reg(s, 0x93, 0xff, 0x30);
  }

  // Flush 20 frames so AEC converges before we start scanning
  oledMsg("QR Scanner", "Settling AEC...");
  for (int i = 0; i < 20; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(80);
  }

  // Allocate quirc for QVGA
  g_quirc = quirc_new();
  if (!g_quirc || quirc_resize(g_quirc, 320, 240) < 0) {
    oledMsg("quirc FAILED", "Check PSRAM", "Halting");
    Serial.println("[qr] quirc init failed");
    while (true) delay(1000);
  }

  oledMsg("QR Scanner", "Ready", "Aim at QR code");
  Serial.println("[qr] ready — scanning");
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { delay(10); return; }

  // Copy pixels into quirc's buffer, then release camera buffer IMMEDIATELY
  // so the next frame can be captured while quirc runs (~75ms on QVGA)
  int qw, qh;
  uint8_t* qbuf = quirc_begin(g_quirc, &qw, &qh);
  memcpy(qbuf, fb->buf, (size_t)qw * qh);
  esp_camera_fb_return(fb);   // ← key: release before the slow quirc_end

  quirc_end(g_quirc);         // detect finder patterns + extract grid

  int n = quirc_count(g_quirc);
  for (int i = 0; i < n; i++) {
    struct quirc_code code;
    struct quirc_data data;
    quirc_extract(g_quirc, i, &code);

    if (quirc_decode(&code, &data) == QUIRC_SUCCESS) {
      const char* payload = (const char*)data.payload;
      Serial.printf("[qr] Found: %s\n", payload);

      oledResult(payload);
      beep();
      neopixelWrite(48, 0, 30, 0);   // green flash
      delay(2000);
      neopixelWrite(48, 0, 0, 0);
      oledMsg("QR Scanner", "Ready", "Aim at QR code");
    }
  }
}
