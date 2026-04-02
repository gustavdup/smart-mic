# Waiter Name/Code via QR Scan — beacon_scanner.ino

## Status: Working ✓

QR scan is integrated and confirmed working. Waiter identity (`name|code`) is set by
scanning a QR code with the built-in OV2640 camera. Values are saved to NVS, included in
WAV filenames, and shown on the OLED waiter screen.

---

## QR Format
`name|code` — e.g. `Alice|W01`

- `waiterName` (32 bytes) — display name
- `waiterCode` (16 bytes) — short code embedded in WAV filename (`rec_YYYYMMDD_HHMMSS_W01.wav`)

---

## Library Required
**ESP32QRCodeReader** (by alvarowolfx) — includes quirc source files.
Install via Arduino IDE: Sketch → Include Library → Add .ZIP Library.
GitHub: https://github.com/alvarowolfx/ESP32QRCodeReader

**Note:** The library is included only to trigger compilation of quirc `.c` files.
`scanQRCode()` calls quirc directly via `#include "quirc/quirc.h"` — the
`ESP32QRCodeReader` class is not used.

---

## GPIO14 Conflict
Camera Y6 = GPIO14 = PDM_PWR (mic power enable). Safe because scanning only happens when
`!isRecording` — mic is powered down and GPIO14 is free for the camera. Restored to HIGH
after `esp_camera_deinit()`.

---

## Key Learnings (what makes it work)

| Setting | Why |
|---------|-----|
| `PIXFORMAT_GRAYSCALE` + `FRAMESIZE_QVGA` (320×240) | quirc needs raw pixels, not JPEG; QVGA is fast enough for decode |
| `fb_count=2` + `CAMERA_GRAB_WHEN_EMPTY` | Allows a second buffer to fill during decode |
| `esp_camera_fb_return()` BEFORE `quirc_end()` | Releases buffer during the ~75ms decode — keeps capture pipeline running |
| 20-frame AEC flush (×80ms) | Lets auto-exposure converge before scanning — blind `delay(2000)` is not enough |
| `ae_level=-1` | Prevents phone screen blow-out |
| `GAINCEILING_4X` (not 8X) | 8X introduces noise → bit errors in QR modules |
| Direct register sharpness fix | `set_sharpness()` is broken on OV2640 (espressif/esp32-camera #196) — must use `set_reg(0xff/0x92/0x93)` |
| `SET_LOOP_TASK_STACK_SIZE(32768)` | Default 8KB loopTask stack overflows when quirc decode succeeds |

---

## Camera Configuration
```cpp
cfg.pixel_format = PIXFORMAT_GRAYSCALE;
cfg.frame_size   = FRAMESIZE_QVGA;   // 320×240
cfg.fb_count     = 2;
cfg.fb_location  = CAMERA_FB_IN_PSRAM;
cfg.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
```

## Sensor Settings
```cpp
s->set_contrast(s, 2);
s->set_sharpness(s, 2);
s->set_saturation(s, -2);
s->set_brightness(s, 0);
s->set_exposure_ctrl(s, 1);
s->set_aec2(s, 1);
s->set_ae_level(s, -1);                 // prevents phone screen blow-out
s->set_gain_ctrl(s, 1);
s->set_gainceiling(s, GAINCEILING_4X);  // 8X = bit errors
s->set_lenc(s, 1);
s->set_bpc(s, 1);
s->set_wpc(s, 1);
s->set_raw_gma(s, 1);
s->set_vflip(s, 1);
s->set_hmirror(s, 0);
// Direct sharpness fix — set_sharpness() is a no-op on OV2640
s->set_reg(s, 0xff, 0xff, 0x00);
s->set_reg(s, 0x92, 0xff, 0x01);
s->set_reg(s, 0x93, 0xff, 0x30);
```

## Core Decode Loop Pattern
```cpp
camera_fb_t* fb = esp_camera_fb_get();
int qw, qh;
uint8_t* qbuf = quirc_begin(q, &qw, &qh);
memcpy(qbuf, fb->buf, (size_t)qw * qh);
esp_camera_fb_return(fb);   // ← release BEFORE slow quirc_end
quirc_end(q);               // ~75ms on QVGA
```

---

## Beep Feedback
- **Success:** 2 kHz (120ms) → 2.8 kHz (80ms) two-tone ascending beep
- **Timeout:** Two 800 Hz beeps

---

## Trigger
Screen 5 (waiter screen, index 4), hold 3s → calls `scanQRCode()`.

---

## Verification
1. Flash, open Serial monitor
2. Navigate to Screen 5 — shows waiter name/code or "No waiter set"
3. Hold 3s — camera initialises, OLED counts down AEC flush then shows "Scan QR code"
4. Hold a QR code reading `Alice|W01` in front of camera
5. Serial shows `[qr] Found: Alice|W01` then `[qr] Saved waiter: Alice | W01`
6. Two-tone beep plays, OLED shows "Waiter set: Alice / W01" for 2s
7. Screen 5 now shows Alice / W01
8. Start recording — filename contains `_W01.wav`
9. Power cycle — waiter restored from NVS on boot
