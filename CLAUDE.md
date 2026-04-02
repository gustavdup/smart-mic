# Bluetooth Scanner — Session Continuity

## Project Overview
ESP32-S3 iBeacon scanner with stereo audio recording, OLED display, and MinIO upload.

**Hardware:** XIAO ESP32-S3 Sense mounted on Seeed Expansion Board v1

### Pin Assignments
| GPIO | Label | Use |
|------|-------|-----|
| 1 | D0 | I2S BCLK (both mics) |
| 2 | D1 | I2S WS/LRCL (both mics) |
| 3 | D2 | I2S DATA (both mics share — tri-state by SEL) |
| 4 | D3 | Button (external, wired D3→GND, INPUT_PULLUP active-low) |
| 5 | D4 | I2C SDA — hardware I2C (OLED, 0x3C) |
| 6 | D5 | I2C SCL — hardware I2C (OLED, 0x3C) |
| 7 | D8 | SPI SCK (SD card) |
| 8 | D9 | SPI MISO (SD card) |
| 9 | D10 | SPI MOSI (SD card) |
| 14 | — | PDM_PWR / CAM_PIN_Y6 — LOW before camera init |
| 21 | — | SD card CS |
| 43 | D6 | (free — do NOT use: UART0 TX, idles HIGH, causes motor buzz) |
| 44 | D7 | Vibration coin motor |
| 10–13, 15–18, 38–40, 47–48 | — | Camera (OV2640) |

### Microphones — Adafruit ICS-43434 (stereo pair)
- **Mic 1 (left channel):** SEL=GND, DATA=GPIO3 — faces table (customer/ambient)
- **Mic 2 (right channel):** SEL=3V3, DATA=GPIO3 (shared — tri-states when not its turn) — faces waiter mouth
- BCLK=GPIO1, WS=GPIO2 shared between both mics
- **DMA buffer layout:** `rawBuf[i*2]` = R slot (mic2), `rawBuf[i*2+1]` = L slot (mic1)
- Stereo 16-bit WAV output, 16kHz default (configurable via `sample_rate=` in config.txt)

### OLED
- **Display:** SSD1306 64×32, hardware I2C, address 0x3C, SDA=GPIO5, SCL=GPIO6
- **Library:** Adafruit SSD1306 + Adafruit GFX. Instance: `Adafruit_SSD1306 oled(64, 32, &Wire, -1)`
- **Rotation:** `oled.setRotation(2)` (180° — mounted upside-down in enclosure)
- **Init:** `Wire.begin(5, 6); oled.begin(SSD1306_SWITCHCAPVCC, 0x3C); oled.setRotation(2);`
- **Layout:** 10 chars wide, 4 lines at y=1/9/17/25 (5×7 font, 8px line height)

### Vibration Motor
- GPIO44 (D7) — coin motor wired direct to GPIO (no transistor, PoC only)
- `gpio_set_drive_capability((gpio_num_t)MOTOR_PIN, GPIO_DRIVE_CAP_3)` — enables 40mA pad drive
- Motor is 30mA typical / 35mA max — within limit for short bursts only
- **Do NOT use GPIO43 (D6)** — UART0 TX, idles HIGH, causes constant buzz
- Patterns: rec start = two 80ms pulses (60ms gap), rec stop = one 200ms pulse
- QR success = 120ms + 80ms ascending, QR timeout = two 100ms equal pulses
- **Known issue:** Motor current draw (and inductive kickback on OFF) glitches the OLED over I2C. Hardware fix: 1N4148 flyback diode across motor terminals (cathode to 3.3V) + transistor driver. PoC workaround: `Wire.end()` / `Wire.begin()` reset after pulses.

## Key Files
- `firmware/beacon_scanner.ino` — main firmware (active)
- `firmware/backups/` — older working versions
- `AUDIO_PIPELINE.md` — audio processing pipeline documentation
- `app.py` — Flask + SocketIO dashboard (Mac side, port 5001)
- `templates/index.html` — live BLE dashboard UI
- `parse_ble.py` — extracts BLE snapshot log from WAV, saves to `ble_logs/` folder
- `CPU_LOAD_TRACKING.md` — design notes on idle-hook CPU measurement and core pinning
- `QR_CODE_SCANNER.md` — QR scanning implementation notes
- `Kalman.md` — plan for Kalman-filtered BLE distance smoothing (not yet implemented)

## Architecture: PSRAM Double-Buffer Recording

Audio is recorded directly into PSRAM — no SD writes during recording. Two **2 MB** PSRAM buffers (`psramBuf[0/1]`). At 16kHz stereo 16-bit, each buffer holds ~30s per segment (~1.83 MB per segment). `recordingTask` writes into the active buffer; at each 30s segment boundary it patches the WAV header + BLE chunk in-buffer, queues the buffer via `segmentQueue`, and flips to the other. `uploadTask` streams the queued buffer directly to MinIO via `uploadFromPSRAM()`. SD is only written as a fallback when WiFi is down or upload fails.

- `activeBuf` — index (0 or 1) of the buffer currently being recorded into
- `psramFill[i]` — audio bytes written into buffer i (not counting WAV header)
- `uploadingBuf` — index of the buffer currently being uploaded (-1 = none); used by display
- `uploadKBps` — live upload speed (KB/s), updated per chunk during streaming
- `segmentQueue` — FreeRTOS queue (depth 2) of `SegmentReady` structs passed from recordingTask → uploadTask

**Segment size at 16kHz stereo 16-bit 30s:** ~1.83 MB audio + 44B WAV header + ~1KB BLE chunk = ~1.85 MB per file.

**PSRAM must be enabled:** Tools → PSRAM → OPI PSRAM.

## Audio Processing Pipeline

See `AUDIO_PIPELINE.md` for full documentation. Summary:

```
Raw 32-bit DMA (ICS-43434, 24-bit MSB-aligned)
  → Extract: (int32_t)(rawBuf >> 8) / 256.0f  → normalised 16-bit float
  → DC blocker (~10Hz, alpha=0.9992) — both channels
  → 3-pole high-pass (~80Hz, alpha=0.9972 cascaded) — both channels
  → Gain + soft limiter → int16_t output
```

Gains: `MIC_GAIN_L=4` (table-facing), `MIC_GAIN_R=4` (waiter-facing — same gain, adjust R upward if waiter channel is quieter).

## Critical Bug Fixes Applied

### Dangling pointer in BLE callback (NimBLE-Arduino 2.4.0)
`device->getManufacturerData().data()` returns a pointer to a temporary `std::string` destroyed immediately.

**Fix:**
```cpp
std::string mfr = device->getManufacturerData();  // keep alive
const uint8_t* d = (const uint8_t*)mfr.data();
size_t len = mfr.size();
```

### Mic — switched from PDM to I2S std (ICS-43434)
Built-in PDM mic replaced with external Adafruit ICS-43434 stereo pair. Uses `driver/i2s_std.h` — `i2s_channel_init_std_mode` with Philips slot format, 32-bit DMA reads.

**ICS-43434 quirks:**
- DMA delivers 32-bit frames — must use `I2S_SLOT_MODE_STEREO` and extract L/R from interleaved buffer
- 24 valid bits MSB-aligned in 32-bit frame. Extract with `(int32_t)(rawBuf[i] >> 8) / 256.0f` — keeps full 24-bit resolution normalised to 16-bit float scale
- `rawBuf[i*2]` = R slot (mic2, SEL=3V3), `rawBuf[i*2+1]` = L slot (mic1, SEL=GND)
- Both mics share DATA line — SEL=GND tri-states during R slot, SEL=3V3 tri-states during L slot

### Motor double-buzz causing OLED power glitch
Motor pulses existed in both the button handler (blocking `delay()`) and `recordingTask` (`vTaskDelay()`). Overlapping current draw crashed the OLED power rail.

**Fix:** Motor pulses only in `recordingTask`. Button handler sets `isRecording = true` then returns immediately.

### GPIO14 (PDM_PWR / CAM_PIN_Y6)
Must be set LOW before `esp_camera_init()` in `scanQRCode()` and restored HIGH after `esp_camera_deinit()`.

### Motor on GPIO43 (D6) causes constant buzz
GPIO43 is UART0 TX — idles HIGH and toggles with every Serial byte. Moved motor to GPIO44 (D7, UART0 RX, idles LOW).

### Arduino preprocessor auto-prototype bug
Arduino IDE auto-generates forward declarations for all non-`static` global functions. If a function uses a type from a `#pragma pack` struct in its signature (e.g. `BLESnapshot*`), the auto-generated prototype appears before the struct definition → compile error `'BLESnapshot' has not been declared`.

**Fix:** mark `writeBLEChunk` as `static` — Arduino preprocessor skips static functions.

### `esp_freertos_hooks.h` include conflict
Including `esp_freertos_hooks.h` breaks parsing of `#pragma pack` structs below it.

**Fix:** forward-declare the function instead:
```cpp
extern "C" esp_err_t esp_register_freertos_idle_hook_for_cpu(bool (*hook)(void), UBaseType_t cpuid);
```

### WiFi DMA cannot access PSRAM
WiFi TCP TX buffers must be in internal SRAM. Using `ps_malloc` silently fails or causes DMA errors.

**Fix:** `heap_caps_malloc(BUF_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)` for all TCP TX buffers.

### Task stack overflows
`uploadToMinIO` puts ~2 KB on stack (canonReq[640] + sts[512] + auth[512] + signing arrays).

**Fix:** `recordingTask` stack 12288, `uploadTask` stack 20480.

### Task watchdog (TWDT) triggering during upload
`tcp.write()` can hold Core 0 for several seconds when TCP send buffer is full.

**Fix:** `esp_task_wdt_reconfigure()` with 30s timeout in `setup()`.

### `setNoDelay` before `connect()`
Calling `tcp.setNoDelay(true)` before `tcp.connect()` has no effect.

**Fix:** Always call `setNoDelay(true)` immediately after a successful `connect()`.

### `currentFile` cross-core String race
`uploadTask` reads `currentFile.c_str()` while `recordingTask` may be assigning it.

**Fix:** Snapshot `currentFile` to a local `char skipFile[48]` under `sdMutex` before the SD scan loop.

### PSRAM streaming loop stall detection
`tcp.write()` blocks indefinitely if TCP send buffer is full.

**Fix:** Track `lastProgress = millis()`. Break with `stalled = true` if no bytes written for 8s or `!tcp.connected()`.

### IntelliSense false errors
`driver/i2s_std.h`, `driver/temperature_sensor.h`, `esp_task_wdt.h`, `quirc/quirc.h` show "cannot open source file" in VSCode. Compile fine in Arduino IDE — ignore.

### Audio saturation from >> 8 without normalisation
`>> 8` gives 24-bit range (±8.3M). With gain 4 this saturated the soft limiter entirely.

**Fix:** divide by 256.0f after shift: `(float)(int32_t)(rawBuf[i] >> 8) / 256.0f` — normalises to 16-bit scale while preserving sub-LSB float precision.

## FreeRTOS Task Layout
| Task | Core | Priority | Stack | Notes |
|------|------|----------|-------|-------|
| WiFi driver | 0 | 23 | — | Always Core 0 — hardwired by ESP-IDF |
| lwIP TCP/IP | 0 | 18 | — | |
| NimBLE host | 0 | ~21 | — | |
| `uploadTask` | 0 | 1 | 20480 | Co-located with WiFi/lwIP — do NOT move to Core 1 |
| `recordingTask` | 1 | 2 | 12288 | Uncontested — preempts loop() |
| `loop()` | 1 | 1 | 32768 | Display, button, BLE queue drain (SET_LOOP_TASK_STACK_SIZE) |

**Core 0 at 99% during upload is expected** — WiFi/BLE drivers are higher priority and still preempt uploadTask.

## OLED Screens (9 screens, 64×32, cycle with single click)

All screens rendered on the 64×32 Adafruit SSD1306 (`oled.setRotation(2)` for 180° enclosure mount). 10 chars wide, 4 lines at y=1/9/17/25.

**Screen 0 (main):** Recording ON/OFF with 3s hint, waiter `W:name`, UL speed or `UL: idle`, SD pending. Hold 3s → start/stop recording. Rec blink dot (r=2) top-right when recording.

**Screen 1 (waiter):** Waiter code, name, `3s:scan QR`, `6s:demo`. Hold 3s → scan QR. Hold 6s → load placeholder `John|d03`. Rec blink dot (r=2) top-right when recording.

**Screen 2 (PSRAM):** B0/B1 PSRAM state (REC/UPL/rdy/idl/fre) + fill in KB or MB (e.g. `1.8M`), UL speed or `UL:idle`, SD pend count. Hold 3s → start/stop recording.

**Screen 3:** BLE beacons 1–3 — format `1-70 3.2m` (no T prefix, space between dB and metres). Stale shown as `1 stale`, inactive as `1 --`.

**Screen 4:** BLE beacons 4–5.

**Screen 5:** System status — R/B/U/W flags, PSRAM+SD pending, waiter name|code.

**Screen 6:** CPU — C0 load %, C1 load % on separate lines, die temp, UL KB/s if active.

**Screen 7:** Speedtest — hold 3s to run. Live Mbps + KB/total progress. Press to cancel. Results persist.

**Screen 8:** Activity log — 4 most recent entries, oldest top, newest bottom.

**Hold progress bar:** `oled.fillRect()` bottom row (y=30, 2px, 64px wide). Shown on screens 0, 1, 2, 7. Full at 3s (screens 0, 2, 7) or 6s (screen 1). Tick mark at x=32 on screen 1 (3s QR threshold).

**Rec blink dot:** `oled.fillCircle(60, 4, 2, WHITE)` — radius 2, top-right corner. Shown on screens 0 and 1.

**PSRAM fill display:** Shows `###K` below 1000KB, `#.#M` at 1000KB and above.

## Pending Upload Counters
Split into two atomic counters, protected by `portENTER_CRITICAL_SAFE(&_pend_mux)`:
- `pendingPSRAM` — segments queued or uploading from PSRAM. `PSRAM_INC()` on queue, `PSRAM_DEC()` after upload/fallback.
- `pendingSD` — WAV files on SD waiting to upload. Pre-scanned at start of each upload session. `SD_INC()` inside `writePSRAMToSD()`, `SD_DEC()` after successful `uploadToMinIO()`.

## WiFi Lifecycle
- **Startup:** Connect → NTP sync → `WiFi.mode(WIFI_OFF)` (after BLE init)
- **Upload:** `WiFi.mode(WIFI_STA)` → connect → 500ms settle → upload queue drain → `WiFi.mode(WIFI_OFF)`
- **Speedtest:** Same as upload, triggered manually from screen 7 (hold 3s). Button press cancels.

## SD Card Config File
Loaded at boot from `/config.txt` on SD, saved to NVS. On subsequent boots without the file, values restored from NVS.

```
wifi_ssid=Home DSL
wifi_pass=123454321
minio_host=192.168.1.72
minio_port=9000
minio_bucket=audio
minio_access=minioadmin
minio_secret=minioadmin
sample_rate=16000
```

Lines starting with `#` ignored. Unknown keys silently skipped. `loadConfig()` returns `"SD"`, `"NVS"`, or `"default"`.

## iBeacon Packet Layout (manufacturer data, 25 bytes)
```
[4C 00]         Apple company ID
[02 15]         iBeacon type / length
[UUID 16 bytes] deployment UUID
[major 2B BE]   values 1–5
[minor 2B BE]
[txPower 1B]    signed dBm
```

## Deployment UUID
`E2C56DB5-DFFB-48D2-B060-D0F5A71096E0` — Majors 1–5 (one per table)

## BLE Chunk in WAV Files
Custom `ble_` chunk appended after audio data in PSRAM before upload. 34 bytes/snapshot: `uint32 offset_ms` + 5×`(bool active, int8 rssi, float distance)`.
- Parse: `python3 parse_ble.py` → file picker → saves to `ble_logs/<name>.txt`

## MinIO
- Default host: `192.168.1.72:9000`, bucket: `audio` — override via `config.txt`
- AWS Signature V4 with `UNSIGNED-PAYLOAD`, region `"us-east-1"` (structurally required)

## Upload Buffer Strategy
`heap_caps_malloc(16384, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)` for TCP TX buffer. PSRAM cannot be read by WiFi DMA — must be internal RAM. PSRAM used for recording buffers.

## Beacon Slot Assignment
`slot = major - 1` — direct mapping.

## CPU Temperature Sensor
`driver/temperature_sensor.h`. Read every 500ms in `updateDisplay()`. Runs 5–15°C above ambient.

## Green LED
`neopixelWrite(48, 0, 0, 0)` at start of `setup()` — turns off built-in RGB LED. No library needed.

## Button Interactions
| Action | Result |
|--------|--------|
| Single click | Next screen (0→1→...→8→0, fires 500ms after release) |
| Double click | Previous screen (8→7→...→0→8) |
| 4 rapid clicks | Toggle uploads on/off — shows "UL enabled" / "UL off" for 1s then returns |
| Hold 3s | Screen 0: start/stop recording. Screen 1: scan QR. Screen 2: start/stop recording. Screen 7: run speedtest. |
| Hold 6s | Screen 1 only: load placeholder waiter `John\|d03` (auto-fires, no release needed) |
| Press during speedtest | Cancel speedtest immediately |

Button state machine: `g_btnState` 0=idle, 1=down, 2=up(counting). `g_clickCount` tracks rapid clicks in current sequence.

## Upload Enable/Disable
`uploadsEnabled` bool (default true). 4-click toggles. When disabled: queue drains to SD, files accumulate. Re-enable wakes `uploadTask` via `xSemaphoreGive(uploadReady)`.

## Segment Duration
30s target. Actual ~28–29s due to finalize/header overhead.

## SD Card Notes
- `SD.begin()` retried up to 5× at boot with 300ms gaps
- SD scan in `uploadTask` reinits SD if `cardSize() == 0`
- Only `.wav` files uploaded — `config.txt` never matched

## Dead Code (not yet removed)
`createWAV()`, `finalizeWAV()`, `writeBLEChunk()` — SD-based WAV helpers from old recording path. Never called. Safe to delete.

## Arduino File Sync
Two copies of the firmware:
- `firmware/beacon_scanner.ino` — repo copy (source of truth)
- `~/Documents/Arduino/beacon_scanner/beacon_scanner.ino` — compiled by Arduino IDE

Both must stay in sync. After every edit: `cp "firmware/beacon_scanner.ino" ~/Documents/Arduino/beacon_scanner/beacon_scanner.ino`

## QR Code Scanning (Waiter Assignment)

**Status: Working.** Screen 1 (waiter screen), hold 3s → `scanQRCode()`.

QR format: `name|code` (e.g. `Alice|W01`). Saved to NVS namespace `"cfg"`, restored on boot.
WAV filename: `rec_YYYYMMDD_HHMMSS_CODE_NAME.wav`.
Placeholder waiter `John|d03` via 6s hold on screen 1.
Recording cannot start without a waiter — error shown, redirects to screen 1.

### What makes QR work
- `PIXFORMAT_GRAYSCALE` + `FRAMESIZE_QVGA` (320×240) — quirc needs raw pixels
- `esp_camera_fb_return()` BEFORE `quirc_end()` — releases buffer during ~75ms decode
- 20-frame AEC flush (×80ms) — lets auto-exposure converge
- `ae_level=-1` — prevents phone screen blow-out
- `GAINCEILING_4X` (not 8X) — 8X causes bit errors in QR modules
- Direct register sharpness fix — `set_sharpness()` broken on OV2640:
  ```cpp
  s->set_reg(s, 0xff, 0xff, 0x00);
  s->set_reg(s, 0x92, 0xff, 0x01);
  s->set_reg(s, 0x93, 0xff, 0x30);
  ```
- `SET_LOOP_TASK_STACK_SIZE(32768)` — default 8KB overflows on quirc success
- `ESP32QRCodeReader` included only to compile quirc `.c` files; class not used

### QR feedback (motor)
- Success: 120ms pulse + 80ms pulse (ascending feel)
- Timeout: two 100ms equal pulses

---

## Pending Tasks
- End-to-end verification: 3+ segments recording + uploading without crash
- Verify BLE chunk present in all uploaded files via parse_ble.py
- Remove dead code: `createWAV`, `finalizeWAV`, `writeBLEChunk`
- Add transistor circuit (2N2222 + 1kΩ) + flyback diode (1N4148) for motor in first prototype
