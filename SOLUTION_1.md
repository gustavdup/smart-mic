# Michelin Insights — Wearable Audio Capture Device

> **Note:** This is an early-stage design document from before the hardware was finalised. Pin assignments, task architecture, button interactions, and many other details differ from the current implementation. See `CLAUDE.md` for the authoritative current state.

## Overview

Michelin Insights is a restaurant conversation intelligence platform. Waiters wear a small ESP32-S3-based device that continuously records stereo audio, attributes conversations to specific tables via BLE beacons, and uploads encrypted WAV files to a local server for transcription and analytics.

**Platform status:** Auth, billing (Stripe), RBAC, feature gating, platform admin, company registration, staff/venue management are all live at `https://michelin.hostingoz.dev/login`. All AI-generated data currently uses `MockDashboardService` — the hardware described here replaces that with a real pipeline.

**Competitor:** TolqAI (`tolqai.com`) — similar wearable mic but no table-level attribution, no local processing, unknown pricing.

---

## System Architecture

```
Restaurant Floor:
  Waiter devices (XIAO ESP32-S3) scan BLE beacons at each table
  → Record stereo audio to SD card
  → Upload via WiFi to local server

Local Server (NUC/Jetson/Mac):
  Minio (S3)     → receives WAV uploads
  Mosquitto MQTT → receives live device telemetry
  Python watcher → extracts BLE data, splits audio, runs Whisper transcription
  Dashboard      → live device monitoring, floor map

Cloud:
  Michelin Insights Platform → analytics, transcripts, reporting
```

---

## Hardware — Wearable Device

### Components

| Component | Part | Interface | Notes |
|---|---|---|---|
| Microcontroller | Seeed XIAO ESP32-S3 Sense (pre-soldered) | — | 21×17.5mm, dual-core 240MHz, WiFi+BLE 5.0, camera, SD, USB-C, 8MB PSRAM |
| Microphone × 2 | Adafruit ICS-43434 I2S MEMS (breakout, product 6049) | I2S shared bus | 65dB SNR, bottom-ported, 16.5×12.7mm board |
| Accelerometer | Adafruit LIS3DH (product 2809) | I2C (0x18) | Speech vibration detection via chest, STEMMA QT |
| Display | 0.49" OLED I2C White 64×32 SSD1306 | I2C (0x3C) | Button-cycled: battery / status / off |
| Haptic motor | Shaftless vibration motor 8×3.4mm 3V 60mA | GPIO direct drive | 3M adhesive backing |
| Battery | LiPo 800mAh 3.7V (802540) | JST-PH 2mm | ~11 hours at ~70mA avg draw |
| SD card | 16GB MicroSD FAT32 | SPI (onboard) | 24h retention after upload |
| Button | Tactile vertical mount switch 10mm | GPIO 1 | Through-enclosure stem |
| Enclosure | 3D printed (base: Printables 1243157) | — | Modified for mic holes, OLED window, button |
| Mic windscreen | Foam earplug disc (5mm circle) | — | Waiter mic only |

### Where to Order

**DigiKey ZA (already ordered):**
- XIAO ESP32-S3 Sense pre-soldered (102010635) × 1
- XIAO ESP32-S3 Sense (113991115) × 3
- ICS-43434 I2S MEMS Mic Adafruit 6049 × 6
- Vibration motor VCLP0820B004L × 3
- XIAO Expansion Base (103030356) × 1
- LIS3DH Accelerometer Adafruit 2809 × 3

**Micro Robotics — robotics.org.za:**
- [OLED 0.49" I2C White 64×32](https://www.robotics.org.za/OLED-049-I2C) × 3
- [Battery LiPo 800mAh 3.7V](https://www.robotics.org.za/802540) × 3
- [DFRobot BLE Beacon 5-pack TEL0149](https://www.robotics.org.za/TEL0149) × 1
- [Shaftless Vibration Motor 8×3.4mm](https://www.robotics.org.za/1637) × 3
- [Tactile Vertical Mount Switch 10mm](https://www.robotics.org.za/MR-C231715) × 1 pack
- [CR2032 Batteries 5-pack](https://www.robotics.org.za/CR2032) × 1
- [Jumper Wires Male-Female 50 pack](https://www.robotics.org.za/JUMP-MF) × 1
- [Pogo pins](https://www.robotics.org.za/P75-E2) (for beacon config jig, optional)

**Other:**
- MicroSD 16GB × 3 (any phone shop or Takealot)
- 3D printed enclosures — device (Printables 1243157 modified) + beacon (Printables 767212)

### Cost Per Device

| Category | Cost |
|---|---|
| DigiKey | ~R2,042 (for 3 devices + extras) |
| Micro Robotics | ~R1,648 |
| Other | ~R200–400 |
| **Per device (components only)** | **~R700–900** |

### Full Restaurant Deployment (5 waiters, 20 tables)

| Item | Qty | Cost |
|---|---|---|
| Waiter devices | 5 (+2 spare) | ~R5,000–6,300 |
| Table beacons | 20 (+5 spare) | ~R1,800–2,500 |
| Local server (NUC) | 1 | R3,000–5,000 |
| **Total** | | **~R9,800–13,800** |
| **Monthly running cost** | | **~R20 (electricity)** |

---

## GPIO Pin Mapping

```
I2S Bus (stereo mics — two ICS-43434):
  BCLK     → GPIO 7
  WS/LRCL  → GPIO 8
  DATA     → GPIO 9
  Mic 1 SEL → GND  (left channel = waiter mic, upward facing)
  Mic 2 SEL → 3.3V (right channel = table mic, outward facing)

I2C Bus (shared — OLED + LIS3DH):
  SDA      → GPIO 5
  SCL      → GPIO 6
  OLED address: 0x3C
  LIS3DH address: 0x18

Haptic motor:   GPIO 3 (direct drive, no transistor)
Status LED:     GPIO 21 (onboard orange LED)
Button:         GPIO 1 (external tactile switch, INPUT_PULLUP)
```

### ICS-43434 Mic Wiring (both mics share same bus)

Both mics connect to the same BCLK, LRCL, DOUT wires (daisy-chained). SEL determines which channel each mic outputs on:

| Pin | Mic 1 (Waiter) | Mic 2 (Table) |
|---|---|---|
| 3V | → 3.3V | → 3.3V |
| GND | → GND | → GND |
| BCLK | → GPIO 7 | → GPIO 7 (shared) |
| LRCL | → GPIO 8 | → GPIO 8 (shared) |
| DOUT | → GPIO 9 | → GPIO 9 (shared) |
| SEL | → GND (solder bridge) | → 3.3V (solder bridge) |

Solder the SEL pin directly to GND or 3V on the mic board — no extra wire needed to the XIAO.

### LIS3DH Wiring

Mount on the **body-facing side** of the enclosure for best chest vibration coupling.

| LIS3DH Pin | Connect to |
|---|---|
| VIN | 3.3V |
| GND | GND |
| SDA | GPIO 5 |
| SCL | GPIO 6 |

---

## Button Interaction Map

| Action | Trigger | Response |
|---|---|---|
| Show battery % | Single click | OLED: battery % + bar graphic for 5s |
| Show status | Double click | OLED: waiter name / table / uptime for 5s |
| QR pair waiter | Triple click | Camera on → "Scan badge..." → scan → "Hi James ✓" |
| Shutdown | Long press 3s | "Shutting down..." → deep sleep |
| Wake up | Long press | Device boots, starts recording |

---

## Enclosure Design Notes

**Base model:** Printables 1243157 (Meshtastic XIAO ESP32S3 case)

**Modifications needed:**
- Remove LoRa module pocket
- Waiter mic hole: top face, 3–4mm hole with 5mm shallow recess for foam windscreen
- Table mic hole: front face, clean 2mm hole aligned with ICS-43434 sound port
- Mic mounting: sealed acoustic sub-housing for each mic board, screwed with M2×6mm screws
- OLED window: top face
- Camera window: front face
- Button hole: side wall, 10mm stem passes through
- Wall thickness: 2mm minimum, 2.5mm recommended

**Weight estimate:** ~55–65g total. Consider shirt clip mount rather than lanyard for better vibration coupling and comfort over an 8-hour shift.

---

## Hardware — BLE Table Beacons

**Part:** DFRobot BLE Sensor Beacon TEL0149 (5-pack)
**Power:** CR2032 coin cell (3–6 months at 1s broadcast interval)
**Enclosure:** Printables 767212 (designed specifically for TEL0149)

### Beacon Configuration

Uses **NanoBeaconConfigTool V3.2.11** (Windows/Mac) via CP2104 USB-TTL adapter.

**Wiring for config (CP2104 → Beacon):**
| CP2104 | Beacon |
|---|---|
| TXD | RX |
| RXD | TX |
| GND | GND |
| VCC | Not connected (power from battery) |

**iBeacon settings per beacon:**

| Beacon | UUID | Major | Minor | Tx Power | Interval |
|---|---|---|---|---|---|
| Table 1 | E2C56DB5-DFFB-48D2-B060-D0F5A71096E0 | 0001 | 0001 | -59 | 1000ms |
| Table 2 | E2C56DB5-DFFB-48D2-B060-D0F5A71096E0 | 0002 | 0001 | -59 | 1000ms |
| Table 3 | E2C56DB5-DFFB-48D2-B060-D0F5A71096E0 | 0003 | 0001 | -59 | 1000ms |
| Table 4 | E2C56DB5-DFFB-48D2-B060-D0F5A71096E0 | 0004 | 0001 | -59 | 1000ms |
| Table 5 | E2C56DB5-DFFB-48D2-B060-D0F5A71096E0 | 0005 | 0001 | -59 | 1000ms |

**⚠️ IMPORTANT:** eFuse memory can only be burned once. Always test with "Run in RAM" first and verify with LightBlue on Mac before clicking "Burn/Program".

**Deployment UUID:** `E2C56DB5-DFFB-48D2-B060-D0F5A71096E0`

---

## Firmware Architecture

**Framework:** Arduino (ESP32 Arduino Core 2.x+)
**Build tool:** PlatformIO (recommended) or Arduino IDE 2.x
**Base fork:** `NamanKansal230505/AI-Voice-Recorder-AWS-S3`

### Module Summary

| Module | Core | Priority | Purpose |
|---|---|---|---|
| audio_recorder | 0 | 5 (highest) | Continuous stereo I2S recording, double-buffered, gapless |
| ble_scanner | 1 | 3 | iBeacon scan every 1s, dominant table state machine |
| imu_speech_detect | 1 | 3 | LIS3DH 400Hz, bandpass 85–300Hz, RMS speaking flag |
| wifi_uploader | 1 | 2 | Upload completed WAV files to Minio, retry, cleanup |
| mqtt_telemetry | 1 | 1 | Publish device status every 60s to Mosquitto |
| ui_task | 1 | 1 | Button debounce, click detection, OLED, haptic, LED |
| battery_monitor | 1 | 0 | ADC voltage, LiPo curve, drain estimate, deep sleep |

### Audio Recording

- **Sample rate:** 16000Hz, 16-bit, stereo (2 channels)
- **Chunk duration:** 60 seconds
- **File size:** ~3.75MB per chunk
- **Buffering:** Double buffer in PSRAM (2× ~3.75MB), gapless swap
- **DSP:** High-pass filter (sub-80Hz), noise gate, applied before SD write
- **Filename:** `mic{N}_table{T}_{YYYY-MM-DD_HH-MM-SS}_part{P}.wav`

### Custom btle WAV Chunk

Each WAV file contains a custom "btle" chunk appended after the data chunk. Standard WAV parsers ignore unknown chunks.

```
btle chunk contents:
  device_id      (uint8)
  waiter_name    (32 bytes, null-padded)
  snapshot_count (uint8, normally 60)
  60 × snapshot:
    timestamp_ms   (uint32)
    beacon_count   (uint8)
    beacons[]      (major uint8, minor uint8, rssi int8) × beacon_count
    speaking       (uint8, 0/1)
```

### BLE State Machine

```
AT_TABLE   → one beacon dominant, RSSI stable, 10+ consecutive seconds
IN_TRANSIT → no dominant beacon or RSSI shifting
AWAY       → all beacons below -85dBm for 10+ seconds
```

### Config (NVS — persists across reboots)

```
device_id, wifi_ssid, wifi_password
minio_endpoint, minio_bucket, minio_access_key, minio_secret_key
mqtt_broker, mqtt_port, mqtt_username, mqtt_password
ble_uuid, speech_threshold
waiter_id, waiter_name (set by QR scan)
```

First boot with no WiFi configured → enters interactive serial setup via USB-C at 115200 baud.

### LED Status

| Pattern | Meaning |
|---|---|
| Solid ON | Recording normally |
| Slow blink (1s/1s) | Uploading to Minio |
| Fast blink (200ms/200ms) | Error |
| OFF | Standby / deep sleep |

### Battery

- LiPo discharge curve lookup (not linear)
- Rolling average of 10 readings
- Drain estimation via voltage delta over 10-minute windows
- Double haptic buzz at 15%
- Deep sleep at 5% to protect LiPo

---

## PlatformIO Setup

```ini
[env:xiao_esp32s3]
platform = espressif32
board = seeed_xiao_esp32s3
framework = arduino
board_build.partitions = huge_app.csv
board_build.arduino.memory_type = qio_opi
build_flags =
    -DBOARD_HAS_PSRAM
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1
monitor_speed = 115200
lib_deps =
    adafruit/Adafruit SSD1306
    adafruit/Adafruit GFX Library
    adafruit/Adafruit LIS3DH
    adafruit/Adafruit Unified Sensor
    knolleary/PubSubClient
    bblanchon/ArduinoJson
```

### Arduino IDE Settings
- Board: XIAO_ESP32S3
- PSRAM: OPI PSRAM
- USB Mode: Hardware CDC and JTAG
- Partition Scheme: Huge APP

---

## Local Server Setup

### Minio (S3-compatible object storage)

```bash
# Start Minio
minio server ~/minio-data --console-address ":9001"

# Console: http://localhost:9001
# Default credentials: minioadmin / minioadmin
# Create bucket: raw-audio
```

### Mosquitto (MQTT broker)

```bash
# Install and start
brew install mosquitto
mosquitto -v

# Subscribe to all device status
mosquitto_sub -t "michelin/#" -v
```

### Python Processing Pipeline

```bash
pip install minio faster-whisper

# Watch bucket and process new WAV files
python3 tools/minio_watcher.py \
  --endpoint localhost:9000 \
  --bucket raw-audio \
  --access-key minioadmin \
  --secret-key minioadmin
```

---

## Server-Side Tools

### extract_btle_chunk.py
```bash
python3 tools/extract_btle_chunk.py mic1_table1_2024-01-01_12-00-00_part1.wav
```
Outputs JSON with device_id, waiter_name, and 60 BLE+IMU snapshots.

### split_stereo.py
```bash
python3 tools/split_stereo.py mic1_table1_2024-01-01_12-00-00_part1.wav
```
Outputs `_waiter.wav` (left channel) and `_table.wav` (right channel).

### minio_watcher.py
Full pipeline: download → extract btle → split stereo → transcribe → store result JSON.

---

## Development Phases

### Phase 1 — Basic Recording (Week 1)
- [ ] Flash firmware, verify I2S recording with one mic
- [ ] Add second mic stereo, verify L/R channel separation in Audacity
- [ ] Implement 1-minute chunking with gapless double buffering
- [ ] Verify WAV files play correctly (stereo, 16kHz, 16-bit)

### Phase 2 — Connectivity (Week 1–2)
- [ ] Minio running on Mac, verify S3 upload from device
- [ ] Mosquitto running on Mac, verify MQTT publish
- [ ] File management: upload queue, retry, 24h retention, cleanup

### Phase 3 — BLE + Telemetry (Week 2)
- [ ] Burn beacons with final UUID/Major values (currently running in RAM)
- [ ] Verify ESP32 picks up beacons by UUID, reads Major correctly
- [ ] Walk between tables — verify dominant table detection
- [ ] Verify custom btle chunk: parse with extract_btle_chunk.py

### Phase 4 — Peripherals (Week 2–3)
- [ ] LIS3DH speech detection calibration
- [ ] OLED display all screens
- [ ] Haptic feedback all patterns
- [ ] Battery monitoring accuracy check

### Phase 5 — QR + Polish (Week 3)
- [ ] QR camera scanning and waiter pairing
- [ ] Deep sleep / wake on long press
- [ ] NVS config manager serial setup
- [ ] End-to-end: boot → QR pair → record → BLE tag → upload → transcribe

---

## Testing Checklist

- [ ] Stereo recording: left = waiter mic, right = table mic (verify in Audacity)
- [ ] Gapless chunking: no audio gaps between consecutive WAV files
- [ ] WAV valid: plays in VLC, Audacity, ffprobe shows correct format
- [ ] btle chunk: Python extractor parses all 60 snapshots correctly
- [ ] Minio upload: files appear in bucket within 5s of chunk completion
- [ ] MQTT telemetry: status messages arrive every 60 seconds
- [ ] BLE scanning: all 5 beacons detected with correct Major values
- [ ] Dominant table: correct table assigned after 10+ seconds at that table
- [ ] OLED: all screens render correctly at 64×32
- [ ] Button: single/double/triple/long press reliably distinguished
- [ ] QR scanning: camera activates, scans badge, updates waiter name
- [ ] Haptic: all patterns distinguishable by feel
- [ ] Battery: percentage roughly matches actual charge level
- [ ] Deep sleep: current draw <20μA, wake on button works
- [ ] 8-hour endurance: records full shift without crash or SD corruption
- [ ] WiFi reconnect: device recovers from dropout, uploads backlog
- [ ] SD card full: device handles gracefully (deletes oldest uploaded files)

---

## Security

| Layer | Method |
|---|---|
| SD card | AES-256 (ESP32-S3 hardware accelerator) |
| WiFi transfer | HTTPS/TLS to Minio (HTTP for prototype) |
| MQTT telemetry | TLS + username/password (plain for prototype) |
| Minio storage | Server-side encryption (SSE) |
| Transcription | 100% local — audio never leaves the building |

**POPIA (South Africa):** Data stays on-premises, encrypted at rest and in transit, no cloud processing required.

---

## Key References

| Resource | URL |
|---|---|
| Base firmware | `github.com/NamanKansal230505/AI-Voice-Recorder-AWS-S3` |
| XIAO ESP32-S3 Sense wiki | `wiki.seeedstudio.com/xiao_esp32s3_getting_started/` |
| ICS-43434 datasheet | Via Adafruit product 6049 |
| LIS3DH datasheet | Via Adafruit product 2809 |
| DFRobot BLE Beacon wiki | `wiki.dfrobot.com/tel0149` |
| Beacon config tool | NanoBeaconConfigTool V3.2.11 |
| Device enclosure base | `printables.com/model/1243157` |
| Beacon enclosure | `printables.com/model/767212` |
| TolqAI (competitor) | `tolqai.com` |
| Michelin platform (live) | `michelin.hostingoz.dev/login` |

---

## Team

| Person | Role |
|---|---|
| Gustav | Hardware/firmware lead |
| Noel | Dev |
| Ashley Anthony | CEO, Isazi Consulting |
| Bruce Watermeyer | Product advisor |
