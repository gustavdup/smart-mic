# CPU Load Tracking — Design & Bug Fix

## How it works

FreeRTOS idle hooks (`idleHook0`, `idleHook1`) increment a counter on each core whenever the scheduler has nothing to run. Every 500 ms, `updateDisplay()` samples those counters and computes a CPU load percentage:

```
cpu_load% = 100 - (idle_count / baseline * 100)
```

A high idle count → core was mostly sleeping → low load.
A low idle count → core was mostly busy → high load.

The counters are reset to zero after each sample so the next window starts fresh.

---

## The Bug (original code)

The original implementation derived the baseline from the **maximum idle count over the last 8 windows** (4 seconds at 500 ms/window):

```cpp
uint32_t base = max over s_idleHist[8]
int cpu% = 100 - (current / base * 100)
```

**This breaks under sustained load.** After ~4 seconds of continuous activity:

1. All 8 history slots fill with low idle counts (e.g. 5 ticks each)
2. `base` drops to 5 (the new "max")
3. `current` is also ~5
4. `100 - (5/5 * 100) = 0%` — display shows **0% CPU even when fully loaded**

This is the opposite of what you want and happens reliably during any recording session.

---

## The Fix (sticky baseline)

Replace the rolling-window max with a **high-water mark** that only ever increases:

```cpp
static uint32_t s_base0 = 1, s_base1 = 1;  // never decreases

// Calibrated once at end of setup():
s_base0 = max(1u, s_idle0);
s_base1 = max(1u, s_idle1);
```

**Why this works:**
- At boot, the ESP32 is idle for a few seconds while connecting to WiFi and initialising BLE. The idle hook fires freely and sets a high baseline.
- Once recording starts, `raw` drops but `s_base` stays at the boot-time idle high-water mark → CPU% reads correctly.
- If the board is ever idle again (between sessions), the baseline can only increase, never shrink.

**Constraint:** Requires at least a brief idle period at startup to calibrate. The existing startup sequence (WiFi connect + NTP sync takes several seconds) provides this naturally.

**Important:** `s_base` must NOT be updated inside `updateDisplay()`. In the first ~10 seconds after `loop()` starts, the FreeRTOS scheduler has transient idle bursts that exceed the calibration baseline. Allowing s_base to ratchet up during those windows inflates the baseline to a value that normal operation never reaches, causing the display to read ~0% CPU initially then jump to ~50% once the baseline stabilises. The calibration at the end of `setup()` is the sole source of truth.

---

## Race condition fix

The original code read and zeroed `s_idle0` in two non-atomic steps. The idle hook running on the other core could fire in between, losing one increment. Fixed with a brief interrupt-disable window:

```cpp
portDISABLE_INTERRUPTS();
uint32_t raw0 = s_idle0; s_idle0 = 0;
uint32_t raw1 = s_idle1; s_idle1 = 0;
portENABLE_INTERRUPTS();
```

On Xtensa, individual 32-bit loads/stores are single instructions (no torn reads), so this is only needed to make the read→zero pair atomic. The window is ~8 ns and only disables interrupts on the calling core.

---

## Core Pinning & Task Distribution

### What Runs Where

| Task | Core | Priority | Stack | Notes |
|------|------|----------|-------|-------|
| WiFi driver | 0 | 23 | — | Always Core 0, not configurable |
| lwIP TCP/IP | 0 | 18 | — | Handles TCP segmentation, ACKs, retransmits |
| NimBLE host | 0 | ~21 | — | Default `CONFIG_BT_NIMBLE_PINNED_TO_CORE = 0` |
| BLE controller (hardware) | 0 | hardwired | — | HCI events fed to NimBLE host |
| `loop()` | 1 | 1 | 32768 | Arduino main task (SET_LOOP_TASK_STACK_SIZE) |
| `recordingTask` | 1 | 2 | 12288 | I2S DMA + PSRAM writes — uncontested on Core 1 |
| `uploadTask` | 0 | 1 | 20480 | Co-located with WiFi/lwIP for best TCP throughput |

---

## Why Core 0 for Upload

During upload, BLE is off — Core 0 has only the WiFi driver and lwIP running. Placing `uploadTask` on the same core:
- Eliminates inter-core IPC for every TCP call (lwIP ↔ uploadTask handoff stays on Core 0)
- Lets recording on Core 1 run completely uncontested
- WiFi driver (priority 23) and lwIP (priority 18) naturally preempt `uploadTask` (priority 1) when needed — correct for a background upload

**Core 0 at 99% during upload is expected** — WiFi/BLE drivers are higher priority and still preempt uploadTask.

---

## BLE vs WiFi Load on Core 0

Core 0 alternates between two modes since WiFi and BLE are never concurrent in this firmware.

**BLE passive scanning** — light:
- `setActiveScan(false)`, `interval == window == 16`
- BLE controller handles RF in hardware; NimBLE host only processes HCI events
- Scan callback does a memcmp + queue push — Core 0 mostly idle between packets

**WiFi upload** — heavy:
- WiFi driver (priority 23) actively manages RF channel, beacons, AMPDU aggregation
- lwIP (priority 18) handles TCP segmentation, ACKs, retransmits for every streamed packet
- Core 0 is genuinely busy for the full upload duration

---

## I2S DMA — Core Pinning Irrelevant

The I2S DMA controller transfers audio buffers independently of which CPU core is running. Task priority matters more than core assignment — `recordingTask` at priority 2 ensures buffers are drained before overflow regardless of what else is on Core 1.

---

## Concurrent WiFi + BLE (not current design)

If ever needed, use ESP-IDF software coexistence (`CONFIG_SW_COEXIST_ENABLE`). Recommended scan parameters to give WiFi airtime: window 49 ms / interval 189 ms.

Known issue: concurrent WiFi HTTP + BLE scanning can cause a Core 1 panic due to NimBLE HCI buffer contention. The current sequential WiFi-off-during-BLE design avoids this entirely.
