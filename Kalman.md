# Plan: Kalman Filter for Real-Time BLE Distance Smoothing

## Context
BLE RSSI measurements are inherently noisy (±3–6 dBm typical), producing distance estimates that jump by 1–5m between readings even for a stationary beacon. A 1D Kalman filter applied per-beacon gives a principled, real-time smoothed distance estimate and automatically rejects outlier measurements via an innovation gate — no manual threshold tuning needed.

Particle filters are overkill: they shine for multi-dimensional position estimation with non-linear dynamics. For 1D distance from a single anchor, Kalman is the correct tool and runs in microseconds.

## Critical File
- `firmware/beacon_scanner.ino` (and its Arduino copy at `~/Documents/Arduino/beacon_scanner/beacon_scanner.ino`)

## Implementation

### 1. Add Kalman constants (near top, with other `#define`s)
```cpp
#define KF_Q  0.1f   // process noise variance — assumes ~0.3 m/s max movement
#define KF_R  2.0f   // measurement noise variance — BLE typically ±1.4m (σ²≈2)
```

### 2. Extend `BeaconState` struct
Add two fields; `distance` stays as raw for logging/comparison:
```cpp
struct BeaconState {
  bool     active;
  int      rssi;
  float    distance;    // raw path-loss estimate (unchanged)
  float    kf_dist;     // Kalman-filtered distance (use for display)
  float    kf_var;      // Kalman posterior variance (uncertainty)
  uint32_t lastSeenMs;
};
```

### 3. Kalman update in `loop()` BLE queue drain
Replace the existing single-line distance assignment + mutex block with:
```cpp
float dist = powf(10.0f, (b.txPower - b.rssi) / 25.0f);  // raw (unchanged)

if (xSemaphoreTake(beaconsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
  BeaconState& s = beacons[slot];
  s.active     = true;
  s.rssi       = b.rssi;
  s.distance   = dist;
  s.lastSeenMs = millis();

  if (!s.kf_var) {
    // First observation — initialise filter cold
    s.kf_dist = dist;
    s.kf_var  = 10.0f;  // high initial uncertainty
  } else {
    // Predict step
    float pred_var = s.kf_var + KF_Q;
    float innov    = dist - s.kf_dist;
    float innov_var = pred_var + KF_R;
    // Innovation gate: reject if measurement > 3σ from prediction
    if (fabsf(innov) <= 3.0f * sqrtf(innov_var)) {
      float K    = pred_var / innov_var;
      s.kf_dist  = s.kf_dist + K * innov;
      s.kf_var   = (1.0f - K) * pred_var;
    } else {
      s.kf_var = pred_var;  // outlier — advance uncertainty, skip update
    }
  }
  xSemaphoreGive(beaconsMutex);
}
```

### 4. Use `kf_dist` on OLED beacon list screen (Screen 2, `updateDisplay()`)
Change the display line from `s.distance` to `s.kf_dist`:
```cpp
snprintf(line, sizeof(line), "T%d %4ddBm %4.1fm %2lus",
         i + 1, s.rssi, s.kf_dist, (unsigned long)age);
```
Raw RSSI stays unchanged — useful to keep as ground truth.

### 5. BLE snapshot logging (`recordingTask`)
Store `kf_dist` in the snapshot instead of raw `distance` — this is the analytically meaningful value:
```cpp
snap.beacon[i].distance = beacons[i].kf_dist;
```

### 6. Reset filter on beacon going stale
When a beacon times out (goes stale / inactive), reset `kf_var = 0` so the next sighting re-initialises cold rather than picking up from stale state. Add to wherever staleness is detected — or simply check `kf_var` being nonzero as the "initialized" flag (already used above).

## Tuning Notes
| Parameter | Value | Effect of increasing |
|-----------|-------|----------------------|
| `KF_Q` | 0.1 | More responsive to movement, noisier |
| `KF_R` | 2.0 | More trust in model vs measurement |
| Gate (3σ) | 3σ | Wider gate accepts more measurements |

At 1 beacon/second update rate, `KF_Q=0.1` allows ~0.3m/s movement before the filter lags. Adjust if beacons move faster.

## Verification
1. Flash firmware, open Serial Monitor
2. Move a beacon toward/away — `kf_dist` should track smoothly, raw `distance` will jitter
3. Briefly occlude a beacon (spike in path loss) — innovation gate should discard the spike
4. Run `parse_ble.py` on a recorded segment — distances should be smoother than before
5. Let a beacon go out of range and come back — filter should re-initialise (kf_var reset)

## Notes
- No new libraries, no heap allocation — purely stack arithmetic
- `sqrtf()` is ~2µs on ESP32-S3 — negligible at 1Hz beacon rate
- The `distance` field is preserved as raw for debugging if needed
- Both firmware copies must be kept in sync (repo + Arduino IDE folder)
