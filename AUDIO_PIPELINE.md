# Audio Processing Pipeline — beacon_scanner

## Hardware
**Microphones:** Adafruit ICS-43434 (stereo pair)
- Mic 1 (SEL=GND) → **left channel** → table-facing (captures customer + ambient)
- Mic 2 (SEL=3V3) → **right channel** → waiter-facing (captures waiter speech)
- Both share DATA line (GPIO3) via I2S tri-state. BCLK=GPIO1, WS=GPIO2.
- 24-bit output, MSB-aligned in a 32-bit I2S frame (bottom 8 bits always zero)
- Sample rate: 16kHz (configurable via `sample_rate=` in config.txt)
- Output format: stereo 16-bit WAV, ~1.85 MB per 30s segment

---

## Pipeline Overview

```
ICS-43434 (24-bit, 32-bit frame)
        │
        ▼
[1] Extract & Normalise           ← both channels
    (int32_t)(rawBuf >> 8) / 256.0f
    → float, 16-bit scale (±32767)
        │
        ▼
[2] DC Blocker (~10Hz)            ← both channels
    Removes slow DC drift from
    vibration and movement
        │
        ▼
[3] 2-Pole High-Pass (~80Hz)      ← both channels
    Cascaded IIR — 40dB/decade rolloff
    Removes handling thuds and
    low-frequency mechanical noise
        │
        ▼
[4] Gain + Soft Limiter           ← both channels
    MIC_GAIN_L=4  /  MIC_GAIN_R=6
    Compresses above ±24000 at 15% slope
    Hard constrain at ±32767
        │
        ▼
int16_t stereo output → PSRAM buffer → WAV file
```

---

## Stage Details

### [1] Extract & Normalise
```cpp
float inL = (float)(int32_t)(rawBuf[i * 2 + 1] >> 8) / 256.0f;
float inR = (float)(int32_t)(rawBuf[i * 2]     >> 8) / 256.0f;
```
The ICS-43434 packs 24 valid bits into a 32-bit frame, MSB-aligned. Shifting right by 8 recovers all 24 bits as an int32_t. Dividing by 256.0f normalises to ±32767 scale while the float representation retains sub-LSB precision from the lower 8 bits — better noise floor at quiet levels.

**DMA slot mapping:** `rawBuf[i*2]` = R slot (mic2, SEL=3V3), `rawBuf[i*2+1]` = L slot (mic1, SEL=GND).

---

### [2] DC Blocker
```cpp
const float DC_ALPHA = 0.9992f;  // ~10Hz cutoff at 16kHz

float dcL = inL - dcPrevInL + DC_ALPHA * dcPrevOutL;
dcPrevInL = inL; dcPrevOutL = dcL; inL = dcL;
```
First-order IIR high-pass at ~10Hz. Removes slow DC drift caused by vibration coupling, movement, and temperature drift. Applied before the main HP filter so the HP filter sees a clean zero-mean signal.

**Cutoff formula:** `fc = (1 - DC_ALPHA) * fs / (2π)` ≈ 10Hz at 16kHz.

---

### [3] 2-Pole High-Pass Filter (~80Hz)
```cpp
const float HP_ALPHA = 0.9972f;  // ~80Hz cutoff at 16kHz

// Pole 1
float outL = HP_ALPHA * (hpPrevOutL + inL - hpPrevInL);
hpPrevInL = inL; hpPrevOutL = outL;

// Pole 2 (cascade)
float s2L = HP_ALPHA * (hp2PrevOutL + outL - hp2PrevInL);
hp2PrevInL = outL; hp2PrevOutL = s2L; outL = s2L;
```
Two first-order IIR high-pass filters in series — 40dB/decade rolloff. Frequencies below 80Hz are attenuated aggressively without affecting speech content.

**Why 80Hz:** Male speech fundamentals start at ~85Hz. Cutting at 80Hz removes almost all handling thuds while keeping all speech. Going higher starts to affect bass in male voices.

**Single-pole formula:** `y[n] = alpha * (y[n-1] + x[n] - x[n-1])`

---

### [4] Gain + Soft Limiter
```cpp
#define MIC_GAIN_L  4   // table-facing (ambient)
#define MIC_GAIN_R  6   // waiter-facing (slightly louder)

auto softClip = [](float x) -> int16_t {
  const float T = 24000.0f;
  if (x >  T) x =  T + (x - T) * 0.15f;
  if (x < -T) x = -T + (x + T) * 0.15f;
  return (int16_t)constrain((long)x, -32768, 32767);
};
audioBuf[i * 2]     = softClip(outL * MIC_GAIN_L);
audioBuf[i * 2 + 1] = softClip(outR * MIC_GAIN_R);
```
Both channels use identical processing — same pipeline, same limiter. R is just 50% louder (gain 6 vs 4) since the waiter mic is the primary speech channel.

Below ±24000 the signal passes through unchanged. Above ±24000 the excess is compressed at 15% slope — loud sounds squashed rather than hard-clipped. Hard constrain at ±32767 is the final safety net.

**Knee at ±24000** = 73% of full scale.

**Tuning gain:** Increase if recordings are too quiet. Decrease if waveform in Audacity is uniformly compressed (no visible dynamic variation between loud and quiet moments).

---

## Output Format
- **Sample rate:** 16000 Hz
- **Channels:** 2 (stereo — L=table, R=waiter)
- **Bit depth:** 16-bit signed PCM
- **Container:** WAV (RIFF) + custom `ble_` chunk appended after audio data
- **Segment duration:** 30s target
- **File size per segment:** ~1.85 MB (1.83 MB audio + 44B WAV header + ~1KB BLE chunk)

---

## Audacity Verification
1. Open WAV file in Audacity
2. Split stereo tracks
3. Both channels should show similar character — peaks during speech, lower amplitude during silence
4. Waveform should have visible dynamic variation — if uniformly compressed, lower gain
5. No flat-topped peaks — if present, gain is too high

---

## Filter State Variables
All state variables are local to `recordingTask()` and reset each time recording starts:
```cpp
float hpPrevInL,  hpPrevOutL;   // HP pole 1, left
float hpPrevInR,  hpPrevOutR;   // HP pole 1, right
float hp2PrevInL, hp2PrevOutL;  // HP pole 2, left
float hp2PrevInR, hp2PrevOutR;  // HP pole 2, right
float dcPrevInL,  dcPrevOutL;   // DC blocker, left
float dcPrevInR,  dcPrevOutR;   // DC blocker, right
```
State resets at recording start so filters don't carry over artefacts between sessions.
