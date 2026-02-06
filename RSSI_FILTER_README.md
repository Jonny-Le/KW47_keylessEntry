# RSSI Filtering Algorithm — KW47 Keyless Entry

## Overview

The RSSI filter module (`rssi_filter.h` / `rssi_filter.c`) processes raw BLE Received Signal Strength Indicator (RSSI) readings from a connected phone and determines whether the device is close enough to trigger an unlock. The algorithm is designed for the **NXP KW47B42ZB7** (Cortex-M33) and follows **MISRA C:2012** / **AUTOSAR** coding guidelines.

All arithmetic uses **Q4 fixed-point** (1 LSB = 1/16 dB). No floating-point. No dynamic memory. Deterministic execution time.

---

## Pipeline

```
Raw RSSI (dBm)
      │
      ▼
┌──────────────┐
│  1. Hampel    │  Outlier rejection — median + MAD in a sliding time window
│     Filter    │  Rejects spikes > K × 1.5 × MAD from the window median
└──────┬───────┘
       │ clean sample (Q4)
       ▼
┌──────────────┐
│  2. Adaptive  │  Exponential Moving Average with time-adaptive alpha
│     EMA       │  Fast alpha for stale samples, slow alpha for rapid bursts
└──────┬───────┘
       │ smoothed value (Q4)
       ▼
┌──────────────┐
│  3. Feature   │  Over a 2-second sliding window of smoothed values:
│  Extraction   │  • Standard deviation (Q4)
│               │  • Percent of samples above enter threshold (Q15)
│               │  • Min / Max / Mean
└──────┬───────┘
       │ features
       ▼
┌──────────────┐
│  4. State     │  Proximity state machine with hysteresis,
│  Machine      │  exit confirmation timer, and post-unlock lockout
└──────────────┘
       │
       ▼
  State + Event
```

---

## Stage Details

### Stage 1: Hampel Filter (Outlier Rejection)

| Parameter | Value | Macro |
|-----------|-------|-------|
| Time window | 800 ms | `RSSI_HAMPEL_WIN_MS` |
| Threshold multiplier K | 3.0 (Q4 = 48) | `RSSI_HAMPEL_K_Q4` |
| MAD floor | 0.5 dB (Q4 = 8) | `RSSI_MAD_EPS_Q4` |

Collects raw samples within the time window, computes the **median** and **Median Absolute Deviation (MAD)** via insertion sort on a scratch buffer. If the new sample deviates from the median by more than `K × 1.4826 × MAD`, it is replaced with the median.

The MAD floor prevents the filter from becoming overly sensitive when the signal is perfectly stable (MAD ≈ 0).

### Stage 2: Adaptive EMA (Smoothing)

| Parameter | Value | Macro |
|-----------|-------|-------|
| Alpha min (dt ≤ 50 ms) | ~0.10 (Q15 = 3277) | `RSSI_EMA_ALPHA_MIN_Q15` |
| Alpha max (dt ≥ 500 ms) | ~0.80 (Q15 = 26214) | `RSSI_EMA_ALPHA_MAX_Q15` |
| Anomaly threshold | 2000 ms | `RSSI_EMA_ANOMALY_DT_MS` |

Alpha is linearly interpolated between min and max based on the time delta between samples:
- **Short dt** (rapid bursts) → low alpha → heavy smoothing
- **Long dt** (stale data) → high alpha → fast adaptation
- **dt > 2 s** → full EMA reset to the new sample (anomaly recovery)

Formula: `ema = ema + alpha × (sample - ema)`, all in Q4/Q15 fixed-point.

### Stage 3: Feature Extraction

| Parameter | Value | Macro |
|-----------|-------|-------|
| Statistics window | 2000 ms | `RSSI_FEAT_WIN_MS` |
| Minimum samples | 6 | `RSSI_MIN_FEAT_SAMPLES` |

Operates on the **smoothed ring buffer**. Computes over all samples within the last 2 seconds:
- **Standard deviation** (Q4) — via integer square root of variance
- **Percent above enter threshold** (Q15) — fraction of samples ≥ -50 dBm
- **Min, Max, Mean** (Q4)

These features feed the stability gate in the state machine.

### Stage 4: State Machine

```
          ┌───────────────────────────────┐
          │            IDLE               │
          │     (no signal received)      │
          └──────────────┬────────────────┘
                         │ first valid EMA
                         ▼
          ┌───────────────────────────────┐
     ┌───►│         FAR (Locked)          │◄────────────┐
     │    │    filtered < -50 dBm         │             │
     │    └──────────────┬────────────────┘             │
     │                   │ filtered ≥ -50 dBm           │
     │                   │ + CandidateStarted event     │
     │                   ▼                              │
     │    ┌───────────────────────────────┐             │
     │    │      CANDIDATE (Approach)     │             │
     │    │                               │             │
     │    │  Stability gate:              │  exit confirm│
     │    │  • stdDev < 2.5 dB            │  (1.5s below │
     │    │  • pctAbove ≥ 50%             │   -60 dBm)  │
     │    │  • hold for 2s                │             │
     │    └───────┬───────────────────────┘             │
     │            │ stable 2s                           │
     │            │ + UnlockTriggered event              │
     │            ▼                                     │
     │    ┌───────────────────────────────┐             │
     │    │     UNLOCKED (Lockout)        │─────────────┘
     │    │                               │  after 5s lockout
     │    │  5-second lockout period      │  + 1.5s exit confirm
     │    │  prevents immediate re-lock   │
     │    └───────────────────────────────┘
```

| Parameter | Value | Macro |
|-----------|-------|-------|
| Enter threshold (near) | -50 dBm | `RSSI_ENTER_NEAR_DBM` |
| Exit threshold (far) | -60 dBm | `RSSI_EXIT_NEAR_DBM` |
| Stability: max std dev | 2.5 dB (Q4 = 40) | `RSSI_STD_TH_Q4` |
| Stability: min pct above | 50% (Q15 = 16384) | `RSSI_PCT_TH_Q15` |
| Stability hold time | 2000 ms | `RSSI_STABLE_MS` |
| Exit confirmation | 1500 ms | `RSSI_EXIT_CONFIRM_MS` |
| Post-unlock lockout | 5000 ms | `RSSI_LOCKOUT_MS` |

**Hysteresis**: The 10 dB gap between enter (-50) and exit (-60) prevents flip-flopping when the signal hovers near a single threshold.

**Exit confirmation**: When in CANDIDATE, the signal must remain below -60 dBm for 1.5 continuous seconds before reverting to FAR. Brief dips are ignored.

**Lockout**: After an unlock event, the state machine stays in UNLOCKED for 5 seconds regardless of signal changes. This prevents rapid lock/unlock cycling (e.g., when a user opens a car door and the signal briefly drops).

#### Events

| Event | Trigger |
|-------|---------|
| `CandidateStarted` | FAR → CANDIDATE transition |
| `UnlockTriggered` | CANDIDATE → UNLOCKED (stability confirmed) |
| `ExitToFar` | CANDIDATE/UNLOCKED → FAR (exit confirmed) |

---

## API

```c
void        RssiFilter_Init(rssiFilter_t *pFilter);
void        RssiFilter_AddMeasurement(rssiFilter_t *pFilter, int8_t rssi);
int8_t      RssiFilter_GetFilteredRssi(const rssiFilter_t *pFilter);
rssiState_t RssiFilter_GetState(const rssiFilter_t *pFilter);
bool_t      RssiFilter_HasStateChanged(rssiFilter_t *pFilter);
void        RssiFilter_Reset(rssiFilter_t *pFilter);
void        RssiFilter_GetFeatures(const rssiFilter_t *pFilter,
                                   uint16_t *pStdQ4, uint8_t *pPctAbove, int8_t *pMean);
rssiEvent_t RssiFilter_GetLastEvent(const rssiFilter_t *pFilter);
```

All functions are NULL-safe. `HasStateChanged` clears the flag on read (read-once semantics). `GetLastEvent` returns the last non-None event and persists until a new event fires.

---

## Memory Layout

The `rssiFilter_t` struct is fully self-contained with no heap allocations:

| Field | Size | Purpose |
|-------|------|---------|
| `raw` ring buffer | 32 entries × (4B time + 1B rssi) = ~160 B | Hampel input window |
| `smooth` ring buffer | 40 entries × (4B time + 2B Q4) = ~240 B | Feature extraction window |
| `tmpA`, `tmpB` scratch | 32 × 2B × 2 = 128 B | Hampel sort buffers |
| `tmpS` scratch | 40 × 2B = 80 B | Feature extraction sort |
| EMA + state machine | ~40 B | Scalars |
| **Total** | **~650 bytes** | Stack-allocated, deterministic |

---

## Running Unit Tests

### Prerequisites

Any C11-compatible compiler on the host machine (e.g., `cc`, `gcc`, `clang`). No embedded toolchain needed — tests mock all SDK dependencies.

### Compile

```bash
cc -std=c11 -Wall -Wextra \
   -I tests/mocks \
   -I /path/to/digital_key_car_anchor_cs \
   -o tests/test_rssi_filter \
   tests/test_rssi_filter.c
```

Replace `/path/to/digital_key_car_anchor_cs` with the actual path to the directory containing `rssi_filter.h` and `rssi_filter.c`. For example:

```bash
cc -std=c11 -Wall -Wextra \
   -I tests/mocks \
   -I ~/mcuxpresso-sdk/mcuxsdk/middleware/wireless/bluetooth/examples/digital_key_car_anchor_cs \
   -o tests/test_rssi_filter \
   tests/test_rssi_filter.c
```

### Run

```bash
# Console only
./tests/test_rssi_filter

# Console + log file
./tests/test_rssi_filter --log tests/test_results.log

# Console + JUnit XML (for CI)
./tests/test_rssi_filter --xml tests/test_results.xml

# Both
./tests/test_rssi_filter --xml tests/test_results.xml --log tests/test_results.log
```

### Mock Headers

Located in `tests/mocks/`:

| File | What it stubs |
|------|---------------|
| `EmbeddedTypes.h` | `bool_t`, `TRUE`/`FALSE`, `uint8_t`, etc. |
| `fsl_component_timer_manager.h` | `TM_GetTimestamp()` — returns a mock clock controlled by `MockTimer_Set()` / `MockTimer_Advance()` |
| `FunctionLib.h` | `FLib_MemSet()` — wraps `memset` |

### Test Coverage (25 tests)

| Category | Tests |
|----------|-------|
| Init & Reset | 2 |
| NULL safety | 1 |
| Hampel filter | 2 |
| EMA smoothing | 3 |
| Feature extraction | 2 |
| State transitions | 4 |
| Exit confirmation | 2 |
| Lockout | 2 |
| Hysteresis / stability | 2 |
| Events & flags | 3 |
| Edge cases | 2 |

---

## Tuning Guide

All tunable parameters are `#define` constants in `rssi_filter.h`. Here's what each one does and how to adjust it.

### Thresholds (most likely to need tuning)

| Macro | Default | Effect |
|-------|---------|--------|
| `RSSI_ENTER_NEAR_DBM` | -50 dBm | RSSI must exceed this to enter CANDIDATE. **Lower** (e.g., -60) for longer range. **Raise** (e.g., -40) for shorter range. |
| `RSSI_EXIT_NEAR_DBM` | -60 dBm | RSSI must drop below this (for 1.5s) to re-lock. Must be lower than enter threshold. The gap between enter and exit is the **hysteresis band** — wider gap = less flip-flopping. |

### Timing

| Macro | Default | Effect |
|-------|---------|--------|
| `RSSI_STABLE_MS` | 2000 ms | How long RSSI must be stable in CANDIDATE before unlock fires. **Increase** for more cautious unlocking. **Decrease** for faster response. |
| `RSSI_EXIT_CONFIRM_MS` | 1500 ms | How long RSSI must stay below exit threshold before locking. Prevents brief signal dips from triggering lock. |
| `RSSI_LOCKOUT_MS` | 5000 ms | Cooldown after unlock. Prevents rapid lock/unlock cycling. |

### Stability Gate

| Macro | Default | Effect |
|-------|---------|--------|
| `RSSI_STD_TH_Q4` | 40 (2.5 dB) | Max allowed standard deviation for unlock. **Lower** = stricter stability requirement. |
| `RSSI_PCT_TH_Q15` | 16384 (~50%) | Min fraction of samples above enter threshold. **Raise** toward 32767 (100%) for stricter. |

### Hampel Filter

| Macro | Default | Effect |
|-------|---------|--------|
| `RSSI_HAMPEL_WIN_MS` | 800 ms | Window for outlier detection. Wider = more robust but slower to adapt. |
| `RSSI_HAMPEL_K_Q4` | 48 (K=3.0) | Outlier threshold multiplier. **Lower** = more aggressive spike rejection. **Higher** = more permissive. |
| `RSSI_MAD_EPS_Q4` | 8 (0.5 dB) | MAD floor. Prevents over-sensitivity when signal is perfectly stable. |

### EMA Smoothing

| Macro | Default | Effect |
|-------|---------|--------|
| `RSSI_EMA_ALPHA_MIN_Q15` | 3277 (~0.10) | Alpha for rapid samples (dt ≤ 50 ms). Lower = heavier smoothing. |
| `RSSI_EMA_ALPHA_MAX_Q15` | 26214 (~0.80) | Alpha for stale samples (dt ≥ 500 ms). Higher = faster adaptation. |
| `RSSI_EMA_ANOMALY_DT_MS` | 2000 ms | If no sample for this long, EMA resets completely. |

### Quick Recipes

- **More responsive unlock** (faster, less safe): Lower `RSSI_STABLE_MS` to 1000, raise `RSSI_STD_TH_Q4` to 64
- **More conservative unlock** (slower, safer): Raise `RSSI_STABLE_MS` to 3000, lower `RSSI_STD_TH_Q4` to 24, raise `RSSI_PCT_TH_Q15` to 24576
- **Longer range**: Lower `RSSI_ENTER_NEAR_DBM` to -60, lower `RSSI_EXIT_NEAR_DBM` to -70
- **Shorter range**: Raise `RSSI_ENTER_NEAR_DBM` to -40, raise `RSSI_EXIT_NEAR_DBM` to -50

---

## Known Limitations / Tech Debt

- **RSSI has not been converted to distance and/or calibrated yet.** The current thresholds (-50 dBm / -60 dBm) are empirical estimates. A proper path-loss model (e.g., log-distance with per-environment calibration) has not been implemented. The thresholds will need to be re-tuned after distance calibration.
- Hampel sort buffer capacity is fixed at compile time (`RSSI_RAW_CAP = 32`).
- No runtime parameter tuning — all thresholds are `#define` constants.
- Test coverage does not include multi-anchor or multi-phone scenarios.
- No power management optimization (filter runs on every sample regardless of state).

---

## Files

| File | Description |
|------|-------------|
| `rssi_filter.h` | Public API, types, all `#define` configuration |
| `rssi_filter.c` | Full pipeline implementation (~460 lines) |
| `rssi_integration.c` | Glue between BLE stack callbacks and filter module |
| `tests/test_rssi_filter.c` | 25 unit tests with JUnit XML + log output |
| `tests/mocks/` | Mock headers for host-side compilation |
