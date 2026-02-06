# ProxRssi — BLE RSSI Proximity Filter

## Overview

The `ProxRssi` module (`ProxRssi.h` / `ProxRssi.c`) processes raw BLE RSSI readings from a connected phone and determines whether the device is close enough to trigger an unlock. Designed for the **NXP KW47B42ZB7** (Cortex-M33), safety-first (automotive-friendly), speed second.

All arithmetic uses **Q4 fixed-point** (1 LSB = 1/16 dB). No floating-point. No dynamic memory. Deterministic execution time. AUTOSAR-style types (`uint8`, `sint16`, `Std_ReturnType`, etc.).

---

## Pipeline

```
Raw RSSI (dBm)
      │  ProxRssi_PushRaw()
      ▼
┌──────────────┐
│  1. Hampel    │  Outlier rejection — median + MAD in a sliding time window
│     Filter    │  Rejects spikes > K × 1.5 × MAD from the window median
└──────┬───────┘
       │ clean sample (Q4)
       ▼
┌──────────────┐
│  2. Adaptive  │  Exponential Moving Average with LUT-based alpha
│     EMA       │  Alpha indexed by dt (ms), Q15 fixed-point
└──────┬───────┘
       │ smoothed value (Q4)
       ▼
┌──────────────┐
│  3. Feature   │  Over a 2-second sliding window of smoothed values:
│  Extraction   │  • Standard deviation (Q4)
│               │  • Percent of samples above enter threshold (Q15)
│               │  • Min / Max
└──────┬───────┘
       │ features
       ▼
┌──────────────┐
│  4. State     │  Proximity state machine with hysteresis,
│  Machine      │  exit confirmation timer, and post-unlock lockout
└──────────────┘
       │  ProxRssi_MainFunction() returns:
       ▼
  ProxRssi_EventType + ProxRssi_FeaturesType
```

---

## Stage Details

### Stage 1: Hampel Filter (Outlier Rejection)

| Parameter | Field | Default |
|-----------|-------|---------|
| Spike window | `wSpikeMs` | 800 ms |
| Threshold multiplier K | `hampelKQ4` | 48 (K=3.0) |
| MAD floor | `madEpsQ4` | 8 (0.5 dB) |

Collects raw samples within the time window, computes the **median** and **Median Absolute Deviation (MAD)** via insertion sort on a scratch buffer. If the latest sample deviates from the median by more than `K × 1.5 × MAD`, it is replaced with the median.

The MAD floor prevents the filter from becoming overly sensitive when the signal is perfectly stable (MAD ≈ 0).

### Stage 2: Adaptive EMA (Smoothing)

| Parameter | Field | Default |
|-----------|-------|---------|
| Alpha LUT | `alphaQ15[]` | Caller-provided, indexed by dt (ms) |
| Anomaly threshold | `maxReasonableDtMs` | 2000 ms |

Alpha is looked up from a caller-provided LUT indexed by the time delta (ms) between samples:
- **Short dt** (rapid bursts) → low alpha → heavy smoothing
- **Long dt** (stale data) → high alpha → fast adaptation
- **dt > maxReasonableDtMs** → full EMA reset (anomaly recovery)

Formula: `ema = ema + alpha × (sample - ema)`, all in Q4/Q15 fixed-point.

### Stage 3: Feature Extraction

| Parameter | Field | Default |
|-----------|-------|---------|
| Statistics window | `wFeatMs` | 2000 ms |
| Minimum samples | `minFeatSamples` | 6 |

Operates on the **smoothed ring buffer**. Computes over all samples within the window:
- **Standard deviation** (Q4) — via integer square root of variance (64-bit intermediate)
- **Percent above enter threshold** (Q15)
- **Min, Max, Last** (Q4)

### Stage 4: State Machine

```
          ┌───────────────────────────────┐
     ┌───►│            FAR                │◄────────────┐
     │    │    filtered < enterNearQ4     │             │
     │    └──────────────┬────────────────┘             │
     │                   │ filtered ≥ enterNearQ4       │
     │                   │ + CANDIDATE_STARTED event    │
     │                   ▼                              │
     │    ┌───────────────────────────────┐             │
     │    │         CANDIDATE             │             │
     │    │                               │  exit confirm│
     │    │  Stability gate:              │  (1.5s below │
     │    │  • pctAbove ≥ pctThQ15        │  exitNearQ4) │
     │    │  • stdDev ≤ stdThQ4           │             │
     │    │  • hold for stableMs          │             │
     │    └───────┬───────────────────────┘             │
     │            │ stable                              │
     │            │ + UNLOCK_TRIGGERED event             │
     │            ▼                                     │
     │    ┌───────────────────────────────┐             │
     │    │         LOCKOUT               │─────────────┘
     │    │                               │  after lockoutMs
     │    │  Cooldown period              │  + exitConfirmMs
     │    │  prevents immediate re-trigger│
     │    └───────────────────────────────┘
```

| Parameter | Field | Default |
|-----------|-------|---------|
| Enter threshold | `enterNearQ4` | -50 dBm (Q4 = -800) |
| Exit threshold | `exitNearQ4` | -60 dBm (Q4 = -960) |
| Hysteresis | `hystQ4` | 10 dB (Q4 = 160) |
| Stability: max std dev | `stdThQ4` | 40 (2.5 dB) |
| Stability: min pct above | `pctThQ15` | 16384 (~50%) |
| Stability hold time | `stableMs` | 2000 ms |
| Exit confirmation | `exitConfirmMs` | 1500 ms |
| Post-unlock lockout | `lockoutMs` | 5000 ms |

#### Events

| Event | Trigger |
|-------|---------|
| `PROX_RSSI_EVT_CANDIDATE_STARTED` | FAR → CANDIDATE transition |
| `PROX_RSSI_EVT_UNLOCK_TRIGGERED` | CANDIDATE → LOCKOUT (stability confirmed) |
| `PROX_RSSI_EVT_EXIT_TO_FAR` | CANDIDATE/LOCKOUT → FAR (exit confirmed) |

---

## API

```c
/* Init: pass context, params struct, alpha LUT, and LUT length */
Std_ReturnType ProxRssi_Init(ProxRssi_CtxType* Ctx,
                             const ProxRssi_ParamsType* Params,
                             const uint16* AlphaQ15Lut,
                             uint32 AlphaLutLen);

/* Push a raw RSSI sample (call from worker thread, not ISR) */
Std_ReturnType ProxRssi_PushRaw(ProxRssi_CtxType* Ctx, uint32 tMs, sint8 rssiDbm);

/* Run the full pipeline: prune, Hampel, EMA, features, state machine */
Std_ReturnType ProxRssi_MainFunction(ProxRssi_CtxType* Ctx, uint32 nowMs,
                                     ProxRssi_EventType* Event,
                                     ProxRssi_FeaturesType* Features);

/* Force state to FAR and clear all buffers */
Std_ReturnType ProxRssi_ForceFar(ProxRssi_CtxType* Ctx);

/* Helpers */
sint16 ProxRssi_DbmToQ4(sint8 dbm);
sint16 ProxRssi_DbToQ4(sint16 db);
```

All functions are NULL-safe (return `E_NOT_OK`). `Features` pointer in `MainFunction` is optional (pass NULL to skip).

---

## Memory Layout

The `ProxRssi_CtxType` struct is fully self-contained with no heap allocations:

| Field | Size | Purpose |
|-------|------|---------|
| `raw` ring buffer | 128 entries × (4B time + 1B rssi) = ~640 B | Hampel input window |
| `smooth` ring buffer | 128 entries × (4B time + 2B Q4) = ~768 B | Feature extraction window |
| `tmpA`, `tmpB` scratch | 128 × 2B × 2 = 512 B | Hampel sort buffers |
| `tmpS` scratch | 128 × 2B = 256 B | Feature extraction sort |
| `alphaQ15` LUT | 1001 × 2B = ~2 KB | EMA alpha lookup |
| EMA + state machine | ~32 B | Scalars |
| **Total** | **~4.2 KB** | Stack-allocated, deterministic |

Buffer capacities are configurable at compile time: `PROX_RSSI_RAW_CAP`, `PROX_RSSI_SMOOTH_CAP`, `PROX_RSSI_ALPHA_LUT_MAX_MS`.

---

## Running Unit Tests

### Prerequisites

Any C11-compatible compiler on the host machine (e.g., `cc`, `gcc`, `clang`). No embedded toolchain or SDK dependencies needed — ProxRssi uses only `<stdint.h>` and `<stdbool.h>`.

### Compile

```bash
cc -std=c11 -Wall -Wextra \
   -I kw47_keyless_entry \
   -o tests/test_prox_rssi \
   tests/test_prox_rssi.c -lm
```

### Run

```bash
# Console only
./tests/test_prox_rssi

# Console + log file
./tests/test_prox_rssi --log tests/test_results.log

# Console + JUnit XML (for CI)
./tests/test_prox_rssi --xml tests/test_results.xml

# Both
./tests/test_prox_rssi --xml tests/test_results.xml --log tests/test_results.log
```

### Test Coverage (19 tests)

| Category | Tests |
|----------|-------|
| Init & NULL safety | 2 |
| PushRaw clamping | 1 |
| Hampel filter | 2 |
| EMA smoothing | 2 |
| Feature extraction | 1 |
| State transitions | 2 |
| Exit confirmation | 2 |
| Lockout | 2 |
| Hysteresis / stability | 2 |
| ForceFar | 1 |
| Full lifecycle | 1 |
| Q4 conversions | 1 |

---

## Tuning Guide

All tunable parameters are fields of `ProxRssi_ParamsType`, passed at init time.

### Thresholds (most likely to need tuning)

| Field | Default | Effect |
|-------|---------|--------|
| `enterNearQ4` | -50 dBm | RSSI must exceed this to enter CANDIDATE. **Lower** for longer range. **Raise** for shorter range. |
| `exitNearQ4` | -60 dBm | RSSI must drop below this (for `exitConfirmMs`) to re-lock. The gap between enter and exit is the **hysteresis band**. |

### Timing

| Field | Default | Effect |
|-------|---------|--------|
| `stableMs` | 2000 ms | How long RSSI must be stable in CANDIDATE before unlock fires. |
| `exitConfirmMs` | 1500 ms | How long RSSI must stay below exit threshold before locking. |
| `lockoutMs` | 5000 ms | Cooldown after unlock. Prevents rapid re-triggering. |

### Stability Gate

| Field | Default | Effect |
|-------|---------|--------|
| `stdThQ4` | 40 (2.5 dB) | Max allowed std dev for unlock. **Lower** = stricter. |
| `pctThQ15` | 16384 (~50%) | Min fraction of samples above enter threshold. **Raise** for stricter. |

### Hampel Filter

| Field | Default | Effect |
|-------|---------|--------|
| `wSpikeMs` | 800 ms | Window for outlier detection. |
| `hampelKQ4` | 48 (K=3.0) | Outlier threshold multiplier. **Lower** = more aggressive rejection. |
| `madEpsQ4` | 8 (0.5 dB) | MAD floor. |

### Quick Recipes

- **More responsive unlock**: Lower `stableMs` to 1000, raise `stdThQ4` to 64
- **More conservative unlock**: Raise `stableMs` to 3000, lower `stdThQ4` to 24, raise `pctThQ15` to 24576
- **Longer range**: Lower `enterNearQ4` to `ProxRssi_DbmToQ4(-60)`, lower `exitNearQ4` accordingly
- **Shorter range**: Raise `enterNearQ4` to `ProxRssi_DbmToQ4(-40)`

---

## Known Limitations / Tech Debt

- **RSSI has not been converted to distance and/or calibrated yet.** Current thresholds are empirical estimates.
- Alpha LUT must be computed and passed by the caller at init time.
- Buffer capacities are fixed at compile time.
- No power management optimization (pipeline runs on every `MainFunction` call).
- RSSI proximity must NOT be the sole unlock criterion — always run a secure cryptographic handshake.

---

## Files

| File | Description |
|------|-------------|
| `kw47_keyless_entry/ProxRssi.h` | Public API, types, params struct, compile-time config |
| `kw47_keyless_entry/ProxRssi.c` | Full pipeline implementation (~580 lines) |
| `tests/test_prox_rssi.c` | 19 unit tests with JUnit XML + log output |
