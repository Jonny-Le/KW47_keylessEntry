# KW47 Keyless Entry System

## Overview

BLE-based passive keyless entry for the **NXP KW47-LOC** board. A phone connects to the KW47 anchor over BLE. The anchor continuously reads the phone's RSSI, filters it through a 4-stage signal processing pipeline (`ProxRssi`), and triggers an unlock when the phone is close and stable for 2 seconds. A 10 dB hysteresis band (-50 dBm enter / -60 dBm exit), 1.5-second exit confirmation timer, and 5-second post-unlock lockout prevent state flip-flopping.

All signal processing runs in **Q4 fixed-point** (1/16 dB resolution). No floating-point. No heap. Deterministic memory. AUTOSAR-style types.

---

## Architecture

```
  Phone (BLE)                       PC Terminal
                                   (screen 115200)
       │                                │
       │  BLE Connection (RSSI)         │  Serial Diagnostics
       ▼                                ▼
┌──────────────────────────────────────────────────────┐
│                  KW47B42ZB7 MCU                      │
│                  (Cortex-M33)                        │
│  ┌────────────────┐     ┌──────────────────────────┐ │
│  │  BLE 6.0 Radio │     │     USB Serial (VCOM)    │ │
│  └───────┬────────┘     └──────────────────────────┘ │
└──────────┼───────────────────────────────────────────┘
           │
           ▼
┌──────────────────────────────────────────────────────┐
│  BLE Stack (NXP SDK)                                 │
│  • GAP Interface — Gap_ReadRssi()                    │
│  • Connection Callback — gConnEvtRssiRead_c          │
└──────────┬───────────────────────────────────────────┘
           │  RSSI event (rssiDbm, timestamp)
           ▼
┌──────────────────────────────────────────────────────┐
│  Application Layer                                   │
│  • digital_key_car_anchor_cs.c                       │
│  • Shell commands: rssi / rssistop                   │
└──────────┬───────────────────────────────────────────┘
           │  ProxRssi_PushRaw(&ctx, tMs, rssiDbm)
           │  ProxRssi_MainFunction(&ctx, tMs, &ev, &feat)
           ▼
┌──────────────────────────────────────────────────────┐
│  ProxRssi Module (ProxRssi.c / .h)                   │
│                                                      │
│  Stage 1: Hampel Filter (outlier rejection)          │
│           Median + MAD, 800 ms window, K = 3.0       │
│                        ↓                             │
│  Stage 2: Adaptive EMA (smoothing)                   │
│           LUT-based alpha, Q15 fixed-point            │
│                        ↓                             │
│  Stage 3: Feature Extraction                         │
│           2 s window: StdDev, PctAbove, Min/Max       │
│                        ↓                             │
│  Stage 4: State Machine                              │
│           FAR → CANDIDATE → LOCKOUT                   │
│           Hysteresis, exit confirm, lockout           │
└──────────┬───────────────────────────────────────────┘
           │  ProxRssi_EventType (NONE / CANDIDATE_STARTED
           │    / UNLOCK_TRIGGERED / EXIT_TO_FAR)
           ▼
       Application logic (start secure handshake, etc.)
```

A full draw.io diagram is available at `architecture.drawio`.

---

## ProxRssi Pipeline

The `ProxRssi` module converts noisy BLE RSSI readings into a stable proximity decision. See `RSSI_FILTER_README.md` for full algorithmic details and a tuning guide.

### Pipeline Summary

| Stage | Purpose | Key Parameters |
|-------|---------|----------------|
| **1. Hampel** | Rejects single-sample spikes using median + MAD | Window 800 ms, K = 3.0 |
| **2. Adaptive EMA** | Smooths signal via LUT-based time-adaptive alpha | Alpha LUT Q15 |
| **3. Features** | Computes stability metrics over a 2 s window | StdDev, PctAbove |
| **4. State Machine** | Maps filtered RSSI + features to proximity state | See below |

### State Machine

| State | Meaning | Entry Condition |
|-------|---------|-----------------|
| **FAR** | Phone is far away | Init, or exit confirmed |
| **CANDIDATE** | Phone may be close, checking stability | Filtered RSSI >= -50 dBm |
| **LOCKOUT** | Unlock fired, cooldown active | Stable in CANDIDATE for 2 s |

**Anti-flip-flop protections:**
- **Hysteresis** — 10 dB gap between enter (-50 dBm) and exit (-60 dBm)
- **Exit confirmation** — signal must stay below -60 dBm for 1.5 s before returning to FAR
- **Lockout** — 5 s cooldown after unlock prevents rapid re-triggering

### Key Parameters

Configured via `ProxRssi_ParamsType` at init time:

```c
p.enterNearQ4    = ProxRssi_DbmToQ4(-50);  /* >= this to enter CANDIDATE */
p.exitNearQ4     = ProxRssi_DbmToQ4(-60);  /* < this for 1.5s to lock */
p.stableMs       = 2000u;                   /* Hold in CANDIDATE to unlock */
p.exitConfirmMs  = 1500u;                   /* Confirm exit duration */
p.lockoutMs      = 5000u;                   /* Post-unlock lockout */
p.pctThQ15       = 16384u;                  /* 50% of samples above enter */
p.stdThQ4        = 40u;                     /* Max 2.5 dB std dev for stability */
```

---

## Methods of Connection

### 1. BLE (Phone to KW47)

The KW47 runs as a **BLE Central** and connects to a phone acting as a peripheral (or vice versa depending on the Digital Key profile). Once connected, the anchor periodically reads the RSSI via `Gap_ReadRssi()`. Once a BLE connection is established, the KW47 automatically starts RSSI monitoring.

### 2. Serial Console (PC to KW47)

The KW47-LOC exposes a **USB VCOM** debug port for diagnostics and shell commands.

**Connection:**
```bash
# macOS
screen /dev/tty.usbmodem* 115200

# Linux
screen /dev/ttyACM0 115200

# Windows
# Use PuTTY or TeraTerm, COMx, 115200 8N1
```

**Serial settings:** 115200 baud, 8 data bits, no parity, 1 stop bit, no flow control.

**Shell commands:**
- `rssi` — Start RSSI monitoring and diagnostic printing
- `rssistop` — Stop RSSI monitoring

**State change output (immediate):**
```
[RSSI] *** STATE: FAR -> CANDIDATE  event=CANDIDATE_STARTED ***
[RSSI] *** STATE: CANDIDATE -> LOCKOUT  event=UNLOCK_TRIGGERED ***
```

### 3. DAPLink / SWD (Debugging)

The KW47-LOC uses an **onboard CMSIS-DAP/DAPLink debug probe** (not J-Link). Flash via **LinkServer** (bundled with MCUXpresso IDE).

See `FLASH_INSTRUCTIONS.md` for complete flashing procedures.

---

## Setup Instructions

### Prerequisites

- **MCUXpresso IDE** (v11.8+) with KW47 SDK installed
- **KW47-LOC** board connected via USB

### 1. Build Firmware

See `BUILD_INSTRUCTIONS.md` for full details.

### 2. Flash to KW47-LOC

See `FLASH_INSTRUCTIONS.md` for flashing via MCUXpresso IDE or LinkServer CLI.

### 3. Connect Serial Terminal

```bash
screen /dev/tty.usbmodem* 115200
```

### 4. Connect a Phone

1. Establish a BLE connection with the KW47-LOC board
2. Walk toward / away from the board
3. Watch state transitions on the serial console

### 5. Run Unit Tests (Host)

No embedded toolchain needed — tests use only `<stdint.h>` and `<stdbool.h>`.

```bash
cc -std=c11 -Wall -Wextra \
   -I kw47_keyless_entry \
   -o tests/test_prox_rssi \
   tests/test_prox_rssi.c -lm

./tests/test_prox_rssi --xml tests/test_results.xml --log tests/test_results.log
```

19 tests covering init, NULL safety, Hampel, EMA, features, state transitions, exit confirmation, lockout, hysteresis, ForceFar, full lifecycle, and Q4 conversions.

---

## File Structure

```
kw47_keyless_entry/
├── README.md                         # This file
├── RSSI_FILTER_README.md             # Detailed algorithm docs + tuning guide
├── BUILD_INSTRUCTIONS.md             # Build procedures
├── FLASH_INSTRUCTIONS.md             # Flashing procedures
├── architecture.drawio               # System architecture diagram
├── CMakeLists.txt                    # Build configuration
├── kw47_keyless_entry/               # Custom source code
│   ├── ProxRssi.c                    # 4-stage pipeline implementation (~580 lines)
│   └── ProxRssi.h                    # Public API, types, params struct
├── tests/
│   └── test_prox_rssi.c             # 19 unit tests (JUnit XML + log)
├── freertos/                         # FreeRTOS build variant
├── digital_key_car_anchor_cs/        # Original NXP example (reference)
├── board_files/                      # KW47-LOC board configuration
└── libs/                             # SDK dependency snapshots
```

---

## Known Limitations / Tech Debt

- **RSSI has not been converted to distance and/or calibrated.** Current thresholds (-50 / -60 dBm) are empirical. A path-loss model with per-environment calibration is needed.
- Alpha LUT must be computed and passed at init time — no built-in generation.
- Single-anchor only — no multi-anchor or multi-phone support yet.
- Channel Sounding (CS) not yet integrated.
- RSSI proximity must NOT be the sole unlock criterion — always run a secure cryptographic handshake.

---

## References

- [NXP KW47-LOC User Guide](https://www.nxp.com/docs/en/user-guide/UM11650.pdf)
- [Bluetooth Digital Key Standard](https://www.bluetooth.org/docman/handlers/downloaddoc.ashx?doc_id=549967)
- [MCUXpresso SDK](https://github.com/nxp-mcuxpresso/mcux-sdk)

## License

BSD-3-Clause (same as NXP SDK)
