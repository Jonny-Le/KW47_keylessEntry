# KW47 Keyless Entry System

## Overview

BLE-based passive keyless entry for the **NXP KW47-LOC** board. A phone running a BLE app (e.g., nRF Connect) connects to the KW47 anchor. The anchor continuously reads the phone's RSSI, filters it through a 4-stage signal processing pipeline, and triggers an unlock when the phone is close and stable for 2 seconds. A 10 dB hysteresis band, 1.5-second exit confirmation timer, and 5-second post-unlock lockout prevent state flip-flopping.

All signal processing runs in **Q4 fixed-point** (1/16 dB resolution). No floating-point. No heap. MISRA C:2012 compliant.

---

## Architecture

```
  iPhone / Phone                    PC Terminal
  (nRF Connect)                    (screen 115200)
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
           │  RSSI event
           ▼
┌──────────────────────────────────────────────────────┐
│  Application Layer                                   │
│  • digital_key_car_anchor_cs.c                       │
│  • Shell commands: rssi / rssistop                   │
└──────────┬───────────────────────────────────────────┘
           │  UpdateRssi(id, rssi)
           ▼
┌──────────────────────────────────────────────────────┐
│  RSSI Integration Layer (rssi_integration.c)         │
│  • Calls RssiFilter_AddMeasurement() on each sample  │
│  • Prints diagnostics every 5 samples                │
│  • Prints state changes + events immediately         │
└──────────┬───────────────────────────────────────────┘
           │  Raw RSSI (dBm)
           ▼
┌──────────────────────────────────────────────────────┐
│  RSSI Filter Module (rssi_filter.c / .h)             │
│                                                      │
│  Stage 1: Hampel Filter (outlier rejection)          │
│           Median + MAD, 800 ms window, K = 3.0       │
│                        ↓                             │
│  Stage 2: Adaptive EMA (smoothing)                   │
│           Time-scaled alpha 0.10–0.80, Q15           │
│                        ↓                             │
│  Stage 3: Feature Extraction                         │
│           2 s window: StdDev, PctAbove, Min/Max/Mean │
│                        ↓                             │
│  Stage 4: State Machine                              │
│           IDLE → FAR → CANDIDATE → UNLOCKED          │
│           Hysteresis, exit confirm, lockout           │
└──────────┬───────────────────────────────────────────┘
           │  State + Event + Filtered RSSI
           ▼
┌──────────────────────────────────────────────────────┐
│  Proximity State Machine (proximity_state_machine.c) │
│  Disconnected → Monitoring → Approach → Proximity    │
│  → Unlock                                            │
│  (Future: Channel Sounding ranging)                  │
└──────────────────────────────────────────────────────┘
```

A full draw.io diagram is available at `architecture.drawio`.

---

## RSSI Filter Pipeline

The filter converts noisy BLE RSSI readings into a stable proximity state. See `RSSI_FILTER_README.md` for full algorithmic details and a tuning guide.

### Pipeline Summary

| Stage | Purpose | Key Parameters |
|-------|---------|----------------|
| **1. Hampel** | Rejects single-sample spikes using median + MAD | Window 800 ms, K = 3.0 |
| **2. Adaptive EMA** | Smooths signal; adapts speed to sample rate | Alpha 0.10–0.80 (Q15) |
| **3. Features** | Computes stability metrics over a 2 s window | StdDev, PctAbove, Mean |
| **4. State Machine** | Maps filtered RSSI + features to proximity state | See below |

### State Machine

| State | Meaning | Entry Condition |
|-------|---------|-----------------|
| **IDLE** | No data yet | Power-on / reset |
| **FAR** (Locked) | Phone is far away | First valid EMA, or exit confirmed |
| **CANDIDATE** (Approach) | Phone may be close, checking stability | Filtered RSSI >= -60 dBm |
| **UNLOCKED** (Lockout) | Unlock fired, cooldown active | Stable in CANDIDATE for 2 s |

**Anti-flip-flop protections:**
- **Hysteresis** — 10 dB gap between enter (-60 dBm) and exit (-70 dBm)
- **Exit confirmation** — signal must stay below -70 dBm for 1.5 s before locking
- **Lockout** — 5 s cooldown after unlock prevents rapid re-lock

### Key Thresholds

All defined in `rssi_filter.h`:

```c
#define RSSI_ENTER_NEAR_DBM   (-60)    // >= this to enter CANDIDATE
#define RSSI_EXIT_NEAR_DBM    (-70)    // < this for 1.5s to lock
#define RSSI_STABLE_MS        (2000U)  // Hold in CANDIDATE to unlock
#define RSSI_EXIT_CONFIRM_MS  (1500U)  // Confirm exit duration
#define RSSI_LOCKOUT_MS       (5000U)  // Post-unlock lockout
```

---

## Methods of Connection

### 1. BLE (Phone to KW47)

The KW47 runs as a **BLE Central** and connects to a phone acting as a peripheral (or vice versa depending on the Digital Key profile). Once connected, the anchor periodically reads the RSSI via `Gap_ReadRssi()`.

**Phone setup:**
- Install **nRF Connect** (iOS/Android) or any BLE peripheral app
- Start advertising or look for the KW47's "Digital Key" advertisement
- Once connected, the KW47 automatically starts RSSI monitoring

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

**Diagnostic output (every 5 samples):**
```
[RSSI] raw=-58 filt=-57 std=1.2dB pct=88% state=CANDIDATE evt=NONE
```

**State change output (immediate):**
```
[RSSI] *** STATE: FAR -> CANDIDATE  event=CANDIDATE_STARTED ***
[RSSI] *** STATE: CANDIDATE -> LOCKOUT  event=UNLOCK_TRIGGERED ***
```

### 3. J-Link / SWD (Debugging)

For flashing and JTAG debugging:
```bash
JLinkExe -device KW47B42ZB7 -if SWD -speed 4000
```

See `FLASH_INSTRUCTIONS.md` for complete flashing procedures.

---

## Setup Instructions

### Prerequisites

- **MCUXpresso SDK** with KW47-LOC board support
- **ARM GCC toolchain** (arm-none-eabi-gcc) or MCUXpresso IDE
- **CMake** 3.22+
- **J-Link** or **MCUXpresso IDE** for flashing
- **KW47-LOC** board connected via USB

### 1. Clone / Obtain the SDK

```bash
# Set your SDK root
export SDK_ROOT=/path/to/mcuxpresso-sdk
```

The RSSI filter source files live at:
```
$SDK_ROOT/mcuxsdk/middleware/wireless/bluetooth/examples/digital_key_car_anchor_cs/
├── rssi_filter.h          # Filter configuration and API
├── rssi_filter.c          # Filter implementation (~460 lines)
├── rssi_integration.h     # Integration layer API
├── rssi_integration.c     # Glue between BLE stack and filter
├── proximity_state_machine.h/.c  # Higher-level proximity logic
└── digital_key_car_anchor_cs.c   # Main application
```

### 2. Build Firmware

**Option A: MCUXpresso IDE (Recommended)**
1. File -> Import -> Existing Projects into Workspace
2. Browse to the SDK example directory
3. Select the `digital_key_car_anchor_cs` project
4. Build (Ctrl+B / Cmd+B)

**Option B: Command Line**
```bash
export ARMGCC_DIR=/opt/homebrew  # or wherever arm-none-eabi-gcc lives

cmake -B build -S freertos \
    -DSdkRootDirPath=$SDK_ROOT \
    -Dboard=kw47loc \
    -Dcore_id=cm33_core0 \
    -DTOOLCHAIN=armgcc \
    -DCMAKE_TOOLCHAIN_FILE=$SDK_ROOT/cmake/toolchain/arm-none-eabi-gcc.cmake

cmake --build build --config Debug
```

See `BUILD_INSTRUCTIONS.md` for full details.

### 3. Flash to KW47-LOC

**MCUXpresso IDE:** Right-click project -> Debug As -> MCU Debug

**J-Link:**
```bash
JLinkExe -device KW47B42ZB7 -if SWD -speed 4000
> loadfile build/digital_key_car_anchor_cs.bin
> r
> g
> exit
```

See `FLASH_INSTRUCTIONS.md` for all flashing methods (LinkServer, OpenOCD, etc.).

### 4. Connect Serial Terminal

```bash
screen /dev/tty.usbmodem* 115200
```

### 5. Connect a Phone

1. Open nRF Connect on your phone
2. Scan for BLE devices
3. Connect to "Digital Key" advertisement
4. Walk toward / away from the KW47-LOC board
5. Watch state transitions on the serial console

### 6. Run Unit Tests (Host)

No embedded toolchain needed — tests mock all SDK headers.

```bash
cc -std=c11 -Wall -Wextra \
   -I tests/mocks \
   -I $SDK_ROOT/mcuxsdk/middleware/wireless/bluetooth/examples/digital_key_car_anchor_cs \
   -o tests/test_rssi_filter \
   tests/test_rssi_filter.c

# Run with JUnit XML + log
./tests/test_rssi_filter --xml tests/test_results.xml --log tests/test_results.log
```

25 tests covering Hampel, EMA, feature extraction, state machine, exit confirmation, lockout, hysteresis, events, and edge cases.

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
├── tests/
│   ├── test_rssi_filter.c            # 25 unit tests (JUnit XML + log)
│   └── mocks/
│       ├── EmbeddedTypes.h           # Mock NXP types
│       ├── fsl_component_timer_manager.h  # Mock timer
│       └── FunctionLib.h             # Mock FLib_MemSet
└── freertos/                         # FreeRTOS build files
```

**Source files** (in the NXP SDK tree):
```
digital_key_car_anchor_cs/
├── rssi_filter.h                     # Public API + all #define config
├── rssi_filter.c                     # 4-stage pipeline implementation
├── rssi_integration.h/.c             # BLE stack ↔ filter glue
├── proximity_state_machine.h/.c      # Higher-level proximity logic
└── digital_key_car_anchor_cs.c       # Main application entry
```

---

## Known Limitations / Tech Debt

- **RSSI has not been converted to distance and/or calibrated.** Current thresholds (-60 / -70 dBm) are empirical. A path-loss model with per-environment calibration is needed.
- No runtime parameter tuning — all thresholds are compile-time `#define` constants.
- Single-anchor only — no multi-anchor or multi-phone support yet.
- Channel Sounding (CS) not yet integrated.

---

## Future Work

- **Distance calibration** — map RSSI to physical distance with log-distance path-loss model
- **Channel Sounding** — enable `gAppBtcsClient_d` for PBR + RTT secure ranging
- **Multi-anchor** — Blue Ravens multi-anchor coordination
- **Accelerometer** — FXLS8964AF integration for motion detection
- **Attack detection** — NADM (Normalized Attack Detector Metric)

---

## References

- [NXP KW47-LOC User Guide](https://www.nxp.com/docs/en/user-guide/UM11650.pdf)
- [Bluetooth Digital Key Standard](https://www.bluetooth.org/docman/handlers/downloaddoc.ashx?doc_id=549967)
- [MCUXpresso SDK](https://github.com/nxp-mcuxpresso/mcux-sdk)

## License

BSD-3-Clause (same as NXP SDK)
