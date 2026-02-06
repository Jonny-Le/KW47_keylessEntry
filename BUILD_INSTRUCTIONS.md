# Build Instructions for KW47 Keyless Entry

## Prerequisites

- **MCUXpresso IDE** (v11.8+) with KW47 SDK installed
- **KW47-LOC** evaluation board (uses onboard DAPLink/CMSIS-DAP debug probe — **not J-Link**)
- USB cable for board connection

## Method 1: MCUXpresso IDE (Recommended)

### 1. Import Project
1. Open MCUXpresso IDE
2. File → Import → General → Existing Projects into Workspace
3. Browse to this project's root directory
4. Select the `freertos` folder
5. Click Finish

### 2. Configure SDK Path
1. Right-click on `kw47_keyless_entry_freertos` project → Properties
2. Under C/C++ Build → MCU Settings, verify the SDK path points to your MCUXpresso SDK installation

### 3. Build Project
1. Right-click on `kw47_keyless_entry_freertos` project
2. Select "Build Project"
3. Wait for build to complete

### 4. Flash to KW47-LOC
1. Connect KW47-LOC board via USB (the onboard DAPLink debugger will enumerate)
2. Right-click project → "Debug As" → "MCUXpresso IDE LinkServer"
3. Select the KW47-LOC target
4. Click "OK" to flash and start debugging

> **Note:** The KW47-LOC board uses an onboard CMSIS-DAP/DAPLink debug probe, not J-Link. Use **LinkServer** (bundled with MCUXpresso IDE) for flashing and debugging.

## Method 2: Command Line (Advanced)

### Prerequisites
```bash
# Set environment variables (adjust paths for your system)
export SDK_ROOT=<path-to-your-mcuxpresso-sdk>/mcuxsdk
export ARMGCC_DIR=<path-to-arm-none-eabi-gcc>
```

### Build Commands
```bash
cd <project-root>/freertos

# Configure with ARM GCC
cmake -B build -S . \
    -DSdkRootDirPath=$SDK_ROOT \
    -Dboard=kw47loc \
    -Dcore_id=cm33_core0 \
    -DTOOLCHAIN=armgcc \
    -DCMAKE_TOOLCHAIN_FILE=$SDK_ROOT/cmake/toolchain/arm-none-eabi-gcc.cmake

# Build
cmake --build build --config Debug
```

### Flash with LinkServer
```bash
# Flash using NXP LinkServer (installed with MCUXpresso IDE)
LinkServer flash KW47B42ZB7:KW47-LOC load build/debug/kw47_keyless_entry_kw47loc.bin
```

## Testing

### 1. Serial Output
- Connect to the KW47-LOC debug serial port (115200 baud, 8N1)
- On macOS: `screen /dev/tty.usbmodem* 115200`
- On Linux: `screen /dev/ttyACM0 115200`
- Monitor state transitions:
```
Proximity State: Disconnected -> Monitoring
RSSI: -78 dBm (Filtered: -80 dBm)
Proximity State: Monitoring -> Approach
RSSI: -62 dBm (Filtered: -64 dBm)
Proximity State: Approach -> Proximity
Stationary for 2000ms - UNLOCK
Proximity State: Proximity -> Unlock
```

### 2. BLE Testing
1. Use a phone BLE scanner app (e.g., nRF Connect)
2. Look for "Digital Key" advertisement
3. Approach the board to test RSSI thresholds
4. Stay stationary for 2 seconds to trigger unlock

### 3. Unit Tests (Host-Side)
```bash
cd <project-root>/tests
gcc -o test_rssi_filter test_rssi_filter.c -I mocks/ -lm
./test_rssi_filter
```

## Troubleshooting

### Build Issues
- Ensure MCUXpresso SDK path is correct and contains KW47 board support
- Check ARM GCC toolchain installation (`arm-none-eabi-gcc --version`)
- Verify the SDK includes the wireless/bluetooth middleware

### Flash Issues
- Ensure the KW47-LOC board enumerates as a USB device (DAPLink)
- Use LinkServer (not J-Link) — the onboard debug probe is CMSIS-DAP
- Try pressing the reset button on the board
- Check `LinkServer probes` to verify the board is detected

### Runtime Issues
- Check serial output for errors
- Verify BLE advertisement is active
- Test with different phone positions

## Project Structure

```
kw47_keyless_entry/
├── CMakeLists.txt                  # Main build config
├── freertos/                       # FreeRTOS build variant (primary build target)
│   ├── CMakeLists.txt
│   ├── prj.conf
│   └── example.yml
├── kw47_keyless_entry/             # Custom source code
│   ├── rssi_filter.c/h            # RSSI filtering (moving avg + Kalman)
│   └── proximity_state_machine.c/h # Tiered proximity state machine
├── digital_key_car_anchor_cs/      # Original NXP example (reference)
├── board_files/                    # KW47-LOC board configuration
├── libs/                           # SDK dependency snapshots (for reference)
├── tests/                          # Host-side unit tests
└── docs (*.md)                     # Documentation
```
