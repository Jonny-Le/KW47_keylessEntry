# Build Instructions for KW47 Keyless Entry

## Method 1: MCUXpresso IDE (Recommended)

### 1. Import Project
1. Open MCUXpresso IDE
2. File → Import → General → Existing Projects into Workspace
3. Browse to: `/Users/jonnyle/CascadeProjects/kw47_keyless_entry`
4. Select the `freertos` folder
5. Click Finish

### 2. Build Project
1. Right-click on `kw47_keyless_entry_freertos` project
2. Select "Build Project"
3. Wait for build to complete

### 3. Flash to KW47-LOC
1. Connect KW47-LOC board via USB
2. Right-click project → "Debug As" → "MCU Debug"
3. Select "KW47-LOC" board
4. Click "Debug" to flash and run

## Method 2: Command Line (Advanced)

### Prerequisites
```bash
# Set environment variables
export SDK_ROOT=/Users/jonnyle/mcuxpresso-sdk
export ARMGCC_DIR=/opt/homebrew
```

### Build Commands
```bash
cd /Users/jonnyle/CascadeProjects/kw47_keyless_entry/freertos

# Build with ARM GCC
cmake -B build -S . \
    -DSdkRootDirPath=$SDK_ROOT \
    -Dboard=kw47loc \
    -Dcore_id=cm33_core0 \
    -DTOOLCHAIN=armgcc \
    -DCMAKE_TOOLCHAIN_FILE=$SDK_ROOT/cmake/toolchain/arm-none-eabi-gcc.cmake

# Build
cmake --build build --config Debug
```

### Flash with J-Link
```bash
# Flash using J-Link Commander
JLinkExe -device KW47B42ZB7 -if SWD -speed 4000
> loadfile build/kw47_keyless_entry_kw47loc.srec
> r
> g
> exit
```

## Testing

### 1. Serial Output
- Connect to KW47-LOC debug port (115200 baud, 8N1)
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
1. Use phone BLE scanner app
2. Look for "Digital Key" advertisement
3. Approach board to test RSSI thresholds
4. Stay stationary for 2 seconds to trigger unlock

### 3. Debug LEDs
- LED1: System active
- LED2: Device connected
- LED3: Proximity detected
- LED4: Unlock condition met

## Troubleshooting

### Build Issues
- Ensure MCUXpresso SDK path is correct
- Check ARM GCC toolchain installation
- Verify KW47-LOC board support files

### Flash Issues
- Check USB connection
- Verify board is in debug mode
- Try pressing reset button

### Runtime Issues
- Check serial output for errors
- Verify BLE advertisement is active
- Test with different phone positions

## Next Steps

1. **Test RSSI Detection**: Verify state transitions work
2. **Adjust Thresholds**: Modify RSSI values in `rssi_filter.h`
3. **Add Accelerometer**: Integrate FXLS8964AF for motion detection
4. **Enable Channel Sounding**: Set `gAppBtcsClient_d 1` in config
