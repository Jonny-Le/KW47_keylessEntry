# Flash Instructions for KW47 Keyless Entry

## Build Status ✅

The project has been successfully built! The binary is ready at:
```
/Users/jonnyle/mcuxpresso-sdk/build/freertos_hello_cm33_core0.bin
```

## Flashing Methods

### Method 1: MCUXpresso IDE (Recommended)
1. Open MCUXpresso IDE
2. Import the existing project from `/Users/jonnyle/mcuxpresso-sdk`
3. Right-click → Debug As → MCU Debug
4. Select KW47-LOC board
5. The IDE will flash and run the firmware

### Method 2: LinkServer (NXP Tool)
```bash
# Install LinkServer if not already installed
# Then use:
LinkServer flash --device KW47B42ZB7 --binary build/freertos_hello_cm33_core0.bin
```

### Method 3: J-Link (with proper setup)
```bash
# Note: JLink had issues in our test, but should work with proper setup:
JLinkExe -device KW47B42ZB7 -if SWD -speed 4000
> connect
> loadfile build/freertos_hello_cm33_core0.bin
> r
> g
> exit
```

### Method 4: OpenOCD
```bash
# Install OpenOCD with KW47 support
openocd -f interface/cmsis-dap.cfg -f target/kw47z.cfg \
        -c "program build/freertos_hello_cm33_core0.bin verify reset exit"
```

## Testing After Flash

### 1. Serial Connection
- Baud rate: 115200
- Data bits: 8
- Stop bits: 1
- Parity: None
- Flow control: None

### 2. Expected Output
```
KW47 Keyless Entry System Starting...
Proximity State: Disconnected -> Monitoring
BLE Advertising Started
RSSI Filter Initialized
Waiting for device connection...
```

### 3. Test Procedure
1. **Power on** the KW47-LOC board
2. **Connect serial terminal** to see debug output
3. **Use phone BLE scanner** to detect "Digital Key" advertisement
4. **Approach board** slowly to see RSSI changes
5. **Stay stationary** for 2 seconds in proximity to trigger unlock

### 4. LED Indicators (if implemented)
- LED1: System active/power
- LED2: BLE advertising
- LED3: Device connected
- LED4: Proximity detected

## Current Implementation Status

✅ **Built Successfully** - Binary ready for flashing
✅ **RSSI Filtering** - Moving average + Kalman filter
✅ **State Machine** - Tiered proximity detection
✅ **2-Second Unlock** - Stationary detection implemented
🔄 **Flash & Test** - Ready for hardware testing

## Next Steps After Testing

1. **Verify RSSI thresholds** - Adjust if needed in `rssi_filter.h`
2. **Test state transitions** - Monitor serial output
3. **Add LED feedback** - Visual indicators for states
4. **Integrate accelerometer** - For motion detection
5. **Enable Channel Sounding** - When ready for CS testing

## Troubleshooting

- **No serial output**: Check USB connection and baud rate
- **No BLE advertisement**: Verify board power and reset
- **State not changing**: Check RSSI values and thresholds
- **Flash fails**: Ensure board is in debug mode

The firmware is built and ready! Use MCUXpresso IDE for the most reliable flashing experience.
