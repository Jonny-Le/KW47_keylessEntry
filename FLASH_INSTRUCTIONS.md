# Flash Instructions for KW47 Keyless Entry

## Debug Probe

The KW47-LOC evaluation board has an **onboard DAPLink/CMSIS-DAP debug probe**. It does **not** use J-Link. Use **LinkServer** (bundled with MCUXpresso IDE) for flashing and debugging.

## Flashing Methods

### Method 1: MCUXpresso IDE (Recommended)
1. Connect the KW47-LOC board via USB
2. Open MCUXpresso IDE and import the project (see `BUILD_INSTRUCTIONS.md`)
3. Right-click project → Debug As → MCUXpresso IDE LinkServer
4. Select the KW47-LOC target
5. Click OK — the IDE will flash and start debugging

### Method 2: LinkServer CLI
```bash
# LinkServer is installed with MCUXpresso IDE
# Verify the board is detected:
LinkServer probes

# Flash the built binary:
LinkServer flash KW47B42ZB7:KW47-LOC load <path-to-build>/kw47_keyless_entry_kw47loc.bin
```

## Testing After Flash

### 1. Serial Connection
- The KW47-LOC debug USB also provides a virtual COM port
- Baud rate: 115200, 8N1, no flow control
- macOS: `screen /dev/tty.usbmodem* 115200`
- Linux: `screen /dev/ttyACM0 115200`

### 2. Expected Output
```
KW47 Keyless Entry System Starting...
Proximity State: Disconnected -> Monitoring
BLE Advertising Started
RSSI Filter Initialized
Waiting for device connection...
```

### 3. Test Procedure
1. **Power on** the KW47-LOC board via USB
2. **Connect serial terminal** to see debug output
3. **Use phone BLE scanner** (e.g., nRF Connect) to detect "Digital Key" advertisement
4. **Approach board** slowly to see RSSI changes
5. **Stay stationary** for 2 seconds in proximity to trigger unlock

## Current Implementation Status

- ✅ **RSSI Filtering** — Moving average + Kalman filter
- ✅ **State Machine** — Tiered proximity detection
- ✅ **2-Second Unlock** — Stationary detection implemented
- 🔄 **Flash & Test** — Ready for hardware testing

## Troubleshooting

- **Board not detected**: Check USB cable and try `LinkServer probes`
- **Flash fails**: Ensure no other debugger is connected; press the reset button
- **No serial output**: Verify baud rate (115200) and correct COM port
- **No BLE advertisement**: Verify board power and reset
- **State not changing**: Check RSSI threshold values in `kw47_keyless_entry/rssi_filter.h`
