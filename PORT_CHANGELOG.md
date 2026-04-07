# Liquid Propulsion Port - Changelog

Quick reference of files created and modified during the liquid propulsion port.

## Files Created (16 new files)

### Header Files (4)
- `include/ValveControl.hpp`
- `include/SparkPlug.hpp`
- `include/PressureTransducer.hpp`
- `include/LoadCell.hpp`

### Hardware Implementation Files (12)
- `hardware/ValveControl.cpp`
- `hardware/SparkPlug.cpp`
- `hardware/PressureTransducer.cpp`
- `hardware/LoadCell.cpp`
- `hardware_simulation/ValveControl.cpp`
- `hardware_simulation/SparkPlug.cpp`
- `hardware_simulation/PressureTransducer.cpp`
- `hardware_simulation/LoadCell.cpp`
- `hardware_test/ValveControl.cpp`
- `hardware_test/SparkPlug.cpp`
- `hardware_test/PressureTransducer.cpp`
- `hardware_test/LoadCell.cpp`

## Files Modified (7 files)

### Include Files (4)
1. `include/Mode.hpp` - Added 3 new states, hardware pointers, and `SetLiquidPropulsionHardware()` method
2. `include/RF.hpp` - Added 17 new commands to enum and extended `ParseCommand()`
3. `include/Telemetry.hpp` - Added `RfSendLiquidPropulsionData()` method declaration
4. `include/MissionConstants.hpp` - Added 17 pin/configuration constants

### Source Files (3)
1. `src/Mode.cpp` - Implemented 3 new state handlers and hardware integration
2. `src/Telemetry.cpp` - Implemented liquid propulsion sensor telemetry
3. `src/Main.cpp` - Added hardware initialization and GPIO setup

## Statistics

- **Total Lines Added:** ~1,500+ lines of code
- **New Classes:** 4 hardware abstraction classes
- **New States:** 3 state machine states
- **New Commands:** 17 RF commands
- **New Constants:** 17 pin/configuration constants

## Code Quality

- ✅ Follows existing ferda code patterns
- ✅ Maintains backwards compatibility
- ✅ No linter errors
- ✅ Consistent naming conventions
- ✅ Proper hardware abstraction (hardware/simulation/test)

## Testing Status

- ✅ Code compiles without errors
- ✅ No linter errors
- ⚠️ Not tested on actual hardware
- ⚠️ ADC reading not implemented (returns 0)
- ⚠️ Pin numbers not configured

---

**For detailed information, see `LIQUID_PROPULSION_PORT.md`**

