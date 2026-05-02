# Liquid Propulsion Quick Start Guide

## Quick Reference

### State Transitions
```
Calibration → (Idle) → Idle → (HotfireIdle) → HotfireIdle
                                                      ↓
                                              (asitest) → ASITest → HotfireIdle
                                              (waterflow) → WaterFlow → HotfireIdle
```

### Essential Commands

**State Control:**
- `"HotfireIdle"` - Enter liquid propulsion mode
- `"Idle"` - Return to solid propulsion mode
- `"ABORT"` - Emergency stop (closes all valves, turns off spark)

**Valve Control (while in HotfireIdle):**
- `"VALVE: [valve name] open"` / `"VALVE: [valve name] close"`
  - Available valves: `nitrogen`, `purge`, `main ethanol`, `main nitrous`, `ASI ethanol`, `ASI oxygen`, `nitrogen bleed`

**Spark Control:**
- `"SPARK: on"` / `"SPARK: off"`

**Test Sequences:**
- `"asitest"` - ASI test sequence
- `"waterflow"` - Water flow sequence

## Before First Use

### 1. Configure Pin Numbers
Edit `include/MissionConstants.hpp` and set all pin constants:
```cpp
const int kNitrogenServoPin = 6;  // Your actual GPIO pin
// ... set all other pins
```

### 2. Implement ADC Reading
Edit `hardware/PressureTransducer.cpp` and `hardware/LoadCell.cpp`:
- Replace `uint16_t rawVal = 0;` with actual ADC reading code
- See `LIQUID_PROPULSION_PORT.md` for details

### 3. Test Hardware
1. Build in Debug mode: `cmake -Bbuild -DCMAKE_BUILD_TYPE=Debug .`
2. Test valve operations individually
3. Verify sensor readings (after ADC implementation)

## Typical Operation Flow

1. **Startup:** System begins in `Calibration` state
2. **Go to Idle:** Send `"Idle"` command
3. **Enter Liquid Mode:** Send `"GoHotfireIdle"` command
4. **Manual Control:** Send valve/spark commands as needed
5. **Or Run Sequence:** Send `"asitest"` or `"waterflow"`
6. **Return to Idle:** Send `"Idle"` when done

## Troubleshooting

**Valves not responding:**
- Check pin numbers in `MissionConstants.hpp`
- Verify GPIO initialization in `Main.cpp`
- Check hardware connections

**Sensors reading zero:**
- ADC implementation required (see "What Still Needs to be Done" in main doc)
- Verify ADC hardware connections
- Check pin assignments

**Commands not working:**
- Must be in `HotfireIdle` state for valve/spark commands
- Check RF communication
- Verify command string format (case-sensitive)

## File Locations

- **Main Documentation:** `LIQUID_PROPULSION_PORT.md`
- **Pin Configuration:** `include/MissionConstants.hpp`
- **Hardware Classes:** `include/*.hpp` and `hardware/*.cpp`
- **State Logic:** `src/Mode.cpp`
- **Command Parsing:** `include/RF.hpp` and `hardware/RF.cpp`

## Need More Details?

See `LIQUID_PROPULSION_PORT.md` for comprehensive documentation.

