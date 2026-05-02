# Liquid Propulsion Port Documentation

## Overview

This document describes the port of liquid propulsion functionality from the Arduino-based `hotfire.ino` codebase into the ferda solid propulsion codebase. The port maintains backwards compatibility with existing solid propulsion functionality while adding new states and hardware abstractions for liquid propulsion operations.

## What Was Done

### 1. Hardware Abstraction Layer

Created four new hardware classes following the existing ferda pattern (hardware/hardware_simulation/hardware_test):

#### ValveControl (`include/ValveControl.hpp`)
- Manages 7 valves: 4 servos (Nitrogen, Purge, Main Ethanol, Main Nitrous) and 3 solenoids (ASI Ethanol, ASI Oxygen, Nitrogen Bleed)
- Provides `OpenValve()` and `CloseValve()` methods with state tracking
- Handles different control logic for servos vs. solenoids (servos use PWM, solenoids use GPIO)
- **Files created:**
  - `include/ValveControl.hpp`
  - `hardware/ValveControl.cpp` (uses pigpio)
  - `hardware_simulation/ValveControl.cpp` (simulation stubs)
  - `hardware_test/ValveControl.cpp` (test stubs)

#### SparkPlug (`include/SparkPlug.hpp`)
- Controls spark ignition system
- Manages relay control (GPIO) and PWM output for RPM signal
- **Files created:**
  - `include/SparkPlug.hpp`
  - `hardware/SparkPlug.cpp`
  - `hardware_simulation/SparkPlug.cpp`
  - `hardware_test/SparkPlug.cpp`

#### PressureTransducer (`include/PressureTransducer.hpp`)
- Reads 7 pressure transducers with different ranges:
  - High-pressure sensors (0-1000 PSI): Nitrogen Line, Ethanol Tank, Nitrous Line, Fuel Inlet, Fuel Outlet, Chamber Pressure
  - Low-pressure sensor (0-200 PSI): Oxygen Line
- Provides `ReadPSI()` and `ReadPSI2()` methods matching original Arduino API
- **Files created:**
  - `include/PressureTransducer.hpp`
  - `hardware/PressureTransducer.cpp` (⚠️ **ADC reading not implemented - see "What Still Needs to be Done"**)
  - `hardware_simulation/PressureTransducer.cpp`
  - `hardware_test/PressureTransducer.cpp`

#### LoadCell (`include/LoadCell.hpp`)
- Reads load cell sensor for thrust measurement
- Converts to pounds (lbs) matching original implementation
- **Files created:**
  - `include/LoadCell.hpp`
  - `hardware/LoadCell.cpp` (⚠️ **ADC reading not implemented - see "What Still Needs to be Done"**)
  - `hardware_simulation/LoadCell.cpp`
  - `hardware_test/LoadCell.cpp`

### 2. Configuration

#### MissionConstants.hpp Updates
Added pin configuration constants for all liquid propulsion hardware:
- Servo pins: `kNitrogenServoPin`, `kPurgeServoPin`, `kMainEthanolServoPin`, `kMainNitrousServoPin`
- Solenoid pins: `kASIEthanolPin`, `kASIOxygenPin`, `kNitrogenBleedPin`
- Spark pins: `kSparkPin`, `kRPMPin`
- PT sensor pins: `kNitrogenLinePTPin`, `kEthanolTankPTPin`, `kNitrousLinePTPin`, `kOxygenLinePTPin`, `kFuelInletPTPin`, `kFuelOutletPTPin`, `kChamberPressurePTPin`
- Load cell pin: `kLoadCellPin`
- Valve angles: `kValveClosedAngle` (179°), `kValveOpenAngle` (91°)

**⚠️ IMPORTANT:** All pin numbers are currently set to `0` as placeholders. These must be updated with actual hardware pin assignments before use.

### 3. State Machine Extensions

#### New States Added to `Mode::Phase` enum:
- `HotfireIdle` - Waiting state for liquid propulsion operations, handles valve/spark commands
- `ASITest` - Executes ASI test sequence (matches original timing: 300ms oxygen+spark, 2s ethanol, then shutdown)
- `WaterFlow` - Executes water flow test sequence (matches original timing: 2s nitrous, 3s ethanol, then shutdown)

#### State Implementation (`src/Mode.cpp`)
- `UpdateHotfireIdle()` - Handles all valve and spark commands via RF, can transition to test sequences
- `UpdateASITest()` - Implements ASI test sequence with proper timing
- `UpdateWaterFlow()` - Implements water flow sequence with proper timing
- All sequences include abort handling and proper cleanup

### 4. RF Command System

Extended `RF::Command` enum with liquid propulsion commands:
- Valve commands: `ValveNitrogenOpen/Close`, `ValvePurgeOpen/Close`, `ValveMainEthanolOpen/Close`, `ValveMainNitrousOpen/Close`, `ValveASIEthanolOpen/Close`, `ValveASIOxygenOpen/Close`, `ValveNitrogenBleedOpen/Close`
- Spark commands: `SparkOn`, `SparkOff`
- Sequence commands: `ASITest`, `WaterFlow`
- State transition: `HotfireIdle`

**Command String Format:** Maintains compatibility with original Arduino code:
- `"VALVE: nitrogen open"` → `ValveNitrogenOpen`
- `"SPARK: on"` → `SparkOn`
- `"asitest"` → `ASITest`
- `"waterflow"` → `WaterFlow`

### 5. Telemetry Integration

#### New Telemetry Method (`src/Telemetry.cpp`)
- `RfSendLiquidPropulsionData()` - Sends PT and load cell data in JSON format
- Data structure matches original binary packet (8 float values)
- Automatically called during liquid propulsion states at ~10ms intervals

#### Telemetry Data Format:
```json
{
  "data_type": "liquid_telem",
  "payload": [
    nitrogen_line_psi,      // 0-1000 PSI
    ethanol_tank_psi,       // 0-1000 PSI
    nitrous_line_psi,       // 0-1000 PSI
    oxygen_line_psi,        // 0-200 PSI
    fuel_inlet_psi,         // 0-1000 PSI
    fuel_outlet_psi,        // 0-1000 PSI
    chamber_pressure_psi,    // 0-1000 PSI
    load_cell_lbs           // Thrust in pounds
  ]
}
```

### 6. Main Program Integration

#### Hardware Initialization (`src/Main.cpp`)
- Creates instances of all liquid propulsion hardware classes
- Initializes hardware via `Mode::SetLiquidPropulsionHardware()`
- GPIO pin setup for all servos, solenoids, and spark control
- Servos initialized to closed position (179°)
- Solenoids initialized to closed state (HIGH for normally-closed, LOW for normally-open)
- Spark initialized to OFF state

## Architecture Decisions

### 1. Hardware Abstraction Pattern
**Decision:** Followed existing ferda pattern with separate implementations for hardware/simulation/test.

**Rationale:** 
- Maintains consistency with existing codebase
- Enables testing without hardware
- Supports software-in-the-loop simulation

**Implementation:**
- All hardware classes have three implementations matching existing pattern
- Simulation and test versions provide stubs/logging

### 2. Optional Hardware Initialization
**Decision:** Liquid propulsion hardware is optional and only initialized in `Main.cpp`, passed to `Mode` via setter method.

**Rationale:**
- Maintains backwards compatibility with solid propulsion
- Hardware only used when in liquid propulsion states
- Allows code to compile and run for solid propulsion missions

**Implementation:**
- `Mode` class stores hardware as optional pointers (default to `nullptr`)
- `SetLiquidPropulsionHardware()` method sets hardware instances
- All hardware access checks for `nullptr` before use

### 3. State-Based Command Handling
**Decision:** Valve and spark commands are handled within `HotfireIdle` state rather than globally.

**Rationale:**
- Commands are context-specific to liquid propulsion operations
- Keeps command handling organized and state-aware
- Prevents accidental valve operations during solid propulsion

**Implementation:**
- Commands processed in `UpdateHotfireIdle()`
- Commands ignored in other states (safe default)

### 4. Timing Preservation
**Decision:** Maintained exact timing from original `hotfire.ino` sequences.

**Rationale:**
- Original sequences were tested and validated
- Timing is critical for safe hotfire operations
- Reduces risk of introducing timing bugs

**Implementation:**
- ASITest: 300ms delay before ethanol, 2000ms ethanol duration, 300ms shutdown delay
- WaterFlow: 2000ms nitrous, 3000ms ethanol, then shutdown
- Uses `currentTime` tracking with static start times

### 5. Command String Compatibility
**Decision:** Maintained original command string format from Arduino code.

**Rationale:**
- Ground control software may already use these strings
- Reduces need for ground control updates
- Maintains operational familiarity

## Challenges and Solutions

### Challenge 1: Arduino to Raspberry Pi Hardware Differences

**Problem:**
- Original code used Arduino `Servo.h` library and `analogRead()`
- Raspberry Pi doesn't have native analog inputs or Arduino servo library
- Need to use pigpio for GPIO/servo control and external ADC for analog reading

**Solution:**
- **Servos:** Used `gpioServo()` from pigpio which takes pulse width in microseconds (1000-2000μs for 0-180°)
  - Conversion: `pulseWidth = 1000 + (angle * 1000 / 180)`
- **Solenoids:** Used `gpioWrite()` for digital control (matching original HIGH/LOW logic)
- **ADC:** Created placeholder functions with TODO comments - actual ADC implementation needed (see "What Still Needs to be Done")

**Files Affected:**
- `hardware/ValveControl.cpp` - Servo control conversion
- `hardware/SparkPlug.cpp` - PWM control using `gpioPWM()`
- `hardware/PressureTransducer.cpp` - ADC placeholder
- `hardware/LoadCell.cpp` - ADC placeholder

### Challenge 2: State Machine Integration

**Problem:**
- Original code was a simple `loop()` with command checking
- Ferda uses a state machine pattern
- Need to integrate liquid propulsion operations without breaking existing states

**Solution:**
- Created new states specifically for liquid propulsion
- Made hardware optional (pointers default to `nullptr`)
- State transitions only occur from appropriate states (e.g., `Idle` → `HotfireIdle`)
- All existing solid propulsion states remain unchanged

**Files Affected:**
- `include/Mode.hpp` - Added new states and hardware pointers
- `src/Mode.cpp` - Implemented new state update functions
- `src/Main.cpp` - Hardware initialization

### Challenge 3: Telemetry Data Format

**Problem:**
- Original code sent binary packets with header/footer
- Ferda uses JSON for telemetry
- Need to maintain data compatibility while using JSON format

**Solution:**
- Created separate telemetry method `RfSendLiquidPropulsionData()`
- Sends JSON with same data structure (8 float values)
- Maintains same sensor order and units as original
- Can be extended to send binary format if needed

**Files Affected:**
- `include/Telemetry.hpp` - Added new method declaration
- `src/Telemetry.cpp` - Implemented JSON telemetry method
- `src/Mode.cpp` - Calls telemetry method during liquid states

### Challenge 4: Pin Configuration Management

**Problem:**
- Original code had pin numbers scattered throughout
- Need centralized configuration for maintainability
- Pin numbers need to be easily changeable

**Solution:**
- Added all pin constants to `MissionConstants.hpp`
- All hardware classes reference constants instead of hardcoded values
- Single location for pin configuration
- Currently set to `0` as placeholders with clear documentation

**Files Affected:**
- `include/MissionConstants.hpp` - Added all pin constants
- All hardware implementation files - Use constants

### Challenge 5: Servo Angle Logic

**Problem:**
- Original code used `Servo.write(angle)` where angle is in degrees
- pigpio `gpioServo()` uses pulse width in microseconds
- Need to convert correctly

**Solution:**
- Standard servo pulse width: 1000μs (0°) to 2000μs (180°)
- Formula: `pulseWidth = 1000 + (angle * 1000 / 180)`
- Verified against existing TVC servo implementation in codebase
- Maintained original angles: 91° (open) and 179° (closed)

**Files Affected:**
- `hardware/ValveControl.cpp` - Servo control implementation

## What Still Needs to be Done

### 1. ADC Hardware Implementation ⚠️ **CRITICAL**

**Status:** Placeholder code exists, actual ADC reading not implemented

**Files Requiring Implementation:**
- `hardware/PressureTransducer.cpp` - `ReadSensor()` method (line ~16)
- `hardware/LoadCell.cpp` - `ReadSensor()` method (line ~8)

**What's Needed:**
The original Arduino code used `analogRead(pin)` which reads 0-1023 (10-bit ADC). Raspberry Pi doesn't have native analog inputs, so you'll need:

1. **External ADC Hardware:**
   - Common options: MCP3008 (SPI), ADS1115 (I2C), or similar
   - Determine which ADC you're using and how it's connected

2. **ADC Reading Implementation:**
   - Replace the placeholder `uint16_t rawVal = 0;` with actual ADC reading
   - Example for MCP3008 via SPI:
     ```cpp
     // Pseudo-code - implement based on your ADC
     uint16_t rawVal = readADC(pin); // Read from SPI/I2C ADC
     ```

3. **Voltage Conversion:**
   - The existing code correctly converts raw ADC value to voltage:
     ```cpp
     float rawVolt = (float)rawVal / 204.6; // 1023 / 5.0 = 204.6
     ```
   - This assumes 10-bit ADC (0-1023) mapping to 0-5V
   - Adjust if using different ADC resolution

4. **Pin Mapping:**
   - Map `MissionConstants` pin numbers to actual ADC channels
   - Update pin constants in `MissionConstants.hpp` with ADC channel numbers

**Testing:**
- Verify ADC readings match expected voltage ranges
- Calibrate sensor readings against known pressures/forces
- Test all 7 PT sensors and load cell

### 2. Pin Number Configuration ⚠️ **REQUIRED**

**Status:** All pins set to `0` as placeholders

**File:** `include/MissionConstants.hpp`

**What's Needed:**
Update all pin constants with actual hardware assignments:

```cpp
// Example - replace with your actual pin numbers
const int kNitrogenServoPin = 6;        // GPIO pin for nitrogen servo
const int kPurgeServoPin = 13;          // GPIO pin for purge servo
// ... etc for all pins
```

**Considerations:**
- GPIO pins on Raspberry Pi (BCM numbering)
- ADC channel numbers (if using external ADC)
- Ensure no pin conflicts with existing solid propulsion hardware
- Document pin assignments for future reference

### 3. Servo Calibration (If Needed)

**Status:** Using original angles (91° open, 179° closed)

**What's Needed:**
- Verify servo angles match your hardware
- Some servos may need calibration/adjustment
- Test valve open/close positions
- Adjust `kValveOpenAngle` and `kValveClosedAngle` if needed

**Files:**
- `include/MissionConstants.hpp` - Valve angle constants
- `hardware/ValveControl.cpp` - Servo control

### 4. Solenoid Logic Verification

**Status:** Implemented based on original code comments

**What's Needed:**
- Verify solenoid logic matches your hardware
- Original code had comments like "want to flip these" - verify actual behavior
- Test normally-open vs normally-closed valve behavior
- Confirm HIGH/LOW logic for each solenoid type

**Files:**
- `hardware/ValveControl.cpp` - Solenoid control (lines ~50-70)

### 5. Integration Testing

**Status:** Code compiles, but not tested on hardware

**What's Needed:**
1. **Unit Testing:**
   - Test each hardware class individually
   - Verify valve open/close operations
   - Test spark control
   - Verify sensor reading (once ADC implemented)

2. **State Machine Testing:**
   - Test state transitions (Idle → HotfireIdle → ASITest/WaterFlow)
   - Verify command handling in HotfireIdle
   - Test abort functionality

3. **Sequence Testing:**
   - Test ASITest sequence timing
   - Test WaterFlow sequence timing
   - Verify proper valve sequencing
   - Test abort during sequences

4. **Telemetry Testing:**
   - Verify sensor data transmission
   - Check data format matches ground control expectations
   - Test telemetry rate/performance

5. **End-to-End Testing:**
   - Full hotfire sequence with actual hardware
   - Verify all valves operate correctly
   - Verify sensor readings are accurate
   - Test ground control communication

### 6. Documentation Updates

**Status:** This document created, but may need updates

**What's Needed:**
- Update main README.md if liquid propulsion becomes standard
- Document pin assignments in a hardware configuration file
- Create operator's manual for liquid propulsion states
- Document any deviations from original hotfire.ino behavior

## Usage Guide

### Entering Liquid Propulsion Mode

1. **Start in Idle State:**
   - System starts in `Calibration` state
   - Transition to `Idle` via `Idle` command

2. **Enter HotfireIdle:**
   - Send RF command: `"HotfireIdle"`
   - System transitions to `HotfireIdle` state
   - Hardware is now ready for liquid propulsion operations

### Manual Valve Control

While in `HotfireIdle`, send commands via RF:
- `"VALVE: nitrogen open"` / `"VALVE: nitrogen close"`
- `"VALVE: purge open"` / `"VALVE: purge close"`
- `"VALVE: main ethanol open"` / `"VALVE: main ethanol close"`
- `"VALVE: main nitrous open"` / `"VALVE: main nitrous close"`
- `"VALVE: ASI ethanol open"` / `"VALVE: ASI ethanol close"`
- `"VALVE: ASI oxygen open"` / `"VALVE: ASI oxygen close"`
- `"VALVE: nitrogen bleed open"` / `"VALVE: nitrogen bleed close"`

### Spark Control

While in `HotfireIdle`:
- `"SPARK: on"` - Turns on spark ignition
- `"SPARK: off"` - Turns off spark ignition

### Running Test Sequences

While in `HotfireIdle`:
- `"asitest"` - Runs ASI test sequence (automated)
- `"waterflow"` - Runs water flow sequence (automated)

Sequences automatically return to `HotfireIdle` when complete.

### Abort

At any time, send `"ABORT"` command to immediately exit (closes all valves, turns off spark).

### Returning to Solid Propulsion

From `HotfireIdle`, send `"Idle"` to return to `Idle` state (solid propulsion mode).

## Code Structure Reference

### New Files Created

```
include/
  ├── ValveControl.hpp
  ├── SparkPlug.hpp
  ├── PressureTransducer.hpp
  └── LoadCell.hpp

hardware/
  ├── ValveControl.cpp
  ├── SparkPlug.cpp
  ├── PressureTransducer.cpp
  └── LoadCell.cpp

hardware_simulation/
  ├── ValveControl.cpp
  ├── SparkPlug.cpp
  ├── PressureTransducer.cpp
  └── LoadCell.cpp

hardware_test/
  ├── ValveControl.cpp
  ├── SparkPlug.cpp
  ├── PressureTransducer.cpp
  └── LoadCell.cpp
```

### Modified Files

```
include/
  ├── Mode.hpp              - Added new states, hardware pointers
  ├── RF.hpp                 - Added new commands
  ├── Telemetry.hpp          - Added liquid propulsion telemetry
  └── MissionConstants.hpp   - Added pin constants

src/
  ├── Mode.cpp               - Implemented new states
  ├── Telemetry.cpp          - Added telemetry method
  └── Main.cpp               - Hardware initialization
```

## Known Limitations

1. **ADC Not Implemented:** Pressure transducers and load cell won't read actual values until ADC code is added
2. **Pin Numbers Placeholder:** All pins set to 0 - must be configured before use
3. **No Error Handling:** ADC failures or hardware errors not currently handled
4. **Telemetry Format:** Uses JSON instead of original binary format (can be changed if needed)
5. **No Calibration:** Sensor calibration not implemented (may need offset/gain adjustments)

## Future Enhancements (Optional)

1. **Sensor Calibration:** Add calibration routines for PT sensors and load cell
2. **Error Handling:** Add robust error handling for hardware failures
3. **Safety Interlocks:** Add safety checks (e.g., prevent certain valve combinations)
4. **Data Logging:** Enhanced logging of valve operations and sensor data
5. **Binary Telemetry:** Option to send binary format matching original
6. **State Persistence:** Save/restore valve states across power cycles
7. **Advanced Sequences:** Additional pre-programmed sequences beyond ASITest/WaterFlow

## Contact and Support

For questions about this port:
- Review this documentation first
- Check code comments in implementation files
- Refer to original `hotfire.ino` for behavioral reference
- Test in simulation/test mode before hardware deployment

## Appendix: Original Code Mapping

| Original (hotfire.ino) | New (ferda) | Notes |
|------------------------|-------------|-------|
| `Servo NitrogenServo` | `ValveControl::Nitrogen` | Servo control via pigpio |
| `digitalWrite(asiEthanolPin, LOW)` | `ValveControl::OpenValve(ASIEthanol)` | Solenoid control |
| `analogRead(pin)` | `PressureTransducer::ReadSensor()` | ⚠️ Needs ADC implementation |
| `ReadSensor()` | `PressureTransducer::ReadPSI()` | 0-200 PSI range |
| `ReadSensor2()` | `PressureTransducer::ReadPSI2()` | 0-1000 PSI range |
| `ReadLoadCell()` | `LoadCell::ReadLBS()` | Thrust in pounds |
| `loop()` commands | `Mode::UpdateHotfireIdle()` | State-based handling |
| `asiTest()` | `Mode::UpdateASITest()` | Sequence state |
| `waterFlow()` | `Mode::UpdateWaterFlow()` | Sequence state |
| `SendData()` | `Telemetry::RfSendLiquidPropulsionData()` | JSON format |

---

**Document Version:** 1.2  
**Last Updated:** 12/10/2025  
**Author:** Port implementation and documentation organized by Benjamin Deutsch
**Reviewed By:** Kemuel Bermudez-Cotto

