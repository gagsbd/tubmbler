# Copilot Instructions for Tumblerv3

## Project Overview
Tumblerv3 is an Arduino-based self-balancing robot with dual-motor control, IMU-based balance stabilization, and autonomous navigation capabilities. The codebase uses command queueing for motion control and multiple operating modes (manual, obstacle avoidance, object following).

## Architecture

### Core Control Flow
1. **Main Loop** ([Tumbllerv3.ino](../Tumbllerv3.ino#L750-L850)): Executes ~200/second with microsecond timing. Processes command queue and updates motor outputs via `updateStatus()`.
2. **Command Queue** ([Tumbllerv3.ino](../Tumbllerv3.ino#L65-L125)): Circular buffer (MAX_COMMANDS=60) of motion directives with optional parameters. Commands execute sequentially.
3. **Balance Control** ([BalanceCar.h](../BalanceCar.h)): Dual feedback loop—Kalman filter estimates angle from MPU6050 accel+gyro, PID controller generates motor PWM.

### Key Components

**Kalman Filter** ([KalmanFilter.cpp](../KalmanFilter.cpp), [KalmanFilter.h](../KalmanFilter.h))
- Fuses accelerometer and gyroscope data (5ms interval, dt=0.005)
- Tuning parameters: Q_angle=0.001, Q_gyro=0.005, R_angle=0.5
- Output: `kalmanfilter_angle` used to detect balance state and trigger stability limits (±22°)

**Motor Control** ([BalanceCar.h](../BalanceCar.h#L80-L190))
- Three control modes: simple `carForward()`, `carBack()`, and `carForwardTrapezoidal()` (trapezoidal acceleration)
- **Critical for balance stability**: `carForwardTrapezoidal()` uses extended coast-down (500ms) at end of motion to maintain balance control—abrupt stops cause toppling. Uses quadratic decay curve to minimize pitch disturbance.
- **Straight-line correction**: Uses MPU6050 gyroscope Z-axis (yaw) to correct motor speeds during motion. Tracks target heading and applies differential motor corrections (PID-based: kp_yaw=0.5, ki_yaw=0.1, kd_yaw=0.2) to keep robot moving straight.
- Yaw is integrated every 5ms in `balanceCar()` from gyro data; `carForwardTrapezoidal()` reads it to apply course corrections.
- Deceleration rate reduced by factor of 2.5x to extend stopping time for self-balancing stability
- Resets motion control variables (`setting_car_speed`, `setting_turn_speed`) at end to hand off to balance control loop
- Trapezoidal mode syncs left/right encoders with proportional gain (syncGain=0.002)
- Direction pins: AIN1/BIN1 for polarity, PWMA_LEFT/PWMB_RIGHT for speed (0-255)
- Encoder sync requires reading `encoder_count_left_a` and `encoder_count_right_a` (volatile)

**Operating Modes** ([mode.h](../mode.h))
- **FUNCTION_MODE**: IDLE, IRREMOTE, OBSTACLE, FOLLOW, BLUETOOTH, FOLLOW2
- **MOTION_MODE**: STANDBY, FORWARD, BACKWARD, TURNLEFT, TURNRIGHT, STOP, START
- Modes decouple input source (IR, Bluetooth, button) from action direction

### Pin Mapping ([Pins.h](../Pins.h))
- Encoders: D2 (left), D4 (right)
- Motors: PWM D5/D6 (speeds), D7/D12 (directions), D8 (standby)
- Sensors: A3 (ultrasonic echo), D11 (trigger), A0-A1 (IR), A2 (voltage)
- RGB: D3 (NeoPixel, 4 pixels), D9 (IR send), D10 (mode button)

## Development Patterns

### Timing & Synchronization
- **Microsecond precision**: `usecElapsed = micros() - usLast` (loop iteration delta)
- **Timer pattern**: `msTimer*` variables accumulate `usecElapsed` for event scheduling (e.g., `timerRunTime`, `timerSonicRange`)
- **Button debounce**: 500ms threshold in [Command.h](../Command.h#L5-L13)

### PID Control Pattern
```cpp
// BalanceCar.h shows three PID chains:
double kp_balance = 55, kd_balance = 0.75;      // Angle to motor correction
double kp_speed = 10, ki_speed = 0.26;          // Target vs actual speed
double kp_turn = 2.5, kd_turn = 0.5;            // Rotation command
```
Each PID is tuned for specific robot dynamics (21mm/tick encoder, balance angle ±22°).

### Command Dispatch Pattern
- Commands added to queue with `cmdQueue.add(VEHICLE_FORWARD, param)`
- `firstScan()` detects new commands; subsequent `current()` calls reuse state
- `next()` advances queue; handle `VEHICLE_FINISHED` to prevent loops

### Sensor Fusion
- Encoders (interrupt-driven, volatile) track distance for motion feedback
- MPU6050 (I2C, 5ms updates via timer) feeds Kalman filter
- Voltage monitoring ([voltage.h](../voltage.h)) for low-battery cutoff

## Important Conventions

1. **Encoder counts**: `ENCODER_COUNTS_90_DEG = 1602`, `ENC_PER_MM = 3`—use for distance targets
2. **Speed values**: Motor commands use 0-255 PWM; tuned speeds (FORWARD=200, turns=80) are preset
3. **Angle limits**: Robot only operates stable between `balance_angle_min=-22°` and `balance_angle_max=22°`; outside range triggers STOP
4. **Interrupt safety**: Encoder counts are volatile; always read fresh values in control loops
5. **Serial debugging**: 9600 baud; use `Serial.print()` for telemetry (line count: 953 total)

## Build & Debug
- **Compiler**: Arduino IDE or PlatformIO (standard AVR toolchain)
- **Board**: Likely Arduino Mega (MsTimer2, dual PWM, 6 analog inputs required)
- **Uploads**: Standard .ino sketch—no special build steps
- **Testing motion**: Commands via `loadCommandQueue()` (implementation in [Tumbllerv3.ino](../Tumbllerv3.ino#L900)); Bluetooth keys ('f','b','l','i') trigger manual control

## Common Tasks

- **Tune balance**: Adjust `kp_balance`, `kd_balance` and Kalman params (Q_angle, Q_gyro, R_angle)
- **Add motion command**: Create constant (e.g., `#define VEHICLE_SPIN`), add case in switch, implement handler
- **Change sensor polling**: Modify timer thresholds (e.g., `timerMPU` for 5ms IMU sync)
- **Synchronize motors**: Tweak `syncGain` in `carForwardTrapezoidal()` or use separate speed targets
