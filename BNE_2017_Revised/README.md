# Brand New Extreme — 2017 Revised Control

This sketch preserves the 2017 hardware concept (Arduino + MPU-6050 + Bluetooth throttle + dual RC ESCs) while replacing the original roll-sign relay logic with a contact-aware hybrid controller.

## Control interpretation

The rider remains the roll-balance controller. Electronics do **not** attempt to hold roll at zero. Instead:

1. A second-order complementary observer estimates board roll.
2. Roll geometry estimates one of three mechanical contact modes: LEFT EDGE / CENTER CASTER / RIGHT EDGE.
3. Propulsion is allocated primarily to the wheel expected to have usable normal load.
4. Yaw rate is intentionally left free for slides and spins until a soft safety envelope is reached.
5. At the hard spin limit, propulsion is cut without blocking sensor updates; re-arm requires low yaw rate and low throttle.

## Nominal geometry

The initial values assume approximately 100 mm half-track and about 16 mm center-wheel rocker:

- edge engagement angle: 9 deg
- contact hysteresis: +/-1 deg
- full edge loading assumption: 13 deg

These are experimental starting points, not universal constants. A useful test set is 12 / 14 / 16 / 18 / 20 mm rocker.

## IMU convention

The revised sketch assumes:

- +X = board forward
- +Y = board left
- +Z = board up
- roll rate = gyro X
- yaw rate = gyro Z

The original `board.ino` used `gyro_z` both for roll integration and for yaw/spin detection. The revised sketch separates these axes. **Verify the actual MPU mounting and signs on the physical board before motor-on testing.**

## Filter

The final archived 2017 code used a PI-corrected second-order complementary observer. This revision keeps that design but makes integration use measured `dt`:

`roll_dot = gyro_roll + Kp * error + Ki * integral(error)`

Initial gains are Kp = 2, Ki = 1. The archived final commit had increased Kp to 4; retune from logged data rather than assuming the larger value is better.

## Spin envelope

Initial values:

- <= 150 deg/s: no intervention
- 150..200 deg/s: smooth propulsion fade
- >= 200 deg/s: hard propulsion cutoff
- re-arm below 80 deg/s, with throttle <= 5% for 500 ms

This preserves deliberate 180/360-degree spin behavior while preventing continued powered acceleration into a runaway spin.

## Bluetooth frame

Compatible with the archived controller protocol:

`a<0..255>z`

The receiver is non-blocking and only refreshes the failsafe heartbeat after a complete valid frame.

## ESC note

The sketch deliberately does **not** auto-run the old ESC calibration sequence. It maps normalized command 0..1 to 1190..1500 us and attaches Servo outputs with a safe 1000..2000 us library range. Confirm the actual ESC neutral/min/max pulse values on a bench before connecting drive belts or riding.

## Suggested validation order

1. Board powered with drive wheels off the ground: verify roll sign, contact mode and yaw sign in telemetry.
2. ESC disconnected: rotate/tilt the board and verify mode hysteresis and spin envelope.
3. ESC connected, belts/wheels unloaded: verify motor assignment for LEFT/RIGHT edge.
4. Low-power ground test with protective equipment and an external emergency stop.
5. Log roll, yaw rate, contact mode and motor command for rocker-height comparison.

The original root `.ino` files are intentionally left unchanged as the historical 2017 artifact.


## Phase 7 control/implementation audit

The first revised commit (`6218371fa59f5c6e813338471f7180403e27371f`) was audited against the hybrid-control model. Phase 7R addresses four implementation mismatches:

1. **Persistent IMU readiness:** a failed boot-time initialization/calibration can no longer be followed by an accidental arm after a later single successful I2C read. Reset is required after boot calibration failure.
2. **Safe FAILSAFE re-arm:** recovering from radio/IMU FAILSAFE now requires both low throttle and yaw rate below the spin re-arm threshold, so a communication dropout during a spin cannot bypass the spin safety condition.
3. **Continuous edge-entry command:** traction engagement now starts at the hysteresis ON angle (10 deg with the nominal settings), eliminating the deterministic ~15.6% engagement jump that occurred when CENTER changed to EDGE.
4. **Transmitter-rate compatibility:** the archived `CTRL.ino` sends only about every 200 ms. With a 250 ms receiver timeout that leaves almost no missed-frame margin. `controller_revised.ino` sends at 50 Hz and receives a one-byte `i` acknowledgement from the board.

## Remaining model/hardware limitations

These are deliberately *not* hidden by software assumptions:

- **SoftwareSerial at 115200 baud on an Uno is a hardware-validation risk**, especially while `Servo` interrupts and I2C/telemetry are active. The archived HC modules appear to have been configured for 115200, so Phase 7R keeps that rate for compatibility. If bench tests show dropped frames, reconfigure both Bluetooth modules to a lower common baud (e.g. 38400) rather than merely lengthening the failsafe timeout.
- **Six-axis IMU roll estimation during aggressive carving/spinning is imperfect.** Lateral/centripetal acceleration contaminates the accelerometer-derived roll reference. The second-order complementary observer is kept because it matches the historical design, but dynamic logs should be checked for false edge-state transitions. Adaptive accelerometer weighting would be a later experimental change, not assumed correct without data.
- `EDGE_ANGLE_DEG`, `FULL_EDGE_ANGLE_DEG`, and LEFT/RIGHT sign mapping are geometric estimates until measured on the physical board under rider load and deck/truck compliance.
- ESC stop/arming/maximum pulse values remain bench-test parameters. Automatic ESC calibration is intentionally not performed at boot.
- The MPU-6050 default ±250 deg/s gyro range is consistent with the current `/131` scale factor and places the 200 deg/s cutoff below saturation, but the actual register configuration should still be verified on hardware.
